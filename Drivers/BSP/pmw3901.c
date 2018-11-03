#include "pmw3901.h"
#include "spi.h"
#include "dwt_stm32_delay.h"
#include "string.h"

// Various time delay needed for PMW3901
#define TIME3901_us_TSWW         45
#define TIME3901_us_TSWR         45
#define TIME3901_us_TSRW_TSRR    20
#define TIME3901_us_TSRAD        35
#define TIME3901_us_TSCLK_NCS_W  35
#define TIME_us_TBEXIT        1    // should be 500ns, but smallest delay Arduino can do is 1us
#define TIME_us_TNCS_SCLK     1    // should be 120ns
#define TIME_us_TSCLK_NCS_R   1    // should be 120ns
// Bit Define
#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)
#define BIT7 (1 << 7)
#define BIT_SET(REG,MASK)          (REG |=  MASK)      //位设置1
#define BIT_CLEAR(REG,MASK)        (REG &= ~MASK)      //位设置0
#define IS_BIT_CLEAR(REG,MASK)    ((REG & MASK)==0x00)
#define IS_BIT_SET(REG,MASK)      ((REG & MASK)==MASK) //判断位为1

float lpfVal = 0.8f;						/*低通系数*/
float pmw3901SumX, pmw3901SumY;			/*累积像素*/
float pmw3901LpfX=0.f, pmw3901LpfY=0.f;	/*累积像素低通*/



MotionBurst_t MotionBurst;

osSemaphoreId Spi2TxRxComplete;
//osSemaphoreId pmw3901ReadIT;
osMutexId Spi2Mutex;
osThreadId pmw3901Handle;
void PMW3901Task(void const * argument);
void pmw3901_run(void);

void PMW3901FreertosInit(void)
{

	pmw3901_init();
	
	osSemaphoreDef(Spi2TxRxComplete);
	Spi2TxRxComplete = osSemaphoreCreate(osSemaphore(Spi2TxRxComplete),1);
	
	osMutexDef(Spi2Mutex);
	Spi2Mutex = osMutexCreate(osMutex(Spi2Mutex));	
	
	osThreadDef(PMW3901Task, PMW3901Task, osPriorityRealtime, 0, 128);
   pmw3901Handle = osThreadCreate(osThread(PMW3901Task), NULL);
	
	//osSemaphoreDef(pmw3901ReadIT);
	//pmw3901ReadIT = osSemaphoreCreate(osSemaphore(pmw3901ReadIT),1);
	
	
}

void PMW3901Task(void const * argument)
{
	
	pmw3901_run();
}
void spiBeginTransaction(void)
{
	osMutexWait(Spi2Mutex,portMAX_DELAY);
	//xSemaphoreTake(spiMutex, portMAX_DELAY);
}

void spiEndTransaction()
{
	osMutexRelease(Spi2Mutex);
	//xSemaphoreGive(spiMutex);
}
void spiExchange(size_t length, uint8_t * data_tx, uint8_t * data_rx)
{
	if(Spi2TxRxComplete != NULL)
	{
		HAL_SPI_TransmitReceive_DMA(&hspi2,data_tx,data_rx,length); //外部中断优先级需要比SPI中断优先级低
		if(osSemaphoreWait(Spi2TxRxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	}
	else
	{
		HAL_SPI_TransmitReceive(&hspi2,data_tx,data_rx,length,10);	
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == SPI2)
	{
		osSemaphoreRelease(Spi2TxRxComplete);
		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		
	}
}



void RegWrite(uint8_t reg, uint8_t value)
{
	//uint8_t send[2];
	reg |= 0x80u; // 最高位为1 写寄存器
	//send[0] = reg;
	//send[1] = value;
	//spiBeginTransaction();
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(TIME_us_TNCS_SCLK);
	//spiExchange(2, send, send);
	HAL_SPI_Transmit(&hspi2,&reg,1,10);
	//spiExchange(1, &reg, &reg);
	DWT_Delay_us(10);
	HAL_SPI_Transmit(&hspi2,&value,1,10);
	//spiExchange(1, &value, &value);
	DWT_Delay_us(TIME3901_us_TSCLK_NCS_W);
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
	//spiEndTransaction();
	DWT_Delay_us(TIME3901_us_TSWW);
}

uint8_t RegRead(uint8_t reg)
{
	uint8_t read = 0;
	reg &= 0x7f;  // 最高位为0 读寄存器
	//spiBeginTransaction();
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(TIME_us_TNCS_SCLK);
	
	HAL_SPI_Transmit(&hspi2,&reg,1,10);
	DWT_Delay_us(TIME3901_us_TSRAD);
	HAL_SPI_Receive(&hspi2,&read,1,10);
	
	//spiExchange(1,&reg,&read);
	DWT_Delay_us(TIME_us_TSCLK_NCS_R);
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);;
	//spiEndTransaction();
	return read;
}


void SwapBytes(unsigned char *c1, unsigned char *c2)
{
  char c = *c2;
  *c2 = *c1;
  *c1 = c;
}

void readMotion(MotionBurst_t * motion)
{
	uint8_t address = 0x16;
	//memset(motion,0x00,sizeof(MotionBurst_t));
   spiBeginTransaction();
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_RESET);
	DWT_Delay_us(TIME_us_TNCS_SCLK);
	spiExchange(1, &address, &address);
	//HAL_SPI_Transmit(&hspi2,&address,1,1);
	
	//DWT_Delay_us(TIME3901_us_TSRAD);
	spiExchange(sizeof(MotionBurst_t), (uint8_t *)motion, (uint8_t *)motion);	
	//HAL_SPI_Receive(&hspi2,(uint8_t *)motion,sizeof(MotionBurst_t),1);
	
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
	spiEndTransaction();
	uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
	realShutter |= (motion->shutter & 0x0ff) << 8;
	motion->shutter = realShutter;
	//SwapBytes((motion->shutter),motion->shutter);
	if ((motion->squal < 25) && (motion->shutter >= 0x1f00))
	{
		motion->deltaX = motion->deltaY = 0;
	}
}



static void InitRegisters(void)
{
	uint8_t r;
	uint8_t c1,c2;
	uint8_t oriention;
	
//	RegWrite(0x7F, 0x00);
//	RegWrite(0x55, 0x01);
//	RegWrite(0x50, 0x07);
//	RegWrite(0x7F, 0x0e);
//   RegWrite(0x43, 0x10);
//	
//	r = RegRead(0x67);
//	if (IS_BIT_SET(r, BIT7))  //if((r&0x40) == 0x40)
//	 RegWrite(0x48, 0x04);
//	else RegWrite(0x48, 0x02);
//	
//	RegWrite(0x7F, 0x00);
//	RegWrite(0x51, 0x7b);
//	RegWrite(0x50, 0x00);
//	RegWrite(0x55, 0x00);
//	
//	RegWrite(0x7F, 0x0e);
//	r = RegRead(0x73);
//	if(r==0x00)
//	{
//		
//		c1 = RegRead(0x70);
//		if(c1>28) c1 = c1 + 11;
//		else c1 = c1 + 14;
//		if(c1>0x3f) c1 = 0x3f;
//		c2 = RegRead(0x71);
//		c2 = ((unsigned short)c2*45)/100;
//		RegWrite(0x7F, 0x00);
//		RegWrite(0x61, 0xAD);
//		RegWrite(0x51, 0x70);
//		RegWrite(0x7F, 0x0e);
//		RegWrite(0x70, c1);
//		RegWrite(0x71, c2);
//	}
//	
	RegWrite(0x7F, 0x00);
	RegWrite(0x61, 0xAD);
	RegWrite(0x7F, 0x03);
	RegWrite(0x40, 0x00);
	RegWrite(0x7F, 0x05);
	RegWrite(0x41, 0xB3);
	RegWrite(0x43, 0xF1);
	RegWrite(0x45, 0x14);
	RegWrite(0x5B, 0x32);
	RegWrite(0x5F, 0x34);
	RegWrite(0x7B, 0x08);
	RegWrite(0x7F, 0x06);
	RegWrite(0x44, 0x1B);
	RegWrite(0x40, 0xBF);
	RegWrite(0x4E, 0x3F);
	RegWrite(0x7F, 0x08);
	RegWrite(0x65, 0x20);
	RegWrite(0x6A, 0x18);
	RegWrite(0x7F, 0x09);
	RegWrite(0x4F, 0xAF);
	RegWrite(0x5F, 0x40);
	RegWrite(0x48, 0x80);
	RegWrite(0x49, 0x80);
	RegWrite(0x57, 0x77);
	RegWrite(0x60, 0x78);
	RegWrite(0x61, 0x78);
	RegWrite(0x62, 0x08);
	RegWrite(0x63, 0x50);
	RegWrite(0x7F, 0x0A);
	RegWrite(0x45, 0x60);
	RegWrite(0x7F, 0x00);
	RegWrite(0x4D, 0x11);
	RegWrite(0x55, 0x80);
	RegWrite(0x74, 0x1F);
	RegWrite(0x75, 0x1F);
	RegWrite(0x4A, 0x78);
	RegWrite(0x4B, 0x78);
	RegWrite(0x44, 0x08);
	RegWrite(0x45, 0x50);
	RegWrite(0x64, 0xFF);
	RegWrite(0x65, 0x1F);
	RegWrite(0x7F, 0x14);
	RegWrite(0x65, 0x67);
	RegWrite(0x66, 0x08);
	RegWrite(0x63, 0x70);
	RegWrite(0x7F, 0x15);
	RegWrite(0x48, 0x48);
	RegWrite(0x7F, 0x07);
	RegWrite(0x41, 0x0D);
	RegWrite(0x43, 0x14);
	RegWrite(0x4B, 0x0E);
	RegWrite(0x45, 0x0F);
	RegWrite(0x44, 0x42);
	RegWrite(0x4C, 0x80);
	RegWrite(0x7F, 0x10);
	RegWrite(0x5B, 0x02);
	RegWrite(0x7F, 0x07);
	RegWrite(0x40, 0x41);
	RegWrite(0x70, 0x00);

	HAL_Delay(10-1); // delay 10ms

	RegWrite(0x32, 0x44);
	RegWrite(0x7F, 0x07);
	RegWrite(0x40, 0x40);
	RegWrite(0x7F, 0x06);
	RegWrite(0x62, 0xF0);
	RegWrite(0x63, 0x00);
	RegWrite(0x7F, 0x0D);
	RegWrite(0x48, 0xC0);
	RegWrite(0x6F, 0xD5);
	RegWrite(0x7F, 0x00);
	RegWrite(0x5B, 0xA0);
	RegWrite(0x4E, 0xA8);
	RegWrite(0x5A, 0x50);
	RegWrite(0x40, 0x80);
	
	/*设置xy数据输出方向*/
	oriention = 0x00;
	/* BIT5为1 反转X轴*/
	BIT_SET(oriention,BIT5);
	
	/* BIT6为1 反转Y轴*/
	BIT_SET(oriention,BIT6);
	
	/* BIT7为1 对调XY轴数据*/
	BIT_SET(oriention,BIT7);
	//HAL_Delay(10);
	RegWrite(0x5B, oriention);
	
	RegWrite(0x7F, 0x14);
	//r = RegRead(0x72);
	RegWrite(0x6F, 0x1C);
	RegWrite(0x7F, 0x00);
}
//float time, speedx;
void pmw3901_IT_Read(void)
{
	//static float time_t = 0.f,speedx_t,speedx_l;
	readMotion(&MotionBurst);
//	time = (HAL_GetTick() - time_t) / HAL_GetTickFreq();
//   time_t = HAL_GetTick();
	
	
	pmw3901SumX += MotionBurst.deltaX;
	pmw3901SumY += MotionBurst.deltaY;
	pmw3901LpfX += (pmw3901SumX - pmw3901LpfX) * lpfVal; // 低通滤波
	pmw3901LpfY += (pmw3901SumY - pmw3901LpfY) * lpfVal;
//	speedx_l = (flowPixelLpfX - speedx_t) / time ;
//	speedx += (speedx_l - speedx) * 0.08f;
//	speedx_t = flowPixelLpfX;
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == MOTION_Pin)
//	{
//		//pmw3901_IT_Read();
//		osSemaphoreRelease(pmw3901ReadIT);
//		HAL_GPIO_WritePin(D3_GPIO_Port,D3_Pin,GPIO_PIN_RESET);
//		//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	}
//}

//#include "usbd_cdc_if.h"
#include <stdio.h>
#include "motoctrl.h"
void pmw3901_init(void)
{
	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(RESET_GPIO_Port,RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port,RESET_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	RegWrite(0x3a, 0x5a); // 上电复位
	HAL_Delay(50);
	InitRegisters();
}
void pmw3901_run(void)
{
	//HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	//portENTER_CRITICAL();
	//PMW3901FreertosInit();
//	HAL_GPIO_WritePin(NSS_GPIO_Port,NSS_Pin,GPIO_PIN_SET);
////	HAL_GPIO_WritePin(NREST_GPIO_Port,NREST_Pin,GPIO_PIN_RESET);
////	osDelay(10);
////	HAL_GPIO_WritePin(NREST_GPIO_Port,NREST_Pin,GPIO_PIN_SET);
	//osDelay(10);
//	//portEXIT_CRITICAL();
//	RegWrite(0x3a, 0x5a); // 上电复位
//	osDelay(50);
//	InitRegisters();
	//HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	//portEXIT_CRITICAL();
	//HAL_SPI_Abort_IT(&hspi2);
	osEvent pmw3901Event;
	while (1)
	{
		pmw3901Event = osSignalWait(0x01,100);
		if(pmw3901Event.status == osEventSignal)
		{
			if(pmw3901Event.value.signals&0x01)
			{
				pmw3901_IT_Read();
				HAL_GPIO_WritePin(D3_GPIO_Port,D3_Pin,GPIO_PIN_SET);
			}
		}
		else if(pmw3901Event.status == osEventTimeout)
		{
			if(HAL_GPIO_ReadPin(MOTION_GPIO_Port,MOTION_Pin) == GPIO_PIN_SET)
			pmw3901_IT_Read();
			//else
			//HAL_SPI_Abort(&hspi2);
			HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
		}
		//osDelay(10);
	}
	//HAL_Delay(200);
}

