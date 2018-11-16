/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "tim.h"
#include "i2c.h"
#include "spi.h"
#include "motoctrl.h"
#include "mpu9250.h"
#include "pmw3901.h"
#include "vl53l0x.h"
#include "ov7670.h"
#include "GUI.h"
#include "DIALOG.h"
#include "DataDisplayDLG.h"
#include "EasyTracer.h"
#include "servoctrl.h"
//#include "picture.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern GUI_CONST_STORAGE GUI_BITMAP bm_20180519160039;
extern uint16_t ov7670ID;
extern uint32_t Frametime;
  uint8_t write,regc,data;
  uint16_t sx;uint16_t sy = 320;
  uint32_t X0; uint32_t Y0;
//extern WM_HWIN CreateWindow(void);
//extern WM_HWIN CreateCameraView(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t lightC;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId startTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint16_t ov7670Frame[160][120];
CCMRAM uint16_t ov7670FrameL[160][120];
RESULT Resu;
TARGET_CONDI Condition={40,60,130,235,155,230,45,45,160,160};  //黄色  {10,67,100,255,63,195,20,20,120,160};{40,60,130,235,155,230,20,20,120,160};
void exchangframelb(uint8_t *B,uint8_t *L,uint32_t num)
{
	for(uint32_t i = 0;i<num;i++)
	{
		L[i+1] = B[i];
		L[i] = B[i+1];
		i++;
	}
}
GUI_CONST_STORAGE GUI_BITMAP camera = {
  120, // xSize
  160, // ySize
  240, // BytesPerLine //每行的字节数
  16, // BitsPerPixel
  (unsigned char *)ov7670FrameL,  // Pointer to picture data
  NULL,  // Pointer to palette
  GUI_DRAW_BMPM565
};


uint8_t cap;
uint8_t SwDis;
/* USER CODE END FunctionPrototypes */

void StartTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//pmw3901_init();
	
	SPIFreertosInit();	
	I2CFreertosInit();
	//PMW3901FreertosInit();
	vl53l0xFreertosInit();
	//while(1); // 测试 减少代码
	OV7670FreertosInit();
	MPU9250FreertosInit();
	//vl53l0xFreertosInit();
	MotoCtrlFreertosInit();
	ServoCtrlFreertosInit();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of startTask */
  osThreadDef(startTask, StartTask, osPriorityNormal, 0, 512);
  startTaskHandle = osThreadCreate(osThread(startTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartTask */
/**
  * @brief  Function implementing the startTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask */
void StartTask(void const * argument)
{

  /* USER CODE BEGIN StartTask */
  /* Infinite loop */
	//portENTER_CRITICAL();
	WM_HWIN hWinDatadisplay;
	//  WM_HWIN hWin;
	//WM_HWIN hCameraView;
	//IMAGE_Handle hcamera;
	
	IMAGE_Handle hlogo;
	TEXT_Handle hyaw;
	TEXT_Handle hleftdis;
	TEXT_Handle hcenterdis;
	TEXT_Handle hrightdis;
	char temp[20];
	GUI_Init();
	//GUI_EnableAlpha(1);
	//WM_MULTIBUF_Enable(1);  /* 使能多缓冲 */
	GUI_SetColor(GUI_RED); 
	//GUI_SetTextMode(GUI_TM_TRANS); 
	//hWin = CreateWindow();
	hWinDatadisplay = CreateDataDisplay(); // 界面1 数据查看

	hlogo = WM_GetDialogItem(hWinDatadisplay,ID_IMAGE_0);
	hyaw = WM_GetDialogItem(hWinDatadisplay,ID_TEXT_0);
	hleftdis = WM_GetDialogItem(hWinDatadisplay,ID_TEXT_1);
	hcenterdis = WM_GetDialogItem(hWinDatadisplay,ID_TEXT_2);
	hrightdis = WM_GetDialogItem(hWinDatadisplay,ID_TEXT_3);
	
//	hCameraView = CreateCameraView(); // 界面2 摄像头
//	WM_HideWindow(hCameraView); // 隐藏界面2 默认界面1
//	hcamera = WM_GetDialogItem(hCameraView,GUI_ID_USER+1); // 获取摄像头图像句柄	
	
	IMAGE_SetBitmap(hlogo,&bm_20180519160039);
	//portEXIT_CRITICAL();
	/* 开外部中断 */
	GUI_Delay(50); // 延时刷屏
	//HAL_NVIC_EnableIRQ(EXTI1_IRQn);
   HAL_NVIC_EnableIRQ(EXTI4_IRQn);
   HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* 开lcd背光 */
	 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,40);
	uint8_t temp1[2];
	temp1[0] = 0x01;
	temp1[1] = 0xFe;
	//WM_HideWindow(hWinDatadisplay);
	osEvent StartTaskEvent;
	uint8_t result;
	 //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)ov7670Frame, 0x2580);
  for(;;)
  { 
	  if(write)
	  {
		  write = 0;
	     CAMERA_IO_Write(OV7670_I2C_ADDRESS, regc, data);
		  //ov7670_drv.Window_Set(OV7670_I2C_ADDRESS,sx,sy,120,160);	//设置窗口
	  }
	 // pmw3901_run();
	 // yawt = Yaw;
	  
	  
//		sprintf(temp,"Yaw: %3.2f %.2f",HeadingDegrees,Yaw);
//	  TEXT_SetText(hyaw,temp);
//	  sprintf(temp,"LeftDis: %3d mm",VL53L0XRangingMeasurementData[0].RangeMilliMeter);
//	  TEXT_SetText(hleftdis,temp);
//	  sprintf(temp,"CenterDis: %3d mm",VL53L0XRangingMeasurementData[1].RangeMilliMeter);
//	  TEXT_SetText(hcenterdis,temp);
//	  sprintf(temp,"RightDis: %x mm",ov7670ID);
//	  TEXT_SetText(hrightdis,temp);
	  //HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	
	 StartTaskEvent = osSignalWait(0x02,100);
	  if(StartTaskEvent.status == osEventSignal)
	  {
		  if(StartTaskEvent.value.signals&0x01)
		  {
			  HAL_GPIO_WritePin(F_CS_GPIO_Port,F_CS_Pin,GPIO_PIN_SET);
				exchangframelb((uint8_t *)ov7670Frame,(uint8_t *)ov7670FrameL,9600*4);
				  if(Trace(&Condition,&Resu))
				  {
						result = 1;
				  }
				 HAL_GPIO_WritePin(F_CS_GPIO_Port,F_CS_Pin,GPIO_PIN_RESET);	
			  if(SwDis == 1)
			  {
				  
				  sprintf(temp,"FPS: %2d",Frametime);
					GUI_DispStringHCenterAt(temp,50,10);
				  if(result)
				  {
					  
					  result = 0;
					  //GUI_DrawRect(Resu.x-Resu.h/2,Resu.y+Resu.h/2,Resu.x+Resu.h/2,Resu.y-Resu.h/2);
					  GUI_DrawCircle(Resu.x,Resu.y,Resu.h/2);
					  sprintf(temp,"X:%d,Y:%d\nH:%d",Resu.x,Resu.y,Resu.h);
					 // GUI_DispString(temp);
					  GUI_DispStringHCenterAt(temp,Resu.x,Resu.y);
				  }
				  //GUI_SelectLayer(0);
				// IMAGE_SetBitmap(hlogo,&camera);	
				 // GUI_DrawBitmapEx(&camera,4,0,0,0,0,0);
				  
				  GUI_DrawBitmap(&camera,4,0);
					//GUI_SelectLayer(1);
				  //IMAGE_SetBitmap(hcamera,&camera);
				 GUI_Delay(5);  //延迟刷屏 
				  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)ov7670Frame, 0x2580);
			  }
			  else if(SwDis == 0)
			  {
				   
				  sprintf(temp,"Yaw: %3.2f %.2f",HeadingDegrees,Yaw);
				  TEXT_SetText(hyaw,temp);
				  sprintf(temp,"%3d|%3d|%3d",VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter,vl53l0x_dis3,VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter);
				  TEXT_SetText(hleftdis,temp);
				  //sprintf(temp,"RightDis: %3d mm",VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter);
				  sprintf(temp,"%.3f,%.3f,%.3f",Yaw,Pitch,Roll);
				  TEXT_SetText(hcenterdis,temp);
				  sprintf(temp,"%d,X:%d,Y:%d",MotionBurst.squal,(int16_t)pmw3901LpfX,(int16_t)pmw3901LpfY);
				  TEXT_SetText(hrightdis,temp);
				  GUI_Delay(5);
			  }
			  else
			  {
				   HAL_UART_Abort(&huart5);
					temp1[0] = 0x01;
					temp1[1] = 0xFe;
				  HAL_UART_Transmit(&huart5,temp1,2,10);
					HAL_UART_Transmit_DMA(&huart5,(uint8_t *)ov7670Frame,sizeof(ov7670Frame));
					GUI_Delay(5000);
				  temp1[0] = 0xfe;
					temp1[1] = 0x01;	
				  HAL_UART_Transmit(&huart5,temp1,2,10);	
				   SwDis = 1;
			  }
			  
		  }
		  //if(StartTaskEvent.value.signals&0x02)
		  if(!HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin))
		  {
			 
			  if(SwDis == 0)
			  {
				  SwDis = 1;
				  //GUI_SetOrientation( GUI_MIRROR_Y|GUI_MIRROR_X|GUI_SWAP_XY);
				  WM_HideWindow(hWinDatadisplay);
				  //WM_ShowWindow(hCameraView);
				  
				  
				 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,50);
			  }
			  else
			  {
				  SwDis = 0;
				  //WM_HideWindow(hCameraView);
				  //GUI_SetOrientation( GUI_MIRROR_Y|GUI_MIRROR_X);
				  WM_ShowWindow(hWinDatadisplay);
				  
				  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,40);
			  }
			  GUI_Delay(200); //消抖
			  //osSignalClear(startTaskHandle,0x02);
			  //osSignalClear(
		  }
		  else if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin))	
		  {
			  static uint8_t switchlight;
				 //GUI_Delay(200);
				if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)) switchlight++;
//				if(switchlight > 20)
//				{
//					lightC = 0;
//					switchlight = 0;	
//				}					
				if(switchlight < 10 && switchlight>1)
				{
					switchlight = 0;
					lightC +=10;
					if(lightC > 100) lightC = 0;
				}
//				 if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin))
//				 {
//					 switchlight = lightC;
//					lightC +=10;
//					 if(lightC > 100) lightC = 10;
//				 }
//				 else	
//				 {
//					 if(!switchlight)
//					 lightC = 0;
//					 else 
//					 {
//						 //lightC = switchlight;
//						 switchlight = 0;
//					 }
//				 }						 
				 ov7670_drv.CameraLight_Set(lightC);
		  }
			
			
	  }
	  else if(StartTaskEvent.status == osEventTimeout)
	  {
		  osSignalSet(startTaskHandle,0x01); //按键界面刷新
		  //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)ov7670Frame, 0x2580);
		  HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	  }
	  static uint32_t X0t;
	  if(X0 != X0t)
	  {
		  X0t = X0;
	  HAL_DCMI_ConfigCrop(&hdcmi, X0*2, Y0, 120*2-1, 160-1); // x0 :0 ~ (320(X0) - 120)*2 Y0 : 640 -160 max 200
	  //HAL_DCMI_Resume(&hdcmi);
	  }
	 //GUI_Delay(50);
	  //IMAGE_SetBMPEx(hlogo,&camera,
//	  HAL_UART_Abort(&huart2);
//	  	temp1[0] = 0x01;
//	   temp1[1] = 0xFe;
//	  HAL_UART_Transmit(&huart2,temp1,2,10);
//		HAL_UART_Transmit_DMA(&huart2,ov7670Frame,sizeof(ov7670Frame));
//		GUI_Delay(5000);
//	  temp1[0] = 0xfe;
//	   temp1[1] = 0x01;
//	  HAL_UART_Transmit(&huart2,temp1,2,10);	
	  //GUI_Delay(100);
	  //HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
    //osDelay(100);
  }
  /* USER CODE END StartTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
