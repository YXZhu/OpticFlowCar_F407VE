#include "vl53l0x.h"

#define Delay HAL_Delay

osThreadId VL53L0XHandle;
//osMessageQId VL53L0XDisMess;
//osMessageQId VL53L0XDisCenter;
//osMessageQId VL53L0XDisRight;
//osPoolId VL53L0XDisPool;
//osMutexId VL53L0XMutex;  //i2c vl53l0x �����ź���
//#pragma arm section zidata = "ram" 
VL53L0X_Dev_t VL53L0XDevs[]={
        {.Id=VL53L0X_DEV_LEFT, .DevLetter='l', .I2cHandle=&hi2c2, .I2cDevAddr=0x52, .comms_type=1, .comms_speed_khz=400},
        {.Id=VL53L0X_DEV_CENTER, .DevLetter='c', .I2cHandle=&hi2c2, .I2cDevAddr=0x52, .comms_type=1, .comms_speed_khz=400},
        {.Id=VL53L0X_DEV_RIGHT, .DevLetter='r', .I2cHandle=&hi2c2, .I2cDevAddr=0x52, .comms_type=1, .comms_speed_khz=400},
};
_vl53l0x_adjust vl53l0x_adjust1[3];
VL53L0X_RangingMeasurementData_t VL53L0XRangingMeasurementData[3];  //�����������

//�ж�ģʽ�����ṹ��
AlrmMode_t AlarmModes[]={
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW , .Name="Lo" , .ThreshLow=1500<<16 ,  .ThreshHigh=0<<16  },
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH, .Name= "hi", .ThreshLow=0<<16   ,  .ThreshHigh=300<<16},
        { .VL53L0X_Mode = VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT , .Name="out", .ThreshLow=300<<16 ,  .ThreshHigh=400<<16},
};

//uint8_t Vl53l0xI2cDevAddr[3] = {0x53,0x54,0x55}; //vl53l0x���õ�ַ

void VL53L0XTask(void const * argument);
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr); //��ַ����
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);
void SetupSingleShot(VL53L0X_Dev_t *dev,RangingConfig_e rangingConfig); //����ģʽ
uint8_t VL53L0XInit(void);
void debug_printf(const char * sFormat, ...);
void SetupInterruptMode(VL53L0X_Dev_t *dev,RangingConfig_e rangingConfig);

void vl53l0xFreertosInit(void) // vl53l0x �����ʼ��
{
	VL53L0XInit();
	//vl53l0x_calibration_LCD();
	//vl53l0x_adjust(&VL53L0XDevs[VL53L0X_DEV_LEFT]);
	//osMutexDef(VL53L0XMutex);
	//VL53L0XMutex = osMutexCreate(osMutex(VL53L0XMutex));
	//osMessageQDef(VL53L0XDisMess,10,vl53l0x_dis_t);
	//VL53L0XDisMess = osMessageCreate(osMessageQ(VL53L0XDisMess),NULL);
//	
//	osMessageQDef(VL53L0XDisCenter,5,VL53L0X_RangingMeasurementData_t);	
//	VL53L0XDisLeft = osMessageCreate(osMessageQ(VL53L0XDisLeft),NULL);
//	
//	osMessageQDef(VL53L0XDisRight,5,VL53L0X_RangingMeasurementData_t);
//	VL53L0XDisLeft = osMessageCreate(osMessageQ(VL53L0XDisLeft),NULL);
	
	//osPoolDef(VL53L0XDisPool,10,vl53l0x_dis_t);
	//VL53L0XDisPool = osPoolCreate(osPool(VL53L0XDisPool));
	
	osThreadDef(VL53L0XTask, VL53L0XTask, osPriorityRealtime, 0, 256);
   VL53L0XHandle = osThreadCreate(osThread(VL53L0XTask), NULL); 
}

/*  osSignal == 0x01 vl53l0x_1 GPIO1_1 �ж϶�ȡ�ź�
*	 osSignal == 0x02 vl53l0x_2 GPIO1_2 �ж϶�ȡ�ź�
*	 osSignal == 0x04 vl53l0x_3 GPIO1_3 �ж϶�ȡ�ź�
*/
//extern float LEFT_disL ,RIGHT_disL,CENTER_disL; // ��ͨ�˲��������ֵ Ԥ��ֵ�����㷨��������
void VL53L0XTask(void const * argument)
{
	
	osEvent vl53l0xEvent;
	VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_LEFT]);// ��������
	//VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_CENTER]);// ��������
	VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_RIGHT]);// ��������
	vl53l0x_uart_rec();  // ��������l53l0x����
	uint8_t time[3];
	//vl53l0x_dis_t *disSend;
	extern osThreadId CarLogicCtrlHandle; // �����߼���������
	for(;;)
	{	
		
		vl53l0xEvent = osSignalWait(0x0F,50);
		if(vl53l0xEvent.status == osEventSignal)
		{
			if(vl53l0xEvent.value.signals&0x01)
			{
				VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[VL53L0X_DEV_LEFT],&VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT]);//��ȡ��������,������ʾ����
				VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_LEFT],0);//���VL53L0X�жϱ�־λ 
				time[VL53L0X_DEV_LEFT] = 0;
				//LEFT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter - LEFT_disL)*0.35f;
				
				//debug_printf("d: %3d mm\r\n",VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter);
			}
			if(vl53l0xEvent.value.signals&0x02)
			{
				VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[VL53L0X_DEV_CENTER],&VL53L0XRangingMeasurementData[VL53L0X_DEV_CENTER]);//��ȡ��������,������ʾ����
				VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_CENTER],0);//���VL53L0X�жϱ�־λ 
				time[VL53L0X_DEV_CENTER] = 0;
			}
			if(vl53l0xEvent.value.signals&0x04)
			{
				//RIGHT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter - RIGHT_disL)*0.35f;
				VL53L0X_GetRangingMeasurementData(&VL53L0XDevs[VL53L0X_DEV_RIGHT],&VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT]);//��ȡ��������,������ʾ����
				VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_RIGHT],0);//���VL53L0X�жϱ�־λ 
				time[VL53L0X_DEV_RIGHT] = 0;
			}
			
			if(time[VL53L0X_DEV_LEFT] > 2) // �жϴ������Ƿ���ϻ��߳�������
			{
				VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter = 2500; 
			}
			else time[VL53L0X_DEV_LEFT]++;
			
			if(time[VL53L0X_DEV_CENTER] > 2) // �жϴ������Ƿ���ϻ��߳�������
			{
				VL53L0XRangingMeasurementData[VL53L0X_DEV_CENTER].RangeMilliMeter = 2500; 
			}
			else time[VL53L0X_DEV_CENTER]++;
			
			if(time[VL53L0X_DEV_RIGHT] > 2) // �жϴ������Ƿ���ϻ��߳�������
			{
				VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter = 2500; 
			}
			else time[VL53L0X_DEV_RIGHT]++;
			osSignalSet(CarLogicCtrlHandle,0x01);
		}	
		else if(vl53l0xEvent.status == osEventTimeout)
		{			
			VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter = 2500; 
			VL53L0XRangingMeasurementData[VL53L0X_DEV_CENTER].RangeMilliMeter = 2500; 
			VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter = 2500;
			osSignalSet(CarLogicCtrlHandle,0x02);
			memset(time,0,sizeof(time));			
			//VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_LEFT],0);//���VL53L0X�жϱ�־λ 
			//VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_CENTER],0);//���VL53L0X�жϱ�־λ 
			//VL53L0X_ClearInterruptMask(&VL53L0XDevs[VL53L0X_DEV_RIGHT],0);//���VL53L0X�жϱ�־λ 
			 //HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
		}
//		disSend = osPoolAlloc(VL53L0XDisPool);
//		disSend->dis1 = VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter;
//		disSend->dis2 = vl53l0x_dis3;
//		disSend->dis3 = VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter;
//		osMessagePut(VL53L0XDisMess,(uint32_t)disSend,1);
		//osDelay(100);
	}
}


uint16_t vl53l0x_dis3;
union vl53l0x_rec_t vl53l0x_rec;

void vl53l0x_uart_rec()
{
	uint8_t sum;
	if(vl53l0x_rec.vl53l0x_3.flag1 == 0x5A && vl53l0x_rec.vl53l0x_3.flag2 == 0x5A)
	{
		sum = 0;
		for(uint8_t i=0; i<(vl53l0x_rec.recchar[3]+4); i++)
		{
			sum += vl53l0x_rec.recchar[i];
		}
		if(sum == vl53l0x_rec.vl53l0x_3.sum)
		{
			vl53l0x_dis3 = vl53l0x_rec.vl53l0x_3.dis_h<<8|vl53l0x_rec.vl53l0x_3.dis_l;
			vl53l0x_dis3 = vl53l0x_dis3*10; //��λתΪMM
			//osSignalSet(VL53L0XHandle,0x08);
		}	
	}
	//HAL_UART_Abort_IT(&huart2);
	HAL_UART_Receive_DMA(&huart2,vl53l0x_rec.recchar,sizeof(vl53l0x_rec.recchar));
}

void usart2_call(void)
{
	//HAL_NVIC_SystemReset();
	vl53l0x_uart_rec();
}


//VL53L0X ���ξ����������
//dev:�豸I2C�����ṹ��
//pdata:����������ݽṹ��
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,VL53L0X_RangingMeasurementData_t *pdata,char *buf)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t RangeStatus;
	
	status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);//ִ�е��β�ಢ��ȡ����������
	if(status !=VL53L0X_ERROR_NONE) return status;
   
	RangeStatus = pdata->RangeStatus;//��ȡ��ǰ����״̬
    memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH);
	VL53L0X_GetRangeStatusString(RangeStatus,buf);//���ݲ���״̬��ȡ״̬�ַ���
	
	//Distance_data = pdata->RangeMilliMeter;//�������һ�β���������
	
    return status;
}

uint8_t VL53L0XInit(void)
{
	HAL_GPIO_WritePin(XSHUT1_GPIO_Port,XSHUT1_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
	HAL_GPIO_WritePin(XSHUT2_GPIO_Port,XSHUT2_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
	HAL_GPIO_WritePin(XSHUT3_GPIO_Port,XSHUT3_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
	
	Delay(10);
	
	vl53l0x_init(&VL53L0XDevs[VL53L0X_DEV_LEFT]); 
	Delay(10);
	//vl53l0x_init(&VL53L0XDevs[VL53L0X_DEV_CENTER]);
	vl53l0x_init(&VL53L0XDevs[VL53L0X_DEV_RIGHT]);
	
	//SetupSingleShot(&VL53L0XDevs[VL53L0X_DEV_LEFT],DEFAULT);
	//SetupSingleShot(&VL53L0XDevs[VL53L0X_DEV_CENTER],DEFAULT);
	//SetupSingleShot(&VL53L0XDevs[VL53L0X_DEV_RIGHT],DEFAULT);
	Delay(10);
	SetupInterruptMode(&VL53L0XDevs[VL53L0X_DEV_LEFT],HIGH_SPEED);
	Delay(10);
	//SetupInterruptMode(&VL53L0XDevs[VL53L0X_DEV_CENTER],HIGH_SPEED);
	SetupInterruptMode(&VL53L0XDevs[VL53L0X_DEV_RIGHT],HIGH_SPEED);
	//Delay(10);
//	VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_LEFT]);//��������
//	VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_CENTER]);//��������
//	//VL53L0X_StartMeasurement(&VL53L0XDevs[VL53L0X_DEV_RIGHT]);//��������
	return HAL_OK;
}

void debug_printf(const char * sFormat, ...)
{
	SEGGER_RTT_printf(0,sFormat);
}



//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ
	switch(dev->Id)
	{
		case VL53L0X_DEV_LEFT:
		{
			HAL_GPIO_WritePin(XSHUT1_GPIO_Port,XSHUT1_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
			osDelay(20);
			HAL_GPIO_WritePin(XSHUT1_GPIO_Port,XSHUT1_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			osDelay(20);
		}
		break;
		case VL53L0X_DEV_CENTER:
		{
			HAL_GPIO_WritePin(XSHUT2_GPIO_Port,XSHUT2_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
			osDelay(20);	
			HAL_GPIO_WritePin(XSHUT2_GPIO_Port,XSHUT2_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			osDelay(20);
		}
		break;
		case VL53L0X_DEV_RIGHT:
		{
			HAL_GPIO_WritePin(XSHUT3_GPIO_Port,XSHUT3_Pin,GPIO_PIN_RESET); //ʧ��VL53L0X
			osDelay(20);	
			HAL_GPIO_WritePin(XSHUT3_GPIO_Port,XSHUT3_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			osDelay(20);
			
		}
		break;
		default: break;
	}
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit(dev);	
}

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	//uint8_t addr;
	if(dev->I2cDevAddr != 0x52) return VL53L0X_ERROR_NONE;
	switch(dev->Id)
	{
		case VL53L0X_DEV_LEFT:
		{
			//addr = 0x54;
			HAL_GPIO_WritePin(XSHUT1_GPIO_Port,XSHUT1_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			Delay(30);
		}
		break;
		case VL53L0X_DEV_CENTER:
		{
			//addr = 0x56;
			HAL_GPIO_WritePin(XSHUT2_GPIO_Port,XSHUT2_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			Delay(30);	
		}
		break;
		case VL53L0X_DEV_RIGHT:
		{
			//addr = 0x58;
			HAL_GPIO_WritePin(XSHUT3_GPIO_Port,XSHUT3_Pin,GPIO_PIN_SET); //ʹ��VL53L0X,�ô��������ڹ���
			Delay(30);	
		}
		break;	
		default: return VL53L0X_ERROR_NONE;
	}

   vl53l0x_Addr_set(dev,0x52+(dev->Id + 1)*2);//����VL53L0X������I2C��ַ
   if(Status!=VL53L0X_ERROR_NONE) goto error;
	Status = VL53L0X_DataInit(dev);//�豸��ʼ��
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	dev->Present = 1; //�豸����
	Delay(2);
	
//	//AT24CXX_Read(0,(uint8_t*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//��ȡ24c02�����У׼����,����У׼ Vl53l0x_data.adjustok==0xAA
//	if(vl53l0x_adjust[dev->Id].adjustok==0xAA)//��У׼
//	  AjustOK=1;	
//	else //ûУ׼	
//	  AjustOK=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//��ӡ������Ϣ
		return Status;
	}
  	
	return Status;
}

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	uint8_t sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
		return VL53L0X_ERROR_NONE;
	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//����I2C��׼ģʽ����
		goto set_error;
	}
	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//��ȡ�Ĵ�������
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//�����豸�µ�I2C��ַ
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//����I2C��ַ����
			goto set_error;
		}
		//�޸Ĳ����ṹ���I2C��ַ
		dev->I2cDevAddr = FinalAddress;
		//����µ�I2C��ַ��д�Ƿ�����
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//��I2C��ַ��д����
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		//print_pal_error(Status);//��ӡ������Ϣ
	}
	if(sta!=0)
	  debug_printf("sta:0x%x\r\n",sta);
	return Status;
}

void SetupInterruptMode(VL53L0X_Dev_t *dev,RangingConfig_e rangingConfig)
{
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;
	uint8_t Mode = 0;
	VL53L0X_StaticInit(dev);
	
	VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
	
	VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
	Delay(2);
	/* Ranging configuration */
	switch(rangingConfig) {
	case LONG_RANGE:
		signalLimit = (FixPoint1616_t)(0.1*65536);
		sigmaLimit = (FixPoint1616_t)(60*65536);
		timingBudget = 33000;
		preRangeVcselPeriod = 18;
		finalRangeVcselPeriod = 14;
		break;
	case HIGH_ACCURACY:
	signalLimit = (FixPoint1616_t)(0.25*65536);
	sigmaLimit = (FixPoint1616_t)(18*65536);
	timingBudget = 200000;
	preRangeVcselPeriod = 14;
	finalRangeVcselPeriod = 10;
	break;
	case HIGH_SPEED:
	signalLimit = (FixPoint1616_t)(0.25*65536);
	sigmaLimit = (FixPoint1616_t)(32*65536);
	timingBudget = 20000;
	preRangeVcselPeriod = 14;
	finalRangeVcselPeriod = 10;
	break;
	case DEFAULT:
	signalLimit = (FixPoint1616_t)(0.25*65536);
	sigmaLimit = (FixPoint1616_t)(18*65536);
	timingBudget = 33000;
	preRangeVcselPeriod = 14;
	finalRangeVcselPeriod = 10;
	break;
	default:
		debug_printf("Not Supported");
	}
	VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev, timingBudget);
	VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
	if( status ){
		debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
	}

	status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
	if( status ){
		debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
	}
				status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

		status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

			status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,  timingBudget);
			//status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,timingBudget);//�����ڲ����ڲ���ʱ��
			if( status ){
				debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
			}

		status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}

			status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
		if( status ){
			debug_printf("VL53L0X_PerformRefCalibration failed\n");
		}
	VL53L0X_StopMeasurement(dev);           // it is safer to do this while sensor is stopped
	 VL53L0X_SetInterruptThresholds(dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ,  AlarmModes[Mode].ThreshLow ,  AlarmModes[Mode].ThreshHigh);
	 status = VL53L0X_SetGpioConfig(dev, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, AlarmModes[Mode].VL53L0X_Mode, VL53L0X_INTERRUPTPOLARITY_LOW);
	 status = VL53L0X_ClearInterruptMask(dev, 0); // clear interrupt pending if any
}
/**
 *  Setup all detected sensors for single shot mode and setup ranging configuration
 */
//VL53L0X ����ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������ �жϲ���
void SetupSingleShot(VL53L0X_Dev_t *dev,RangingConfig_e rangingConfig){
 
    int status;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    uint32_t refSpadCount;
	uint8_t isApertureSpads;
	FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25*65536);
	FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18*65536);
	uint32_t timingBudget = 33000;
	uint8_t preRangeVcselPeriod = 14;
	uint8_t finalRangeVcselPeriod = 10;

	  if(dev->Present){
			status=VL53L0X_StaticInit(dev);
			if( status ){
				 debug_printf("VL53L0X_StaticInit %d failed\n",dev->Id);
			}

			status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
		if( status ){
			debug_printf("VL53L0X_PerformRefCalibration failed\n");
		}

		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
		if( status ){
			debug_printf("VL53L0X_PerformRefSpadManagement failed\n");
		}

			status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING); //ʹ����������ģʽ,//���β���
			if( status ){
				debug_printf("VL53L0X_SetDeviceMode failed\n");
			}
//			status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,timingBudget);//�����ڲ����ڲ���ʱ��
//			if( status ){
//				debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
//			}
			status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1); // Enable Sigma limit
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
		}

		status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1); // Enable Signa limit
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckEnable failed\n");
		}
		/* Ranging configuration */
			switch(rangingConfig) {
			case LONG_RANGE:
				signalLimit = (FixPoint1616_t)(0.1*65536);
				sigmaLimit = (FixPoint1616_t)(60*65536);
				timingBudget = 33000;
				preRangeVcselPeriod = 18;
				finalRangeVcselPeriod = 14;
				break;
			case HIGH_ACCURACY:
			signalLimit = (FixPoint1616_t)(0.25*65536);
			sigmaLimit = (FixPoint1616_t)(18*65536);
			timingBudget = 200000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
			case HIGH_SPEED:
			signalLimit = (FixPoint1616_t)(0.25*65536);
			sigmaLimit = (FixPoint1616_t)(32*65536);
			timingBudget = 20000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
			case DEFAULT:
			signalLimit = (FixPoint1616_t)(0.25*65536);
			sigmaLimit = (FixPoint1616_t)(18*65536);
			timingBudget = 33000;
			preRangeVcselPeriod = 14;
			finalRangeVcselPeriod = 10;
			break;
			default:
				debug_printf("Not Supported");
			}

			status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

		status = VL53L0X_SetLimitCheckValue(dev,  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
		if( status ){
			debug_printf("VL53L0X_SetLimitCheckValue failed\n");
		}

			status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,  timingBudget);
			//status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,timingBudget);//�����ڲ����ڲ���ʱ��
			if( status ){
				debug_printf("VL53L0X_SetMeasurementTimingBudgetMicroSeconds failed\n");
			}

			status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}

			status = VL53L0X_SetVcselPulsePeriod(dev,  VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
		if( status ){
			debug_printf("VL53L0X_SetVcselPulsePeriod failed\n");
		}
		status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
		if( status ){
			debug_printf("VL53L0X_PerformRefCalibration failed\n");
		}
		
//		status = VL53L0X_StopMeasurement(dev);//ֹͣ����
//		if( status ){
//			debug_printf("VL53L0X_PerformRefCalibration failed\n");
//		}		
//		 status = VL53L0X_SetInterruptThresholds(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,AlarmModes.ThreshLow, AlarmModes.ThreshHigh);//�趨�����ж��ϡ�����ֵ
//		if( status ){
//			debug_printf("VL53L0X_PerformRefCalibration failed\n");
//		}	
//		 status = VL53L0X_SetGpioConfig(dev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,AlarmModes.VL53L0X_Mode,VL53L0X_INTERRUPTPOLARITY_LOW);//�趨�����ж�ģʽ �½���
//		if( status ){
//			debug_printf("VL53L0X_PerformRefCalibration failed\n");
//		}	
//		 status = VL53L0X_ClearInterruptMask(dev,0);//���VL53L0X�жϱ�־λ
//		if( status ){
//			debug_printf("VL53L0X_PerformRefCalibration failed\n");
//		}	
			dev->LeakyFirst=1;
		 //VL53L0X_StartMeasurement(dev);//��������
	  }
}
