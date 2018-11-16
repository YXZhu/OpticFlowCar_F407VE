#include "servoctrl.h"

#define S1_OUT TIM_CHANNEL_1
#define S2_OUT TIM_CHANNEL_2
#define S3_OUT TIM_CHANNEL_4
#define SxTime htim3

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //大小限制
	
ServoCtrl_t ServoCtrlS1;
ServoCtrl_t ServoCtrlS2;
ServoCtrl_t ServoCtrlS3;
//uint32_t timer;
//enum 
//{
//	ServoS1 =0,
//	ServoS2,
//	ServoS3
//};

osThreadId ServoCtrlHandle;

uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange); 
uint8_t AngleRun(ServoCtrl_t *ServoCtr);
void ServoCtrlTask(void const * argument);

void ServoCtrlFreertosInit(void)
{
	/* 初始化舵机*/
	ServoCtrlS1.MinAngleSet = 0;
	ServoCtrlS1.MaxAngleSet = 180;
	ServoCtrlS1.ServoAngleSet = 180;
	ServoCtrlS1.ServoAngleNow = 180;
	ServoCtrlS1.ServoSpeed = 180;
	
	ServoCtrlS2.MinAngleSet = 15;
	ServoCtrlS2.MaxAngleSet = 90;
	ServoCtrlS2.ServoAngleSet = 15;
	ServoCtrlS2.ServoAngleNow = 15;
	ServoCtrlS2.ServoSpeed = 180;
	
	ServoCtrlS3.MinAngleSet = 18;
	ServoCtrlS3.MaxAngleSet = 180;
	ServoCtrlS3.ServoAngleSet = 95;
	ServoCtrlS2.ServoAngleNow = 95;
	ServoCtrlS3.ServoSpeed = 180;
	
  osThreadDef(ServoCtrlTask, ServoCtrlTask, osPriorityNormal, 0, 128);
  ServoCtrlHandle = osThreadCreate(osThread(ServoCtrlTask), NULL);
  
}
uint32_t plues = 1500;
void ServoCtrlTask(void const * argument)
{
	osEvent event;
	osDelay(500);
	HAL_TIM_PWM_Start_IT(&SxTime,S1_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S2_OUT);
	HAL_TIM_PWM_Start_IT(&SxTime,S3_OUT);
	
	//ServoCtrlS1.TimeLast = osKernelSysTick();
	//ServoCtrlS2.TimeLast = osKernelSysTick();
	//ServoCtrlS3.TimeLast = osKernelSysTick();
	
	__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
	__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
   
  for(;;)
  {

	  
	  event = osSignalWait(0x0f,40);
	  if(event.status == osEventSignal)
	  {
		  if(event.value.signals&0x01)
			 // __HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,plues);
				__HAL_TIM_SET_COMPARE(&SxTime,S1_OUT,Angle2Pulse(AngleRun(&ServoCtrlS1),0));
		  if(event.value.signals&0x02)
				__HAL_TIM_SET_COMPARE(&SxTime,S2_OUT,Angle2Pulse(AngleRun(&ServoCtrlS2),0));
		  if(event.value.signals&0x04)
				__HAL_TIM_SET_COMPARE(&SxTime,S3_OUT,Angle2Pulse(AngleRun(&ServoCtrlS3),0));
	  }
	  else if(event.status == osEventTimeout)
	  {
		  
	  }
    //osDelay(5);
  }
}

/* angle 角度 exchange 设置反方向为0度 */
uint32_t Angle2Pulse(uint8_t angle,uint8_t exchange)
{
	#define MaxPulse 2500.f
	#define MinPulse 600.f
	if(angle > 180 ) return 0;
	if(exchange)
		return  (uint32_t)((180 - angle)*((MaxPulse - MinPulse)/180.f)+MinPulse);
	else
		return (uint32_t)(angle*((MaxPulse - MinPulse)/180.f)+MinPulse);
}

/* 角度控制 速度控制 */
uint8_t AngleRun(ServoCtrl_t *ServoCtr)
//uint8_t AngleRun(uint8_t angleSet,uint8_t angleSpeed,uint8_t *nowangle,uint32_t *time)
{
	float temp;
	
	if(ServoCtr->ServoAngleSet > ServoCtr->MaxAngleSet) // 判断是不是在该舵机设置范围内
	{
		ServoCtr->ServoAngleSet = ServoCtr->MaxAngleSet;
	}
	else if(ServoCtr->ServoAngleSet < ServoCtr->MinAngleSet)
	{
		ServoCtr->ServoAngleSet = ServoCtr->MinAngleSet;
	}
	
	if(ServoCtr->ServoAngleSet == ServoCtr->ServoAngleNow)
	{
		ServoCtr->TimeLast = osKernelSysTick(); // 保存当前时间
		return ServoCtr->ServoAngleSet;
	}
	else
	{
//		if(ServoCtr->TimeLast == 0) // 未获取时间
//		{
//			ServoCtr->TimeLast = osKernelSysTick();
//			return ServoCtr->ServoAngleNow;
//		}
		
		temp = osKernelSysTick() - ServoCtr->TimeLast; // 获取当前时间之差
		temp = 1000.f / temp; //  计算当前设置数值
		ServoCtr->TimeLast = osKernelSysTick(); // 保存当前时间
		//if(temp == 0) return 255; // 计算速度过快
		temp = (float)ServoCtr->ServoSpeed / temp;
		
		if(ServoCtr->ServoAngleSet > ServoCtr->ServoAngleNow)
		{	
			temp = ServoCtr->ServoAngleNow + temp;	
			if(temp >= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		else
		{
			temp = ServoCtr->ServoAngleNow - temp;
			if(temp <= ServoCtr->ServoAngleSet) 
				ServoCtr->ServoAngleNow = ServoCtr->ServoAngleSet;	
			else
				ServoCtr->ServoAngleNow = temp;
		}
		return ServoCtr->ServoAngleNow;
	}
}

