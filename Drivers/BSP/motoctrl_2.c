#include "freertos.h"
#include "motoctrl.h"
#include "pmw3901.h"
#include "vl53l0x.h"
#include "mpu9250.h"
#include "arm_math.h"

#define ABS(x)   ((x)>=0?(x):-(x))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //��С����

#define OpticFlow_Ctrl 0
#define debug_moto
union n16bit angle;
union cmd_t cmd;

/* ϵͳ������� */
osSemaphoreId motoctrlTx;
osThreadId motoctrlHandle;
osThreadId CarLogicCtrlHandle; // �����߼���������
osMutexId mototurnMute;  //����ת�以����

void MotoCtrlFreertosInit(void);
void CarLogicCtrlTask(void const * argument);
static void CameraServoInit(void); // ����ͷ������Ƴ�ʼ��
CCMRAM arm_pid_instance_f32 cameraservoctrl_pid;
#if OpticFlow_Ctrl == 1
CCMRAM arm_pid_instance_f32 speedyctrlpid;
CCMRAM arm_pid_instance_f32 speedxctrlpid;

static void SpeedXCtrl_OpticFlow_init(void);
static void SpeedXCtrl_OpticFlow(int16_t setspeedx);

static void SpeedYCtrl_OpticFlow_init(void);
static void SpeedYCtrl_OpticFlow(int16_t setspeedy);

int16_t spdy=400;
int16_t spdx=400;

#else
CCMRAM arm_pid_instance_f32 speedactrlpid_tim;
CCMRAM arm_pid_instance_f32 speedbctrlpid_tim;
CCMRAM arm_pid_instance_f32 directionctrlpid_yaw;  // ����������PID ʹ��YAW // ��ĳ���Ƕ�����
CCMRAM arm_pid_instance_f32 straightctrlpid;  // �켣����PID //��ǽ����
CCMRAM arm_pid_instance_f32 straightspeedctrlpid;  // �켣�ٶȿ��� 
/* ������ƺ��� */
static void DirectionCtrl_YAW(int16_t* D_angle ,uint8_t *reset);
/* A��PID���ƺ��� */
static void SpeedACtrl_Time(int8_t setspeedA); // setspeedA ��λ cm/s
/* B��PID���ƺ��� */
static void SpeedBCtrl_Time(int8_t setspeedB);
/* �켣���ƺ��� */  
static void StraightCtrl(uint8_t distance);  // �켣���� ��ǽ�� // ѭ��
void Moto_TimeInit(void);
#endif
uint8_t testmoto = 0; // testmoto


CCMRAM int16_t angle_Turn = 0; // �������з������� ����
CCMRAM uint8_t angle_TurnReset = 1; //���ӷ���λ ���õ�ǰ�Ƕ�Ϊ0

CCMRAM float speedA_Tim,speedB_Tim; // AB�ֵ�ǰ�ٶ�
CCMRAM int8_t spdC = 0;  // ���ó����ٶ� // 0 ~ 120
//CCMRAM float D_errorpx = 1.8f; // AB�ֲ������������ 1.8

CCMRAM uint16_t CameraServoAngleSet = 90; // ����ͷ����Ƕ����� ��0 - 180 ��
//uint8_t cal_tim =10;

void MotoCtrlFreertosInit(void)  // ������������ʼ������
{
#if OpticFlow_Ctrl == 0
	Moto_TimeInit(); // �������ٶ�ʱ�� ���벶��
#else
	 SpeedYCtrl_OpticFlow_init();
	 SpeedXCtrl_OpticFlow_init();
#endif	
	//CameraServoInit();
	
	osThreadDef(MotoCtrlTask, MotoCtrlTask, osPriorityHigh, 0, 512);
   motoctrlHandle = osThreadCreate(osThread(MotoCtrlTask), NULL);
	
	/* С���߼��������� */
	osThreadDef(CarLogicCtrlTask,CarLogicCtrlTask,osPriorityAboveNormal,0,512);
	CarLogicCtrlHandle = osThreadCreate(osThread(CarLogicCtrlTask),NULL);
	
	osSemaphoreDef(motoctrlTx);
	motoctrlTx = osSemaphoreCreate(osSemaphore(motoctrlTx),1);
	
	osMutexDef(mototurnMute);
	mototurnMute = osMutexCreate(osMutex(mototurnMute));
}
static void CameraServoInit()
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	cameraservoctrl_pid.Kp = 0.2;
	cameraservoctrl_pid.Ki = 0.007;
	cameraservoctrl_pid.Kd = 0;
	arm_pid_init_f32(&cameraservoctrl_pid, 1);
}

extern RESULT Resu;
#ifdef debug_moto
uint8_t cameradebug;
#endif
void CameraServoctrl()
{
	static int16_t outputerror;
	int16_t error;
	error = Resu.y;
	error = 60 - error;
	outputerror = arm_pid_f32(&cameraservoctrl_pid,error);
	
	CameraServoAngleSet =  90 + LIMIT(outputerror,-90,90);
	
	//__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,LIMIT((CameraServoAngleSet*10+500),500,2300)); // 500 - 2300 ��Ӧ 0 - 180��
	
	#ifdef debug_moto
	if(cameradebug == 1)
	{
		cameradebug = 0;
		arm_pid_init_f32(&cameraservoctrl_pid, 1);
		
	}
	#endif
	
}

void CarLogicCtrlTask(void const * argument)
{
	osEvent event;
	while(motoctrlHandle == NULL) osDelay(100);
	for(;;)
	{	
		event = osSignalWait(0x0f,100);
		if(event.status == osEventSignal)
		{
			if(event.value.signals&0x04) // ת������ͷ�ת�书��
			{
				osMutexRelease(mototurnMute);
			}
			if(event.value.signals&0x01)
			{
				StraightCtrl(13);	
			}
			if(event.value.signals&0x02)
			{
				
			}
		}
		else if(event.status == osEventTimeout)
		{
			
			
		}
		//StraightCtrl(13);	
		//osDelay(5);
	}
	
}
void MotoCtrlTask(void const * argument)
{
	uint32_t time;
#if OpticFlow_Ctrl == 0
	uint16_t moto_run;
	float spdA_t,spdB_t;
#else
	SpeedYCtrl_OpticFlow_init();
	SpeedXCtrl_OpticFlow_init();
#endif
	cmd.send.aa = 0xaa;  //����֡ͷ
	cmd.send.ed = 0xed;  //����֡β
	angle.i = 1500; //��ʼ��С��ת��Ƕ� angle��1350 ~ 1650
	cmd.send.SpeedA = 0;  //��ʼ����ߵ���ٶ�Ϊ0  speedA ��-100 ~ 100
	cmd.send.SpeedB = 0;  //��ʼ���ұߵ���ٶ�Ϊ0  speedB ��-100 ~ 100

	osDelay(200); // �ӳ٣����������ȶ�
	time = osKernelSysTick();
	for(;;)
	{
#if OpticFlow_Ctrl == 0	
		if(moto_run%1 == 0)
		{
			//StraightCtrl(13);	
		}
		if(moto_run%2 == 0)  // ÿXx5msִ��һ�� ���ݳ����ٶȵ��ڷ�Ӧ�ٶ� //(uint16_t)(-0.1667*spdA + 26.667) 
		{
			//CameraServoctrl();
			DirectionCtrl_YAW(&angle_Turn, &angle_TurnReset);
		}
		if(moto_run%5 == 0)  // ÿ10x5msִ��һ��
		{
			if(angle.i != 1500) // ���ټ���
			{
				float error; //���
				error = arm_pid_f32(&straightspeedctrlpid,1500 - angle.i);
				//spdA_t = spdC + error;
				//spdB_t = spdC - error;
				//error= (1500.f - angle.i)/150.f;
				//error = error / D_errorpx;
				spdA_t = spdC*(1+LIMIT(error,-1.2f,0));
				spdB_t = spdC*(1-LIMIT(error,0,1.2f));
				
			}
			SpeedACtrl_Time((int8_t)LIMIT(spdA_t,-100,100));   // �����޷�
			SpeedBCtrl_Time((int8_t)LIMIT(spdB_t,-100,100));
			
		}
		if(moto_run%80 == 0) // �����ת ת������ ÿ60x5msִ��һ��
		{
			static float speedA_old,speedB_old;
			if(speedA_old == speedA_Tim)
			{
				speedA_old = speedA_Tim = 0;
				//cmd.send.SpeedA = 0; 
			}
			else speedA_old = speedA_Tim;
			
			if(speedB_old == speedB_Tim)
			{
				speedB_old = speedB_Tim = 0;
				//cmd.send.SpeedB = 0;
			}
			else speedB_old = speedB_Tim;				
		}
		if(moto_run%400 == 0)
		{
//			static uint8_t a;
//			if(a)
//				spdA = spdB = 30,a=0;
//			else
//				spdA = spdB = 10,a=1;
		}
		if(moto_run >= 1000) moto_run = 0;
		else moto_run ++;
#else
		SpeedYCtrl_OpticFlow(spdy);
		SpeedXCtrl_OpticFlow(spdx);
#endif
		cmd.send.anglt_H = angle.c[1];
		cmd.send.anglt_L = angle.c[0];
		HAL_UART_Transmit_IT(&huart4,cmd.d,(sizeof(cmd.d)));
		osSemaphoreWait(motoctrlTx,100);	
		osDelayUntil(&time,5);
		

	}
}
uint8_t debug = 0;
#if OpticFlow_Ctrl == 0
CCMRAM float outpida,outpidb;

static void Moto_TimeInit(void)
{
	/* A��PID���� */
	speedactrlpid_tim.Kp = 1.05f;
	speedactrlpid_tim.Ki = 0.245f;
	speedactrlpid_tim.Kd = 0.83f;
	arm_pid_init_f32(&speedactrlpid_tim, 1);
	/* B��PID���� */
	speedbctrlpid_tim.Kp = 1.05f;  //0.358 0.45
	speedbctrlpid_tim.Ki = 0.245f; //0.0765 0.085
	speedbctrlpid_tim.Kd = 0.83f;  //0.178 0.25
	arm_pid_init_f32(&speedbctrlpid_tim, 1);
	
	/* �������PID���� */
	directionctrlpid_yaw.Kp = 5.1f; //5
	directionctrlpid_yaw.Ki = 0;
	directionctrlpid_yaw.Kd = 2.f;
	arm_pid_init_f32(&directionctrlpid_yaw, 1);
	
	/* �켣����PID���� */
	straightctrlpid.Kp = 3.5f;  //3.85
	straightctrlpid.Ki = 0.006f; //0.02
	straightctrlpid.Kd = 0.0f;  // 0.08
	arm_pid_init_f32(&straightctrlpid, 1);	
	
	/* �켣�ٶȿ���PID���� */
	straightspeedctrlpid.Kp = 0.0035f;
	straightspeedctrlpid.Ki = 0;
	straightspeedctrlpid.Kd = 0.00;
	arm_pid_init_f32(&straightspeedctrlpid, 1);	
	
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1); // ����AB�ֲ������벶��
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
	
}
 uint8_t dbug,sbug;

//#define LEFT_dis  VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter
//#define RIGHT_dis  VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter
//#define CENTER_dis  vl53l0x_dis3  // ��λmm
#define RunWide 40 // �ܵ���� ��λcm

uint16_t MaxRunDis = 35; //�����켣�����㷨���������� ��λcm
uint16_t FMinRunDis = 40; //�����켣�����㷨ǰ����С����
float DisError;   // �����������ܵ���Ե��� ��λcm
/* �켣���ƺ��� */  
// ���Ҵ�������� 14cm �ܵ���� 40cm 40-14 26 13  //

union flag_t StraightCtrlFlag;
#define SaveOldAngleFlag StraightCtrlFlag.flag0
#define SaveOldSpdCFlag StraightCtrlFlag.flag1
#define StartRunFlag StraightCtrlFlag.flag2
#define StartTrunFlag StraightCtrlFlag.flag3  // StartTrunFlag = 0 δת�� 1 ����ת��
#define UnlockMutexFlag StraightCtrlFlag.flag4 
#define TestStop StraightCtrlFlag.flag5
//#define TrunError // ����ת�����У��
int16_t outpidangle;   // PD �Ƕ�У����� //����Ϊ�� ���ҿ�Ϊ��
CCMRAM float pxs = 0.8f;
int16_t outpidspeed;  //�켣�ٶ���� PID

int16_t LEFT_dis,RIGHT_dis,CENTER_dis;  // ������ֵ���� ���ڱȽ�
CCMRAM float LEFT_disf,RIGHT_disf,CENTER_disf;  // ��ͨ�˲��������ֵתΪ����
CCMRAM float LEFT_disL ,RIGHT_disL,CENTER_disL; // ��ͨ�˲��������ֵ Ԥ��ֵ�����㷨��������
CCMRAM float angle_Turn_old;
//CCMRAM float testaa;
//extern osMessageQId VL53L0XDisMess;
//extern osPoolId VL53L0XDisPool;
int16_t spdC_old; 
uint8_t TurnNum = 0;  // ��¼ת�����
CCMRAM float anglecaltmp;

static void StraightCtrl(uint8_t distance)  // �켣���� ��ǽ�� // ѭ��
{
	//static int16_t spdC_old;
	static int8_t direction = 0;  // >0 ��ת <0 ��ת
	//static 
	static uint32_t timeTurn = 0; // ��¼ת��ʱ��
	static uint16_t timeSaveAngle; // ��¼���浱ǰ�Ƕ���Ҫ�ȴ���ʱ��
	
	/* ��ͨ�˲����� */
	LEFT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter - LEFT_disL)*pxs;
	RIGHT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter - RIGHT_disL)*pxs;
	CENTER_disL = vl53l0x_dis3;
	
	/* ת���� */ 
	LEFT_disf = LEFT_disL/10.f;//(LEFT_disL)*arm_cos_f32((HeadingDegrees - angle_Turn_old)*0.0174532925f)/10.f;  // ���������ǽǶ�У�� ������
	arm_abs_f32(&LEFT_disf,&LEFT_disf,1);
	RIGHT_disf = RIGHT_disL/10.f;//RIGHT_disL*arm_cos_f32((angle_Turn_old - HeadingDegrees)*0.0174532925f)/10.f;  
	arm_abs_f32(&RIGHT_disf,&RIGHT_disf,1);
	CENTER_disf = CENTER_disL/10.f;

	/* תΪ���� ���ж� */
	LEFT_dis =  LEFT_disf;
	RIGHT_dis = RIGHT_disf;
	CENTER_dis = CENTER_disf;

	if(SaveOldAngleFlag == 0) // ����֮ǰ�ĵĽǶȽ��нǶ�У��
	{
		SaveOldAngleFlag = 1;	
		angle_Turn_old = HeadingDegrees;
	}
	if(SaveOldSpdCFlag == 0)
	{
		SaveOldSpdCFlag = 1;
		spdC_old = spdC;
		osMutexWait(mototurnMute,0);
	}
	
	#ifdef debug_moto
	if(TestStop)
	{
		TestStop = 0;
		spdC = 0;
		spdC_old = 0;
	}
							//spdC = 40;  // �����ٶ�Ϊ40cm/s����
	#endif
	if(LEFT_dis < MaxRunDis || RIGHT_dis < MaxRunDis || CENTER_dis < FMinRunDis)  // ���Ϲ켣�㷨
	{
		//MaxRunDis = 20;
		
		if(!StartTrunFlag)  // ��StartTrunFlag = 0ʱ δת�� Ĭ���ٶ���ʻ
		{
			direction = 0;
			//spdC = spdC_old;
		}
		
		if(LEFT_dis < MaxRunDis)
		{ 
			if(RIGHT_dis < MaxRunDis)  // ���С��MaxRunDis �ұ�С��MaxRunDis ֱ�� ֱ��
			{
				if(direction == 0)
				{
					DisError = (LEFT_disf - RIGHT_disf)/2.f;  // ʹ����λ���ܵ��м�
				}
				else if(direction < 0)
					DisError = LEFT_disf - distance;  // ��ת��
				else
					DisError = distance - RIGHT_disf;
				if(CENTER_dis < FMinRunDis) // ����С��Ԥ��ֵ ǰ��С��Ԥ��ֵ
				{

				}
			}
			else  // ���С��MaxRunDis �ұߴ���MaxRunDis ������· ����
			{
				if(direction<=0) // ��ת��ʱ��ʹ�����У��
				DisError = LEFT_disf - distance;
					//StartTrunFlag = 0;
				if(CENTER_dis > FMinRunDis)
				{
					// �������
					if(TurnNum == 2 ||
					   TurnNum == 6 )		
					{
						//if((!StartTrunFlag) && StartRunFlag &&(!TrunErrorFlag))  // ת�� ת��ʹ�� StartRunFlag = 1
						if(osMutexWait(mototurnMute,0) == osOK)
						{
							StartTrunFlag = 1;
							TurnNum ++;
							angle_Turn_old = angle_Turn_old + 90;
							direction = 1;
							#ifndef debug_moto
							//spdC = 40;  // �����ٶ�Ϊ40cm/s����
							#endif
						}
					}
				}
				else
				{
					// �������
					if(TurnNum == 0 ||
					   TurnNum == 1 ||
						TurnNum == 10 )		
					{
						if(!StartRunFlag)
						{
							StartRunFlag = 1;
							osMutexRelease(mototurnMute);
						}
						//if(TurnNum<11)
						//if(!StartTrunFlag &&(!TrunErrorFlag))
						if(osMutexWait(mototurnMute,0) == osOK)
						{						
							//static uint8_t TurnNum = 0; // ��¼ת�����
							if((HAL_GetTick() - timeTurn) > 1000)
							{
								TurnNum ++;
								StartTrunFlag = 1;
								StartRunFlag = 1; // �߹�
								//#define Turnangle 90
								//if(TurnNum == 2 )
									//spdC = 60;
									angle_Turn_old = angle_Turn_old + 90,direction = 1;
								#ifndef debug_moto 
									//spdC = 60;
								#endif
								timeTurn = HAL_GetTick();
							}
						}
					}
				}					
			}
		}
		else  // ��ߴ���MaxRunDis������· ����
		{
			if(RIGHT_dis < MaxRunDis)  // ��ߴ���MaxRunDis �ұ�С��MaxRunDis
			{
				if(direction>=0) // ��ת��ʱ��ʹ���ұ�У��
				DisError = distance - RIGHT_disf;
				if(CENTER_dis > FMinRunDis)  // ǰ�����FMinRunDis
				{
					// �������
					if(TurnNum == 4 ||
					   TurnNum == 8 )		
					{
						//if((!StartTrunFlag) && StartRunFlag &&(!TrunErrorFlag))  // ת�� ת��ʹ�� StartRunFlag = 1
						if(osMutexWait(mototurnMute,0) == osOK)
						{
							if((HAL_GetTick() - timeTurn) > 1000)
							{
								StartTrunFlag = 1;
								TurnNum ++;
								angle_Turn_old = angle_Turn_old - 90;
								direction = -1;
								#ifndef debug_moto
								//spdC = 40;  // �����ٶ�Ϊ40cm/s����
								#endif
								timeTurn = HAL_GetTick();
							}
						}
					}
				}
			}
			else  // �������ߴ���MaxRunDis ����  �����ֲ�� 
			{
				 DisError = 0; //�������
				arm_pid_reset_f32(&straightctrlpid);
				// �������
				if(TurnNum == 3 ||
					TurnNum == 5 ||
					TurnNum == 7 ||
					TurnNum == 9 ||
					TurnNum == 11 )		
				{
					//if(!StartTrunFlag &&  StartRunFlag && (!TrunErrorFlag))
					if(osMutexWait(mototurnMute,0) == osOK)
					{
						//static uint32_t timeTurn = 0;
						if((HAL_GetTick() - timeTurn) > 1000)
						{
							
							StartTrunFlag = 1;
							if(TurnNum == 3 || TurnNum == 7 || TurnNum == 11)
								angle_Turn_old = angle_Turn_old - 90,direction = -1; // ��
							else
								angle_Turn_old = angle_Turn_old + 90,direction = 1;
							#ifndef debug_moto 
								//spdC = 60;
							#endif
							timeTurn = HAL_GetTick();
							TurnNum ++;
						}
					}
				}
				//return;
			}
		}
		
		if(ABS(DisError)<0.5f)
		{
			timeSaveAngle ++;  //����5msִ��һ��
			if(timeSaveAngle == 100) // �����ֱ��ʱ�䳬��1.5��
			{
				timeSaveAngle = 0;
				if(StartTrunFlag != 1)
				{
					//angle_TurnReset = 1;
					angle_Turn_old = HeadingDegrees;
				}
				//SaveOldAngleFlag = 0; // �ѵ�ǰ�Ƕ�����ΪĬ�Ͻ�
			}
			
		}
		else 
		{
//			static float HeadingDegrees_t,temp;
//			if(HeadingDegrees_t > HeadingDegrees)
//			{
//				temp =  ;
//			}
//			else
//			{
//				
//			}
//			HeadingDegrees_t = HeadingDegrees;
			timeSaveAngle = 0;
			
		}
		

		
		//if(ABS(outpidangle)<40)
			outpidangle = arm_pid_f32(&straightctrlpid,DisError);
//		else
//		{
//			if(ABS(DisError)<1.5f)
//			{
//				outpidangle = 0;
//				arm_pid_reset_f32(&straightctrlpid);
//			}
//		}
		
		if(sbug == 1)
		{
			sbug = 0;
			arm_pid_init_f32(&straightctrlpid,1);
		}
		else if(sbug == 2)
		{
			//dbug = 0;
			outpidangle  = 0;
			arm_pid_reset_f32(&straightctrlpid);
		}
	
		if(angle_Turn_old >= 360) angle_Turn_old = angle_Turn_old - 360;
		else if(angle_Turn_old < 0) angle_Turn_old = angle_Turn_old + 360;
		
		angle_Turn = angle_Turn_old - LIMIT(outpidangle,-30,30);
		//static int8_t speed;
		
		/* Ԥ���� */ 
		if(CENTER_dis < (FMinRunDis + 40) || TurnNum == 2 || TurnNum == 4 || TurnNum == 6 || TurnNum == 8 || TurnNum == 12)  // Ԥ����
		{				
			//outpidspeed = arm_pid_f32(&straightspeedctrlpid,40 - CENTER_dis);
			if(CENTER_dis < FMinRunDis-10)
			{
				//spdC = spdC_old;
			}
			else
				spdC = LIMIT((spdC_old*CENTER_disf/100.f),20,45);
		}	
		else spdC = spdC_old;
	}
	
	else //if(LEFT_dis > MaxRunDis && RIGHT_dis > MaxRunDis && CENTER_dis > FMinRunDis)// �������붼������ 
	{
		if(StartRunFlag == 1&& TurnNum == 12)  // �����߳��ܵ� //�Ѿ���ʼ
		{
			static uint16_t endtime;
			//spdC = 30;
			if(endtime>100) 
			{
				spdC = 0; //10 * ����ʱ�� ms��ͣ��
				spdC_old = 0;
			}
			else endtime ++;
				
		}
		else //���ÿ�ʼ�ٶ� ΪXX �ҵ��ܵ��Զ�����ΪĬ���ٶ�
		{
			#ifndef debug_moto
			spdC = 40;  // �����ٶ�Ϊ40cm/s����
			#endif	
		}
		spdC = 30;
		LEFT_disL = 300;
		RIGHT_disL = 500;
		//angle_Turn = 0;
		//outpidangle = 0;
		DisError = 0; //��λ����
		return;
	}

//	if(sbug == 1)
//	{
//		dbug = 0;
//		arm_pid_init_f32(&straightctrlpid,1);
//	}
//	else if(sbug == 2)
//	{
//		//dbug = 0;
//		arm_pid_reset_f32(&straightctrlpid);
//	}
}
	//uint8_t pd = 1; 
/* ������ƺ��� */	float HeadingDegrees_new;
	int16_t outpid ,D_angle_old;
float seterror,seterrorT;  // Ŀ�������ڵ����ֵ seterrorT ����ֵ
static void DirectionCtrl_YAW(int16_t *D_angle ,uint8_t *reset) // D_angle�� �Ƕ� 0 ~ 360 ��-180 ~ 180��-180 = 180 reset 1�� ��λ���Զ���λ0 D_angleҲ��0
{
	static float HeadingDegrees_old;  // ��ʼ�Ƕȼ�¼���� ��Ϊ���ӳ�ʼ�� 
	//static float HeadingDegrees_new;
	if(dbug == 1)
	{
		dbug = 0;
		arm_pid_init_f32(&directionctrlpid_yaw,1);
		arm_pid_init_f32(&straightspeedctrlpid, 1);
	}
	
	//if(ABS(outpid) >180 ) // ������λ
	//	arm_pid_reset_f32(&directionctrlpid_yaw);
	if(D_angle_old != *D_angle || *reset == 1)  // �жϽǶ������Ƿ�ı� �Ƿ���λ
	{
		
		if(*reset == 1)
		{
			*reset = 0;
			*D_angle = 0;
			D_angle_old = 0;
			HeadingDegrees_old = HeadingDegrees;
		}
		//outpid = 0;
		/*seterror = *D_angle - D_angle_old;
		if(seterror < -180 ) seterror = 360 + seterror;	
		else if(seterror > 180 ) seterror = -360 + seterror;
		outpid = 0;  // ���ٴ���ȸı䷽��ʱ outpid ���� //����һ�����Ƕ�һ�¸��Ƕ�ʱ����
		HeadingDegrees_new = seterror + HeadingDegrees_new;
		if(HeadingDegrees_new>=360) HeadingDegrees_new = HeadingDegrees_new - 360;*/
		//outpid = 0; // ������� PD������ ���� ֻ�����һ��
		HeadingDegrees_new = *D_angle + HeadingDegrees_old;  // ����Ŀ��Ƕ�
		if(HeadingDegrees_new < -180 ) HeadingDegrees_new = 360 + HeadingDegrees_new;	
		else if(HeadingDegrees_new > 180 ) HeadingDegrees_new = -360 + HeadingDegrees_new;
		//if(HeadingDegrees_new>=360) HeadingDegrees_new = HeadingDegrees_new - 360;
		D_angle_old = *D_angle;
	}
	

	/* ���Ԥ���� */
	seterror = HeadingDegrees_new - HeadingDegrees;
	if(seterror < -180 ) seterror = 360 + seterror;	
	else if(seterror > 180 ) seterror = -360 + seterror;
		
   outpid = arm_pid_f32(&directionctrlpid_yaw,seterror);  //PD���� ��������  if(ABS(outpid) <= 150)
	
	arm_abs_f32(&seterror,&seterrorT,1);  // ȡ����ֵ�ж� �Ƿ�ﵽĿ��Ƕ� ������ں��� ǰ�����seterror
	if(seterrorT < 2) 
	{
		if(StartRunFlag&&StartTrunFlag)
		{
			StartTrunFlag = 0;
			osSignalSet(CarLogicCtrlHandle,0x04);
			//UnlockMutexFlag = 1;
			//osMutexRelease(mototurnMute);
		}
		// д�뵽��Ԥ���Ƕ�ʱ���޸ı���
		
	}
	//if(ABS(outpid) > 150)  // �����ҴﵽĿ�� ��λPD����
//	{
//		arm_abs_f32(&seterror,&seterror,1);  // �Ѳ��ٴ���PD������ ȡ����ֵ���ж�
	//	if(seterrorT < 5) 
	//	{
	//		arm_pid_reset_f32(&directionctrlpid_yaw);
	//		outpid = 0;
	//	}
	//}
	if(spdC >= 0)
		angle.i = LIMIT(1500 - outpid,1300,1700);
	else
		angle.i = LIMIT(1500 + outpid,1300,1700);
}

/* A��PID���ƺ��� */
static void SpeedACtrl_Time(int8_t setspeedA)
{
	int8_t px = ABS(setspeedA);
	static int8_t setspeedA_old;
	float outpida_t;
	//int8_t px_o = ABS(setspeedA_old);
	arm_abs_f32(&outpida,&outpida_t,1);
	if(setspeedA_old/setspeedA<0 || setspeedA == 0 || outpida_t > 150) // �жϵ������ת �����仯���� ������λ
	{
		setspeedA_old = setspeedA;
		cmd.send.SpeedA = 0;
		outpida = 0;
		arm_pid_reset_f32(&speedactrlpid_tim);
		//arm_pid_init_f32(&speedactrlpid_tim, 1);
		return;
	}
	//else if(px > px_o && px - px_o >20) px = 0.5*px; // ���ò����仯���� �������ò���
	else setspeedA_old = setspeedA;
	outpida = arm_pid_f32(&speedactrlpid_tim,(px - speedA_Tim)*(-0.00625f*px+1.0625f)); ////��̬���� *(-0.01875f*px+2.6875f) //
	
	if(debug == 1) // �رյ�����
	{
		//debug = 0;
		cmd.send.SpeedA = 0;
		
	}
	else if(debug == 2)  // ��λPID�㷨
	{
		arm_pid_init_f32(&speedactrlpid_tim, 1);
		debug = 0;
	}
	else
	{
		//if(!setspeedA) cmd.send.SpeedA = 0;
		if(setspeedA < 0) 
		{
			outpida = - outpida;
			cmd.send.SpeedA = LIMIT(outpida,-100,10);
		}
		else
		{
			cmd.send.SpeedA = LIMIT(outpida,-10,100);
		}
	}
}

/* B��PID���ƺ��� */
static void SpeedBCtrl_Time(int8_t setspeedB)
{
	int8_t px = ABS(setspeedB);
	static int8_t setspeedB_old;
	float outpidb_t;
	//int8_t px_o = ABS(setspeedB_old);
	arm_abs_f32(&outpidb,&outpidb_t,1);
	if(setspeedB_old/setspeedB<0  || setspeedB == 0 || outpidb_t > 200) // �жϵ������ת
	{
		setspeedB_old = setspeedB;
		cmd.send.SpeedB = 0;
		outpidb  = 0;
		arm_pid_reset_f32(&speedbctrlpid_tim);
		//arm_pid_init_f32(&speedbctrlpid_tim, 1);
		return;
	}
	//else if(px > px_o && px - px_o >20) px = 0.5*px; // ���ò����仯���� �������ò���
	else setspeedB_old = setspeedB;
	outpidb = arm_pid_f32(&speedbctrlpid_tim,(px - speedB_Tim)*(-0.00625f*px+1.0625f)); //��̬���� ���PID�������2 ��setspeedB==10 ϵ��Ϊ2 *(-0.01875f*ABS(px)+2.6875f)
	if(debug == 1) // �رյ�����
	{
		//debug = 0;
		cmd.send.SpeedB = 0;
		
	}
	else if(debug == 2)  // ��λPID�㷨
	{
		arm_pid_init_f32(&speedbctrlpid_tim, 1);
		debug = 0;
	}
	else
	{
		//if(!setspeedB) cmd.send.SpeedB = 0;
		if(setspeedB < 0)
		{
			outpidb = - outpidb;
			cmd.send.SpeedB = LIMIT(outpidb,-100,10);
		}
		else cmd.send.SpeedB = LIMIT(outpidb,-10,100);
	}
}
/* 
	�������� 20�� speedA_Tim��ÿ��ʱ�� ��ֱ̥����6.6cm �ܳ���20.7345CM 
	ÿ��1.036cm �� �ٶȵ�λΪ cm/s
*/
void SpeedA_Callback(void) // A�� �ٶȻص����� 
{
	//speedA_Tim = 100;
	static uint8_t flag;
	static uint32_t speedA_tim_o,time;
	if(flag)
	{
		flag = 0;
		if((HAL_GetTick() -time)<=200) //δ����200ms
		{
			uint32_t speedA_t = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
			if(speedA_tim_o < speedA_t) speedA_Tim = speedA_t - speedA_tim_o;
			else speedA_Tim = (__HAL_TIM_GET_AUTORELOAD(&htim4) - speedA_tim_o + 1) + speedA_t;
			speedA_Tim = 103672.5f/(speedA_Tim);  //1.036725f/(speedA_Tim/100000.f)
		}
		else
		{
			speedA_Tim = 0;
		}
	}
	else
	{
		flag =~flag;
		time = HAL_GetTick(); // ��õ�ǰϵͳʱ��
		speedA_tim_o = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
	}
}

void SpeedB_Callback(void) // B�� �ٶȻص�����
{
	static uint8_t flag;
	static uint32_t speedB_tim_o,time;
	if(flag)
	{
		flag = 0;
		if((HAL_GetTick() - time)<=200) //δ����200ms
		{
			uint32_t speedB_t = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
			if(speedB_tim_o < speedB_t) speedB_Tim = speedB_t - speedB_tim_o;
			else speedB_Tim = (__HAL_TIM_GET_AUTORELOAD(&htim4) - speedB_tim_o + 1) + speedB_t;
			speedB_Tim = 103672.5f/(speedB_Tim);
		}
		else
		{
			speedB_Tim = 0;
		}
	}
	else
	{
		flag =~flag;
		time = HAL_GetTick(); // ��õ�ǰϵͳʱ��
		speedB_tim_o = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
	}
	
}

#else
/*С��Y��ֱ��PID��ʼ��*/
static void SpeedYCtrl_OpticFlow_init(void)
{
	speedyctrlpid.Kp = 0.298;
	speedyctrlpid.Ki = 0.001376;
	speedyctrlpid.Kd = 0.3722;
	arm_pid_init_f32(&speedyctrlpid, 1);
} 

CCMRAM float lastpmw3901Y,speedy,outpidy;
uint8_t spd;
static void SpeedYCtrl_OpticFlow(int16_t setspeedy)
{
	//static float lastpmw3901Y,speedy;
	speedy += (((pmw3901LpfY - lastpmw3901Y)*100) - speedy) * 0.08f;
	lastpmw3901Y = pmw3901LpfY;
	if(MotionBurst.squal>50 && ABS(speedy) >20) outpidy = arm_pid_f32(&speedyctrlpid,setspeedy/2.f - speedy);
	//else if(outpidy > 100) outpidy = 80;
	else outpidy = 0;
	//if(MotionBurst.maxRawData == 0&&MotionBurst.minRawData == 0) outpidy = 0;
	//if(outpidy<-2000) debug = 1;
	if(debug == 1) // �رյ�����
	{
		//debug = 0;
		cmd.send.SpeedA = cmd.send.SpeedB = 0;
		
	}
	else if(debug == 2)  // ��λPID�㷨
	{
		arm_pid_init_f32(&speedyctrlpid, 1);
		debug = 0;
	}
	else
	{
		spd = cmd.send.SpeedA = cmd.send.SpeedB = LIMIT(outpidy,-30,100);
	}
		
}
CCMRAM float lastpmw3901X,speedx,outpidx;

static void SpeedXCtrl_OpticFlow_init(void)
{
	speedxctrlpid.Kp = 0.298;
	speedxctrlpid.Ki = 0.001376;
	speedxctrlpid.Kd = 0.3722;
	arm_pid_init_f32(&speedxctrlpid, 1);
}

static void SpeedXCtrl_OpticFlow(int16_t setspeedx)
{
	speedx += (((pmw3901LpfX - lastpmw3901X)*100) - speedx) * 0.08f;
	lastpmw3901X = pmw3901LpfX;
	if(MotionBurst.squal>50 ) outpidx = arm_pid_f32(&speedxctrlpid,setspeedx/2.f - speedx);
	//else if(outpidx>150 ) outpidx = 149;
	else outpidx = 0;
	angle.i = 1500 - LIMIT(outpidx,-145,155);
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == UART4)
	{
		//TX_Uart1Ready=SET;
		osSemaphoreRelease(motoctrlTx);
	}
	
	
}
