#include "freertos.h"
#include "motoctrl.h"
#include "pmw3901.h"
#include "vl53l0x.h"
#include "mpu9250.h"
#include "arm_math.h"

#define ABS(x)   ((x)>=0?(x):-(x))
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) ) //大小限制

#define OpticFlow_Ctrl 0
#define debug_moto
union n16bit angle;
union cmd_t cmd;

/* 系统句柄定义 */
osSemaphoreId motoctrlTx;
osThreadId motoctrlHandle;
osThreadId CarLogicCtrlHandle; // 车子逻辑控制任务
osMutexId mototurnMute;  //车子转弯互斥锁

void MotoCtrlFreertosInit(void);
void CarLogicCtrlTask(void const * argument);
static void CameraServoInit(void); // 摄像头舵机控制初始化
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
CCMRAM arm_pid_instance_f32 directionctrlpid_yaw;  // 舵机方向控制PID 使用YAW // 朝某个角度运行
CCMRAM arm_pid_instance_f32 straightctrlpid;  // 轨迹控制PID //沿墙行走
CCMRAM arm_pid_instance_f32 straightspeedctrlpid;  // 轨迹速度控制 
/* 舵机控制函数 */
static void DirectionCtrl_YAW(int16_t* D_angle ,uint8_t *reset);
/* A轮PID控制函数 */
static void SpeedACtrl_Time(int8_t setspeedA); // setspeedA 单位 cm/s
/* B轮PID控制函数 */
static void SpeedBCtrl_Time(int8_t setspeedB);
/* 轨迹控制函数 */  
static void StraightCtrl(uint8_t distance);  // 轨迹控制 沿墙走 // 循迹
void Moto_TimeInit(void);
#endif
uint8_t testmoto = 0; // testmoto


CCMRAM int16_t angle_Turn = 0; // 车子运行方向设置 度数
CCMRAM uint8_t angle_TurnReset = 1; //车子方向复位 设置当前角度为0

CCMRAM float speedA_Tim,speedB_Tim; // AB轮当前速度
CCMRAM int8_t spdC = 0;  // 设置车子速度 // 0 ~ 120
//CCMRAM float D_errorpx = 1.8f; // AB轮差速误差带入比例 1.8

CCMRAM uint16_t CameraServoAngleSet = 90; // 摄像头舵机角度设置 ：0 - 180 °
//uint8_t cal_tim =10;

void MotoCtrlFreertosInit(void)  // 电机驱动任务初始化函数
{
#if OpticFlow_Ctrl == 0
	Moto_TimeInit(); // 开启测速定时器 输入捕获
#else
	 SpeedYCtrl_OpticFlow_init();
	 SpeedXCtrl_OpticFlow_init();
#endif	
	//CameraServoInit();
	
	osThreadDef(MotoCtrlTask, MotoCtrlTask, osPriorityHigh, 0, 512);
   motoctrlHandle = osThreadCreate(osThread(MotoCtrlTask), NULL);
	
	/* 小车逻辑控制任务 */
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
	
	//__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,LIMIT((CameraServoAngleSet*10+500),500,2300)); // 500 - 2300 对应 0 - 180°
	
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
			if(event.value.signals&0x04) // 转弯完成释放转弯功能
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
	cmd.send.aa = 0xaa;  //设置帧头
	cmd.send.ed = 0xed;  //设置帧尾
	angle.i = 1500; //初始化小车转向角度 angle：1350 ~ 1650
	cmd.send.SpeedA = 0;  //初始化左边电机速度为0  speedA ：-100 ~ 100
	cmd.send.SpeedB = 0;  //初始化右边电机速度为0  speedB ：-100 ~ 100

	osDelay(200); // 延迟，待传感器稳定
	time = osKernelSysTick();
	for(;;)
	{
#if OpticFlow_Ctrl == 0	
		if(moto_run%1 == 0)
		{
			//StraightCtrl(13);	
		}
		if(moto_run%2 == 0)  // 每Xx5ms执行一次 根据车子速度调节反应速度 //(uint16_t)(-0.1667*spdA + 26.667) 
		{
			//CameraServoctrl();
			DirectionCtrl_YAW(&angle_Turn, &angle_TurnReset);
		}
		if(moto_run%5 == 0)  // 每10x5ms执行一次
		{
			if(angle.i != 1500) // 差速计算
			{
				float error; //误差
				error = arm_pid_f32(&straightspeedctrlpid,1500 - angle.i);
				//spdA_t = spdC + error;
				//spdB_t = spdC - error;
				//error= (1500.f - angle.i)/150.f;
				//error = error / D_errorpx;
				spdA_t = spdC*(1+LIMIT(error,-1.2f,0));
				spdB_t = spdC*(1-LIMIT(error,0,1.2f));
				
			}
			SpeedACtrl_Time((int8_t)LIMIT(spdA_t,-100,100));   // 输入限幅
			SpeedBCtrl_Time((int8_t)LIMIT(spdB_t,-100,100));
			
		}
		if(moto_run%80 == 0) // 电机不转 转速置零 每60x5ms执行一次
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
	/* A轮PID参数 */
	speedactrlpid_tim.Kp = 1.05f;
	speedactrlpid_tim.Ki = 0.245f;
	speedactrlpid_tim.Kd = 0.83f;
	arm_pid_init_f32(&speedactrlpid_tim, 1);
	/* B轮PID参数 */
	speedbctrlpid_tim.Kp = 1.05f;  //0.358 0.45
	speedbctrlpid_tim.Ki = 0.245f; //0.0765 0.085
	speedbctrlpid_tim.Kd = 0.83f;  //0.178 0.25
	arm_pid_init_f32(&speedbctrlpid_tim, 1);
	
	/* 舵机控制PID参数 */
	directionctrlpid_yaw.Kp = 5.1f; //5
	directionctrlpid_yaw.Ki = 0;
	directionctrlpid_yaw.Kd = 2.f;
	arm_pid_init_f32(&directionctrlpid_yaw, 1);
	
	/* 轨迹控制PID参数 */
	straightctrlpid.Kp = 3.5f;  //3.85
	straightctrlpid.Ki = 0.006f; //0.02
	straightctrlpid.Kd = 0.0f;  // 0.08
	arm_pid_init_f32(&straightctrlpid, 1);	
	
	/* 轨迹速度控制PID参数 */
	straightspeedctrlpid.Kp = 0.0035f;
	straightspeedctrlpid.Ki = 0;
	straightspeedctrlpid.Kd = 0.00;
	arm_pid_init_f32(&straightspeedctrlpid, 1);	
	
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1); // 开启AB轮测速输入捕获
	HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_2);
	
}
 uint8_t dbug,sbug;

//#define LEFT_dis  VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter
//#define RIGHT_dis  VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter
//#define CENTER_dis  vl53l0x_dis3  // 单位mm
#define RunWide 40 // 跑道宽度 单位cm

uint16_t MaxRunDis = 35; //开启轨迹控制算法左右最大距离 单位cm
uint16_t FMinRunDis = 40; //开启轨迹控制算法前方最小距离
float DisError;   // 两传感器距跑道边缘误差 单位cm
/* 轨迹控制函数 */  
// 左右传感器间隔 14cm 跑道宽度 40cm 40-14 26 13  //

union flag_t StraightCtrlFlag;
#define SaveOldAngleFlag StraightCtrlFlag.flag0
#define SaveOldSpdCFlag StraightCtrlFlag.flag1
#define StartRunFlag StraightCtrlFlag.flag2
#define StartTrunFlag StraightCtrlFlag.flag3  // StartTrunFlag = 0 未转弯 1 正在转弯
#define UnlockMutexFlag StraightCtrlFlag.flag4 
#define TestStop StraightCtrlFlag.flag5
//#define TrunError // 开启转弯错误校正
int16_t outpidangle;   // PD 角度校正误差 //往左靠为正 往右靠为负
CCMRAM float pxs = 0.8f;
int16_t outpidspeed;  //轨迹速度输出 PID

int16_t LEFT_dis,RIGHT_dis,CENTER_dis;  // 距离数值整形 用于比较
CCMRAM float LEFT_disf,RIGHT_disf,CENTER_disf;  // 低通滤波后距离数值转为厘米
CCMRAM float LEFT_disL ,RIGHT_disL,CENTER_disL; // 低通滤波后距离数值 预赋值避免算法后面误判
CCMRAM float angle_Turn_old;
//CCMRAM float testaa;
//extern osMessageQId VL53L0XDisMess;
//extern osPoolId VL53L0XDisPool;
int16_t spdC_old; 
uint8_t TurnNum = 0;  // 记录转弯次数
CCMRAM float anglecaltmp;

static void StraightCtrl(uint8_t distance)  // 轨迹控制 沿墙走 // 循迹
{
	//static int16_t spdC_old;
	static int8_t direction = 0;  // >0 右转 <0 左转
	//static 
	static uint32_t timeTurn = 0; // 记录转弯时间
	static uint16_t timeSaveAngle; // 记录保存当前角度需要等待的时间
	
	/* 低通滤波处理 */
	LEFT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_LEFT].RangeMilliMeter - LEFT_disL)*pxs;
	RIGHT_disL += (VL53L0XRangingMeasurementData[VL53L0X_DEV_RIGHT].RangeMilliMeter - RIGHT_disL)*pxs;
	CENTER_disL = vl53l0x_dis3;
	
	/* 转厘米 */ 
	LEFT_disf = LEFT_disL/10.f;//(LEFT_disL)*arm_cos_f32((HeadingDegrees - angle_Turn_old)*0.0174532925f)/10.f;  // 加入陀螺仪角度校正 弧度制
	arm_abs_f32(&LEFT_disf,&LEFT_disf,1);
	RIGHT_disf = RIGHT_disL/10.f;//RIGHT_disL*arm_cos_f32((angle_Turn_old - HeadingDegrees)*0.0174532925f)/10.f;  
	arm_abs_f32(&RIGHT_disf,&RIGHT_disf,1);
	CENTER_disf = CENTER_disL/10.f;

	/* 转为整形 好判断 */
	LEFT_dis =  LEFT_disf;
	RIGHT_dis = RIGHT_disf;
	CENTER_dis = CENTER_disf;

	if(SaveOldAngleFlag == 0) // 保存之前的的角度进行角度校正
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
							//spdC = 40;  // 设置速度为40cm/s过弯
	#endif
	if(LEFT_dis < MaxRunDis || RIGHT_dis < MaxRunDis || CENTER_dis < FMinRunDis)  // 符合轨迹算法
	{
		//MaxRunDis = 20;
		
		if(!StartTrunFlag)  // 当StartTrunFlag = 0时 未转弯 默认速度行驶
		{
			direction = 0;
			//spdC = spdC_old;
		}
		
		if(LEFT_dis < MaxRunDis)
		{ 
			if(RIGHT_dis < MaxRunDis)  // 左边小于MaxRunDis 右边小于MaxRunDis 直线 直走
			{
				if(direction == 0)
				{
					DisError = (LEFT_disf - RIGHT_disf)/2.f;  // 使车子位于跑道中间
				}
				else if(direction < 0)
					DisError = LEFT_disf - distance;  // 左转弯
				else
					DisError = distance - RIGHT_disf;
				if(CENTER_dis < FMinRunDis) // 两边小于预设值 前面小于预设值
				{

				}
			}
			else  // 左边小于MaxRunDis 右边大于MaxRunDis 可能有路 快跑
			{
				if(direction<=0) // 右转弯时不使能左边校正
				DisError = LEFT_disf - distance;
					//StartTrunFlag = 0;
				if(CENTER_dis > FMinRunDis)
				{
					// 弯道计数
					if(TurnNum == 2 ||
					   TurnNum == 6 )		
					{
						//if((!StartTrunFlag) && StartRunFlag &&(!TrunErrorFlag))  // 转弯 转弯使能 StartRunFlag = 1
						if(osMutexWait(mototurnMute,0) == osOK)
						{
							StartTrunFlag = 1;
							TurnNum ++;
							angle_Turn_old = angle_Turn_old + 90;
							direction = 1;
							#ifndef debug_moto
							//spdC = 40;  // 设置速度为40cm/s过弯
							#endif
						}
					}
				}
				else
				{
					// 弯道计数
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
							//static uint8_t TurnNum = 0; // 记录转弯次数
							if((HAL_GetTick() - timeTurn) > 1000)
							{
								TurnNum ++;
								StartTrunFlag = 1;
								StartRunFlag = 1; // 走过
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
		else  // 左边大于MaxRunDis可能有路 快跑
		{
			if(RIGHT_dis < MaxRunDis)  // 左边大于MaxRunDis 右边小于MaxRunDis
			{
				if(direction>=0) // 左转弯时不使能右边校正
				DisError = distance - RIGHT_disf;
				if(CENTER_dis > FMinRunDis)  // 前面大于FMinRunDis
				{
					// 弯道计数
					if(TurnNum == 4 ||
					   TurnNum == 8 )		
					{
						//if((!StartTrunFlag) && StartRunFlag &&(!TrunErrorFlag))  // 转弯 转弯使能 StartRunFlag = 1
						if(osMutexWait(mototurnMute,0) == osOK)
						{
							if((HAL_GetTick() - timeTurn) > 1000)
							{
								StartTrunFlag = 1;
								TurnNum ++;
								angle_Turn_old = angle_Turn_old - 90;
								direction = -1;
								#ifndef debug_moto
								//spdC = 40;  // 设置速度为40cm/s过弯
								#endif
								timeTurn = HAL_GetTick();
							}
						}
					}
				}
			}
			else  // 左右两边大于MaxRunDis 可能  遇到分叉口 
			{
				 DisError = 0; //误差清零
				arm_pid_reset_f32(&straightctrlpid);
				// 弯道计数
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
								angle_Turn_old = angle_Turn_old - 90,direction = -1; // 左
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
			timeSaveAngle ++;  //程序5ms执行一次
			if(timeSaveAngle == 100) // 如果走直线时间超过1.5秒
			{
				timeSaveAngle = 0;
				if(StartTrunFlag != 1)
				{
					//angle_TurnReset = 1;
					angle_Turn_old = HeadingDegrees;
				}
				//SaveOldAngleFlag = 0; // 把当前角度设置为默认角
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
		
		/* 预减速 */ 
		if(CENTER_dis < (FMinRunDis + 40) || TurnNum == 2 || TurnNum == 4 || TurnNum == 6 || TurnNum == 8 || TurnNum == 12)  // 预减速
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
	
	else //if(LEFT_dis > MaxRunDis && RIGHT_dis > MaxRunDis && CENTER_dis > FMinRunDis)// 三个距离都不符合 
	{
		if(StartRunFlag == 1&& TurnNum == 12)  // 可能走出跑道 //已经开始
		{
			static uint16_t endtime;
			//spdC = 30;
			if(endtime>100) 
			{
				spdC = 0; //10 * 运行时间 ms后停车
				spdC_old = 0;
			}
			else endtime ++;
				
		}
		else //设置开始速度 为XX 找到跑道自动设置为默认速度
		{
			#ifndef debug_moto
			spdC = 40;  // 设置速度为40cm/s过弯
			#endif	
		}
		spdC = 30;
		LEFT_disL = 300;
		RIGHT_disL = 500;
		//angle_Turn = 0;
		//outpidangle = 0;
		DisError = 0; //复位参数
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
/* 舵机控制函数 */	float HeadingDegrees_new;
	int16_t outpid ,D_angle_old;
float seterror,seterrorT;  // 目标与现在的误差值 seterrorT 绝对值
static void DirectionCtrl_YAW(int16_t *D_angle ,uint8_t *reset) // D_angle： 角度 0 ~ 360 或-180 ~ 180，-180 = 180 reset 1： 复位并自动置位0 D_angle也置0
{
	static float HeadingDegrees_old;  // 初始角度记录变量 设为车子初始角 
	//static float HeadingDegrees_new;
	if(dbug == 1)
	{
		dbug = 0;
		arm_pid_init_f32(&directionctrlpid_yaw,1);
		arm_pid_init_f32(&straightspeedctrlpid, 1);
	}
	
	//if(ABS(outpid) >180 ) // 超调复位
	//	arm_pid_reset_f32(&directionctrlpid_yaw);
	if(D_angle_old != *D_angle || *reset == 1)  // 判断角度设置是否改变 是否方向复位
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
		outpid = 0;  // 快速大幅度改变方向时 outpid 清零 //或者一下正角度一下负角度时清零
		HeadingDegrees_new = seterror + HeadingDegrees_new;
		if(HeadingDegrees_new>=360) HeadingDegrees_new = HeadingDegrees_new - 360;*/
		//outpid = 0; // 输出清零 PD控制器 线性 只需计算一次
		HeadingDegrees_new = *D_angle + HeadingDegrees_old;  // 计算目标角度
		if(HeadingDegrees_new < -180 ) HeadingDegrees_new = 360 + HeadingDegrees_new;	
		else if(HeadingDegrees_new > 180 ) HeadingDegrees_new = -360 + HeadingDegrees_new;
		//if(HeadingDegrees_new>=360) HeadingDegrees_new = HeadingDegrees_new - 360;
		D_angle_old = *D_angle;
	}
	

	/* 误差预处理 */
	seterror = HeadingDegrees_new - HeadingDegrees;
	if(seterror < -180 ) seterror = 360 + seterror;	
	else if(seterror > 180 ) seterror = -360 + seterror;
		
   outpid = arm_pid_f32(&directionctrlpid_yaw,seterror);  //PD控制 过调不算  if(ABS(outpid) <= 150)
	
	arm_abs_f32(&seterror,&seterrorT,1);  // 取绝对值判断 是否达到目标角度 必须放在后面 前面计算seterror
	if(seterrorT < 2) 
	{
		if(StartRunFlag&&StartTrunFlag)
		{
			StartTrunFlag = 0;
			osSignalSet(CarLogicCtrlHandle,0x04);
			//UnlockMutexFlag = 1;
			//osMutexRelease(mototurnMute);
		}
		// 写入到达预定角度时的修改变量
		
	}
	//if(ABS(outpid) > 150)  // 过调且达到目标 复位PD控制
//	{
//		arm_abs_f32(&seterror,&seterror,1);  // 已不再带入PD函数， 取绝对值好判断
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

/* A轮PID控制函数 */
static void SpeedACtrl_Time(int8_t setspeedA)
{
	int8_t px = ABS(setspeedA);
	static int8_t setspeedA_old;
	float outpida_t;
	//int8_t px_o = ABS(setspeedA_old);
	arm_abs_f32(&outpida,&outpida_t,1);
	if(setspeedA_old/setspeedA<0 || setspeedA == 0 || outpida_t > 150) // 判断电机正反转 参数变化过大 参数复位
	{
		setspeedA_old = setspeedA;
		cmd.send.SpeedA = 0;
		outpida = 0;
		arm_pid_reset_f32(&speedactrlpid_tim);
		//arm_pid_init_f32(&speedactrlpid_tim, 1);
		return;
	}
	//else if(px > px_o && px - px_o >20) px = 0.5*px; // 设置参数变化过大 调节设置参数
	else setspeedA_old = setspeedA;
	outpida = arm_pid_f32(&speedactrlpid_tim,(px - speedA_Tim)*(-0.00625f*px+1.0625f)); ////动态参数 *(-0.01875f*px+2.6875f) //
	
	if(debug == 1) // 关闭电机输出
	{
		//debug = 0;
		cmd.send.SpeedA = 0;
		
	}
	else if(debug == 2)  // 复位PID算法
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

/* B轮PID控制函数 */
static void SpeedBCtrl_Time(int8_t setspeedB)
{
	int8_t px = ABS(setspeedB);
	static int8_t setspeedB_old;
	float outpidb_t;
	//int8_t px_o = ABS(setspeedB_old);
	arm_abs_f32(&outpidb,&outpidb_t,1);
	if(setspeedB_old/setspeedB<0  || setspeedB == 0 || outpidb_t > 200) // 判断电机正反转
	{
		setspeedB_old = setspeedB;
		cmd.send.SpeedB = 0;
		outpidb  = 0;
		arm_pid_reset_f32(&speedbctrlpid_tim);
		//arm_pid_init_f32(&speedbctrlpid_tim, 1);
		return;
	}
	//else if(px > px_o && px - px_o >20) px = 0.5*px; // 设置参数变化过大 调节设置参数
	else setspeedB_old = setspeedB;
	outpidb = arm_pid_f32(&speedbctrlpid_tim,(px - speedB_Tim)*(-0.00625f*px+1.0625f)); //动态参数 最大PID输入乘以2 当setspeedB==10 系数为2 *(-0.01875f*ABS(px)+2.6875f)
	if(debug == 1) // 关闭电机输出
	{
		//debug = 0;
		cmd.send.SpeedB = 0;
		
	}
	else if(debug == 2)  // 复位PID算法
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
	测速码盘 20格 speedA_Tim：每格时间 轮胎直径：6.6cm 周长：20.7345CM 
	每格1.036cm 则 速度单位为 cm/s
*/
void SpeedA_Callback(void) // A轮 速度回调函数 
{
	//speedA_Tim = 100;
	static uint8_t flag;
	static uint32_t speedA_tim_o,time;
	if(flag)
	{
		flag = 0;
		if((HAL_GetTick() -time)<=200) //未超过200ms
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
		time = HAL_GetTick(); // 获得当前系统时间
		speedA_tim_o = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
	}
}

void SpeedB_Callback(void) // B轮 速度回调函数
{
	static uint8_t flag;
	static uint32_t speedB_tim_o,time;
	if(flag)
	{
		flag = 0;
		if((HAL_GetTick() - time)<=200) //未超过200ms
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
		time = HAL_GetTick(); // 获得当前系统时间
		speedB_tim_o = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_2);
	}
	
}

#else
/*小车Y轴直走PID初始化*/
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
	if(debug == 1) // 关闭电机输出
	{
		//debug = 0;
		cmd.send.SpeedA = cmd.send.SpeedB = 0;
		
	}
	else if(debug == 2)  // 复位PID算法
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
