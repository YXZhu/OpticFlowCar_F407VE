#ifndef __motoctrl_H
#define __motoctrl_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "stm32f1xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"
#include "EasyTracer.h"
#if defined(__CC_ARM) 
	#pragma anon_unions	/*用于支持结构体联合体*/
#endif
typedef struct
{
  uint8_t aa;
  int8_t SpeedA;
  int8_t SpeedB;
  uint8_t anglt_H;
  uint8_t anglt_L;
  //uint16_t c;
  uint8_t ed;
}rec_t; //stm8为大端模式

union cmd_t
{ 
  
  uint8_t d[sizeof(rec_t)];
	rec_t send;
 
};
union flag_t  // 标志联合体
{
     unsigned char flag;
     struct 
     {
          unsigned char flag0:1,
                        flag1:1,
                        flag2:1,  
                        flag3:1,
                        flag4:1,
                        flag5:1,
                        flag6:1,
                        flag7:1;
     };
};
extern union cmd_t cmd;
extern union n16bit angle;
extern osThreadId motoctrlHandle;
void MotoCtrlTask(void const * argument);
void MotoCtrlFreertosInit(void);
#ifdef __cplusplus
}
#endif
#endif

