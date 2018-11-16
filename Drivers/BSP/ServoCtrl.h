#ifndef __servoctrl_H
#define __servoctrl_H

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"


typedef struct 
{
	float ServoSpeed; // 度每秒	
	float ServoAngleNow; // 当前角度
	uint32_t TimeLast;  // 上次设置角度时间
	uint8_t ServoAngleSet; // 0 ~ 180°
	uint8_t MaxAngleSet; // 最大设置角度
	uint8_t MinAngleSet; // 最小设置角度
} ServoCtrl_t;

void ServoCtrlFreertosInit(void);

extern ServoCtrl_t ServoCtrlS1;
extern ServoCtrl_t ServoCtrlS2;
extern ServoCtrl_t ServoCtrlS3;

#endif

