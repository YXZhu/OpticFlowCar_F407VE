#ifndef __servoctrl_H
#define __servoctrl_H

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"


typedef struct 
{
	float ServoSpeed; // ��ÿ��	
	float ServoAngleNow; // ��ǰ�Ƕ�
	uint32_t TimeLast;  // �ϴ����ýǶ�ʱ��
	uint8_t ServoAngleSet; // 0 ~ 180��
	uint8_t MaxAngleSet; // ������ýǶ�
	uint8_t MinAngleSet; // ��С���ýǶ�
} ServoCtrl_t;

void ServoCtrlFreertosInit(void);

extern ServoCtrl_t ServoCtrlS1;
extern ServoCtrl_t ServoCtrlS2;
extern ServoCtrl_t ServoCtrlS3;

#endif

