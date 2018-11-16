#ifndef __pmw3901_H
#define __pmw3901_H

#ifdef __cplusplus
 extern "C" {
#endif

//#include "stm32f1xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#if defined(__CC_ARM) 
	#pragma anon_unions	/*用于支持结构体联合体*/
#endif
typedef __packed struct motionBurst_s  // __packed 告诉编译器按照结构体数据定义大小编译
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} MotionBurst_t;
extern osThreadId pmw3901Handle;
extern float pmw3901LpfX, pmw3901LpfY;	/*累积像素低通*/
extern MotionBurst_t MotionBurst;
extern osSemaphoreId pmw3901ReadIT;
void PMW3901FreertosInit(void);
void pmw3901_run(void);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void pmw3901_init(void);
#ifdef __cplusplus
}
#endif
#endif

