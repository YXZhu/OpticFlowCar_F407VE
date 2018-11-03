#ifndef __mpu9250_H
#define __mpu9250_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "eeprom.h"
extern uint8_t c;
extern osThreadId mpu9250Handle;
extern osSemaphoreId INTReady;
extern CCMRAM float Pitch,Roll,Yaw;
extern CCMRAM float HeadingDegrees;
extern CCMRAM float linear_accelX,linear_accelY,linear_accelZ;
extern CCMRAM float Gravity_X,Gravity_Y,Gravity_Z;
extern CCMRAM float GYRO_Spx,GYRO_Spy,GYRO_Spz;
extern CCMRAM float Compass_X,Compass_Y,Compass_Z;

//int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
//int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void MPU9250Task(void const * argument);
void MPU9250FreertosInit(void);
int get_tick_count(unsigned long *count);
void gyro_data_ready_cb(void);
#ifdef __cplusplus
}
#endif
#endif

