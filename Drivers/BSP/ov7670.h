#ifndef __ov7670_H
#define __ov7670_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "sccb.h"
#include "dcmi.h"
#include "usart.h"
#include "lcd.h"
//#include <stdint.h>

#define OV7670_I2C_ADDRESS                    (0X42)
#define OV7670_ID    								 0x7376
/** @defgroup CAMERA_Driver_structure  Camera Driver structure
  * @{
  */
typedef struct
{
  void     (*Init)(uint16_t, uint8_t);
  uint16_t (*ReadID)(uint16_t);  
  void     (*Config)(uint16_t, uint32_t, uint32_t, uint32_t);
  void     (*Window_Set)(uint16_t,uint16_t,uint16_t ,uint16_t ,uint16_t);
  void     (*CameraLight_Set)(uint32_t);
}CAMERA_DrvTypeDef;

extern CAMERA_DrvTypeDef   ov7670_drv;

void OV7670FreertosInit(void);
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg);
void CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
void ov7670_Init(uint16_t DeviceAddr, uint8_t resolution);
void ov7670_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t brightness_value);
uint16_t ov7670_ReadID(uint16_t DeviceAddr);
void ov7670_Window_Set(uint16_t DeviceAddr,uint16_t sx,uint16_t sy,uint16_t width,uint16_t height);
void ov7670_CameraLight_Set(uint32_t CameraLight);
#ifdef __cplusplus
}
#endif

#endif 

