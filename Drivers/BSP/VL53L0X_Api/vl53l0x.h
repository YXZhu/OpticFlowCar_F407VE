#ifndef __vl53l0x_H
#define __vl53l0x_H
#ifdef __cplusplus
 extern "C" {
#endif

//#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_cali.h"
#include "eeprom.h"

extern osThreadId VL53L0XHandle;
//extern osMutexId VL53L0XMutex;
enum VL53L0X_dev_e{
    VL53L0X_DEV_LEFT =  0,    //!< left satellite device P21 header : 'l'
    VL53L0X_DEV_CENTER  =  1, //!< center (built-in) vl053 device : 'c"
    VL53L0X_DEV_RIGHT=  2,     //!< Right satellite device P22 header : 'r'
};
typedef enum {
	LONG_RANGE 		= 0, /*!< Long range mode */
	HIGH_SPEED 		= 1, /*!< High speed mode */
	HIGH_ACCURACY	= 2, /*!< High accuracy mode */
	DEFAULT        = 3, /*!< Default mode */
} RangingConfig_e;

////vl53l0x������У׼��Ϣ�ṹ�嶨��
//typedef __packed struct
//{
//	uint8_t  adjustok;                    //У׼�ɹ���־��0XAA����У׼;������δУ׼
//	uint8_t  isApertureSpads;             //ApertureSpadsֵ
//	uint8_t  VhvSettings;                 //VhvSettingsֵ
//	uint8_t  PhaseCal;                    //PhaseCalֵ
//	uint32_t XTalkCalDistance;            //XTalkCalDistanceֵ
//	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCpsֵ
//	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeterֵ
//	int32_t  OffsetMicroMeter;            //OffsetMicroMeterֵ
//	uint32_t refSpadCount;                //refSpadCountֵ
//	
//}_vl53l0x_adjust;

//�����޾���ֵ ��λ:mm
#define Thresh_Low  60
#define Thresh_High 150

//�ж�ģʽ�����ṹ��
typedef struct 
{
    const int VL53L0X_Mode;//ģʽ
	 const char *Name;
	 uint32_t ThreshLow;    //����ֵ
	 uint32_t ThreshHigh;   //����ֵ
}AlrmMode_t; 

/* ���ڽ��սṹ�� */
typedef struct{
	uint8_t flag1;
	uint8_t flag2;
	uint8_t data;
	uint8_t size;
	uint8_t dis_h;
	uint8_t dis_l;
	uint8_t mode;
	uint8_t temp;
	uint8_t sum;
} vl53l0x_uart_rec_t;

union vl53l0x_rec_t{
	uint8_t recchar[sizeof(vl53l0x_uart_rec_t)];
	vl53l0x_uart_rec_t vl53l0x_3;
};
extern VL53L0X_Dev_t VL53L0XDevs[];
extern union vl53l0x_rec_t vl53l0x_rec;
void vl53l0xFreertosInit(void);
void vl53l0x_uart_rec(void);
extern VL53L0X_RangingMeasurementData_t VL53L0XRangingMeasurementData[3];  // �������ݽ��
extern uint16_t vl53l0x_dis3; // ���ڲ������
#ifdef __cplusplus
}
#endif

#endif 

