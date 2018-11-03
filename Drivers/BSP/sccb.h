#ifndef __SCCB_H
#define __SCCB_H

#include "main.h"
#include "i2c.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ο�������guanfu_wang���롣
//ALIENTEKս��STM32������V3
//SCCB ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/18
//�汾��V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////
  
//#define SCCB_SDA_IN()  {GPIOG->CRH&=0XFF0FFFFF;GPIOG->CRH|=0X00800000;}
//#define SCCB_SDA_OUT() {GPIOG->CRH&=0XFF0FFFFF;GPIOG->CRH|=0X00300000;}

   
#define SCCB_ID   			0X42  			//OV7670��ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
uint8_t SCCB_WR_Byte(uint8_t dat);
uint8_t SCCB_RD_Byte(void);
uint8_t SCCB_WR_Reg(uint8_t DeviceAddr,uint8_t reg,uint8_t data);
uint8_t SCCB_RD_Reg(uint8_t DeviceAddr,uint8_t reg);

#endif













