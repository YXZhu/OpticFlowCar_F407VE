#include "sccb.h"
#include "dwt_stm32_delay.h"

#define delay_us DWT_Delay_us
#define SCCB_SDA_SET HAL_GPIO_WritePin(SIO_D_GPIO_Port,SIO_D_Pin,GPIO_PIN_SET)   // SIO_D
#define SCCB_SDA_RESET HAL_GPIO_WritePin(SIO_D_GPIO_Port,SIO_D_Pin,GPIO_PIN_RESET)
#define SCCB_READ_SDA HAL_GPIO_ReadPin(SIO_D_GPIO_Port,SIO_D_Pin)
#define SCCB_SDA_IN(...) SCCB_SDA_SET
#define SCCB_SCL_SET HAL_GPIO_WritePin(SIO_C_GPIO_Port,SIO_C_Pin,GPIO_PIN_SET)   // SIO_C
#define SCCB_SCL_RESET HAL_GPIO_WritePin(SIO_C_GPIO_Port,SIO_C_Pin,GPIO_PIN_RESET)

//////////////////////////////////////////////////////////////////////////////////	 
//������ο�������guanfu_wang���롣
//ALIENTEKս��STM32������V3
//SCCB ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/18
//�汾��V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////
 
 #define clocktime 10
 #define delaytime 20
//��ʼ��SCCB�ӿ�
//CHECK OK
void SCCB_Init(void)
{				   
}			 

//SCCB��ʼ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĸߵ���,ΪSCCB��ʼ�ź�
//�ڼ���״̬��,SDA��SCL��Ϊ�͵�ƽ
void SCCB_Start(void)
{
	  
    SCCB_SDA_SET;    //�����߸ߵ�ƽ	   
    SCCB_SCL_SET;	    //��ʱ���߸ߵ�ʱ���������ɸ�����
    delay_us(clocktime);  
    SCCB_SDA_RESET;
    delay_us(clocktime);	 
    SCCB_SCL_RESET;	    //�����߻ָ��͵�ƽ��������������Ҫ	  
}

//SCCBֹͣ�ź�
//��ʱ��Ϊ�ߵ�ʱ��,�����ߵĵ͵���,ΪSCCBֹͣ�ź�
//����״����,SDA,SCL��Ϊ�ߵ�ƽ
void SCCB_Stop(void)
{
    SCCB_SDA_RESET;
    delay_us(clocktime);	 
    SCCB_SCL_SET;	
    delay_us(clocktime); 
    SCCB_SDA_SET;	
    delay_us(clocktime);
}  
//����NA�ź�
void SCCB_No_Ack(void)
{
	delay_us(clocktime);
	SCCB_SDA_SET;	
	SCCB_SCL_SET;	
	delay_us(clocktime);
	SCCB_SCL_RESET;	
	delay_us(clocktime);
	SCCB_SDA_RESET;	
	delay_us(clocktime);
}
//SCCB,д��һ���ֽ�
//����ֵ:0,�ɹ�;1,ʧ��. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res;	 
	for(j=0;j<8;j++) //ѭ��8�η�������
	{
		if(dat&0x80)SCCB_SDA_SET;	
		else SCCB_SDA_RESET;
		dat<<=1;
		delay_us(clocktime);
		SCCB_SCL_SET;	
		delay_us(clocktime);
		SCCB_SCL_RESET;		   
	}			 
	SCCB_SDA_IN();		//����SDAΪ���� 
	delay_us(clocktime);
	SCCB_SCL_SET;			//���յھ�λ,���ж��Ƿ��ͳɹ�
	delay_us(clocktime);
	if(SCCB_READ_SDA)res=1;  //SDA=1����ʧ�ܣ�����1
	else res=0;         //SDA=0���ͳɹ�������0
	SCCB_SCL_RESET;		 
	//SCCB_SDA_OUT;		//����SDAΪ���    
	return res;  
}	 
//SCCB ��ȡһ���ֽ�
//��SCL��������,��������
//����ֵ:����������
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN();		//����SDAΪ����  
	for(j=8;j>0;j--) 	//ѭ��8�ν�������
	{		     	  
		delay_us(clocktime);
		SCCB_SCL_SET;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		delay_us(clocktime);
		SCCB_SCL_RESET;
	}	
	//SCCB_SDA_OUT();		//����SDAΪ���    
	return temp;
} 							    
//д�Ĵ���
//����ֵ:0,�ɹ�;1,ʧ��.
uint8_t SCCB_WR_Reg(uint8_t DeviceAddr,uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//����SCCB����
	if(SCCB_WR_Byte(DeviceAddr))res=1;	//д����ID	  
	delay_us(delaytime);
  	if(SCCB_WR_Byte(reg))res=1;		//д�Ĵ�����ַ	  
	delay_us(delaytime);
  	if(SCCB_WR_Byte(data))res=1; 	//д����	 
  	SCCB_Stop();	  
  	return	res;
}		  					    
//���Ĵ���
//����ֵ:�����ļĴ���ֵ
uint8_t SCCB_RD_Reg(uint8_t DeviceAddr,uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//����SCCB����
	SCCB_WR_Byte(DeviceAddr);		//д����ID	  
	delay_us(delaytime);	 
  	SCCB_WR_Byte(reg);			//д�Ĵ�����ַ	  
	delay_us(delaytime);	  
	SCCB_Stop();   
	delay_us(delaytime);	   
	//���üĴ�����ַ�󣬲��Ƕ�
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//���Ͷ�����	  
	delay_us(delaytime);
  	val=SCCB_RD_Byte();		 	//��ȡ����
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}















