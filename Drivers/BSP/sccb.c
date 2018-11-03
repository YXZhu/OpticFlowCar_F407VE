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
//本程序参考自网友guanfu_wang代码。
//ALIENTEK战舰STM32开发板V3
//SCCB 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/18
//版本：V1.0		    							    							  
//////////////////////////////////////////////////////////////////////////////////
 
 #define clocktime 10
 #define delaytime 20
//初始化SCCB接口
//CHECK OK
void SCCB_Init(void)
{				   
}			 

//SCCB起始信号
//当时钟为高的时候,数据线的高到低,为SCCB起始信号
//在激活状态下,SDA和SCL均为低电平
void SCCB_Start(void)
{
	  
    SCCB_SDA_SET;    //数据线高电平	   
    SCCB_SCL_SET;	    //在时钟线高的时候数据线由高至低
    delay_us(clocktime);  
    SCCB_SDA_RESET;
    delay_us(clocktime);	 
    SCCB_SCL_RESET;	    //数据线恢复低电平，单操作函数必要	  
}

//SCCB停止信号
//当时钟为高的时候,数据线的低到高,为SCCB停止信号
//空闲状况下,SDA,SCL均为高电平
void SCCB_Stop(void)
{
    SCCB_SDA_RESET;
    delay_us(clocktime);	 
    SCCB_SCL_SET;	
    delay_us(clocktime); 
    SCCB_SDA_SET;	
    delay_us(clocktime);
}  
//产生NA信号
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
//SCCB,写入一个字节
//返回值:0,成功;1,失败. 
uint8_t SCCB_WR_Byte(uint8_t dat)
{
	uint8_t j,res;	 
	for(j=0;j<8;j++) //循环8次发送数据
	{
		if(dat&0x80)SCCB_SDA_SET;	
		else SCCB_SDA_RESET;
		dat<<=1;
		delay_us(clocktime);
		SCCB_SCL_SET;	
		delay_us(clocktime);
		SCCB_SCL_RESET;		   
	}			 
	SCCB_SDA_IN();		//设置SDA为输入 
	delay_us(clocktime);
	SCCB_SCL_SET;			//接收第九位,以判断是否发送成功
	delay_us(clocktime);
	if(SCCB_READ_SDA)res=1;  //SDA=1发送失败，返回1
	else res=0;         //SDA=0发送成功，返回0
	SCCB_SCL_RESET;		 
	//SCCB_SDA_OUT;		//设置SDA为输出    
	return res;  
}	 
//SCCB 读取一个字节
//在SCL的上升沿,数据锁存
//返回值:读到的数据
uint8_t SCCB_RD_Byte(void)
{
	uint8_t temp=0,j;    
	SCCB_SDA_IN();		//设置SDA为输入  
	for(j=8;j>0;j--) 	//循环8次接收数据
	{		     	  
		delay_us(clocktime);
		SCCB_SCL_SET;
		temp=temp<<1;
		if(SCCB_READ_SDA)temp++;   
		delay_us(clocktime);
		SCCB_SCL_RESET;
	}	
	//SCCB_SDA_OUT();		//设置SDA为输出    
	return temp;
} 							    
//写寄存器
//返回值:0,成功;1,失败.
uint8_t SCCB_WR_Reg(uint8_t DeviceAddr,uint8_t reg,uint8_t data)
{
	uint8_t res=0;
	SCCB_Start(); 					//启动SCCB传输
	if(SCCB_WR_Byte(DeviceAddr))res=1;	//写器件ID	  
	delay_us(delaytime);
  	if(SCCB_WR_Byte(reg))res=1;		//写寄存器地址	  
	delay_us(delaytime);
  	if(SCCB_WR_Byte(data))res=1; 	//写数据	 
  	SCCB_Stop();	  
  	return	res;
}		  					    
//读寄存器
//返回值:读到的寄存器值
uint8_t SCCB_RD_Reg(uint8_t DeviceAddr,uint8_t reg)
{
	uint8_t val=0;
	SCCB_Start(); 				//启动SCCB传输
	SCCB_WR_Byte(DeviceAddr);		//写器件ID	  
	delay_us(delaytime);	 
  	SCCB_WR_Byte(reg);			//写寄存器地址	  
	delay_us(delaytime);	  
	SCCB_Stop();   
	delay_us(delaytime);	   
	//设置寄存器地址后，才是读
	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID|0X01);	//发送读命令	  
	delay_us(delaytime);
  	val=SCCB_RD_Byte();		 	//读取数据
  	SCCB_No_Ack();
  	SCCB_Stop();
  	return val;
}















