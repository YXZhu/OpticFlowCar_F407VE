#include "ov7670.h"

#define ov7670Delay HAL_Delay
CAMERA_DrvTypeDef   ov7670_drv = 
{
  ov7670_Init,
  ov7670_ReadID,  
  ov7670_Config,
  ov7670_Window_Set,
  ov7670_CameraLight_Set,
};

const uint8_t OV7670_RGB_reg[][2]=
{	 
	//{0x3a, 0x04}, 
	{0x8c, 0x00}, //RGB444
	{0x40, 0xd0},  //通用控制15
	{0x12, 0x14}, // QVGA COM7  0x14
	
	
	{0x32, 0x80}, //0x80  //HREF
	{0x17, 0x17},  // 0x17
	{0x18, 0x05},// 0x05 // HSTOP
	{0x19, 0x02},  //VSTART //0x02
	{0x1a, 0x7a},//0x7a //VSTOP //VREF = VSTOP - VSTART = 2 x HIGH
	{0x03, 0x0a},//0x0a		//VREF
	{0x0c, 0x04},  // [6] 输出数据进行 MSB和LSB 交换	//0x00
	/* 缩放设置 */
	{0x3e, 0x19},//  qvga 0x00
	{0x70, 0x3A},  //qvga 0x00
	{0x71, 0x35}, //qvga 0x01
	{0x72, 0x11}, //qvga0x11
	{0x73, 0xF1}, // 0x00
	
	{0xa2, 0x02}, 
	{0x11, 0x01},   //分频 0x03 //0x06 分7频
	{0x7a, 0x20}, 
	{0x7b, 0x1c}, 
	{0x7c, 0x28}, 
	{0x7d, 0x3c}, 
	{0x7e, 0x55}, 
	{0x7f, 0x68}, 
	{0x80, 0x76}, 
	{0x81, 0x80}, 
	{0x82, 0x88}, 
	{0x83, 0x8f}, 
	{0x84, 0x96}, 
	{0x85, 0xa3}, 
	{0x86, 0xaf}, 
	{0x87, 0xc4}, 
	{0x88, 0xd7}, 
	{0x89, 0xe8}, 
	{0x13, 0xe0}, 
	{0x00, 0x00},//AGC
	
	{0x10, 0x00}, 
	{0x0d, 0x00}, 
	{0x14, 0x28},//0x38, limit the max gain /0x28
	{0xa5, 0x05}, 
	{0xab, 0x07},
	
	{0x24, 0x75}, 
	{0x25, 0x63}, 
	{0x26, 0xA5}, 
	{0x9f, 0x78}, 
	{0xa0, 0x68}, 
	{0xa1, 0x03},//0x0b, 
	{0xa6, 0xdf},//0xd8, 
	{0xa7, 0xdf},//0xd8, 
	{0xa8, 0xf0}, 
	{0xa9, 0x90}, 
	{0xaa, 0x94}, 
	{0x13, 0xe5}, 
	{0x0e, 0x61}, 
	{0x0f, 0x4b}, 
	{0x16, 0x02}, 
	{0x1e, 0x07},//0x07, 
	{0x21, 0x02}, 
	{0x22, 0x91}, 
	{0x29, 0x07}, 
	{0x33, 0x0b}, 
	{0x35, 0x0b}, 
	{0x37, 0x1d}, 
	{0x38, 0x71}, 
	{0x39, 0x2a}, 
	{0x3c, 0x78}, 
	{0x4d, 0x40}, 
	{0x4e, 0x20}, 
	{0x69, 0x00}, 
	{0x6b, 0x9a},//PLL 重要参数 x6 0x9a   //0x4a x4
	{0x74, 0x19}, 
	{0x8d, 0x4f}, 
	{0x8e, 0x00}, 
	{0x8f, 0x00}, 
	{0x90, 0x00}, 
	{0x91, 0x00}, 
	{0x92, 0x00},//0x19,//0x66 
	{0x96, 0x00}, 
	{0x9a, 0x80}, 
	{0xb0, 0x84}, 
	{0xb1, 0x0c}, 
	{0xb2, 0x0e}, 
	{0xb3, 0x82}, 
	{0xb8, 0x0a}, 
	{0x43, 0x14}, 
	{0x44, 0xf0}, 
	{0x45, 0x34}, 
	{0x46, 0x58}, 
	{0x47, 0x28}, 
	{0x48, 0x3a}, 
	{0x59, 0x88}, 
	{0x5a, 0x88}, 
	{0x5b, 0x44}, 
	{0x5c, 0x67}, 
	{0x5d, 0x49}, 
	{0x5e, 0x0e}, 
	{0x64, 0x04}, 
	{0x65, 0x20}, 
	{0x66, 0x05}, 
	{0x94, 0x04}, 
	{0x95, 0x08}, 
	{0x6c, 0x0a}, 
	{0x6d, 0x55}, 
	{0x6e, 0x11}, 
	{0x6f, 0x9e},//0x9e for advance AWB 
	{0x6a, 0x40}, 
	{0x01, 0x40}, 
	{0x02, 0x40}, 
	{0x13, 0xe7}, 
	{0x15, 0x00}, //重要参数 
	{0x4f, 0x80}, 
	{0x50, 0x80}, 
	{0x51, 0x00}, 
	{0x52, 0x22}, 
	{0x53, 0x5e}, 
	{0x54, 0x80}, 
	{0x55, 0x0A},//亮度 
	{0x56, 0x4f},//对比度 
	{0x58, 0x9e},	
	{0x41, 0x08}, 
	{0x3f, 0x05},//边缘增强调整 0x05
	{0x75, 0x05}, 
	{0x76, 0xe1}, 
	{0x4c, 0x0f},//噪声抑制强度 0x0f
	{0x77, 0x0a}, 
	{0x3d, 0xc0},//0xc0, 
	{0x4b, 0x09}, 
	{0xc9, 0x60}, 
	{0x41, 0x38}, 
	{0x34, 0x11}, 
	{0x3b, 0x02},//0x00,//0x02, 
	{0xa4, 0x88},//0x88, 
	{0x96, 0x00}, 
	{0x97, 0x30}, 
	{0x98, 0x20}, 
	{0x99, 0x30}, 
	{0x9a, 0x84}, 
	{0x9b, 0x29}, 
	{0x9c, 0x03}, 
	{0x9d, 0x4c}, 
	{0x9e, 0x3f}, 
	{0x78, 0x04},	
	{0x79, 0x01}, 
	{0xc8, 0xf0}, 
	{0x79, 0x0f}, 
	{0xc8, 0x00}, 
	{0x79, 0x10}, 
	{0xc8, 0x7e}, 
	{0x79, 0x0a}, 
	{0xc8, 0x80}, 
	{0x79, 0x0b}, 
	{0xc8, 0x01}, 
	{0x79, 0x0c}, 
	{0xc8, 0x0f}, 
	{0x79, 0x0d}, 
	{0xc8, 0x20}, 
	{0x79, 0x09}, 
	{0xc8, 0x80}, 
	{0x79, 0x02}, 
	{0xc8, 0xc0}, 
	{0x79, 0x03}, 
	{0xc8, 0x40}, 
	{0x79, 0x05}, 
	{0xc8, 0x30}, 
	{0x79, 0x26}, 
	{0x09, 0x00}, 
	//{0x3b, 0xc2},//0x82,//0xc0,//0xc2,	//night mode 
	
	///////////////////////////////////////////////////////////////////////
//以下部分代码由开源电子网网友:duanzhang512 提出
//添加此部分代码将可以获得更好的成像效果,但是最下面一行会有蓝色的抖动.
//如不想要,可以屏蔽此部分代码.然后将:OV7670_Window_Set(12,176,240,320);
//改为:OV7670_Window_Set(12,174,240,320);,即可去掉最下一行的蓝色抖动
	{0x6a, 0x40},
	{0x01, 0x40},
	{0x02, 0x40},
	{0x13, 0xe7},
	{0x15, 0x00},  
	
		
	{0x58, 0x9e},
	
	{0x41, 0x08},
	{0x3f, 0x00},
	{0x75, 0x05},
	{0x76, 0xe1},
	{0x4c, 0x00},
	{0x77, 0x01},
	{0x3d, 0xc2},	
	{0x4b, 0x09},
	{0xc9, 0x60},
	{0x41, 0x38},
	
	{0x34, 0x11},
	{0x3b, 0x02}, 

	{0xa4, 0x89},
	{0x96, 0x00},
	{0x97, 0x30},
	{0x98, 0x20},
	{0x99, 0x30},
	{0x9a, 0x84},
	{0x9b, 0x29},
	{0x9c, 0x03},
	{0x9d, 0x4c},
	{0x9e, 0x3f},
	{0x78, 0x04},
	
	{0x79, 0x01},
	{0xc8, 0xf0},
	{0x79, 0x0f},
	{0xc8, 0x00},
	{0x79, 0x10},
	{0xc8, 0x7e},
	{0x79, 0x0a},
	{0xc8, 0x80},
	{0x79, 0x0b},
	{0xc8, 0x01},
	{0x79, 0x0c},
	{0xc8, 0x0f},
	{0x79, 0x0d},
	{0xc8, 0x20},
	{0x79, 0x09},
	{0xc8, 0x80},
	{0x79, 0x02},
	{0xc8, 0xc0},
	{0x79, 0x03},
	{0xc8, 0x40},
	{0x79, 0x05},
	{0xc8, 0x30},
	{0x79, 0x26}, 
	{0x09, 0x00},
///////////////////////////////////////////////////////////////////////
};
uint16_t ov7670Frame[160][120];
uint16_t ov7670ID;
extern uint8_t cap;

////////////////////////////////////////////////////////////////////////////
//OV7670功能设置
//白平衡设置
//0:自动
//1:太阳sunny
//2,阴天cloudy
//3,办公室office
//4,家里home
void OV7670_Light_Mode(uint8_t mode)
{
	uint8_t reg13val=0XE7;//默认就是设置为自动白平衡
	uint8_t reg01val=0;
	uint8_t reg02val=0;
	switch(mode)
	{
		case 1://sunny
			reg13val=0XE5;
			reg01val=0X5A;
			reg02val=0X5C;
			break;	
		case 2://cloudy
			reg13val=0XE5;
			reg01val=0X58;
			reg02val=0X60;
			break;	
		case 3://office
			reg13val=0XE5;
			reg01val=0X84;
			reg02val=0X4c;
			break;	
		case 4://home
			reg13val=0XE5;
			reg01val=0X96;
			reg02val=0X40;
			break;	
	}
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X13,reg13val);//COM8设置 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X01,reg01val);//AWB蓝色通道增益 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X02,reg02val);//AWB红色通道增益 
}				  
//色度设置
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Color_Saturation(uint8_t sat)
{
	uint8_t reg4f5054val=0X80;//默认就是sat=2,即不调节色度的设置
 	uint8_t reg52val=0X22;
	uint8_t reg53val=0X5E;
 	switch(sat)
	{
		case 0://-2
			reg4f5054val=0X40;  	 
			reg52val=0X11;
			reg53val=0X2F;	 	 
			break;	
		case 1://-1
			reg4f5054val=0X66;	    
			reg52val=0X1B;
			reg53val=0X4B;	  
			break;	
		case 3://1
			reg4f5054val=0X99;	   
			reg52val=0X28;
			reg53val=0X71;	   
			break;	
		case 4://2
			reg4f5054val=0XC0;	   
			reg52val=0X33;
			reg53val=0X8D;	   
			break;	
	}
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X4F,reg4f5054val);	//色彩矩阵系数1
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X50,reg4f5054val);	//色彩矩阵系数2 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X51,0X00);			//色彩矩阵系数3  
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X52,reg52val);		//色彩矩阵系数4 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X53,reg53val);		//色彩矩阵系数5 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X54,reg4f5054val);	//色彩矩阵系数6  
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X58,0X9E);			//MTXS 
}
//亮度设置
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Brightness(uint8_t bright)
{
	uint8_t reg55val=0X00;//默认就是bright=2
  	switch(bright)
	{
		case 0://-2
			reg55val=0XB0;	 	 
			break;	
		case 1://-1
			reg55val=0X98;	 	 
			break;	
		case 3://1
			reg55val=0X18;	 	 
			break;	
		case 4://2
			reg55val=0X30;	 	 
			break;	
	}
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X55,reg55val);	//亮度调节 
}
//对比度设置
//0:-2
//1:-1
//2,0
//3,1
//4,2
void OV7670_Contrast(uint8_t contrast)
{
	uint8_t reg56val=0X40;//默认就是contrast=2
  	switch(contrast)
	{
		case 0://-2
			reg56val=0X30;	 	 
			break;	
		case 1://-1
			reg56val=0X38;	 	 
			break;	
		case 3://1
			reg56val=0X50;	 	 
			break;	
		case 4://2
			reg56val=0X60;	 	 
			break;	
	}
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X56,reg56val);	//对比度调节 
}
//特效设置
//0:普通模式    
//1,负片
//2,黑白   
//3,偏红色
//4,偏绿色
//5,偏蓝色
//6,复古	    
void OV7670_Special_Effects(uint8_t eft)
{
	uint8_t reg3aval=0X04;//默认为普通模式
	uint8_t reg67val=0XC0;
	uint8_t reg68val=0X80;
	switch(eft)
	{
		case 1://负片
			reg3aval=0X24;
			reg67val=0X80;
			reg68val=0X80;
			break;	
		case 2://黑白
			reg3aval=0X14;
			reg67val=0X80;
			reg68val=0X80;
			break;	
		case 3://偏红色
			reg3aval=0X14;
			reg67val=0Xc0;
			reg68val=0X80;
			break;	
		case 4://偏绿色
			reg3aval=0X14;
			reg67val=0X40;
			reg68val=0X40;
			break;	
		case 5://偏蓝色
			reg3aval=0X14;
			reg67val=0X80;
			reg68val=0XC0;
			break;	
		case 6://复古
			reg3aval=0X14;
			reg67val=0XA0;
			reg68val=0X40;
			break;	 
	}
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X3A,reg3aval);//TSLB设置 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X68,reg67val);//MANU,手动U值 
	SCCB_WR_Reg(OV7670_I2C_ADDRESS,0X67,reg68val);//MANV,手动V值 
}	

void OV7670FreertosInit(void)
{

	HAL_GPIO_WritePin(RESTE_GPIO_Port,RESTE_Pin,GPIO_PIN_SET); // 复位端口（正常使用拉高）
	HAL_GPIO_WritePin(PWDN_GPIO_Port,PWDN_Pin,GPIO_PIN_RESET); //功耗选择模式 正常拉低
	
	ov7670Delay(5);
	//CAMERA_IO_Write(OV7670_I2C_ADDRESS,0x12,0x80);
	//ov7670Delay(5);
	ov7670ID = ov7670_drv.ReadID(OV7670_I2C_ADDRESS);
	if(ov7670ID == OV7670_ID)
	{
		ov7670_drv.Init(OV7670_I2C_ADDRESS,0); // OV7670初始化
		//ov7670Delay(100);
		OV7670_Light_Mode(2);
		OV7670_Color_Saturation(2);
		OV7670_Brightness(2);
		OV7670_Contrast(2);
		OV7670_Special_Effects(0);	
		//ov7670_drv.Window_Set(OV7670_I2C_ADDRESS,0,320,160,120);	//设置窗口
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		//ov7670_drv.CameraLight_Set(10);  // 外部LED亮度设置
	}
	//HAL_GPIO_TogglePin(F_CS_GPIO_Port,F_CS_Pin);
	ov7670Delay(100);
	
	HAL_DCMI_ConfigCROP(&hdcmi,120,0,120*2-1,160-1);  //400	
	HAL_DCMI_EnableCrop(&hdcmi);
	if(HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)ov7670Frame, 0x2580) != HAL_OK)
	{
		Error_Handler();
	}
}
void ov7670_CameraLight_Set(uint32_t CameraLight)  // LED亮度设置 0 ~ 100
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,CameraLight);
}
extern osThreadId startTaskHandle;
uint32_t Frametime;
void BSP_CAMERA_FrameEventCallback()
{
	static uint32_t Frametime_old;
	Frametime = 1000/(HAL_GetTick() - Frametime_old);
	Frametime_old = HAL_GetTick();
	osSignalSet(startTaskHandle,0x01);
	//cap = 0;
	//HAL_GPIO_TogglePin(F_CS_GPIO_Port,F_CS_Pin);
	//HAL_UART_Transmit_DMA(&huart2,ov7670Frame,sizeof(ov7670Frame));
}
void CAMERA_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
	SCCB_WR_Reg(Addr,Reg,Value);
}
uint8_t CAMERA_IO_Read(uint8_t Addr, uint8_t Reg)
{ 
	return SCCB_RD_Reg(Addr,Reg);
}


/**
  * @brief  Initializes the OV7670 CAMERA component.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  resolution: Camera resolution
  * @retval None
  */
void ov7670_Init(uint16_t DeviceAddr, uint8_t resolution)
{
	uint32_t index;
	 CAMERA_IO_Write(DeviceAddr, 0x12,0x80);
	ov7670Delay(10);
	switch(resolution)
	{
		case 0:
		{
			for(index=0; index<(sizeof(OV7670_RGB_reg)/2); index++)
			{
			  CAMERA_IO_Write(DeviceAddr, OV7670_RGB_reg[index][0], OV7670_RGB_reg[index][1]);
			  //ov7670Delay(1);
			} 
			break;
		 } 
	 }		
}

/**
  * @brief  Configures the OV7670 camera feature.
  * @param  DeviceAddr: Device address on communication Bus.
  * @param  feature: Camera feature to be configured
  * @param  value: Value to be configured
  * @param  brightness_value: Brightness value to be configured
  * @retval None
  */
void ov7670_Config(uint16_t DeviceAddr, uint32_t feature, uint32_t value, uint32_t brightness_value)
{
	
}
/**
  * @brief  Read the OV7670 Camera identity.
  * @param  DeviceAddr: Device address on communication Bus.
  * @retval the OV7670 ID
  */
uint16_t ov7670_ReadID(uint16_t DeviceAddr)
{
	uint16_t ReadID;
	/* Prepare the sensor to read the Camera ID */
	ReadID = CAMERA_IO_Read(DeviceAddr, 0x0b);
	//ov7670Delay(1);
	ReadID = CAMERA_IO_Read(DeviceAddr, 0x0a)|ReadID<<8 ;
	return ReadID;
}

//设置图像输出窗口
//对QVGA设置。
void ov7670_Window_Set(uint16_t DeviceAddr,uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
	uint16_t endx;
	uint16_t endy;
	uint8_t temp; 
	endx=sx+width*2;	//V*2
 	endy=sy+height*2;
	if(endy>784)endy-=784;
	temp=CAMERA_IO_Read(DeviceAddr,0X03);				//读取Vref之前的值
	temp&=0XF0;
	temp|=((endx&0X03)<<2)|(sx&0X03);
	CAMERA_IO_Write(DeviceAddr,0X03,temp);				//设置Vref的start和end的最低2位
	CAMERA_IO_Write(DeviceAddr,0X19,sx>>2);			//设置Vref的start高8位
	CAMERA_IO_Write(DeviceAddr,0X1A,endx>>2);			//设置Vref的end的高8位

	temp=CAMERA_IO_Read(DeviceAddr,0X32);				//读取Href之前的值
	temp&=0XC0;
	temp|=((endy&0X07)<<3)|(sy&0X07);
	CAMERA_IO_Write(DeviceAddr,0X17,sy>>3);			//设置Href的start高8位
	CAMERA_IO_Write(DeviceAddr,0X18,endy>>3);			//设置Href的end的高8位
}
