#include "vl53l0x_cali.h"
#include "tim.h"

#define delay_ms HAL_Delay
#define ToggleDisLED  HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin)
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 阿波罗STM32F429开发板
//VL53L0X-校准模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

_vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数据写入24c02)
_vl53l0x_adjust Vl53l0x_data;   //校准数据24c02读缓存区（用于系统初始化时向24C02读取数据）

#define adjust_num 5//校准错误次数

static void LCD_printf(char *cha)
{
	LCD_Fill(0,45+12,160,45+24,BLACK);
	//LCD_Fill(0,45,160,80,BLACK);
	//LCD_Fill(0,20,160,128,BLACK);
	//LCD_ShowString(20,45+12,150,12,12,(uint8_t *)"                      ");
	LCD_ShowString(0,45+12,150,12,12,(uint8_t *)cha);
	delay_ms(1000); //显示等待
}
//VL53L0X校准函数
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev)
{
	char chart[25];
	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100<<16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	uint8_t i=0;

	
	VL53L0X_StaticInit(dev);//数值恢复默认,传感器处于空闲状态
    //LED1=0;
	//SPADS校准----------------------------
	spads:
	delay_ms(10);
	//printf("The SPADS Calibration Start...\r\n");
	LCD_printf((char *)"Calibration Start");
	Status = VL53L0X_PerformRefSpadManagement(dev,&refSpadCount,&isApertureSpads);//执行参考Spad管理
	if(Status == VL53L0X_ERROR_NONE)
	{
		 
		 sprintf(chart,"refSpadCount=%d",refSpadCount);
		 LCD_printf(chart);
		
	    //printf("refSpadCount = %d\r\n",refSpadCount);
	    Vl53l0x_adjust.refSpadCount = refSpadCount;
		 sprintf(chart,"isApertureSpads=%d",isApertureSpads);
		 LCD_printf(chart);
	    //printf("isApertureSpads = %d\r\n",isApertureSpads);	
	    Vl53l0x_adjust.isApertureSpads = isApertureSpads;
		 sprintf(chart," Calibration Finish");
		 LCD_printf(chart);
       // printf("The SPADS Calibration Finish...\r\n\r\n");		
	    i=0;
	}
	else
	{
	    i++;
	    if(i==adjust_num) return Status;
		 LCD_printf((char *)"Calibration Error");
	    //printf("SPADS Calibration Error,Restart this step\r\n");
	    goto spads;
	}
	//设备参考校准---------------------------------------------------
	ref:
	delay_ms(10);
	LCD_printf("Ref Calibration Start");
	//printf("The Ref Calibration Start...\r\n");
	Status = VL53L0X_PerformRefCalibration(dev,&VhvSettings,&PhaseCal);//Ref参考校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		sprintf(chart,"VhvSettings = %d",VhvSettings);
		LCD_printf(chart);
		//printf("VhvSettings = %d\r\n",VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		sprintf(chart,"PhaseCal = %d",PhaseCal);
		LCD_printf(chart);		
		//printf("PhaseCal = %d\r\n",PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		LCD_printf("The Ref Calibration Finish");
		//printf("The Ref Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		LCD_printf("Ref Calibration Error");
		//printf("Ref Calibration Error,Restart this step\r\n");
		goto ref;
	}
	//偏移校准------------------------------------------------
	offset:
	delay_ms(10);
	LCD_printf("Offset Calibration:10CM");
	LCD_printf("Offset Calibration Start");
	//printf("Offset Calibration:need a white target,in black space,and the distance keep 100mm!\r\n");
	//printf("The Offset Calibration Start...\r\n");
	
	Status = VL53L0X_PerformOffsetCalibration(dev,CalDistanceMilliMeter,&OffsetMicroMeter);//偏移校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		sprintf(chart,"CalDistanceMM=%dmm",CalDistanceMilliMeter);
		LCD_printf(chart);
		//printf("CalDistanceMilliMeter = %d mm\r\n",CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		sprintf(chart,"OffsetMm=%dmm",OffsetMicroMeter);
		LCD_printf(chart);
		//printf("OffsetMicroMeter = %d mm\r\n",OffsetMicroMeter);	
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		LCD_printf("Offset Calibration Finish");
		//printf("The Offset Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		LCD_printf("Offset Calibration Error");
		//printf("Offset Calibration Error,Restart this step\r\n");
		goto offset;
	}
	//串扰校准-----------------------------------------------------
	xtalk:
	delay_ms(20);
	LCD_printf("Cross Talk Cal:grey");
	///printf("Cross Talk Calibration:need a grey target\r\n");
	//printf("The Cross Talk Calibration Start...\r\n");
	LCD_printf("Cross Talk Cal Start");
	Status = VL53L0X_PerformXTalkCalibration(dev,XTalkCalDistance,&XTalkCompensationRateMegaCps);//串扰校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		sprintf(chart,"XTalkCalDis=%dmm",XTalkCalDistance);
		LCD_printf(chart);
		//printf("XTalkCalDistance = %d mm\r\n",XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		sprintf(chart,"XTalkComRateMeC=%d",XTalkCompensationRateMegaCps);
		LCD_printf(chart);
		//printf("XTalkCompensationRateMegaCps = %d\r\n",XTalkCompensationRateMegaCps);	
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		LCD_printf("Cross Talk Cal Finish");
		//printf("The Cross Talk Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		LCD_printf("Cross Talk Cal Error");
		//printf("Cross Talk Calibration Error,Restart this step\r\n");
		goto xtalk;
	}
	//LED1=1;
	LCD_printf("All Calibration Finished");
	//printf("All the Calibration has Finished!\r\n");
	//printf("Calibration is successful!!\r\n");
	LCD_printf("Calibration is successful");
	Vl53l0x_adjust.adjustok = 0xAA;//校准成功
	//AT24CXX_Write(0,(uint8_t*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数据保存到24c02
	memcpy(&Vl53l0x_data,&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数据复制到Vl53l0x_data结构体
	
		  VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,Vl53l0x_data.isApertureSpads);//设定Spads校准值
        delay_ms(2);		 
	    VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,Vl53l0x_data.PhaseCal);//设定Ref校准值
		 delay_ms(2);
		 VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,Vl53l0x_data.OffsetMicroMeter);//设定偏移校准值

		 delay_ms(2);
		VL53L0X_SetXTalkCompensationRateMegaCps(dev,Vl53l0x_data.XTalkCompensationRateMegaCps);//设定串扰校准值
         delay_ms(2);
	return Status;
}

void vl53l0x_calibration_LCD(void)
{
	uint16_t key1sum = 0;
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,10);
//	LCD_WR_REG(0x36); //MX, MY, RGB mode
//	LCD_WR_DATA8(0xA0);  // 横屏  X: 160 Y: 128
//	/* 设置显示器宽度 高度 */
//	lcddev.width = 160;
//	lcddev.height = 120;

	if(!HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)|| !HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)) //如果有按键按下进入校正模式 否则启动系统
	{
		LCD_Dir(Vertical_LCD);
		LCD_Clear(BLACK);
		POINT_COLOR = RED;
		LCD_DrawLine(0,13,160,13);
		//LCD_Fill(0,0,159,12,WHITE);
		LCD_ShowString(0,0,160,12,12,(uint8_t *)"   The SPADS Calibration  ");
		POINT_COLOR = WHITE;
		//LCD_ShowString(20,20,140,12,12,(uint8_t *)" Calibration Start ");	
		LCD_ShowString(20,20+15,140,12,12,(uint8_t *)"Press K0 Calibration");
		LCD_ShowString(30,20+12+15,130,12,12,(uint8_t *)"VL53L0X_DEV_RIGHT");
		LCD_ShowString(20,46+15,160,12,12,(uint8_t *)"Press K1 Calibration");
		LCD_ShowString(30,46+12+15,130,12,12,(uint8_t *)"VL53L0X_DEV_LEFT");
		POINT_COLOR = YELLOW;
		LCD_ShowString(20,65+12+15,140,12,12,(uint8_t *)"Long Press K1 To Quit");
		while(!HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)||!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)); //等待释放按键	
		while(1)
		{
			if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)) key1sum++;
			else
			{			
				if(key1sum > 0)
				{
						LCD_Fill(0,20,160,128,BLACK);
						//key1sum = 0;
						LCD_ShowString(20,45+12,140,12,12,(uint8_t *)"VL53L0X_DEV_LEFT");
						delay_ms(1000);
					   vl53l0x_adjust(&VL53L0XDevs[VL53L0X_DEV_LEFT]);
					
					
				}
				 key1sum = 0;
			}
				if(key1sum>1000) 
				{
					LCD_Fill(0,20,160,128,BLACK);
					POINT_COLOR = RED;
					LCD_ShowString(10,45+12,150,12,12,(uint8_t *)"Release K1 To Quit");
					while(!HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin));
					LCD_Dir(Horizon_LCD);
					//LCD_Fill(10,77,160,128,BLACK);
					//LCD_ShowString(20,65+12,160,12,12,(uint8_t *)"Quit");
					break;
				}
			delay_ms(0);
			
		}
	}
	
	
}
//vl53l0x校准测试
//dev:设备I2C参数结构体
void vl53l0x_calibration_test(VL53L0X_Dev_t *dev)
{  
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	uint8_t key=0;
	uint8_t i=0;
	
//	LCD_Fill(30,170,320,300,WHITE);
//	POINT_COLOR=RED;//设置字体为红色 
	printf("need a white target,and ");
   printf("the distance keep 100mm.\r\n");
//	POINT_COLOR=BLUE;//设置字体为蓝色 
//	LCD_ShowString(30,220,200,16,16,"KEY_UP: Return menu");
//	LCD_ShowString(30,240,200,16,16,"KEY1:   Calibration");
	while(1)
	{
		//key = KEY_Scan(0);
		if(key==0)
		{
			//POINT_COLOR=RED;//设置字体为红色 
			//LCD_ShowString(30,260,200,16,16,"Start calibration...");
			status = vl53l0x_adjust(dev);//进入校准
			if(status!=VL53L0X_ERROR_NONE)//校准失败
			{ 
				 printf("Calibration is error!!\r\n");
				 i=3;
				 while(i--)
				 {
					  delay_ms(500);
//					  LCD_ShowString(30,260,200,16,16,"                    ");
//					  delay_ms(500);
//					  LCD_ShowString(30,260,200,16,16,"Calibration is error");
				 }
			}
			else
				 printf("Calibration is complete!!\r\n");
			delay_ms(500);

			break;
				
		 }
//		 else if(key==WKUP_PRES)
//		 {
//			 LED1=1;
//			 break;//返回上一菜单
//		 }		 
		 delay_ms(200);
		 //LED0=!LED0;
		
	}
		
}
