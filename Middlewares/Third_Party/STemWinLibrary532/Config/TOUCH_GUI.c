/**
  *****************************************************************************
  *                              emWin触摸值获取
  *
  *                       (C) Copyright 2000-2020, ***
  *                             All Rights Reserved
  *****************************************************************************
  *
  * @File    : emWin_Touch.c
  * @By      : Sam Chan
  * @Version : V1.0
  * @Date    : 2014 / 06 / 17
  *
  *****************************************************************************
  *
  *                                    Update
  *
  * @Version : ***
  * @By      : ***
  * @Date    : 20** / ** / **
  * @Revise  : ***
  *
  *****************************************************************************
**/


/******************************************************************************
                                 外部函数头文件                        
******************************************************************************/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "gui.h"
#include "touch.h"
#include "TOUCH_GUI.h"

/******************************************************************************
                               定义触摸用到的变量                  
******************************************************************************/

#define Touch_Adjust_Base     0xf0

//下面定义的校准值可以通过 USMART串口调试组件下载到24C02中
//在EEPROM中存储的数值对应位置
//Touch_Adjust_Base     存的是 TOUCH_AD_y0 的值 --> = (TOUCH_AD_x0 << 8) + TOUCH_AD_y0
//Touch_Adjust_Base + 1 存的是 TOUCH_AD_x0 的值
//Touch_Adjust_Base + 2 存的是 TOUCH_AD_x1 的低8位
//Touch_Adjust_Base + 3 存的是 TOUCH_AD_x1 的高8位
//Touch_Adjust_Base + 4 存的是 TOUCH_AD_y1 的低8位
//Touch_Adjust_Base + 5 存的是 TOUCH_AD_y1 的高8位

int TOUCH_AD_y0;  //X   == 221;
int TOUCH_AD_y1;  // == 3900;

int TOUCH_AD_x0;  //Y == 160;
int TOUCH_AD_x1;  // == 3883;

/**
  *****************************************************************************
  * @Name   : GUI准备X轴测量
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void GUI_TOUCH_X_ActivateX(void)
{
	//TP_Get_Adjdata();
}

/**
  *****************************************************************************
  * @Name   : GUI准备Y轴测量
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void GUI_TOUCH_X_ActivateY(void)
{
}

/**
  *****************************************************************************
  * @Name   : GUI禁用轴测量
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void GUI_TOUCH_X_Disable  (void)
{
}

/**
  *****************************************************************************
  * @Name   : GUI获取X轴值
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : X轴数值
  *****************************************************************************
**/
int GUI_TOUCH_X_MeasureX(void) 
{
	uint16_t  x,y;
   tp_dev.scan(1);
	//TP_Read_XY2(&x,&y);

	return tp_dev.y;
}

/**
  *****************************************************************************
  * @Name   : GUI获取Y轴值
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : Y轴数值
  *****************************************************************************
**/
int GUI_TOUCH_X_MeasureY(void) 
{
	uint16_t  x,y;

	//TP_Read_XY2(&x,&y);

	return tp_dev.x;
}

/**
  *****************************************************************************
  * @Name   : GUI获取触摸坐标值
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void GUI_TOUCH_X_GetXY(void)
{
	
// 	TOUCH_AD_x0   = tp_dev.xfac;  //得到x校准参数
// 	TOUCH_AD_y0    = tp_dev.yfac;  //得到y校准参数
// 	
// 	TOUCH_AD_x1  = tp_dev.xoff;  //得到x偏移量
// 	TOUCH_AD_y1 = tp_dev.yoff;  //得到y偏移量
	#define TOUCH_AD_TOP 120
#define TOUCH_AD_BOTTOM 1960
#define TOUCH_AD_LEFT 120
#define TOUCH_AD_RIGHT 1896
	//在EEPROM中存储的数值对应位置
	//Touch_Adjust_Base     存的是 TOUCH_AD_y0 的值
	//Touch_Adjust_Base + 1 存的是 TOUCH_AD_x0 的值
	//Touch_Adjust_Base + 2 存的是 TOUCH_AD_x1 的低8位
	//Touch_Adjust_Base + 3 存的是 TOUCH_AD_x1 的高8位
	//Touch_Adjust_Base + 4 存的是 TOUCH_AD_y1 的低8位
	//Touch_Adjust_Base + 5 存的是 TOUCH_AD_y1 的高8位
	TP_Get_Adjdata();
//	TOUCH_AD_x0 =(int)tp_dev.xfac;
//	TOUCH_AD_y0 = (int)tp_dev.yfac;
//	
//	TOUCH_AD_x1 = (int)tp_dev.xoff;  //读取数据
//	TOUCH_AD_y1 = (int)tp_dev.yoff;  //读取数据
	GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 240, TOUCH_AD_TOP, TOUCH_AD_BOTTOM);  //横屏 GUI_TOUCH_X_MeasureY = tp_dev.x GUI_TOUCH_X_MeasureX = tp_dev.Y
    GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 320, TOUCH_AD_RIGHT, TOUCH_AD_LEFT);     
//	 GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 240, TOUCH_AD_TOP, TOUCH_AD_BOTTOM); //竖屏
//    GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 320, TOUCH_AD_LEFT, TOUCH_AD_RIGHT);  
	//GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 240, TOUCH_AD_y0, TOUCH_AD_y1);  //读取Y值
	//GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 320, TOUCH_AD_x0, TOUCH_AD_x1);  //读取X值
}

/**
  *****************************************************************************
  * @Name   : GUI触摸屏校准
  *
  * @Brief  : none
  *
  * @Input  : none
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void Touch_Adjust(void)
{
	GUI_CURSOR_Show();
	GUI_CURSOR_Select(&GUI_CursorCrossL);
	GUI_SetBkColor(GUI_WHITE);
	GUI_SetColor(GUI_BLACK);
	GUI_Clear();
	GUI_DispString("Measurement of\nA/D converter values");

	while (1)
	{
		GUI_PID_STATE TouchState;
		int xPhys, yPhys;
		GUI_TOUCH_GetState(&TouchState);  /* Get the touch position in pixel */
		xPhys = GUI_TOUCH_GetxPhys();     /* Get the A/D mesurement result in x */
		yPhys = GUI_TOUCH_GetyPhys();     /* Get the A/D mesurement result in y */
		
		/* Display the measurement result */
		GUI_SetColor(GUI_BLUE);		
		GUI_DispStringAt("Analog input:\n", 0, 20);
		GUI_GotoY(GUI_GetDispPosY() + 2);
		GUI_DispString("x:");
		GUI_DispDec(xPhys, 4);
		GUI_DispString(", y:");
		GUI_DispDec(yPhys, 4);
		
		/* Display the according position */
		GUI_SetColor(GUI_RED);
		GUI_GotoY(GUI_GetDispPosY() + 4);
		GUI_DispString("\nPosition:\n");
		GUI_GotoY(GUI_GetDispPosY() + 2);
		GUI_DispString("x:");
		GUI_DispDec(TouchState.x,4);
		GUI_DispString(", y:");
		GUI_DispDec(TouchState.y,4);
		GUI_DispString(", y:");
		GUI_DispDec((int)tp_dev.xfac,4);		
		/* Wait a while */
		GUI_Delay(100);
	};
}
