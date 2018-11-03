#ifndef __TOUCH_GUI_H  
#define __TOUCH_GUI_H    

void GUI_TOUCH_X_ActivateX(void);
void GUI_TOUCH_X_ActivateY(void);
void GUI_TOUCH_X_Disable (void);
int GUI_TOUCH_X_MeasureX(void);  // 获取触摸 X 坐标值
int GUI_TOUCH_X_MeasureY(void);  //获取触摸 Y 坐标值
void GUI_TOUCH_X_GetXY(void);  //获取触摸参数的
void Touch_Adjust(void);

#endif

