#ifndef __TOUCH_GUI_H  
#define __TOUCH_GUI_H    

void GUI_TOUCH_X_ActivateX(void);
void GUI_TOUCH_X_ActivateY(void);
void GUI_TOUCH_X_Disable (void);
int GUI_TOUCH_X_MeasureX(void);  // ��ȡ���� X ����ֵ
int GUI_TOUCH_X_MeasureY(void);  //��ȡ���� Y ����ֵ
void GUI_TOUCH_X_GetXY(void);  //��ȡ����������
void Touch_Adjust(void);

#endif

