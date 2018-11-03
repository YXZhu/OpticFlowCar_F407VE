#ifndef __datadisplay_H
#define __datadisplay_H	

#include "WM.h"

#define ID_FRAMEWIN_0   (GUI_ID_USER + 0x02)
#define ID_TEXT_0   (GUI_ID_USER + 0x03)
#define ID_TEXT_1   (GUI_ID_USER + 0x04)
#define ID_TEXT_2   (GUI_ID_USER + 0x05)
#define ID_TEXT_3   (GUI_ID_USER + 0x06)
#define ID_IMAGE_0   (GUI_ID_USER + 0x07)

#define ID_IMAGE_0_IMAGE_0   0x00

WM_HWIN CreateDataDisplay(void);

#endif
