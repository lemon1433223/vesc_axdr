#ifndef __APP_FREERTOS_H__
#define __APP_FREERTOS_H__
#include "cmsis_os.h"
extern uint8_t USB_RxBuffer[128];
extern osThreadId_t confgeneralTaskId;
extern osThreadId_t printTaskInfoId;
extern osThreadId_t defaultTaskHandle;
extern osThreadId_t osTaskManagerId;
extern osThreadId_t displayTaskId;
extern uint32_t LCD_PAGE;
#endif

