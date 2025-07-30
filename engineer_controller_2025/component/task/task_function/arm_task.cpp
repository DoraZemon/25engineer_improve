/**
  ******************************************************************************
  * @file           : arm_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "arm_task.h"


arm_device g_arm;

void arm_task(void *argument) {
    g_arm.init();
    for (;;) {
        g_arm.update_control(true);
#if not JY_ME02
        g_arm.send_msg();
#endif
        osDelay(10);
    }
}