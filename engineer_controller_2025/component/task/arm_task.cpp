/**
  ******************************************************************************
  * @file           : arm_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-5
  ******************************************************************************
  */


#include "arm_task.h"


arm_device g_arm;

void arm_task(void *argument) {
    g_arm.init();
    osDelay(2000); //等待电机初始化完成
    for (;;) {
        g_arm.update_control(true);
        g_arm.send_msg();
        osDelay(10);
    }
}