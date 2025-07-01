/**
  ******************************************************************************
  * @file           : gimbal_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-1
  ******************************************************************************
  */


#include "gimbal_task.h"

gimbal_device g_gimbal;


void gimbal_task(void *argument) {
    osDelay(200); //等待其他任务初始化完成
    for (;;) {
        g_gimbal.update_control();
        osDelay(1);
    }
}
