/**
  ******************************************************************************
  * @file           : lost_check_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#include "lost_check_task.h"
#include "drv_arm.h"

extern arm_device g_arm;

void lost_check_task(void *argument) {
    osDelay(200);
    for (;;) {
        g_arm.check_motor_loss();
        osDelay(6);
    }
}