/**
  ******************************************************************************
  * @file           : lost_check_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "lost_check_task.h"
#include "drv_dm_Imu.h"
#include "drv_communicate.h"

extern arm_device g_arm;
extern dm_imu_device g_imu;
extern communicate_device g_communicate;

void lost_check_task(void *argument) {
    osDelay(200);
    for (;;) {
        g_arm.check_motor_loss();
//        g_imu.check_imu_for_loss();
        g_communicate.check_for_loss();
        osDelay(20);
    }
}