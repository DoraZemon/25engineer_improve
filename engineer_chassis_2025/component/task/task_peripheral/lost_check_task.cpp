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
#include "rtos_inc.h"
#include "drv_chassis.h"
#include "drv_dm_Imu.h"
#include "drv_communicate.h"
extern chassis_device g_chassis;
extern dm_imu_device g_imu;
extern communicate_device g_communicate;

void lost_check_task(void *argument) {
    osDelay(200);
    for (;;) {
        g_chassis.check_motor_lost();
//        g_imu.check_imu_for_loss();
        g_communicate.check_for_loss();
        osDelay(20);
    }
}