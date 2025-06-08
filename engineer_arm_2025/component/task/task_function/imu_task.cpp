/**
  ******************************************************************************
  * @file           : imu_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-6
  ******************************************************************************
  */


#include "imu_task.h"
#include "GlobalCfg.h"

dm_imu_device g_imu;

void imu_task(void *argument) {
    g_imu.init(&IMU_CAN, IMU_SLAVE_ID, IMU_MASTER_ID, IMUUpdateBinarySemHandle);
    osDelay(200);
    for (;;) {
#if DM_IMU
        g_imu.request_imu_data();
#endif
        osDelay(3);
    }
}