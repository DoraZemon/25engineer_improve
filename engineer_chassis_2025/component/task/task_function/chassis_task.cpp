/**
  ******************************************************************************
  * @file           : chassis_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "chassis_task.h"
#include "GlobalCfg.h"
#include "drv_dm_Imu.h"
#include "drv_communicate.h"

extern communicate_device g_communicate;


chassis_device g_chassis;
dm_imu_device g_imu;

void chassis_task(void *argument) {

    g_chassis.init(&CHASSIS_CAN);
    g_imu.init(&IMU_CAN, IMU_SLAVE_ID, IMU_MASTER_ID, IMUUpdateBinarySemHandle);//can总线上另一个设备发起数据请求
    osDelay(1000);
    while (!g_chassis.check_init_completely()) {
        osDelay(1);
    }
    for (;;) {
        g_chassis.update_ready();
        if (g_communicate.check_is_rc_online() && g_chassis.check_enable()) {
            if (g_chassis.check_can_use()) {//仅开机做一次自检，后续只管offset
                g_chassis.update_speed_control();
            } else {
                continue;
            }
        } else {
            g_chassis.set_free();
        }
#if CHASSIS
        g_chassis.can_set();
#endif
        osDelay(1);
    }

}

