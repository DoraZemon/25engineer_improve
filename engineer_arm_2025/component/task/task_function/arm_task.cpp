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
#include "drv_rc.h"
#include "drv_pc.h"

extern pc_device g_pc;
extern rc_device g_rc;
arm_device g_arm;

void arm_task(void *argument) {
    osDelay(200); //等待其他任务遥控器初始化完成
    g_arm.init();
//    while (!(g_rc.check_ready() && !g_pc.check_lost())) { //等待遥控器准备就绪
//        g_arm.update_data();
//        g_arm.send_msg();
//        osDelay(1);
//    }
    for (;;) {
        g_arm.recover_dm();
        g_arm.update_control(g_rc.check_ready() && !g_pc.check_lost());
//        g_arm.update_control(false);
//
        g_arm.send_msg();
        osDelay(2);
    }
}