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
    g_arm.init();
    for (;;) {
        g_arm.update_control(g_rc.check_ready() && !g_pc.check_lost());
        g_arm.send_msg();
        osDelay(1);
    }
}