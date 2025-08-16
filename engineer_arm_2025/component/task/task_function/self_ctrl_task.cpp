/**
  ******************************************************************************
  * @file           : self_ctrl_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-8-1
  ******************************************************************************
  */


#include "self_ctrl_task.h"
extern rc_device g_rc;
extern controller_device g_controller;
extern arm_device g_arm;
extern communicate_device g_communicate;
extern gimbal_device g_gimbal;
extern hi229um_device g_hi229um;

extern pc_device g_pc;

robot_device g_robot;

void self_kb_event_task(void* argument){
    osDelay(5000);
    for(;;){
        g_robot.do_check_event(g_rc);
        osDelay(1);
    }
}


void self_kb_state_task(void * argument){
    osDelay(5000);
    for(;;){
        g_robot.do_check_state(g_rc);
        osDelay(1);
    }
}


void self_ctrl_task(void * argument){
    osDelay(5000);
    for(;;){
        g_robot.update_control(g_pc,g_rc,g_arm,g_controller,g_communicate,g_hi229um,g_gimbal);
        osDelay(1);
    }
}