/**
  ******************************************************************************
  * @file           : drv_pc.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "drv_pc.h"
#include "GlobalCfg.h"
#include "usbd_cdc_if.h"

void pc_device::set_connected() {
    is_lost = false;
}

void pc_device::set_lost() {
    is_lost = true;
}

bool pc_device::check_lost() {
    return is_lost;
}

void pc_device::update_data(rc_device &rc, arm_device &arm,controller_device & controller) {
    normal_tx_data.frame_head = PC_Normal_Frame_Head;
    normal_tx_data.is_rc_online = rc.check_ready();
//    normal_tx_data.is_rc_online = true;
    normal_tx_data.is_from_dt7 = true;
    memcpy(normal_tx_data.remote_ctrl,rc.raw_data.buff, sizeof(normal_tx_data.remote_ctrl));
    normal_tx_data.joint1 = arm.data.joint_states.joint1;
    normal_tx_data.joint2 = arm.data.joint_states.joint2;
    normal_tx_data.joint3 = arm.data.joint_states.joint3;
    normal_tx_data.joint4 = arm.data.joint_states.joint4;
    normal_tx_data.joint5 = arm.data.joint_states.joint5;
    normal_tx_data.joint6 = arm.data.joint_states.joint6;
    //todo 其它待完善
    normal_tx_data.frame_tail = PC_Frame_Tail;

    controller_tx_data.frame_head = PC_Controller_Frame_Head;
    controller_tx_data.is_controller_valid = controller.raw_data.is_data_valid && !controller.check_lost();
    if(controller.raw_data.is_data_valid && !controller.check_lost()){//通信协议中加入控制器有效性判断
        controller_tx_data.joint1 = controller.raw_data.joint1;
        controller_tx_data.joint2 = controller.raw_data.joint2;
        controller_tx_data.joint3 = controller.raw_data.joint3;
        controller_tx_data.joint4 = controller.raw_data.joint4;
        controller_tx_data.joint5 = controller.raw_data.joint5;
        controller_tx_data.joint6 = controller.raw_data.joint6;
    }else{
        controller_tx_data.joint1 = 0.0f;
        controller_tx_data.joint2 = 0.0f;
        controller_tx_data.joint3 = 0.0f;
        controller_tx_data.joint4 = 0.0f;
        controller_tx_data.joint5 = 0.0f;
        controller_tx_data.joint6 = 0.0f;
    }
    controller_tx_data.frame_tail = PC_Frame_Tail;

    arm.set_joint1_target(rx_data.joint1);
    arm.set_joint2_target(rx_data.joint2);
    arm.set_joint3_target(rx_data.joint3);
    arm.set_joint4_target(rx_data.joint4);
    arm.set_joint5_target(rx_data.joint5);
    arm.set_joint6_target(rx_data.joint6);
    arm.set_arm_ctrl_enable(rx_data.arm_ctrl_enable);
}


void pc_device::transmit_data() {

    // 更新计数器
    normal_counter++;
    controller_counter++;

    // 发送normal_tx_data (100Hz)
    if (normal_counter >= 10) {  // 1000Hz / 100Hz = 10
        CDC_Transmit_FS((uint8_t *) (&normal_tx_data), sizeof(normal_tx_data));
        normal_counter = 0;  // 重置计数器
    }

    // 发送controller_tx_data (30Hz)
    if (controller_counter >= 33) {  // 1000Hz / 30Hz ≈ 33.33 -> 取33
        CDC_Transmit_FS((uint8_t *) (&controller_tx_data), sizeof(controller_tx_data));
        controller_counter = 0;  // 重置计数器
    }
}