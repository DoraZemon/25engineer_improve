/**
  ******************************************************************************
  * @file           : drv_controller.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "drv_controller.h"


void controller_device::init() {

}

void controller_device::update() {
}

void controller_device::set_lost() {
    is_lost = true;
}

void controller_device::set_connected() {
    is_lost = false;
}

bool controller_device::check_lost() const {
    return is_lost;
}

bool controller_device::judge_transfer_rx_callback(custom_judge_raw_msg *judge_raw_msg) {
    raw_data.joint1 = judge_raw_msg->joint1;
    raw_data.joint2 = judge_raw_msg->joint2;
    raw_data.joint3 = judge_raw_msg->joint3;
    raw_data.joint4 = judge_raw_msg->joint4;
    raw_data.joint5 = judge_raw_msg->joint5;
    raw_data.joint6 = judge_raw_msg->joint6;
    raw_data.is_data_valid = judge_raw_msg->is_data_valid;
    raw_data.life_flag = judge_raw_msg->life_flag;

    if (raw_data.is_data_valid) {
        last_valid_raw_data = raw_data; //保存上一次接收到的数据
    }

    if (ABS(raw_data.life_flag - last_life_flag) < 1) {
        last_life_flag = raw_data.life_flag; //更新生命检测标志位
        return false; //数据没有变化
    }

    last_life_flag = raw_data.life_flag; //更新生命检测标志位

    return true;
}