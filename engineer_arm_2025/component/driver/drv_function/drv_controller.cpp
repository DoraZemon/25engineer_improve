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

    return true;
}