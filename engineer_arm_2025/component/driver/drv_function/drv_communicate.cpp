/**
  ******************************************************************************
  * @file           : drv_communicate.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-6
  ******************************************************************************
  */


#include "drv_communicate.h"


void communicate_device::init(CAN_HandleTypeDef *hcan_, uint32_t tx_id, uint32_t rx_id, osSemaphoreId_t rxsem) {
    this->hcan = hcan_;
    can_device.init_rx(hcan, rx_id, std::bind(&communicate_device::update_rx_data, this, std::placeholders::_1), rxsem);
    can_device.init_tx(hcan, 8, tx_id, (uint8_t *) &tx_data);
}


void communicate_device::update(rc_device &rc) {
    data.is_rc_online = rc.check_ready();


}

void communicate_device::update_rx_data(uint8_t *rx_data) {
    if (rx_data[0] != COMMUNICATE_FRAME_HEAD) {
        return; //帧头不匹配，丢弃数据
    }
    auto raw_data = reinterpret_cast<communicate_rx_data_t *>(rx_data);
    data.is_arm_pump_holding = raw_data->is_arm_pump_holding;
    data.is_left_pump_holding = raw_data->is_left_pump_holding;
    data.is_right_pump_holding = raw_data->is_right_pump_holding;
    data.is_chassis_motor1_error = raw_data->chassis_motor1_error;
    data.is_chassis_motor2_error = raw_data->chassis_motor2_error;
    data.is_chassis_motor3_error = raw_data->chassis_motor3_error;
    data.is_chassis_motor4_error = raw_data->chassis_motor4_error;

    data.chassis_error = data.is_chassis_motor1_error || data.is_chassis_motor2_error ||
                         data.is_chassis_motor3_error || data.is_chassis_motor4_error;
}

void communicate_device::check_for_loss() {
    osStatus_t stat;
    stat = osSemaphoreAcquire(this->can_device.rx_sem, 500);
    if (stat != osOK) {
        lost_num++;
        this->is_lost = true;
    } else {
        if (this->is_lost) {
            this->is_lost = false;
        }
    }
}

void communicate_device::send_msg() {
    taskENTER_CRITICAL();
    tx_data.frame_head = COMMUNICATE_FRAME_HEAD;
    tx_data.is_arm_pump_open = data.is_arm_pump_open;
    tx_data.is_left_pump_open = data.is_left_pump_open;
    tx_data.is_right_pump_open = data.is_right_pump_open;
    tx_data.is_rc_online = data.is_rc_online;
    tx_data.speed_x = data.speed_x;
    tx_data.speed_y = data.speed_y;
    tx_data.speed_spin = data.speed_spin;
    taskEXIT_CRITICAL();
    can_device.send_msg();
}


void communicate_device::set_chassis_ctrl(int16_t speed_x,
                                          int16_t speed_y,
                                          int8_t speed_spin) {
    data.speed_x = speed_x;
    data.speed_y = speed_y;
    data.speed_spin = speed_spin;
}

void communicate_device::set_pump_ctrl(bool is_arm_pump_open, bool is_left_pump_open, bool is_right_pump_open) {
    data.is_arm_pump_open = is_arm_pump_open;
    data.is_left_pump_open = is_left_pump_open;
    data.is_right_pump_open = is_right_pump_open;
}


bool communicate_device::check_arm_pump_holding() {
    return data.is_arm_pump_holding;
}

bool communicate_device::check_left_pump_holding() {
    return data.is_left_pump_holding;
}

bool communicate_device::check_right_pump_holding() {
    return data.is_right_pump_holding;
}

bool communicate_device::check_chassis_error() {
    return data.chassis_error;
}

bool communicate_device::check_lost() {
    return is_lost;
}