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
#include "GlobalCfg.h"


void communicate_device::init(CAN_HandleTypeDef *hcan_, uint32_t tx_id, uint32_t rx_id, osSemaphoreId_t rxsem) {
    this->hcan = hcan_;
    can_device.init_rx(hcan, rx_id, std::bind(&communicate_device::update_rx_data, this, std::placeholders::_1), rxsem);
    can_device.init_tx(hcan, 8, tx_id, (uint8_t *) &tx_data);
}


void communicate_device::update(chassis_device &chassis, pump_device &pump) {
    data.is_arm_pump_holding = pump.check_arm_pump_holding();
    data.is_left_pump_holding = pump.check_left_pump_holding();
    data.is_right_pump_holding = pump.check_right_pump_holding();

    data.is_chassis_motor1_error = chassis.wheel[0].lost_flag;
    data.is_chassis_motor2_error = chassis.wheel[1].lost_flag;
    data.is_chassis_motor3_error = chassis.wheel[2].lost_flag;
    data.is_chassis_motor4_error = chassis.wheel[3].lost_flag;


    pump.set_arm_pump_open_state(data.is_arm_pump_open);
    pump.set_left_pump_open_state(data.is_left_pump_open);
    pump.set_right_pump_open_state(data.is_right_pump_open);

        chassis.set_speed_x(data.speed_x);
        chassis.set_speed_y(data.speed_y);
        chassis.set_speed_spin(data.speed_spin);

}

void communicate_device::update_rx_data(uint8_t *rx_data) {
    if (rx_data[0] != COMMUNICATE_FRAME_HEAD) {
        return; //帧头不匹配，丢弃数据
    }
    auto raw_data = reinterpret_cast<communicate_rx_data_t *>(rx_data);
    data.is_arm_pump_open = raw_data->is_arm_pump_open;
    data.is_left_pump_open = raw_data->is_left_pump_open;
    data.is_right_pump_open = raw_data->is_right_pump_open;
    data.is_rc_online = raw_data->is_rc_online;

    data.speed_x = float(raw_data->speed_x) / powf(2.0,15);
    data.speed_y = float(raw_data->speed_y) / powf(2.0,15);
    data.speed_spin = float(raw_data->speed_spin) / powf(2.0,7);
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

bool communicate_device::check_is_rc_online() {
    return data.is_rc_online;
}

void communicate_device::send_msg() {
    taskENTER_CRITICAL();
    tx_data.frame_head = COMMUNICATE_FRAME_HEAD;
    tx_data.is_left_pump_holding = data.is_left_pump_holding;
    tx_data.is_right_pump_holding = data.is_right_pump_holding;
    tx_data.is_arm_pump_holding = data.is_arm_pump_holding;
    tx_data.chassis_motor1_error = data.is_chassis_motor1_error;
    tx_data.chassis_motor2_error = data.is_chassis_motor2_error;
    tx_data.chassis_motor3_error = data.is_chassis_motor3_error;
    tx_data.chassis_motor4_error = data.is_chassis_motor4_error;
    taskEXIT_CRITICAL();
    can_device.send_msg();
}
