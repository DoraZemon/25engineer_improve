/**
  ******************************************************************************
  * @file           : drv_JY_ME02.cpp
  * @author         : 34147
  * @brief          : 维特的JY-ME02-CAN编码器的底层
  * @attention      : None
  * @date           : 2024/7/1
  ******************************************************************************
  */



#include "drv_JY_ME02.h"


JY_ME02_encoder_device::JY_ME02_encoder_device() : raw_data({0.0f, 0, 0.0f,0.0f}),lost_flag(false){}


void JY_ME02_encoder_device::init(CAN_HandleTypeDef *_hcan, uint32_t rx_id, osSemaphoreId_t rx_sem) {
    can_device.init_rx(_hcan,rx_id,std::bind(&JY_ME02_encoder_device::JY_ME02_rx_data_callback,this,std::placeholders::_1),rx_sem);
}


void JY_ME02_encoder_device::JY_ME02_rx_data_callback( uint8_t *rx_data){
    if(rx_data[0] == UNIVERSAL_HEAD && rx_data[1] == ANGLE_HEAD){//分两个包发送接收
        this->raw_data.angle = float ((rx_data[3] << 8) | rx_data[2]) * 360.f / 32768.f;
        this->raw_data.ang_vel = float ((rx_data[5] << 8) | rx_data[4]) * 360.f / 32768.f/ENCODER_SAMPLING_TIME;
        this->raw_data.round_cnt = int16_t((rx_data[7] << 8) | rx_data[6]);
    }else if(rx_data[0] == UNIVERSAL_HEAD && rx_data[1] == TEMP_HEAD){
        this->raw_data.temperature = float((rx_data[3] << 8) | rx_data[2]) /100.f;
    }
}

float JY_ME02_encoder_device::get_angle() const {
    return this->raw_data.angle;
}

float JY_ME02_encoder_device::get_total_angle() const {
    return raw_data.angle + raw_data.round_cnt * 360.f;
}

void JY_ME02_encoder_device::check_for_loss() {
    osStatus_t stat;
    stat = osSemaphoreAcquire(this->can_device.rx_sem, 400);
    if (stat != osOK) {
        this->lost_flag = true;
    } else {
        if (this->lost_flag) {
            this->lost_flag = false;
        }
    }

}

bool JY_ME02_encoder_device::check_lost() const {
    return this->lost_flag;
}