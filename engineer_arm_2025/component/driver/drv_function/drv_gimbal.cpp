/**
  ******************************************************************************
  * @file           : drv_gimbal.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-1
  ******************************************************************************
  */


#include "drv_gimbal.h"


gimbal_device::gimbal_device() : data({ GIMBAL_SERVO_PITCH_HORIZONTAL_1000, GIMBAL_SERVO_YAW_FORWARD_1000}),
                                 pitch_servo(&SERVO_UART, GIMBAL_PITCH_SERVO_ID),
                                 yaw_servo(&SERVO_UART, GIMBAL_YAW_SERVO_ID) {}


void gimbal_device::update_control() {
    static int16_t s_pitch_last_msg = 0;


    if (ABS(s_pitch_last_msg - this->data.servo_set_pitch_1000) > 0.005f) {
        this->pitch_servo.set_pos(this->data.servo_set_pitch_1000);
        s_pitch_last_msg = this->data.servo_set_pitch_1000;
    }


    static int16_t s_yaw_last_msg = 0;

    if (ABS(s_yaw_last_msg - this->data.servo_set_yaw_1000) > 0.005f) {
        this->yaw_servo.set_pos(this->data.servo_set_yaw_1000);
        s_yaw_last_msg = this->data.servo_set_yaw_1000;
    }

}


void gimbal_device::set_pitch_target(int16_t set) {
    data.servo_set_pitch_1000 = set;
    VAL_LIMIT(data.servo_set_pitch_1000, GIMBAL_PITCH_1000_MIN, GIMBAL_PITCH_1000_MAX);
}

void gimbal_device::set_yaw_target(int16_t set) {
    data.servo_set_yaw_1000 = set;
    VAL_LIMIT(data.servo_set_yaw_1000, GIMBAL_YAW_1000_MIN, GIMBAL_YAW_1000_MAX);
}




