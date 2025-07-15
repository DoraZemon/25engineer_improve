/**
  ******************************************************************************
  * @file           : drv_dm_imu.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "drv_dm_imu.h"
#include "cstring"
#include "user_lib.h"

const float deg2round = (float) (1 / 360.0);


void dm_imu_device::init(CAN_HandleTypeDef *hcan_, uint32_t canid, uint32_t master_id, osSemaphoreId_t rxsem) {
    is_using_quaternion = false;
    is_using_accel = false;
    can_device.init_rx(hcan_, master_id, std::bind(&dm_imu_device::update_data, this, std::placeholders::_1), rxsem);
    can_device.init_tx(hcan_, 4, 0x6FF, cmd);
    hcan = hcan_;
    can_id = canid;
}

void dm_imu_device::update_data(uint8_t *rx_data) {
    switch (rx_data[0]) {
        case Accel:update_accel(rx_data);
            break;
        case Gyro:update_gyro(rx_data);
            break;
        case Euler:update_euler(rx_data);
            update_euler_cnt();
            break;
        case Quaternion:update_quaternion(rx_data);
            break;
        default:break;

    }
}

void dm_imu_device::update_accel(uint8_t *pdata) {
    data.accel[0] = uint_to_float((pdata[3] << 8) | pdata[2], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    data.accel[1] = uint_to_float((pdata[5] << 8) | pdata[4], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
    data.accel[2] = uint_to_float((pdata[7] << 8) | pdata[6], ACCEL_CAN_MIN, ACCEL_CAN_MAX, 16);
}

void dm_imu_device::update_gyro(uint8_t *pdata) {
    data.gyro[0] = uint_to_float((pdata[3] << 8) | pdata[2], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    data.gyro[1] = uint_to_float((pdata[5] << 8) | pdata[4], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
    data.gyro[2] = uint_to_float((pdata[7] << 8) | pdata[6], GYRO_CAN_MIN, GYRO_CAN_MAX, 16);
}

void dm_imu_device::update_euler(uint8_t *pdata) {
    data.pitch = uint_to_float((pdata[3] << 8) | pdata[2], PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
    data.yaw = uint_to_float((pdata[5] << 8) | pdata[4], ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
    data.roll = uint_to_float((pdata[7] << 8) | pdata[6], YAW_CAN_MIN, YAW_CAN_MAX, 16);

}

void dm_imu_device::update_euler_cnt() {
    if (!this->is_zero_offset && this->msg_cnt > 50) {
        this->msg_cnt = 0;
    }//保证直接用zero offset即可初始化

    this->msg_cnt++;
    if (this->msg_cnt > 50) {
        this->is_zero_offset = true;
    } else {
        this->is_zero_offset = false;
    }

    /* initial value */
    if (!this->is_zero_offset) {

        this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;
        this->data.euler.pitch.current_ang = (float) this->data.pitch;
        this->data.euler.pitch.zero_offset_deg = this->data.euler.pitch.current_ang;//可以删掉，因为pitch以0为offset(六轴)
        this->data.euler.pitch.round_cnt = 0;

        this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
        this->data.euler.roll.current_ang = (float) this->data.roll;
        this->data.euler.roll.zero_offset_deg = this->data.euler.roll.current_ang;//可以删掉，因为roll以0为offset(六轴)
        this->data.euler.roll.round_cnt = 0;

        this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.current_ang = (float) this->data.yaw;
        this->data.euler.yaw.zero_offset_deg = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.round_cnt = 0;

        return;
    }

    this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
    this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
    this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;

    this->data.euler.yaw.current_ang = (float) this->data.yaw;
    this->data.euler.pitch.current_ang = (float) this->data.pitch;
    this->data.euler.roll.current_ang = (float) this->data.roll;

    if (this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang < -YAW_RANGE) {
        this->data.euler.yaw.round_cnt++;
    } else if (this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang > YAW_RANGE) {
        this->data.euler.yaw.round_cnt--;
    }

    this->data.euler.yaw.total_rounds =
        (float(this->data.euler.yaw.round_cnt) * YAW_RANGE * 2 + this->data.euler.yaw.current_ang
         - this->data.euler.yaw.zero_offset_deg) * deg2round;

    if (this->data.euler.roll.current_ang - this->data.euler.roll.last_ang < -ROLL_RANGE) {
        this->data.euler.roll.round_cnt++;
    } else if (this->data.euler.roll.current_ang - this->data.euler.roll.last_ang > ROLL_RANGE) {
        this->data.euler.roll.round_cnt--;
    }

    this->data.euler.roll.total_rounds =
        (float(this->data.euler.roll.round_cnt) * ROLL_RANGE * 2 + this->data.euler.roll.current_ang
         - this->data.euler.roll.zero_offset_deg) * deg2round;

    if (this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang < -PITCH_RANGE) {
        this->data.euler.pitch.round_cnt++;
    } else if (this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang > PITCH_RANGE) {
        this->data.euler.pitch.round_cnt--;
    }

    this->data.euler.pitch.total_rounds =
        (float(this->data.euler.pitch.round_cnt) * PITCH_RANGE * 2 + this->data.euler.pitch.current_ang
         - this->data.euler.pitch.zero_offset_deg) * deg2round;
}

void dm_imu_device::update_quaternion(uint8_t *pdata) {
    int w = pdata[1] << 6 | ((pdata[2] & 0xF8) >> 2);
    int x = (pdata[2] & 0x03) << 12 | (pdata[3] << 4) | ((pdata[4] & 0xF0) >> 4);
    int y = (pdata[4] & 0x0F) << 10 | (pdata[5] << 2) | (pdata[6] & 0xC0) >> 6;
    int z = (pdata[6] & 0x3F) << 8 | pdata[7];

    data.q[0] = uint_to_float(w, Quaternion_MIN, Quaternion_MAX, 14);
    data.q[1] = uint_to_float(x, Quaternion_MIN, Quaternion_MAX, 14);
    data.q[2] = uint_to_float(y, Quaternion_MIN, Quaternion_MAX, 14);
    data.q[3] = uint_to_float(z, Quaternion_MIN, Quaternion_MAX, 14);
}

/**
 * @brief 一发一答式
 */
void dm_imu_device::request_imu_data() {
    for (uint8_t i = 1; i <= 4; i++) {
        if ((!is_using_quaternion && i == 4) || (!is_using_accel && i == 1)) {
            continue;
        }
        can_device.tx_member.id = 0x6FF;
        can_device.tx_member.len = 4;
        uint8_t cmd_[4] = {(uint8_t) can_id, (uint8_t) (can_id >> 8), i, 0xCC};
        memcpy(can_device.tx_member.buf_data, cmd_, 4);
        can_device.send_msg();
        osDelay(2);
    }
}

// 假设这里返回的都是归一化的值
float dm_imu_device::get_pitch_raw() {
    return data.euler.pitch.current_ang / (360.f);
}

float dm_imu_device::get_roll() {
    return data.roll;
}

float dm_imu_device::get_yaw() {
    return data.euler.yaw.total_rounds;
}

float dm_imu_device::get_gyro_x() {
    return data.gyro[0];
}

float dm_imu_device::get_gyro_y() {
    return data.gyro[1];
}

float dm_imu_device::get_gyro_z() {
    return data.gyro[2];
}

void dm_imu_device::check_imu_for_loss() {
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

bool dm_imu_device::check_lost() {
    return is_lost;
}