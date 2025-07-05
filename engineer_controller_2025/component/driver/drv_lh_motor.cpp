/**
  ******************************************************************************
  * @file           : drv_lh_motor.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#include <cstring>
#include "drv_lh_motor.h"
#include "user_lib.h"
#include "bsp_buzzer.h"

void lh_motor_device::init(FDCAN_HandleTypeDef *hfdcan_,
                           uint32_t id_,
                           bool is_reverse_,
                           lh_motor_device::control_mode_e control_mode,
                           osSemaphoreId_t rx_sem_) {
    hfdcan = hfdcan_;
    id = id_;
    is_reverse = is_reverse_;
    motor_state = ENABLED;
    this->fdcan_device.init_rx(hfdcan_,
                               id_ + FEEDBACK_OFFSET + REQUEST_FEEDBACK,
                               std::bind(&lh_motor_device::update_data, this, std::placeholders::_1),
                               rx_sem_);
    this->fdcan_device.init_tx(hfdcan_, FDCAN_DLC_BYTES_16, id_ + CONTROL, (uint8_t *) &this->tx_buff);
}

void lh_motor_device::update_data(uint8_t *can_rx_data) {

    raw_data.data_length = can_rx_data[0];
    raw_data.enable_flag = can_rx_data[1];
    raw_data.control_mode = can_rx_data[2];

    raw_data.pos_ecd = big_endian_to_float(&can_rx_data[3]);
    raw_data.speed = big_endian_to_float(&can_rx_data[7]);
    raw_data.current = big_endian_to_float(&can_rx_data[11]);

    raw_data.error_code = big_endian_to_uint32(&can_rx_data[15]);
    raw_data.temperature = big_endian_to_uint16(&can_rx_data[19]);
    raw_data.voltage = big_endian_to_uint16(&can_rx_data[21]);

    raw_data.limit_flag = can_rx_data[23];

    uint8_t temp_array[24];

    memcpy(temp_array, can_rx_data, 24);


    if (msg_cnt < 50 && !is_zero_offset) {
        data.last_round = data.current_round;

        data.current_round = raw_data.pos_ecd;//todo 归一化

        data.zero_offset_round = data.current_round;

        data.last_speed = data.current_speed;

        data.current_speed = raw_data.speed;
        data.fb_torque_current = raw_data.current;
        data.fb_voltage = raw_data.voltage / 10.f;
        data.temperature = raw_data.temperature / 10.f;
        msg_cnt++;
        return;
    }

    is_zero_offset = true;

    data.last_round = data.current_round;
    data.current_round = raw_data.pos_ecd;//todo 归一化

    data.last_speed = data.current_speed;
    data.current_speed = raw_data.speed;

    if (data.current_round - data.last_round > 180.f) {
        data.round_cnt = data.round_cnt - 1;
    } else if (data.current_round - data.last_round < -180.f) {
        data.round_cnt = data.round_cnt + 1;
    }
    data.total_rounds = (float) data.round_cnt * 360.f + data.current_round
                        - data.zero_offset_round;//为0
    data.total_rounds_without_offset =
        (float) data.round_cnt * 360.f + data.current_round;

    data.fb_torque_current = raw_data.current;
    data.fb_voltage = raw_data.voltage / 10.f;
    data.temperature = raw_data.temperature / 10.f;
}

void lh_motor_device::check_motor_for_loss() {

    osStatus_t stat;
    stat = osSemaphoreAcquire(fdcan_device.rx_sem, 50);
    if (stat != osOK) {
        // 超时，认为电机丢失
        is_lost = true;
    } else {
        if(is_lost){
            is_lost = false;
        }
    }

}


float lh_motor_device::get_total_rounds() const {
    if (!is_reverse) {
        return data.total_rounds;
    } else {
        return -data.total_rounds;
    }
}


float lh_motor_device::get_current_round() {
    if (!is_reverse) {
        return data.current_round;
    } else {
        return -data.current_round;
    }
}

float lh_motor_device::get_speed() {
    if (!is_reverse) {
        return data.current_speed;
    } else {
        return -data.current_speed;
    }
}

void lh_motor_device::reset_total_rounds_zero_offset(float total_rounds) {
    float round_f = total_rounds - (float) ((int) (total_rounds));//取出total_rounds的小数部分
    if (!this->is_reverse)//正转
    {
        this->data.round_cnt = (int) total_rounds;
        this->data.zero_offset_round = this->data.current_round - round_f;
    } else//反转
    {
        this->data.round_cnt = -(int) total_rounds;
        this->data.zero_offset_round = this->data.current_round + round_f;
    }
    this->is_zero_offset = true;
}

void lh_motor_device::update_ready() {
    if (!this->is_lost && this->is_zero_offset) {
        this->is_ready = true;
    } else {
        this->is_ready = false;
    }
}

void lh_motor_device::request_feedback() {
    memset(tx_buff, 0, 16);
    fdcan_device.reset_tx(FDCAN_DLC_BYTES_1, id + REQUEST_FEEDBACK);
    fdcan_device.send_msg();
}

void lh_motor_device::send_control() {
    memset(tx_buff, 0, 16);

    tx_buff[0] = 0x0E;
    tx_buff[1] = motor_state;
    tx_buff[2] = control_mode;

    float_to_big_endian_bytes(ctrl_data.pos, &tx_buff[3]);
    float_to_big_endian_bytes(ctrl_data.vel, &tx_buff[7]);
    float_to_big_endian_bytes(ctrl_data.current, &tx_buff[11]);

    fdcan_device.reset_tx(FDCAN_DLC_BYTES_16, id + CONTROL);
    fdcan_device.send_msg();
}

void lh_motor_device::function_operation(uint8_t cmd_id) {
    memset(tx_buff, 0, 16);
    tx_buff[0] = 1;
    tx_buff[1] = cmd_id;
    fdcan_device.reset_tx(FDCAN_DLC_BYTES_2, id + FUNCTION_OPERATION);
    fdcan_device.send_msg();
}

void lh_motor_device::parameter_read(uint16_t para_add, float *data) {

    memset(tx_buff, 0, 16);
    tx_buff[0] = 3;
    tx_buff[1] = 0x01; // 0x01表示读取参数

    tx_buff[2] = (uint8_t) (para_add >> 8); // 高字节
    tx_buff[3] = (uint8_t) (para_add & 0xFF); // 低字节

    fdcan_device.reset_tx(FDCAN_DLC_BYTES_4, id + PARAMETER_READ);
    fdcan_device.send_msg();

    auto receive_param_func = [this, data, para_add](uint8_t *can_rx_data) {
      if (can_rx_data[2] == (uint8_t) (para_add >> 8) && can_rx_data[3] == (uint8_t) (para_add & 0xFF)) {
          if (can_rx_data[4] == INT) {
          } else if (can_rx_data[4] == FLOAT) {
              *data = big_endian_to_float(&can_rx_data[5]);

          }
      }
    };

    parameter_read_device.init_rx(hfdcan,
                                  id + PARAMETER_READ + FEEDBACK_OFFSET,
                                  receive_param_func,
                                  nullptr);
}

void lh_motor_device::parameter_read(uint16_t para_add, int32_t *data) {

    memset(tx_buff, 0, 16);
    tx_buff[0] = 3;
    tx_buff[1] = 0x01; // 0x01表示读取参数

    tx_buff[2] = (uint8_t) (para_add >> 8); // 高字节
    tx_buff[3] = (uint8_t) (para_add & 0xFF); // 低字节

    fdcan_device.reset_tx(FDCAN_DLC_BYTES_4, id + PARAMETER_READ);
    fdcan_device.send_msg();

    auto receive_param_func = [this, data, para_add](uint8_t *can_rx_data) {
      if (can_rx_data[2] == (uint8_t) (para_add >> 8) && can_rx_data[3] == (uint8_t) (para_add & 0xFF)) {
          if (can_rx_data[4] == INT) {
              *data = big_endian_to_int32(&can_rx_data[5]);
          } else if (can_rx_data[4] == FLOAT) {
          }
      }
    };

    parameter_read_device.init_rx(hfdcan,
                                  id + PARAMETER_READ + FEEDBACK_OFFSET,
                                  receive_param_func,
                                  nullptr);
}

void lh_motor_device::parameter_write(uint16_t para_add, float data) {
    memset(tx_buff, 0, 16);
    tx_buff[0] = 8;
    tx_buff[1] = 0x02; // 0x02表示写入参数

    tx_buff[2] = (uint8_t) (para_add >> 8); // 高字节
    tx_buff[3] = (uint8_t) (para_add & 0xFF); // 低字节

    tx_buff[4] = FLOAT;

    float_to_big_endian_bytes(data, &tx_buff[5]);

    fdcan_device.reset_tx(FDCAN_DLC_BYTES_12, id + PARAMETER_WRITE);
    fdcan_device.send_msg();
}

void lh_motor_device::parameter_write(uint16_t para_add, int32_t data) {
    memset(tx_buff, 0, 16);
    tx_buff[0] = 8;
    tx_buff[1] = 0x02; // 0x02表示写入参数

    tx_buff[2] = (uint8_t) (para_add >> 8); // 高字节
    tx_buff[3] = (uint8_t) (para_add & 0xFF); // 低字节

    tx_buff[4] = INT;

    int32_to_big_endian(data, &tx_buff[5]);

    fdcan_device.reset_tx(FDCAN_DLC_BYTES_12, id + PARAMETER_WRITE);
    fdcan_device.send_msg();
}


void lh_motor_device::set_vel(float set) {
    if (control_mode == CURRENT) {
        set_current(velpid.pid_calculate(set,get_speed()));
    } else if (control_mode == POSITION) {
//        ctrl_data.pos = set;
    } else if (control_mode == VELOCITY) {
        ctrl_data.vel = set;
    }
}

void lh_motor_device::float_to_big_endian_bytes(float value, uint8_t *out) {
    uint8_t *p = (uint8_t *) &value;
    out[0] = p[3];
    out[1] = p[2];
    out[2] = p[1];
    out[3] = p[0];
}

float lh_motor_device::big_endian_to_float(uint8_t *data) {
    float value;
    uint8_t *p = (uint8_t *) &value;
    p[0] = data[3];
    p[1] = data[2];
    p[2] = data[1];
    p[3] = data[0];
    return value;
}

uint16_t lh_motor_device::big_endian_to_uint16(uint8_t *data) {
    return ((uint16_t) data[0] << 8) | data[1];
}

uint32_t lh_motor_device::big_endian_to_uint32(uint8_t *data) {
    return ((uint32_t) data[0] << 24) | ((uint32_t) data[1] << 16) |
           ((uint32_t) data[2] << 8) | data[3];
}


void lh_motor_device::int32_to_big_endian(int32_t value, uint8_t *out) {
    out[0] = (value >> 24) & 0xFF;
    out[1] = (value >> 16) & 0xFF;
    out[2] = (value >> 8) & 0xFF;
    out[3] = (value) & 0xFF;
}


int32_t lh_motor_device::big_endian_to_int32(uint8_t *data) {
    return ((int32_t) data[0] << 24) | ((int32_t) data[1] << 16) |
           ((int32_t) data[2] << 8) | data[3];
}