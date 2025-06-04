/**
  ******************************************************************************
  * @file           : drv_dji_motor.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "drv_dji_motor.h"

uint8_t dji_motor_device::can1_tx_buff_0x200[8] = {};
uint8_t dji_motor_device::can1_tx_buff_0x1ff[8] = {};
uint8_t dji_motor_device::can1_tx_buff_0x2ff[8] = {};
uint8_t dji_motor_device::can2_tx_buff_0x200[8] = {};
uint8_t dji_motor_device::can2_tx_buff_0x1ff[8] = {};
uint8_t dji_motor_device::can2_tx_buff_0x2ff[8] = {};

dji_motor_device::dji_motor_device() : data({0,}), stall_flag(false), low_pass_alpha(1.0) {}

void dji_motor_device::init(CAN_HandleTypeDef *_hcan,
                            bool reverse_flag,
                            uint32_t _id,
                            enum DJI_MOTOR_TYPE type,
                            osSemaphoreId_t rx_sem,
                            float stall_current,
                            float stall_speed,
                            float offset_current) {
    this->lost_flag = true;
    this->ready_flag = false;
    this->is_zero_offset = false;
    this->msg_cnt = 0;
    this->type = type;
    this->reverse_flag = reverse_flag;
    this->data.stall_cnt = 0;
    this->data.stall_current_max = stall_current;
    this->data.stall_speed_min = stall_speed;
    this->data.offset_current = offset_current;
    if (_id <= 8 && _id >= 1) {
        this->id = _id;
    } else {
        return;
    }
    if (this->type == DJI_M3508 || this->type == DJI_M2006) {
        if (_hcan == &hcan1) {
            if (_id <= 4) {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x200;
                this->can_device.tx_member.id = 0x200;
            } else {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x1ff;
                this->can_device.tx_member.id = 0x1FF;
            }
        } else if (_hcan == &hcan2) {
            if (_id <= 4) {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x200;
                this->can_device.tx_member.id = 0x200;
            } else {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x1ff;
                this->can_device.tx_member.id = 0x1FF;
            }
        }
        this->can_device.tx_member.len = 8;
        this->can_device.init_rx(_hcan,
                                 0x200 + this->id,
                                 std::bind(&dji_motor_device::update_data, this, std::placeholders::_1),
                                 rx_sem);
    } else if (type == DJI_GM6020) {
        if (_hcan == &hcan1) {
            if (_id <= 4) {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x1ff;
                this->can_device.tx_member.id = 0x1ff;
            } else {
                this->can_device.tx_member.buf_data = can1_tx_buff_0x2ff;
                this->can_device.tx_member.id = 0x2ff;
            }
        } else if (_hcan == &hcan2) {
            if (_id <= 4) {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x1ff;
                this->can_device.tx_member.id = 0x1ff;
            } else {
                this->can_device.tx_member.buf_data = can2_tx_buff_0x2ff;
                this->can_device.tx_member.id = 0x2ff;
            }
        }
        this->can_device.tx_member.len = 8;
        this->can_device.init_rx(_hcan,
                                 0x204 + this->id,
                                 std::bind(&dji_motor_device::update_data, this, std::placeholders::_1),
                                 rx_sem);
    }
}

void dji_motor_device::check_motor_for_stall() {
    if (ABS(this->data.torque_current) > ABS(this->data.stall_current_max)
        && ABS(this->data.current_speed) < ABS(this->data.stall_speed_min)) {
        this->data.stall_cnt++;
    } else {
        this->data.stall_cnt = 0;
    }

    if (this->data.stall_cnt > 20) {
        this->stall_flag = true;
    } else {
        this->stall_flag = false;
    }
}

void dji_motor_device::update_data(uint8_t *rx_data) {
    dji_motor_data_t *data = &(this->data);
    dji_motor_raw_data_t *raw = &(this->raw_data);

    raw->encoder = (uint16_t) (rx_data[0] << 8 | rx_data[1]);
    raw->speed_rpm = (int16_t) (rx_data[2] << 8 | rx_data[3]);
    raw->torque_current = (int16_t) (rx_data[4] << 8 | rx_data[5]);
    raw->temperature = rx_data[6];

    if (this->lost_flag) {
        this->msg_cnt = 0;
        this->is_zero_offset = false;
    }
    if (this->msg_cnt < 50 && !this->is_zero_offset) {
        switch (this->type) {
            case DJI_M3508:data->current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_M3508;
                break;
            case DJI_M2006:data->current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_M2006;
                break;
            case DJI_GM6020:data->current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_GM6020;
                break;
            default:break;
        }
        VAL_LIMIT(data->current_speed, -1.0f, 1.0f);
        data->current_round = (float) raw->encoder * ENCODER_TO_ROUND;
        VAL_LIMIT(data->current_round, -1.0f, 1.0f);
        data->zero_offset_round = data->current_round;
        this->msg_cnt++;
        this->data.stall_cnt = 0;
        return;
    }
    this->is_zero_offset = true;

    data->last_round = data->current_round;
    data->current_round = (float) raw->encoder * ENCODER_TO_ROUND;
    VAL_LIMIT(data->current_round, -1.0f, 1.0f);

    if (data->current_round - data->last_round > 0.5f) {
        data->round_cnt--;
    }
    if (data->current_round - data->last_round < -0.5f) {
        data->round_cnt++;
    }

    data->total_rounds = (float) data->round_cnt + data->current_round - data->zero_offset_round;//归一化之后走的总圈数

    switch (this->type) {
        case DJI_M3508:data->raw_current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_M3508;
            break;
        case DJI_M2006:data->raw_current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_M2006;
            break;
        case DJI_GM6020:data->raw_current_speed = (float) raw->speed_rpm / DJI_MOTOR_MAX_SPEED_GM6020;
            break;
        default:break;
    }
    data->current_speed = data->raw_current_speed * low_pass_alpha + data->last_speed * (1 - low_pass_alpha);
    data->last_speed = data->current_speed;

    VAL_LIMIT(data->current_speed, -1.0f, 1.0f);

    data->torque_current = raw->torque_current;

    check_motor_for_stall();
}

void dji_motor_device::update_ready() {
    if (!this->lost_flag && this->is_zero_offset) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }
}

bool dji_motor_device::check_ready() const {
    return this->ready_flag;
}

void dji_motor_device::set_current(float current) {
    float current_motor_set = current + this->data.offset_current;
    VAL_LIMIT(current_motor_set, -1.0f, 1.0f);
    if (this->reverse_flag) {
        switch (this->type) {
            case DJI_M3508:this->current_set = (int16_t) (-current_motor_set * DJI_MOTOR_MAX_CURRENT_M3508);
                break;
            case DJI_M2006:this->current_set = (int16_t) (-current_motor_set * DJI_MOTOR_MAX_CURRENT_M2006);
                break;
            case DJI_GM6020:this->current_set = (int16_t) (-current_motor_set * DJI_MOTOR_MAX_CURRENT_GM6020);
                break;
            default:break;
        }
    } else {
        switch (this->type) {
            case DJI_M3508:this->current_set = (int16_t) (current_motor_set * DJI_MOTOR_MAX_CURRENT_M3508);
                break;
            case DJI_M2006:this->current_set = (int16_t) (current_motor_set * DJI_MOTOR_MAX_CURRENT_M2006);
                break;
            case DJI_GM6020:this->current_set = (int16_t) (current_motor_set * DJI_MOTOR_MAX_CURRENT_GM6020);
                break;
            default:break;
        }
    }
    set_current_to_can_tx_buff();

}

float dji_motor_device::get_total_rounds() const {
    if (this->reverse_flag) {
        return -this->data.total_rounds;
    } else {
        return this->data.total_rounds;
    }

}

float dji_motor_device::get_current_round() {
    if (this->reverse_flag) {
        return -this->data.current_round;
    } else {
        return this->data.current_round;
    }
}

float dji_motor_device::get_speed() {
    float speed = 0;
    if (is_using_external_speed) {
        speed = *data.external_speed;
    } else {
        speed = data.current_speed;
    }
    if (this->reverse_flag) {
        return -speed;
    } else {
        return speed;
    }
}

float dji_motor_device::get_speed_without_external() {
    if (this->reverse_flag) {
        return -data.current_speed;
    } else {
        return data.current_speed;
    }
}

void dji_motor_device::set_pos(float set_pos) {

    this->set_vel(this->pospid.pid_calculate(set_pos, this->get_total_rounds()));

}

void dji_motor_device::set_vel(float set_vel) {
    this->set_current(this->velpid.pid_calculate(set_vel, this->get_speed()));
}

void dji_motor_device::set_current_to_can_tx_buff() const {
    uint32_t id = this->id;
    uint8_t index;
    if (id < 5) {
        index = id * 2 - 1;
    } else {
        index = (id - 4) * 2 - 1;
    }
    this->can_device.tx_member.buf_data[index - 1] = HIGH_BYTE(this->current_set);
    this->can_device.tx_member.buf_data[index] = LOW_BYTE(this->current_set);
}

void dji_motor_device::set_free() {
    this->msg_cnt = 0;
    this->is_zero_offset = false;
    this->current_set = 0;
    this->set_current_to_can_tx_buff();
}

void dji_motor_device::set_current_zero() {
    this->current_set = 0;
    this->set_current_to_can_tx_buff();
}

void dji_motor_device::reset_total_rounds_zero_offset(float total_rounds) {
    float round_f = total_rounds - (float) ((int) (total_rounds));//取出total_rounds的小数部分
    if (!this->reverse_flag)//正转
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

void dji_motor_device::set_reverse() {
    if (!this->reverse_flag) {
        this->reverse_flag = true;
        this->reset_total_rounds_zero_offset(this->get_total_rounds());
    }
}

void dji_motor_device::set_forward() {
    if (this->reverse_flag) {
        this->reverse_flag = false;
        this->reset_total_rounds_zero_offset(this->get_total_rounds());
    }
}

void dji_motor_device::send_can_msg() {
    this->can_device.send_msg();
}

bool dji_motor_device::check_lost() const {
    return this->lost_flag;
}

void dji_motor_device::check_motor_for_loss() {
    osStatus_t stat;
    stat = osSemaphoreAcquire(this->can_device.rx_sem, 50);
    if (stat != osOK) {
        lost_num++;
        this->lost_flag = true;
    } else {
        if (this->lost_flag) {
            this->lost_flag = false;
//            this->reset_total_rounds_zero_offset(0);
        }
    }
}

bool dji_motor_device::check_reverse() const {
    return this->reverse_flag;
}

bool dji_motor_device::check_stall() const {
    return this->stall_flag;
}

void dji_motor_device::set_stall_parameter(int16_t _stall_current_max, float _stall_speed_min) {
    this->data.stall_speed_min = _stall_speed_min;
    this->data.stall_current_max = _stall_current_max;
}

void dji_motor_device::set_pid(pid_param pospid, pid_param velpid) {
//    this->velpid.init(velpid.max_out,velpid.integral_limit,velpid.kp,velpid.ki,velpid.kd);
//    this->pospid.init(pospid.max_out,pospid.integral_limit,pospid.kp,pospid.ki,pospid.kd);
    this->velpid.pid_reset(velpid.max_out,
                           velpid.integral_higher_limit,
                           velpid.p,
                           velpid.i,
                           velpid.d,
                           velpid.ap,
                           velpid.bp,
                           velpid.cp);
    this->pospid.pid_reset(pospid.max_out,
                           pospid.integral_higher_limit,
                           pospid.p,
                           pospid.i,
                           pospid.d,
                           pospid.ap,
                           pospid.bp,
                           pospid.cp);
}

void dji_motor_device::change_speed_source(float *speed) {
    is_using_external_speed = true;
    this->data.external_speed = speed;
}

void dji_motor_device::set_low_pass_alpha(float alpha) {
    low_pass_alpha = alpha;
}


void dji_motor_device::set_offset_current(float offset_current) {
    data.offset_current = offset_current;
}