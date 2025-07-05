/**
  ******************************************************************************
  * @file           : drv_dm_motor.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/4/21
  ******************************************************************************
  */


#include "drv_dm_motor.h"

float dm_motor_device::uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

int dm_motor_device::float_to_uint(float x, float x_min, float x_max, int bits) {
/// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

void dm_motor_device::init(FDCAN_HandleTypeDef *hfdcan,
                           uint32_t slaveId,
                           uint16_t masterId,
                           dm_motor_mode_e motor_mode,
                           bool is_reverse,
                           DM_MOTOR_TYPE type_, osSemaphoreId_t rxSem) {
    slave_id = slaveId;
    master_id = masterId;
    this->is_reverse = is_reverse;
    mode = motor_mode;
    switch (motor_mode) {

        case DM_MIT:mit_mode = DM_MIT_Torque;
            fdcan_device.init_rx(hfdcan,
                                 masterId,
                                 std::bind(&dm_motor_device::update_data, this, std::placeholders::_1),
                                 rxSem);
            fdcan_device.init_tx(hfdcan, FDCAN_DLC_BYTES_8, slaveId, tx_buff);
            break;
        case DM_PV:
            fdcan_device.init_rx(hfdcan,
                                 masterId,
                                 std::bind(&dm_motor_device::update_data, this, std::placeholders::_1),
                                 rxSem);
            fdcan_device.init_tx(hfdcan, FDCAN_DLC_BYTES_8, 0x100 + slaveId, tx_buff);
            break;
        case DM_VO:
            fdcan_device.init_rx(hfdcan,
                                 masterId,
                                 std::bind(&dm_motor_device::update_data, this, std::placeholders::_1),
                                 rxSem);
            fdcan_device.init_tx(hfdcan, FDCAN_DLC_BYTES_8, 0x200 + slaveId, tx_buff);
            break;
        default:break;
    }
    type = type_;
    switch (type) {

        case DM_J3507_2EC:basic_info.v_max = DM_J3507_2EC_V_MAX;
            basic_info.p_max = DM_J3507_2EC_P_MAX;
            basic_info.t_max = DM_J3507_2EC_T_MAX;
            basic_info.kp_min = DM_J3507_2EC_KP_MIN;
            basic_info.kp_max = DM_J3507_2EC_KP_MAX;
            basic_info.kd_min = DM_J3507_2EC_KD_MIN;
            basic_info.kd_max = DM_J3507_2EC_KD_MAX;
            break;
        case DM_J4310_2EC:basic_info.v_max = DM_J4310_2EC_V_MAX;
            basic_info.p_max = DM_J4310_2EC_P_MAX;
            basic_info.t_max = DM_J4310_2EC_T_MAX;
            basic_info.kp_min = DM_J4310_2EC_KP_MIN;
            basic_info.kp_max = DM_J4310_2EC_KP_MAX;
            basic_info.kd_min = DM_J4310_2EC_KD_MIN;
            basic_info.kd_max = DM_J4310_2EC_KD_MAX;
            break;
    }
    set_motor_enable();
}

void dm_motor_device::set_motor_enable() {
    fdcan_device.tx_member.buf_data[0] = 0xff;
    fdcan_device.tx_member.buf_data[1] = 0xff;
    fdcan_device.tx_member.buf_data[2] = 0xff;
    fdcan_device.tx_member.buf_data[3] = 0xff;
    fdcan_device.tx_member.buf_data[4] = 0xff;
    fdcan_device.tx_member.buf_data[5] = 0xff;
    fdcan_device.tx_member.buf_data[6] = 0xff;
    fdcan_device.tx_member.buf_data[7] = 0xfc;
    for (uint8_t tmp = 0; tmp < 10; tmp++) {
        if (fdcan_device.send_msg() == osOK) {
            __NOP();
        }
        osDelay(3);
    }
    is_enable = true;
    motor_state = Enabled;
    osDelay(20);
}

void dm_motor_device::set_motor_disable() {
    fdcan_device.tx_member.buf_data[0] = 0xff;
    fdcan_device.tx_member.buf_data[1] = 0xff;
    fdcan_device.tx_member.buf_data[2] = 0xff;
    fdcan_device.tx_member.buf_data[3] = 0xff;
    fdcan_device.tx_member.buf_data[4] = 0xff;
    fdcan_device.tx_member.buf_data[5] = 0xff;
    fdcan_device.tx_member.buf_data[6] = 0xff;
    fdcan_device.tx_member.buf_data[7] = 0xfd;
    for (uint8_t tmp = 0; tmp < 10; tmp++) {
        if (fdcan_device.send_msg() == osOK) {
            __NOP();
        }
        osDelay(3);
    }
    is_enable = false;
    motor_state = Disabled;
    is_zero_offset = false;
    ctrl_data.torq = 0;
    ctrl_data.kp = 0;
    ctrl_data.kd = 0;
}

void dm_motor_device::set_motor_save_zero_offset() {
    fdcan_device.tx_member.buf_data[0] = 0xff;
    fdcan_device.tx_member.buf_data[1] = 0xff;
    fdcan_device.tx_member.buf_data[2] = 0xff;
    fdcan_device.tx_member.buf_data[3] = 0xff;
    fdcan_device.tx_member.buf_data[4] = 0xff;
    fdcan_device.tx_member.buf_data[5] = 0xff;
    fdcan_device.tx_member.buf_data[6] = 0xff;
    fdcan_device.tx_member.buf_data[7] = 0xfe;
    for (uint8_t tmp = 0; tmp < 6; tmp++) {
        if (fdcan_device.send_msg() == osOK) {
            __NOP();
        }
        osDelay(2);
    }
}

void dm_motor_device::set_motor_clear_error() {
    fdcan_device.tx_member.buf_data[0] = 0xff;
    fdcan_device.tx_member.buf_data[1] = 0xff;
    fdcan_device.tx_member.buf_data[2] = 0xff;
    fdcan_device.tx_member.buf_data[3] = 0xff;
    fdcan_device.tx_member.buf_data[4] = 0xff;
    fdcan_device.tx_member.buf_data[5] = 0xff;
    fdcan_device.tx_member.buf_data[6] = 0xff;
    fdcan_device.tx_member.buf_data[7] = 0xfb;
    for (uint8_t tmp = 0; tmp < 6; tmp++) {
        if (fdcan_device.send_msg() == osOK) {
            __NOP();
        }
        osDelay(2);
    }
}

//不断调用
void dm_motor_device::set_ctrl_to_can_tx_buff() {
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint8_t *pbuf, *vbuf;
    switch (mode) {
        case DM_MIT://MIT模式
            pos_tmp = float_to_uint(ctrl_data.p_des, -basic_info.p_max, basic_info.p_max, 16);
            vel_tmp = float_to_uint(ctrl_data.v_des, -basic_info.v_max, basic_info.v_max, 12);
            tor_tmp = float_to_uint(ctrl_data.torq, -basic_info.t_max, basic_info.t_max, 12);
            kp_tmp = float_to_uint(ctrl_data.kp, basic_info.kp_min, basic_info.kp_max, 12);
            kd_tmp = float_to_uint(ctrl_data.kd, basic_info.kd_min, basic_info.kd_max, 12);

            fdcan_device.tx_member.buf_data[0] = (pos_tmp >> 8);
            fdcan_device.tx_member.buf_data[1] = pos_tmp;
            fdcan_device.tx_member.buf_data[2] = (vel_tmp >> 4);
            fdcan_device.tx_member.buf_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            fdcan_device.tx_member.buf_data[4] = kp_tmp;
            fdcan_device.tx_member.buf_data[5] = (kd_tmp >> 4);
            fdcan_device.tx_member.buf_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            fdcan_device.tx_member.buf_data[7] = tor_tmp;
            break;
        case DM_PV://位置速度模式
            pbuf = (uint8_t *) &ctrl_data.p_des;
            vbuf = (uint8_t *) &ctrl_data.v_des;
            fdcan_device.tx_member.buf_data[0] = *pbuf;
            fdcan_device.tx_member.buf_data[1] = *(pbuf + 1);
            fdcan_device.tx_member.buf_data[2] = *(pbuf + 2);
            fdcan_device.tx_member.buf_data[3] = *(pbuf + 3);
            fdcan_device.tx_member.buf_data[4] = *vbuf;
            fdcan_device.tx_member.buf_data[5] = *(vbuf + 1);
            fdcan_device.tx_member.buf_data[6] = *(vbuf + 2);
            fdcan_device.tx_member.buf_data[7] = *(vbuf + 3);
            break;
        case DM_VO://速度模式
            vbuf = (uint8_t *) &ctrl_data.v_des;
            fdcan_device.tx_member.buf_data[0] = *vbuf;
            fdcan_device.tx_member.buf_data[1] = *(vbuf + 1);
            fdcan_device.tx_member.buf_data[2] = *(vbuf + 2);
            fdcan_device.tx_member.buf_data[3] = *(vbuf + 3);
            break;
        default:break;
    }
}

bool dm_motor_device::recover_the_motor() {
    if ((dm_motor_state_enum) raw_data.err >= OverVoltage && (dm_motor_state_enum) raw_data.err <= Overload) {
        motor_state = (dm_motor_state_enum) raw_data.err;
        set_motor_clear_error();
        set_motor_disable();
        set_motor_enable();
        return true;
    } else if ((dm_motor_state_enum) raw_data.err == Disabled) {
        motor_state = (dm_motor_state_enum) raw_data.err;
        set_motor_enable();
        return true;
    }
    return false;
}

// --------------------- MIT --------------------- //

void dm_motor_device::MIT_inter_set_motor_normalization_torque(float torque) {
    torque_set = torque + offset_current;
    ABS_LIMIT(torque_set, 1.0f);
    if (is_reverse) {
        ctrl_data.torq = -torque_set * basic_info.t_max;
    } else {
        ctrl_data.torq = torque_set * basic_info.t_max;
    }
    ctrl_data.kp = 0;
    ctrl_data.kd = 0.4f;//0.3或0.4  原来是 0
}

void dm_motor_device::MIT_outer_set_motor_speed(float speed) {
    this->MIT_inter_set_motor_normalization_torque(velpid.pid_calculate(speed, get_speed()));
}

void dm_motor_device::MIT_outer_set_motor_total_rounds(float total_rounds) {
    this->MIT_outer_set_motor_speed(pospid.pid_calculate(total_rounds, get_total_rounds_without_offset()));
}

void dm_motor_device::MIT_inter_set_motor_speed(float speed) {
    ABS_LIMIT(speed, 1.0f);
    ctrl_data.v_des = speed * basic_info.v_max;
    ctrl_data.kp = 0; //->p_ves = 0
    //kd不为0
    ctrl_data.kd = 1;//可改
    ctrl_data.torq = 0;//可改
}

void dm_motor_device::MIT_inter_set_motor_round(float rounds) {
    float p_des = rounds * 2 * PI;
    ABS_LIMIT(p_des, basic_info.p_max);
    ctrl_data.p_des = p_des;
    ctrl_data.v_des = 0;//待测
    ctrl_data.kp = 1;//kp不能为0
    ctrl_data.kd = 1;//kd不能为0
    ctrl_data.torq = 0;//不确定
}

void dm_motor_device::MIT_ctrl_position_and_torque(float pos, float torque, float kp) {
    VAL_LIMIT(pos, 0.0f, 1.0f);
    ABS_LIMIT(torque, 1.0f);
    ctrl_data.p_des = pos * 2 * PI;
    if (is_reverse) {
        ctrl_data.torq = -torque * basic_info.t_max;
    } else {
        ctrl_data.torq = torque * basic_info.t_max;
    }
    ctrl_data.kp = kp;
    ctrl_data.kd = 0.4f;//0.3或0.4
}

// --------------------- PV --------------------- //
void dm_motor_device::PV_inter_set_motor_total_rounds(float total_rounds, float speed) {//30*0.2f = 6
    ABS_LIMIT(total_rounds, basic_info.p_max);
    ctrl_data.p_des = total_rounds * 2 * PI;
    ABS_LIMIT(speed, 1.0f);
    ctrl_data.v_des = speed * basic_info.v_max;
}

// --------------------- VO --------------------- //
void dm_motor_device::VO_inter_set_motor_speed(float speed) {
    ABS_LIMIT(speed, 1.0f);
    ctrl_data.v_des = speed * basic_info.v_max;
}

float dm_motor_device::get_speed() {
    float speed = 0;
    if (is_using_external_speed) {
        speed = *data.external_speed;
    } else {
        speed = data.current_speed;
    }
    if (this->is_reverse) {
        return -speed;
    } else {
        return speed;
    }
}

float dm_motor_device::get_speed_without_external() {
    if (this->is_reverse) {
        return -data.current_speed;
    } else {
        return data.current_speed;
    }
}

float dm_motor_device::get_total_rounds() const {
    if (is_reverse) {
        return (float) -data.total_rounds;
    } else {
        return (float) data.total_rounds;
    }
}

float dm_motor_device::get_current_round() {
    if (is_reverse) {
        return (float) -data.current_round;
    } else {
        return (float) data.current_round;
    }
}

float dm_motor_device::get_total_rounds_without_offset() const {
    if (is_reverse) {
        return (float) -data.total_rounds_without_offset;
    } else {
        return (float) data.total_rounds_without_offset;
    }
}

void dm_motor_device::reset_total_rounds_zero_offset(float total_rounds) {
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

void dm_motor_device::set_reverse() {
    if (!this->is_reverse) {
        this->is_reverse = true;
        this->reset_total_rounds_zero_offset(this->get_total_rounds());
    }
}

void dm_motor_device::set_forward() {
    if (this->is_reverse) {
        this->is_reverse = false;
        this->reset_total_rounds_zero_offset(this->get_total_rounds());
    }
}

void dm_motor_device::set_toggle() {
    this->is_reverse = !is_reverse;
    this->reset_total_rounds_zero_offset(this->get_total_rounds());
}

void dm_motor_device::set_stall_parameter(float _stall_current_max, float _stall_speed_max) {
    stall_speed_max = _stall_speed_max;
    stall_torque_max = _stall_current_max;
}

void dm_motor_device::update_data(uint8_t *can_rx_data) {
    dm_motor_data_t *ptr = &(data);
    dm_motor_raw_data_t *raw = &raw_data;

    raw->id = can_rx_data[0] & 0xF;
    raw->err = can_rx_data[0] >> 4;
    //若id>0x0f,则解算err出错
    raw->pos_rad = (can_rx_data[1] << 8) | can_rx_data[2];
    raw->vel_rad_s = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
    raw->torque = ((can_rx_data[4] & 0xF) << 8) | can_rx_data[5];
    raw->t_mos = can_rx_data[6];
    raw->t_rotor = can_rx_data[7];

    if (msg_cnt < 50 && !is_zero_offset) {
        //round
        ptr->last_round = ptr->current_round;//电机编码反馈值
        ptr->current_round = (uint_to_float(raw->pos_rad, -basic_info.p_max, basic_info.p_max, 16) + basic_info.p_max)
                             * RAD2ROUND; // (-3.14,3.14) //电机编码反馈值
        ptr->zero_offset_round = ptr->current_round;

        //speed
        ptr->last_speed = ptr->current_speed;
        ptr->raw_current_speed =
            uint_to_float(raw->vel_rad_s, -basic_info.v_max, basic_info.v_max, 12) / basic_info.v_max; // (-30.0,30.0)
        ptr->current_speed = (1 - low_pass_alpha) * ptr->last_speed + low_pass_alpha * ptr->raw_current_speed;//自带滤波
        VAL_LIMIT(ptr->current_speed, -1.0f, 1.0f);
        msg_cnt++;

        return;

    }
    is_zero_offset = true;


    // 这里计圈好像有点问题 尤其是在第一次开的那一下 spill cnt 可能为1或者-1
    // round
    ptr->last_round = ptr->current_round;//电机编码反馈值
    ptr->current_round = (uint_to_float(raw->pos_rad, -basic_info.p_max, basic_info.p_max, 16) + basic_info.p_max)
                         * RAD2ROUND; // (-3.14,3.14) //电机编码反馈值
    if (ptr->current_round - ptr->last_round > basic_info.p_max * RAD2ROUND) {
        ptr->round_cnt = ptr->round_cnt - 1;
    } else if (ptr->current_round - ptr->last_round < -basic_info.p_max * RAD2ROUND) {
        ptr->round_cnt = ptr->round_cnt + 1;
    }
    ptr->total_rounds = (float) ptr->round_cnt * (2.0f * basic_info.p_max * RAD2ROUND) + ptr->current_round
                        - ptr->zero_offset_round;//为0
    ptr->total_rounds_without_offset =
        (float) ptr->round_cnt * (2.0f * basic_info.p_max * RAD2ROUND) + ptr->current_round;

    //speed
    ptr->last_speed = ptr->current_speed;
    ptr->raw_current_speed =
        uint_to_float(raw->vel_rad_s, -basic_info.v_max, basic_info.v_max, 12) / basic_info.v_max; // (-30.0,30.0)
    ptr->current_speed = (1 - low_pass_alpha) * ptr->last_speed + low_pass_alpha * ptr->raw_current_speed;//自带滤波
    VAL_LIMIT(ptr->current_speed, -1.0f, 1.0f);

    //temperature
    ptr->temperature = (int8_t) raw->t_rotor;


    //torque
    ptr->fb_torque_current = uint_to_float(raw->torque, -basic_info.t_max, basic_info.t_max, 12); // (-10.0,10.0);
    if (ABS(ptr->fb_torque_current) > ABS(stall_torque_max) && ABS(ptr->current_speed) < ABS(stall_speed_max)) {
        stall_cnt++;
    } else {
        stall_cnt = 0;
    }

}

void dm_motor_device::send_can_msg() {
    set_ctrl_to_can_tx_buff();
    this->fdcan_device.send_msg();
}

void dm_motor_device::change_speed_source(float *speed) {
    is_using_external_speed = true;
    this->data.external_speed = speed;
}

void dm_motor_device::set_low_pass_alpha(float alpha) {
    low_pass_alpha = alpha;
}

void dm_motor_device::set_vel(float set_vel) {
    if (mode == DM_MIT) {
        if (mit_mode == DM_MIT_Torque) {
            MIT_outer_set_motor_speed(set_vel);
        } else if (mit_mode == DM_MIT_Velocity) {
            MIT_outer_set_motor_speed(set_vel);
        }
    } else if (mode == DM_VO) {
        VO_inter_set_motor_speed(set_vel);
    }

}

void dm_motor_device::set_pos(float set_pos) {
    if (mode == DM_MIT) {
        if (mit_mode == DM_MIT_Torque) {
            MIT_outer_set_motor_total_rounds(set_pos);
        } else if (mit_mode == DM_MIT_Position) {
            MIT_inter_set_motor_round(set_pos);
        }
    } else if (mode == DM_PV) {
        PV_inter_set_motor_total_rounds(set_pos, 0.15f);//0.15 * 30 rad/s很快了
    }
}

bool dm_motor_device::check_lost() const {
    return this->lost_flag;
}

void dm_motor_device::check_motor_for_loss() {
    osStatus_t stat;
    stat = osSemaphoreAcquire(this->fdcan_device.rx_sem, 50);
    if (stat != osOK) {
        lost_num++;
        this->lost_flag = true;
        this->set_motor_enable();
    } else {
        if (this->lost_flag) {
            this->lost_flag = false;
            this->set_motor_enable();
//            this->reset_total_rounds_zero_offset(0);
        }
    }
}

bool dm_motor_device::check_reverse() const {
    return this->is_reverse;
}

bool dm_motor_device::check_stall() const {
    return this->stall_flag;
}

void dm_motor_device::update_ready() {
    if (!this->lost_flag && this->is_zero_offset) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }
}

bool dm_motor_device::check_ready() const {
    return this->ready_flag;
}

void dm_motor_device::set_free() {
    MIT_inter_set_motor_normalization_torque(0.0f);
}

void dm_motor_device::set_offset_current(float current) {
    offset_current = current;
}