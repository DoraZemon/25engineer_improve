//
// Created by 34147 on 2024/4/13.
//

#include "drv_hi229um.h"

const char mode_cmd_91[] = "AT+SETPTL=91\r\n";
const char mode_cmd_90[] = "AT+SETPTL=90,A0,B0,C0,D0,F0\r\n";

const char mode_cmd_6_axis[] = "AT+MODE=0\r\n";
const char mode_cmd_9_axis[] = "AT+MODE=1\r\n";

const char dir_cmd_00[] = "AT+URFR=1,0,0,0,1,0,0,0,1\r\n";//陀螺仪
const char dir_cmd_01[] = "AT+URFR=1,0,0,0,0,1,0,-1,0\r\n";//x 90
const char dir_cmd_02[] = "AT+URFR=1,0,0,0,0,-1,0,1,0\r\n";//x -90
const char dir_cmd_03[] = "AT+URFR=1,0,0,0,-1,0,0,0,-1\r\n";//x 180
const char dir_cmd_04[] = "AT+URFR=0,0,-1,0,1,0,1,0,0\r\n";//y 90
const char dir_cmd_05[] = "AT+URFR=0,0,1,0,1,0,-1,0,0\r\n";//y -90
const char dir_cmd_06[] = "AT+URFR=-1,0,0,0,1,0,0,0,-1\r\n";//y 180
const char dir_cmd_07[] = "AT+URFR=0,1,0,-1,0,0,0,0,1\r\n";//z 90
const char dir_cmd_08[] = "AT+URFR=0,-1,0,1,0,0,0,0,1\r\n";//z -90
const char dir_cmd_09[] = "AT+URFR=-1,0,0,0,-1,0,0,0,1\r\n";//z 180
const float g = 9.80665f;//重力加速度
const float deg2rad = (float) (PI / 180.0);
const float deg2round = (float) (1 / 360.0);
static uint16_t U2(uint8_t *p) {
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

hi229um_device::hi229um_device(UART_HandleTypeDef *huart)
    : enable_flag(true), zero_offset_flag(false), lost_flag(true), huart(huart), ready_flag(false) {}

void hi229um_device::init() {
    usart_send_buf(this->huart, (uint8_t *) &dir_cmd_07, strlen(dir_cmd_07));

}

#if HI229UM_91
void hi229um_device::update_data() {

    this->last_time = this->current_time;
    this->current_time = get_time_ms_us();
    this->delta_t = (float) (this->current_time - this->last_time) * 0.001f;

    taskENTER_CRITICAL();
    if (!this->zero_offset_flag && this->data.msg_cnt > 50) {
        this->data.msg_cnt = 0;
    }//保证直接用zero offset即可初始化

    this->data.msg_cnt++;
    if (this->data.msg_cnt > 50) {
        this->zero_offset_flag = true;
    } else {
        this->zero_offset_flag = false;
    }
    taskEXIT_CRITICAL();

    /* initial value */
    if (!this->zero_offset_flag) {
        this->delta_t = 0.0f;

        this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;
        this->data.euler.pitch.current_ang = (float) this->raw_data.euler_ang.pitch;
        this->data.euler.pitch.zero_offset_deg = this->data.euler.pitch.current_ang;//可以删掉，因为pitch以0为offset(六轴)
        this->data.euler.pitch.round_cnt = 0;

        this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
        this->data.euler.roll.current_ang = (float) this->raw_data.euler_ang.roll;
        this->data.euler.roll.zero_offset_deg = this->data.euler.roll.current_ang;//可以删掉，因为roll以0为offset(六轴)
        this->data.euler.roll.round_cnt = 0;

        this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.current_ang = (float) this->raw_data.euler_ang.yaw;
        this->data.euler.yaw.zero_offset_deg = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.round_cnt = 0;

        return;
    }

    this->update_euler();

    this->data.ang_v.x = (float) this->raw_data.ang_v.x * 0.1f;
    this->data.ang_v.y = (float) this->raw_data.ang_v.y * 0.1f;
    this->data.ang_v.z = (float) this->raw_data.ang_v.z * 0.1f;

    this->data.WCS_acc.x = (float) this->raw_data.acc.x * 0.001f * g;
    this->data.WCS_acc.y = (float) this->raw_data.acc.y * 0.001f * g;
    this->data.WCS_acc.z = (float) this->raw_data.acc.z * 0.001f * g;

    this->data.WCS_vel.x += this->data.WCS_acc.x * this->delta_t;
    this->data.WCS_vel.y += this->data.WCS_acc.y * this->delta_t;
    this->data.WCS_vel.z += this->data.WCS_acc.z * this->delta_t;

    this->data.WCS_pos.x += this->data.WCS_vel.x * this->delta_t;
    this->data.WCS_pos.y += this->data.WCS_vel.y * this->delta_t;
    this->data.WCS_pos.z += this->data.WCS_vel.z * this->delta_t;
}

void hi229um_device::update_euler() {
    this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
    this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
    this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;

    this->data.euler.yaw.current_ang = (float) this->raw_data.euler_ang.yaw;
    this->data.euler.pitch.current_ang = (float) this->raw_data.euler_ang.pitch;
    this->data.euler.roll.current_ang = (float) this->raw_data.euler_ang.roll;

    if (this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang < -HI229UM_YAW_RANGE) {
        this->data.euler.yaw.round_cnt++;
    } else if (this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang > HI229UM_YAW_RANGE) {
        this->data.euler.yaw.round_cnt--;
    }

    this->data.euler.yaw.total_rounds =
        (float(this->data.euler.yaw.round_cnt) * HI229UM_YAW_RANGE * 2 + this->data.euler.yaw.current_ang
            - this->data.euler.yaw.zero_offset_deg) * deg2round;

    if (this->data.euler.roll.current_ang - this->data.euler.roll.last_ang < -HI229UM_ROLL_RANGE) {
        this->data.euler.roll.round_cnt++;
    } else if (this->data.euler.roll.current_ang - this->data.euler.roll.last_ang > HI229UM_ROLL_RANGE) {
        this->data.euler.roll.round_cnt--;
    }

    this->data.euler.roll.total_rounds =
        (float(this->data.euler.roll.round_cnt) * HI229UM_ROLL_RANGE * 2 + this->data.euler.roll.current_ang
            - this->data.euler.roll.zero_offset_deg) * deg2round;

    if (this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang < -HI229UM_PITCH_RANGE) {
        this->data.euler.pitch.round_cnt++;
    } else if (this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang > HI229UM_PITCH_RANGE) {
        this->data.euler.pitch.round_cnt--;
    }

    this->data.euler.pitch.total_rounds =
        (float(this->data.euler.pitch.round_cnt) * HI229UM_PITCH_RANGE * 2 + this->data.euler.pitch.current_ang
            - this->data.euler.pitch.zero_offset_deg) * deg2round;
}
#else
void hi229um_device::update_data() {

    this->last_time = this->current_time;
    this->current_time = get_time_ms_us();
    this->delta_t = (float) (this->current_time - this->last_time) * 0.001f;

    taskENTER_CRITICAL();
    if(!this->zero_offset_flag && this->data.msg_cnt > 50){
        this->data.msg_cnt = 0;
    }//保证直接用zero offset即可初始化

    this->data.msg_cnt++;
    if (this->data.msg_cnt > 50) {
        this->zero_offset_flag = true;
    }
    else {
        this->zero_offset_flag = false;
    }
    taskEXIT_CRITICAL();

    /* initial value */
    if (!this->zero_offset_flag) {
        this->delta_t = 0.0f;

        this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;
        this->data.euler.pitch.current_ang = (float) this->raw_data.euler_ang.pitch * 0.01f;
        this->data.euler.pitch.zero_offset_deg = this->data.euler.pitch.current_ang;//可以删掉，因为pitch以0为offset(六轴)
        this->data.euler.pitch.round_cnt = 0;

        this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
        this->data.euler.roll.current_ang = (float) this->raw_data.euler_ang.roll * 0.01f;
        this->data.euler.roll.zero_offset_deg = this->data.euler.roll.current_ang;//可以删掉，因为roll以0为offset(六轴)
        this->data.euler.roll.round_cnt = 0;

        this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.current_ang = (float) this->raw_data.euler_ang.yaw * 0.1f;
        this->data.euler.yaw.zero_offset_deg = this->data.euler.yaw.current_ang;
        this->data.euler.yaw.round_cnt = 0;

        return;
    }

    this->update_euler();

    this->data.ang_v.x = (float) this->raw_data.ang_v.x * 0.1f;
    this->data.ang_v.y = (float) this->raw_data.ang_v.y * 0.1f;
    this->data.ang_v.z = (float) this->raw_data.ang_v.z * 0.1f;

    this->data.WCS_acc.x = (float) this->raw_data.acc.x * 0.001f * g;
    this->data.WCS_acc.y = (float) this->raw_data.acc.y * 0.001f * g;
    this->data.WCS_acc.z = (float) this->raw_data.acc.z * 0.001f * g;

    this->data.WCS_vel.x += this->data.WCS_acc.x * this->delta_t;
    this->data.WCS_vel.y += this->data.WCS_acc.y * this->delta_t;
    this->data.WCS_vel.z += this->data.WCS_acc.z * this->delta_t;

    this->data.WCS_pos.x += this->data.WCS_vel.x * this->delta_t;
    this->data.WCS_pos.y += this->data.WCS_vel.y * this->delta_t;
    this->data.WCS_pos.z += this->data.WCS_vel.z * this->delta_t;
}

void hi229um_device::update_euler() {
    this->data.euler.yaw.last_ang = this->data.euler.yaw.current_ang;
    this->data.euler.roll.last_ang = this->data.euler.roll.current_ang;
    this->data.euler.pitch.last_ang = this->data.euler.pitch.current_ang;

    this->data.euler.yaw.current_ang = (float)this->raw_data.euler_ang.yaw / 10.0f;
    this->data.euler.pitch.current_ang = (float)this->raw_data.euler_ang.pitch / 100.0f;
    this->data.euler.roll.current_ang = (float)this->raw_data.euler_ang.roll / 100.0f;

    if(this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang < - HI229UM_YAW_RANGE){
        this->data.euler.yaw.round_cnt++;
    }else if(this->data.euler.yaw.current_ang - this->data.euler.yaw.last_ang > HI229UM_YAW_RANGE){
        this->data.euler.yaw.round_cnt--;
    }

    this->data.euler.yaw.total_rounds = (float(this->data.euler.yaw.round_cnt) * HI229UM_YAW_RANGE *2 + this->data.euler.yaw.current_ang - this->data.euler.yaw.zero_offset_deg) * deg2round;

    if(this->data.euler.roll.current_ang - this->data.euler.roll.last_ang < - HI229UM_ROLL_RANGE){
        this->data.euler.roll.round_cnt++;
    }else if(this->data.euler.roll.current_ang - this->data.euler.roll.last_ang > HI229UM_ROLL_RANGE){
        this->data.euler.roll.round_cnt--;
    }

    this->data.euler.roll.total_rounds = ( float(this->data.euler.roll.round_cnt) * HI229UM_ROLL_RANGE *2 + this->data.euler.roll.current_ang - this->data.euler.roll.zero_offset_deg) * deg2round;

    if(this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang < - HI229UM_PITCH_RANGE){
        this->data.euler.pitch.round_cnt++;
    }else if(this->data.euler.pitch.current_ang - this->data.euler.pitch.last_ang > HI229UM_PITCH_RANGE){
        this->data.euler.pitch.round_cnt--;
    }

    this->data.euler.pitch.total_rounds = ( float(this->data.euler.pitch.round_cnt) * HI229UM_PITCH_RANGE *2 + this->data.euler.pitch.current_ang - this->data.euler.pitch.zero_offset_deg) * deg2round;
}
#endif
void hi229um_device::set_current_as_offset() {
    this->data.msg_cnt = 0;
    this->zero_offset_flag = false;
}

void hi229um_device::update_ready() {
    if (!this->lost_flag && this->zero_offset_flag) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }
}

void hi229um_device::receive_dma() {
    usart_start_receive_dma(this->huart, this->raw_data.buf, DRV_HI229UM_BUF_SIZE);
}

bool hi229um_device::check_legal() {
    if (this->raw_data.frame_head.head[0] == CHSYNC1 && this->raw_data.frame_head.head[1] == CHSYNC2
        && this->check_crc_passing(&this->raw_data))
        return true;
    else
        return false;
}

bool hi229um_device::check_crc_passing(hi229um_raw *raw) {
    uint16_t crc = 0;

    /* checksum */
    crc16_update(&crc, raw->buf, 4);
    crc16_update(&crc, raw->buf + 6, raw->frame_head.len);
    if (crc != U2(raw->buf + 4)) {
        return false;
    }

    return true;
}

void hi229um_device::enable() {
    this->enable_flag = true;
}

void hi229um_device::disable() {
    this->enable_flag = false;
}

bool hi229um_device::check_enable() {
    return enable_flag;
}

void hi229um_device::set_lost() {
//    this->data.msg_cnt = 0;
//    this->zero_offset_flag = false;// 丢失之后不重置陀螺仪，因为上位机会发生突变
    this->lost_flag = true;
}

void hi229um_device::set_connect() {
    this->lost_flag = false;
}

bool hi229um_device::check_lost() {
    return this->lost_flag;
}

float hi229um_device::get_yaw_total_deg() {
    return this->data.euler.yaw.total_rounds;
}

bool hi229um_device::check_zero_offset() {
    return this->zero_offset_flag;
}

void hi229um_device::set_nine_axis_mode() {
    for(int i =0;i<5;i++){
        usart_send_buf_dma(this->huart,(uint8_t*)mode_cmd_9_axis,sizeof(mode_cmd_9_axis));
        osDelay(5);
    }
}