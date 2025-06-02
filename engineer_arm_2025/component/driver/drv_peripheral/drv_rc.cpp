/**
  ******************************************************************************
  * @file           : drv_rc.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "drv_rc.h"

rc_device::rc_device(UART_HandleTypeDef *huart)
    : lost_flag(true), huart(huart), using_kb_flag(false) {}

void rc_device::update_data() {
    taskENTER_CRITICAL();
    memcpy(&this->last_data, &this->data, sizeof(rc_data_t));

    if (this->raw_data.ch0 < 1044 && this->raw_data.ch0 > 1004) this->raw_data.ch0 = 1024;
    if (this->raw_data.ch1 < 1044 && this->raw_data.ch1 > 1004) this->raw_data.ch1 = 1024;
    if (this->raw_data.ch2 < 1044 && this->raw_data.ch2 > 1004) this->raw_data.ch2 = 1024;
    if (this->raw_data.ch3 < 1044 && this->raw_data.ch3 > 1004) this->raw_data.ch3 = 1024;

    if (this->raw_data.ch0 > 1684 || this->raw_data.ch0 < 364) {
        taskEXIT_CRITICAL();
        return;
    }

    if (this->raw_data.ch1 > 1684 || this->raw_data.ch1 < 364) {
        taskEXIT_CRITICAL();
        return;
    }

    if (this->raw_data.ch2 > 1684 || this->raw_data.ch2 < 364) {
        taskEXIT_CRITICAL();
        return;
    }

    if (this->raw_data.ch3 > 1684 || this->raw_data.ch3 < 364) {
        taskEXIT_CRITICAL();
        return;
    }

    this->data.right_rocker.x = (float(this->raw_data.ch0) - 1024.0f) / 660.0f;
    ABS_LIMIT(this->data.right_rocker.x, 1);
    this->data.right_rocker.y = (float(this->raw_data.ch1) - 1024.0f) / 660.0f;
    ABS_LIMIT(this->data.right_rocker.y, 1);
    this->data.left_rocker.x = (float(this->raw_data.ch2) - 1024.0f) / 660.0f;
    ABS_LIMIT(this->data.left_rocker.x, 1);
    this->data.left_rocker.y = (float(this->raw_data.ch3) - 1024.0f) / 660.0f;
    ABS_LIMIT(this->data.left_rocker.y, 1);

    this->data.left_sw = (enum RC_SW) this->raw_data.s2;
    this->data.right_sw = (enum RC_SW) this->raw_data.s1;

    this->data.mouse.x = float(this->raw_data.mouse_x) / 500.0f;
    ABS_LIMIT(this->data.mouse.x, 1);
    this->data.mouse.y = float(this->raw_data.mouse_y) / 500.0f;
    ABS_LIMIT(this->data.mouse.y, 1);
    this->data.mouse.z = float(this->raw_data.mouse_z) / 20.0f;
    ABS_LIMIT(this->data.mouse.z, 1);

    this->data.mouse.right_button = (enum RC_BUTTON) this->raw_data.right_click;
    this->data.mouse.left_button = (enum RC_BUTTON) this->raw_data.left_click;

    this->data.kb.key_code = this->raw_data.keys;

    if (this->raw_data.mouse_x == 0 && this->raw_data.mouse_y == 0 && this->raw_data.mouse_z == 0 &&
        this->raw_data.right_click == 0 && this->raw_data.left_click == 0 && this->raw_data.keys == 0) {
        this->using_kb_flag = false;
    } else {
        this->using_kb_flag = true;
    }

    this->data.wheel = -(float(this->raw_data.wheel) - 1024.0f) / 660.0f;
    taskEXIT_CRITICAL();
}

void rc_device::update_event() {
    if (lost_flag) {
        return;
    }
    taskENTER_CRITICAL();

    this->event.keys_down_act = (~this->last_data.kb.key_code) & (this->data.kb.key_code);
    this->event.keys_up_act = (this->last_data.kb.key_code) & (~this->data.kb.key_code);

    this->event.left_button_down_act = (!this->last_data.mouse.left_button) && (this->data.mouse.left_button);
    this->event.left_button_up_act = (this->last_data.mouse.left_button) && (!this->data.mouse.left_button);
    this->event.right_button_down_act = (!this->last_data.mouse.right_button) && (this->data.mouse.right_button);
    this->event.right_button_up_act = (this->last_data.mouse.right_button) && (!this->data.mouse.right_button);

    if (this->data.right_sw == RC_SW_UP && this->last_data.right_sw == RC_SW_MID) {
        this->event.sw_act &= ~RC_SW_R_AREA;
        this->event.sw_act |= RC_SW_R_MID2UP;
    } else if (this->data.right_sw == RC_SW_MID && this->last_data.right_sw == RC_SW_UP) {
        this->event.sw_act &= ~RC_SW_R_AREA;
        this->event.sw_act |= RC_SW_R_UP2MID;
    } else if (this->data.right_sw == RC_SW_MID && this->last_data.right_sw == RC_SW_DOWN) {
        this->event.sw_act &= ~RC_SW_R_AREA;
        this->event.sw_act |= RC_SW_R_DOWN2MID;
    } else if (this->data.right_sw == RC_SW_DOWN && this->last_data.right_sw == RC_SW_MID) {
        this->event.sw_act &= ~RC_SW_R_AREA;
        this->event.sw_act |= RC_SW_R_MID2DOWN;
    }

    if (this->data.left_sw == RC_SW_UP && this->last_data.left_sw == RC_SW_MID) {
        this->event.sw_act &= ~RC_SW_L_AREA;
        this->event.sw_act |= RC_SW_L_MID2UP;
    } else if (this->data.left_sw == RC_SW_MID && this->last_data.left_sw == RC_SW_UP) {
        this->event.sw_act &= ~RC_SW_L_AREA;
        this->event.sw_act |= RC_SW_L_UP2MID;
    } else if (this->data.left_sw == RC_SW_MID && this->last_data.left_sw == RC_SW_DOWN) {
        this->event.sw_act &= ~RC_SW_L_AREA;
        this->event.sw_act |= RC_SW_L_DOWN2MID;
    } else if (this->data.left_sw == RC_SW_DOWN && this->last_data.left_sw == RC_SW_MID) {
        this->event.sw_act &= ~RC_SW_L_AREA;
        this->event.sw_act |= RC_SW_L_MID2DOWN;
    }
    taskEXIT_CRITICAL();
}

void rc_device::update_ready() {
    if (!this->lost_flag) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }
}

bool rc_device::check_key_down_state(enum keyMap key) {
    switch (key) {
        case keyA:return this->data.kb.key_bit_state.A;
            break;
        case keyB:return this->data.kb.key_bit_state.B;
            break;
        case keyC:return this->data.kb.key_bit_state.C;
            break;
        case keyCTRL:return this->data.kb.key_bit_state.CTRL;
            break;
        case keyD:return this->data.kb.key_bit_state.D;
            break;
        case keyE:return this->data.kb.key_bit_state.E;
            break;
        case keyF:return this->data.kb.key_bit_state.F;
            break;
        case keyG:return this->data.kb.key_bit_state.G;
            break;
        case keyQ:return this->data.kb.key_bit_state.Q;
            break;
        case keyR:return this->data.kb.key_bit_state.R;
            break;
        case keyS:return this->data.kb.key_bit_state.S;
            break;
        case keySHIFT:return this->data.kb.key_bit_state.SHIFT;
            break;
        case keyV:return this->data.kb.key_bit_state.V;
            break;
        case keyW:return this->data.kb.key_bit_state.W;
            break;
        case keyX:return this->data.kb.key_bit_state.X;
            break;
        case keyZ:return this->data.kb.key_bit_state.Z;
            break;
        default:return false;
    }
}

bool rc_device::check_key_up_state(enum keyMap key) {
    return !this->check_key_down_state(key);
}

bool rc_device::check_key_down_event(enum keyMap key) {
    uint16_t key_state = (1 << key);
    taskENTER_CRITICAL();
    if ((this->event.keys_down_act & key_state) == key_state) {
        this->event.keys_down_act &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_key_up_event(enum keyMap key) {
    uint16_t key_state = (1 << key);
    taskENTER_CRITICAL();
    if ((this->event.keys_up_act & key_state) == key_state) {
        this->event.keys_up_act &= (~key_state);
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_mouse_left_click_down_event() {
    taskENTER_CRITICAL();
    if (this->event.left_button_down_act) {
        this->event.left_button_down_act = 0;
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_mouse_left_click_up_event() {
    taskENTER_CRITICAL();
    if (this->event.left_button_up_act) {
        this->event.left_button_up_act = 0;
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_mouse_right_click_down_event() {
    taskENTER_CRITICAL();
    if (this->event.right_button_down_act) {
        this->event.right_button_down_act = 0;
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_mouse_right_click_up_evnet() {
    taskENTER_CRITICAL();
    if (this->event.right_button_up_act) {
        this->event.right_button_up_act = 0;
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_sw_state(enum RC_SW_STATE sw_state) {
    if (sw_state <= 3) {
        if (sw_state - RC_SW_L_UP == this->data.left_sw - RC_SW_UP)
            return true;
    } else {
        if (sw_state - RC_SW_R_UP == this->data.right_sw - RC_SW_UP)
            return true;
    }
    return false;
}

bool rc_device::check_sw_event(uint16_t sw_event) {
    taskENTER_CRITICAL();
    if ((this->event.sw_act & sw_event) == sw_event) {
        this->event.sw_act &= (~(sw_event & 0x00FF));
        taskEXIT_CRITICAL();
        return true;
    } else {
        taskEXIT_CRITICAL();
        return false;
    }
}

bool rc_device::check_wheel_state(enum RC_WHEEL_STATE wheel_state) {
    if (this->data.wheel > 0.85f && wheel_state == RC_WHEEL_UP) {
        return true;
    } else if (this->data.wheel < -0.85f && wheel_state == RC_WHEEL_DOWN) {
        return true;
    } else {
        return false;
    }
}

bool rc_device::check_ready() {
    return this->ready_flag;
}

void rc_device::set_lost() {
    this->lost_flag = true;
}

void rc_device::set_connect() {
    this->lost_flag = false;
}

bool rc_device::check_lost() const {
    return this->lost_flag;
}

float rc_device::get_mouse_x() const {
    return this->data.mouse.x;
}

float rc_device::get_mouse_y() const {
    return this->data.mouse.y;
}

float rc_device::get_mouse_z() const {
    return this->data.mouse.z;
}

float rc_device::get_right_rocker_x() const {
    return this->data.right_rocker.x;
}

float rc_device::get_right_rocker_y() const {
    return this->data.right_rocker.y;
}

float rc_device::get_left_rocker_x() const {
    return this->data.left_rocker.x;
}

float rc_device::get_left_rocker_y() const {
    return this->data.left_rocker.y;
}

bool rc_device::check_mouse_left_click_state() const {
    return this->data.mouse.left_button;
}

bool rc_device::check_mouse_right_click_state() const {
    return this->data.mouse.right_button;
}

