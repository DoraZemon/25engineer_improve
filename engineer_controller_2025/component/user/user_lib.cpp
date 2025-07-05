//
// Created by 34147 on 2023/11/13.
//

#include "user_lib.h"

/**
 * @brief 将输入变量x的值限制在min和max之间，改变其原值
 * @param x
 * @param min
 * @param max
 */
void val_limit(float *x, float min, float max) {
    if (*x < min) {
        *x = min;
    }
    if (*x > max) {
        *x = max;
    }
}

/**
 * @brief 限制输入变量x的绝对值小于abs_max
 * @param x
 * @param abs_max
 */
void abs_limit(float *x, float abs_max) {
    if (*x > abs_max) {
        *x = abs_max;
    }
    if (*x < -abs_max) {
        *x = -abs_max;
    }
}

//Hal库时钟自己定为1000
uint32_t get_time_us() {
    return TIM7->CNT;
}

uint32_t get_time_ms(void) {
    return HAL_GetTick();
}

float get_time_ms_us() {
    return ((float) get_time_ms() + (float) get_time_us() / 1000.0f);
}

uint16_t unsigned_16(uint8_t *p) {
    uint16_t u;
    memcpy(&u, p, 2);
    return u;
}

/**
 * @brief 返回限制在一个区间的值
 * @param val
 * @param min
 * @param max
 * @return
 */
float limited_val(float val, float min, float max) {
    VAL_LIMIT(val, min, max);
    return val;
}


void low_pass(float &target, float current, float alpha) {
    float tmp = current - target;
    target = target + alpha * tmp;
}
