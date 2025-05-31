/**
  ******************************************************************************
  * @file           : user_lib.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


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

uint32_t get_time_ms(void)
{
    return HAL_GetTick();
}

float get_time_ms_us() {
    return ((float)get_time_ms() + (float)get_time_us()/1000.0f);
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


void low_pass(float &target,float current,float alpha){
    float tmp = current - target;
    target = target + alpha * tmp;
}


int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}