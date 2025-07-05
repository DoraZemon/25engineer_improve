//
// Created by 34147 on 2023/11/23.
//

#include "spd_plan.h"

/*--------------------------斜坡函数------------------------*/
/**
 * @brief 初始化斜坡函数的相关参数
 * @param slope_plan
 * @param out
 * @param acc
 * @param dec
 * @param out_max
 * @param target
 */
void slope_speed_init(slope_speed_t *slope_plan, float out, float acc, float dec, float out_max, float target) {
    slope_plan->out = out;
    slope_plan->acc = acc;
    slope_plan->dec = dec;
    slope_plan->out_max = out_max;
    slope_plan->target = target;
}

/**
 * @brief 斜坡函数的计算
 * @param slope_plan
 * @param target_speed
 * @return
 */
float get_slope_speed(slope_speed_t *slope_plan) {
    float target_speed = slope_plan->target;
    ABS_LIMIT(target_speed, slope_plan->out_max);
    //判断加减速和速度方向
    bool is_acc = true;
    int dir; // 1向前，-1向后

    if (slope_plan->out > 0) {
        dir = 1;
        if (target_speed < slope_plan->out) {
            is_acc = false;
        }
    } else {
        dir = -1;
        if (target_speed > slope_plan->out) {
            is_acc = false;
        }
    }

    float speed_err = fabsf(target_speed - slope_plan->out);

    float acc;

    if (is_acc) {
        acc = slope_plan->acc;
    } else {
        acc = -slope_plan->dec;
    }

    if (speed_err < fabsf(acc)) {
        slope_plan->out = target_speed;
    } else {
        slope_plan->out = fabsf(slope_plan->out) + acc;        //不用计算出总步长，以增量式慢慢推进，走一步看一步
        slope_plan->out = slope_plan->out * (float) dir;
    }
    return slope_plan->out;
}

void update_slope_spd(slope_speed_t *slope, float acc, float dec, float out_max) {
    slope->acc = acc;
    slope->dec = dec;
    slope->out_max = out_max;
}


/*-------------------------------sigmoid函数-----------------------*/
/**
 * @brief 初始化sigmoid函数
 * @param sigmoid_plan
 * @param len
 * @param flexible
 * @param cnt
 */
void sigmoid_speed_init(sigmoid_speed_t *sigmoid_plan, uint16_t len, uint8_t flexible, uint32_t cnt) {
    sigmoid_plan->start_speed = 0;
    sigmoid_plan->end_speed = 0;
    sigmoid_plan->len = len;
    sigmoid_plan->flexible = flexible;
    sigmoid_plan->cnt = cnt;
}

/**
 * @brief sigmoid函数的计算
 * @param sigmoid_plan
 * @param total_time_ms
 * @return
 */
float get_sigmoid_speed(sigmoid_speed_t *sigmoid_plan, uint32_t total_time_ms) {
    float ret_speed = 0.0f;
    uint16_t num = sigmoid_plan->len / 2;

    sigmoid_plan->current_time = HAL_GetTick();
    if (sigmoid_plan->current_time - sigmoid_plan->last_time >= total_time_ms / (uint32_t) sigmoid_plan->len) {
        sigmoid_plan->last_time = sigmoid_plan->current_time;
        sigmoid_plan->cnt++;
        if (sigmoid_plan->cnt > 30000) {
            sigmoid_plan->cnt = 30000;
        }
    }

    ret_speed = sigmoid_plan->start_speed + (sigmoid_plan->end_speed - sigmoid_plan->start_speed) /
                                            (1 + expf(-((float) sigmoid_plan->flexible / (float) num) *
                                                      (float) sigmoid_plan->cnt
                                                      + (float) sigmoid_plan->flexible));

    if (ABS(ret_speed - sigmoid_plan->end_speed) < 0.00001f) {
        ret_speed = sigmoid_plan->end_speed;
    }
    return ret_speed;
}