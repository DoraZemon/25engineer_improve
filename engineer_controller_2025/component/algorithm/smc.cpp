/**
  ******************************************************************************
  * @file           : smc.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/5/1
  ******************************************************************************
  */


#include "smc.h"


void SimpleSMC::reset_smc(float max_out, float l, float k, float p) {
    lambda = l;
    K = k;
    phi = p;
    this->max_out = max_out;
//        prev_error = 0;
//        filtered_deriv = 0;
}

float SimpleSMC::sat(float x) {

    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;

}


float SimpleSMC::calculate(float target, float current, float dt) {

    this->target = target;
    this->current = current;
    // 计算误差及导数（含简单滤波）
    float error = target - current;
    float raw_deriv = (error - prev_error) / dt;
    prev_error = error;

    // 对导数低通滤波（截止频率约50Hz）
    float alpha = 2.f * PI * 50.f * dt;
    filtered_deriv = alpha * raw_deriv + (1 - alpha) * filtered_deriv;

    // 计算滑模面
    float s = filtered_deriv + lambda * error;

    // 等效控制简化（假设无模型知识）
    float u_eq = 0.0f;  // 实际可保留部分经验项

    // 切换控制（带边界层的饱和函数）
    float u_sw = K * sat(s / phi);

    out = u_eq + u_sw;
    // 限幅
    ABS_LIMIT(out, max_out);
    // 总控制量
    return out;
}