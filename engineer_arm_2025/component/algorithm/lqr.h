/**
  ******************************************************************************
  * @file           : lqr.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef LQR_H_
#define LQR_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"

class SimpleLQR {
 private:


  float max_out;  // 最大输出限制（根据执行器能力设定）
  float i_max;
  float i_max_ratio; // 积分限幅（防止积分饱和）

  // 状态变量
  float prev_target = 0;
  float target_vel_est = 0;
  float integral = 0;
  float out = 0;

  float d_term = 0;
  float ff_term = 0;
  float p_term = 0;
  float i_term = 0;
  float sigmoid_gain = 0;
  float error = 0;

  float set = 0;
  float get = 0;

 public:
    // 可调参数（通过注释说明每个参数的影响）
  float Kp;       // 位置误差增益：越大回正越快，但可能引发超调
  float Kd;       // 速度阻尼增益：增大可抑制振荡，建议Kd ≈ 0.3*Kp开始调试
  float Ki;       // 积分增益：消除稳态误差，从0.1*Kp开始尝试
  float Kff;      // 前馈增益：提升跟踪响应，建议0.5~1.0之间
  SimpleLQR() = default;


  void reset_lqr(float kp, float kd, float ki, float kff, float i_max_ratio_, float max_out_) {
      // 设置参数
      Kp = kp;
      Kd = kd;
      Ki = ki;
      Kff = kff;

      max_out = max_out_;
      i_max_ratio = i_max_ratio_;
      i_max = i_max_ratio * max_out / fmaxf(Ki, 1e-6f);

      // 状态清零
      prev_target = 0.0f;
      target_vel_est = 0.0f;
      integral = 0.0f;
      out = 0.0f;
  }
  // 重置控制器状态（当发生异常时调用）


  // 核心控制函数
  float calculate(float current_pos, float current_vel,
                  float target_pos, float dt) {
      // 更新限制
      i_max = i_max_ratio * max_out / fmaxf(Ki, 1e-6f);
      set = target_pos;
      get = current_pos;
      // 目标速度估计（带低通滤波）
      float raw_target_vel = (target_pos - prev_target) / fmaxf(dt, 1e-3f);
      target_vel_est = 0.7f * target_vel_est + 0.3f * raw_target_vel;

      // 误差计算
      error = target_pos - current_pos;

      // 智能积分（仅在小误差时累加）
      if (fabsf(error) < max_out / fmaxf(Kp, 1e-3f)) {
//        integral += error * dt;
          integral += error;
          integral = fminf(fmaxf(integral, -i_max), i_max);  // 限幅
      } else {
          integral *= 0.9f;  // 抑制积分增长
      }

      // 微分项（基于速度差，适合云台）
//     d_term = Kd * (target_vel_est - current_vel);
      d_term = Kd * (-current_vel);

      // 前馈项（基于目标速度）
      ff_term = Kff * target_vel_est;

      // 动态增益调整（误差小时提升响应）
      sigmoid_gain = 1.5f - 0.5f / (1.0f + expf(-20.0f * fabsf(error)));

//     p_term = sigmoid_gain * Kp * error;

      p_term = Kp * error;

      i_term = Ki * integral;

      // 控制量合成
      float u = p_term + ff_term + d_term + i_term;

      u = fminf(fmaxf(u, -max_out), max_out);

      // 状态更新
      prev_target = target_pos;
      out = u;
      return out;
  }


};

#endif //LQR_H_
