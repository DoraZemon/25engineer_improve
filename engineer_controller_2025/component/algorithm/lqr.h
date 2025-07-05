/**
  ******************************************************************************
  * @file           : lqr.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/5/1
  ******************************************************************************
  */


#ifndef AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_LQR_H_
#define AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_LQR_H_
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
  // 可调参数（通过注释说明每个参数的影响）
  float Kp;       // 位置误差增益：越大回正越快，但可能引发超调
  float Kd;       // 速度阻尼增益：增大可抑制振荡，建议Kd ≈ 0.3*Kp开始调试
  float Ki;       // 积分增益：消除稳态误差，从0.1*Kp开始尝试
  float Kff;      // 前馈增益：提升跟踪响应，建议0.5~1.0之间

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
  SimpleLQR() = default;


  void reset_lqr(float kp, float kd, float ki, float kff, float i_max_ratio_, float max_out_);
  // 重置控制器状态（当发生异常时调用）


  // 核心控制函数
  float calculate(float current_pos, float current_vel,
                  float target_pos, float dt);


};


#endif //AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_LQR_H_
