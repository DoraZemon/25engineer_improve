/**
  ******************************************************************************
  * @file           : smc.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/5/1
  ******************************************************************************
  */


#ifndef AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_SMC_H_
#define AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_SMC_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

#include "user_lib.h"

class SimpleSMC {
 private:
  // 可调参数（通过注释说明调参方向）
  float lambda;    // 滑模面系数：增大可加快误差收敛，但可能引发振荡
  float K;         // 切换增益：增大增强抗扰性，但会增加抖振
  float phi;       // 边界层厚度：增大可减少抖振，但会降低跟踪精度

  // 内部状态
  float prev_error = 0;
  float filtered_deriv = 0;

  float target = 0.0;
  float current = 0.0;

  float max_out = 0.0;

  float out = 0.0;

 public:
  SimpleSMC() = default;

  // 实时调参接口


  void reset_smc(float max_out, float l, float k, float p);

  // 控制量计算（需目标速度、当前速度、控制周期dt）
  float calculate(float target, float current, float dt);

 private:
  // 饱和函数（替代sign函数减少抖振）
  float sat(float x);
};


#endif //AERIAL_GIMBAL_2025_COMPONENT_ALGORITHM_SMC_H_
