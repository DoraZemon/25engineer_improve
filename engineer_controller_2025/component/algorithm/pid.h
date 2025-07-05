//
// Created by 34147 on 2024/1/25.
//

#ifndef ENGINEER_CHASSIS_2024_PID_H
#define ENGINEER_CHASSIS_2024_PID_H
#ifdef __cplusplus
extern "C" {
#endif
//C

#include "math.h"
#include "user_lib.h"
#ifdef __cplusplus
}
#endif
//C++





struct pid_param {
  float p;
  float i;
  float d;
  float ap;
  float bp;
  float cp;
  float input_max_err;//不用
  float max_out;
  float integral_higher_limit; //积分限幅 正
  float integral_lower_limit;  //积分限幅 负
};

class pid {

 public:
  pid(float max_out_ = 1.0,
      float integral_limit = 0.1,

      float kp = 5.0,
      float ki = 0.001,
      float kd = 0.5,
      float ap = 0,
      float bp = 0,
      float cp = 0);

  float pid_calculate(float set_, float get_);

  float pid_calculate_for_iout(float set_, float get_);

  void pid_reset(float max_out, float integral_limit, float kp, float ki, float kd, float ap, float bp, float cp);

  void pid_clear();

 protected:

  bool enable;

  float set;
  float get;

  float err;
  float last_err;
  float penultimate_err;

  float pout;
  float iout;
  float dout;

 public:
  pid_param param;
  float out;

};

#endif //ENGINEER_CHASSIS_2024_PID_H
