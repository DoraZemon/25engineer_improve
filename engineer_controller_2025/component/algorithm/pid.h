/**
  ******************************************************************************
  * @file           : pid.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef PID_H_
#define PID_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "math.h"
#include "user_lib.h"


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
      float cp = 0) : param{kp, ki, kd, ap, bp, cp, 0, max_out_, integral_limit, -integral_limit},//结构体初始化格式
                      enable(true),
                      set(0), get(0),
                      err(0), last_err(0), penultimate_err(0),
                      pout(0), iout(0), dout(0), out(0) {}

  float pid_calculate(float set_, float get_) {
      get = get_;
      set = set_;
      err = set - get;
      // if (fabsf(param.input_max_err) > 0.0)
      //     abs_limit(&err, param.input_max_err);
//    v->Kp = v->Ap + v->Bp * (1- exp(-v->Cp * fabs(v->Err)));
      if (fabsf(param.ap) >= 0.001 && fabsf(param.cp) >= 0)
          param.p = param.ap + param.bp * (1 - exp(-param.cp * fabs(err)));
      pout = param.p * err;
      iout += param.i * err;
//    pid->iout += pid->param.i * abs_zero(pid->err,0.1);
      dout = param.d * (err - last_err);

//    abs_limit(&(pid->iout), pid->param.integral_limit);
      val_limit(&iout, param.integral_lower_limit, param.integral_higher_limit);
      out = pout + iout + dout;
      abs_limit(&out, param.max_out);
      last_err = err;

      if (enable == 0) {
          out = 0;
      }
      return (out);
  }

  float pid_calculate_for_iout(float set_, float get_) {
      get = get_;
      set = set_;
      err = set - get;
      if (fabsf(err) > 0.07)//为了ioutput只给极小范围的振动使用
          err = 0;
      pout = param.p * err;
      iout += param.i * err;
      //    pid->iout += pid->param.i * abs_zero(pid->err,0.1);
      dout = param.d * (err - last_err);

      //    abs_limit(&(pid->iout), pid->param.integral_limit);
      val_limit(&iout, param.integral_lower_limit, param.integral_higher_limit);
      out = pout + iout + dout;
      abs_limit(&out, param.max_out);
      last_err = err;

      if (enable == 0) {
          iout = 0;
      }
      return (iout);
  }

  void pid_reset(float max_out, float integral_limit, float kp, float ki, float kd, float ap, float bp, float cp) {
      if (kp >= 0)param.p = kp;
      if (fabsf(ki) <= 1e-9) {
          param.i = 0;
          iout = 0;
      } else if (ki > 0)
          param.i = ki;
      if (kd >= 0)param.d = kd;
      param.ap = ap;
      param.bp = bp;
      param.cp = cp;
      if (max_out > 0) param.max_out = max_out;
      if (integral_limit > 0) {
          param.integral_higher_limit = integral_limit;
          param.integral_lower_limit = -integral_limit;
      } else {
          param.integral_higher_limit = -integral_limit;
          param.integral_lower_limit = integral_limit;
      }

  }

  void pid_clear() {
      iout = 0;
  }

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

#endif //PID_H_
