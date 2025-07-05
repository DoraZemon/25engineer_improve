//
// Created by 34147 on 2024/1/25.
//

#include "pid.h"

///PID构造函数
//设置了 PID 控制器的初始参数
pid::pid(float max_out_, float integral_limit, float kp, float ki, float kd, float ap, float bp, float cp)
    : param{kp, ki, kd, ap, bp, cp, 0, max_out_, integral_limit, -integral_limit},//结构体初始化格式
      enable(true),
      set(0), get(0),
      err(0), last_err(0), penultimate_err(0),
      pout(0), iout(0), dout(0), out(0) {}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output   0-outmax
  */ //0-outmax已限幅
__RAM_FUNC
float pid::pid_calculate(float set_, float get_) {
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

float pid::pid_calculate_for_iout(float set_, float get_) {
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

void pid::pid_reset(float max_out, float integral_limit, float kp, float ki, float kd, float ap, float bp, float cp) {
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

void pid::pid_clear() {
    iout = 0;
}