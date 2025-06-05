/**
  ******************************************************************************
  * @file           : mecanum.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#ifndef MECANUM_H_
#define MECANUM_H_

#include "drv_dji_motor.h"

#ifdef __cplusplus
extern "C" {
#endif
//C
#include "arm_math.h"
#include <math.h>

/* the radius of wheel(mm) */
#ifndef RADIUS
#define RADIUS 77
#endif
/* the perimeter of wheel(mm) */  //车轮周长 2*PI*R
#define PERIMETER 483.8f

/* chassis motor use 3508 */
/* the deceleration ratio of chassis motor */
#define MOTOR_DECELE_RATIO (1.0f / 19.0f)

#define WHEELPERIMETER 395.0f

#define WHEELPERIMETER_X  395.f
#define WHEELPERIMETER_Y  373.f

/* wheel track distance(mm) */ //轨距 左右轮之间的距离
#define WHEELTRACK (450.49f)
/* wheelbase distance(mm) */ //轮距 前后轮之车轮轴距离
#define WHEELBASE (362.9f)

/* gimbal is relative to chassis center x axis offset(mm) */
#define ROTATE_X_OFFSET (-165.0f)
/* gimbal is relative to chassis center y axis offset(mm) */
#define ROTATE_Y_OFFSET (0.0f)

#define ECD_RATIO  (WHEELPERIMETER * MOTOR_DECELE_RATIO)

#define ECD_RATIO_X  (WHEELPERIMETER_X * MOTOR_DECELE_RATIO)
#define ECD_RATIO_Y  (WHEELPERIMETER_Y * MOTOR_DECELE_RATIO)


#ifndef RADIAN_COEF
#define RADIAN_COEF 57.3f //弧度到角度
#endif
#ifdef __cplusplus
}
#endif
//C++

void chassisMotorSolverSet(dji_motor_device wheels[], float vel_x, float vel_y, float spin);

void SuperChassisMotorSolverSet(dji_motor_device wheels[], float vel_x, float vel_y, float spin);

__RAM_FUNC void chassisMotorPositionSolverSet(dji_motor_device wheels[], float x, float y, float w);


#endif //MECANUM_H_
