/**
  ******************************************************************************
  * @file           : mecanum.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "mecanum.h"

//无功率环
__RAM_FUNC void chassisMotorSolverSet(dji_motor_device wheels[], float vel_x, float vel_y, float spin) {
    float speeds[4];
    float speed_correction = 1;//这个是限制轮子的最大速度不是车体的速度,也可以直接在这里限制车速
    float w = 0.8f;
    float v;
    ABS_LIMIT(vel_x, 1.0f);
    ABS_LIMIT(vel_y, 1.0f);
    arm_sqrt_f32(vel_x * vel_x + vel_y * vel_y, &v);

    //其实这一步可以没有，因为有这一步其实是改变了w和x和y的比例分配问题（最大功率有限制），实际上可以改spin的值来做
    if (v > 1) {
        float theta = atanf(vel_y / vel_x);
        vel_x = arm_cos_f32(theta);
        vel_y = arm_sin_f32(theta);
    }
    //逆时针

    speeds[0] = (vel_x - vel_y - spin * w);//define CHASSIS_MOTOR_LF    (0x201)
    speeds[1] = (vel_x + vel_y - spin * w);//define CHASSIS_MOTOR_LB    (0x202)
    speeds[2] = (vel_x - vel_y + spin * w);//define CHASSIS_MOTOR_RB    (0x203)
    speeds[3] = (vel_x + vel_y + spin * w);//define CHASSIS_MOTOR_RF    (0x204)

    for (float speed : speeds) {//取最大值，这个比较合理，我之前是在speed出来之前进行分配，这个是出来之后进行分配，是一样的。
        if (fabsf(speed) > speed_correction)
            speed_correction = fabsf(speed);
    }
    speed_correction = 1.0f / speed_correction;
    //这个又进行了一次处理，是取最大的那个按比例来做

    for (int i = 0; i < 4; i++) {
        wheels[i].set_vel(speed_correction * speeds[i]);
    }
}

/**
  * @brief mecanum glb_chassis velocity decomposition.F:forword; B:backword; L:left; R:right
  * @param input : ccx=+vx(mm/s)  ccy=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR  Y前后 X左右
  */
__RAM_FUNC void SuperChassisMotorSolverSet(dji_motor_device wheels[], float vel_x, float vel_y, float spin) {
    static float rotate_ratio_rf, rotate_ratio_lf, rotate_ratio_lb, rotate_ratio_rb;
    float speeds[4];
    float speed_correction = 1;//这个是限制轮子的最大速度不是车体的速度,也可以直接在这里限制车速
    float v;
    ABS_LIMIT(vel_x, 1.0f);
    ABS_LIMIT(vel_y, 1.0f);
    arm_sqrt_f32(vel_x * vel_x + vel_y * vel_y, &v);
    //其实这一步可以没有，因为有这一步其实是改变了w和x和y的比例分配问题（最大功率有限制），实际上可以改spin的值来做
    if (v > 1) {//留着先不改
        float theta = atanf(vel_y / vel_x);
        vel_x = arm_cos_f32(theta);
        vel_y = arm_sin_f32(theta);
    }

    float ratio_ratio_max = (WHEELBASE + WHEELTRACK) / 2.0f + ABS(ROTATE_Y_OFFSET) + ABS(ROTATE_X_OFFSET);

    //先y后x
    rotate_ratio_rf = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_Y_OFFSET + ROTATE_X_OFFSET) / ratio_ratio_max;
    rotate_ratio_lf = ((WHEELBASE + WHEELTRACK) / 2.0f - ROTATE_Y_OFFSET - ROTATE_X_OFFSET) / ratio_ratio_max;
    rotate_ratio_lb = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_Y_OFFSET - ROTATE_X_OFFSET) / ratio_ratio_max;
    rotate_ratio_rb = ((WHEELBASE + WHEELTRACK) / 2.0f + ROTATE_Y_OFFSET + ROTATE_X_OFFSET) / ratio_ratio_max;

    //解算出角度
    speeds[0] = (vel_x - vel_y - spin * rotate_ratio_lf);//define CHASSIS_MOTOR_LF    (0x201)
    speeds[1] = (vel_x + vel_y - spin * rotate_ratio_lb);//define CHASSIS_MOTOR_LB    (0x202)
    speeds[2] = (vel_x - vel_y + spin * rotate_ratio_rb);//define CHASSIS_MOTOR_RB    (0x203)
    speeds[3] = (vel_x + vel_y + spin * rotate_ratio_rf);//define CHASSIS_MOTOR_RF    (0x204)


    for (int i = 0; i < 4; i++) {//取最大值，这个比较合理，我之前是在speed出来之前进行分配，这个是出来之后进行分配，是一样的。
        if (fabsf(speeds[i]) > speed_correction)//speed_correction是轮子最大速度限制
            speed_correction = fabsf(speeds[i]);
    }
    speed_correction = 1.0f / speed_correction;
    //取最大为1 按比例输出速度
    for (int i = 0; i < 4; i++) {
        wheels[i].set_vel(speed_correction * speeds[i]);
    }
}

/**
 * @brief 给定底盘需要移动的距离，反解除电机需要转的圈数
 * @param wheels
 * @param x 底盘坐标系
 * @param y
 * @param w 旋转角度 (rad)
 */

float pre_w;

__RAM_FUNC void chassisMotorPositionSolverSet(dji_motor_device wheels[], float x, float y, float w)//机械臂的坐标系和底盘坐标系xy相反
{
    float pos[4];
    float rotate_ratio = (WHEELBASE + WHEELTRACK) / 2.f;
    float ecd_ratio_w = (ECD_RATIO_X + ECD_RATIO_Y) / 2.f;
    float pre_x = x / ECD_RATIO_X;
    float pre_y = y / ECD_RATIO_Y;
    pre_w = w / ecd_ratio_w * rotate_ratio;
//    pos[0] = (x - y) / ECD_RATIO;                                       //define CHASSIS_MOTOR_LF    (0x201)
//    pos[1] = (x + y) / ECD_RATIO;                                        //define CHASSIS_MOTOR_LB    (0x202)
//    pos[2] = (x - y) / ECD_RATIO;                                        //define CHASSIS_MOTOR_RB    (0x203)
//    pos[3] = (x + y) / ECD_RATIO;                                        //define CHASSIS_MOTOR_RF    (0x204)
    pos[0] = pre_x - pre_y - pre_w;
    pos[1] = pre_x + pre_y - pre_w;
    pos[2] = pre_x - pre_y + pre_w;
    pos[3] = pre_x + pre_y + pre_w;

    for (int i = 0; i < 4; i++) {
        wheels[i].set_pos(pos[i]);
    }
}