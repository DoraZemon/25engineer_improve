/**
  ******************************************************************************
  * @file           : drv_arm.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef DRV_ARM_H_
#define DRV_ARM_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "drv_dm_motor.h"
#include "median_filter.h"
#include "drv_dji_motor.h"

constexpr float Arm_Motor1_Offset = 0.9474f; //电机1偏置
constexpr float Arm_Motor2_Offset = -0.5394f; //电机2偏置
constexpr float Arm_Motor3_Offset = -0.2276f; //电机3偏置
constexpr float Arm_Motor4_Offset = 0.2413f; //电机4偏置
constexpr float Arm_Motor5_Offset = -0.5805f; //电机5偏置
constexpr float Arm_Motor6_Offset = 0.f; //电机6偏置

#define Arm_Motor1_Can (hcan1)
#define Arm_Motor2_Can (hcan2)
#define Arm_Motor3_Can (hcan2)

#define Arm_Motor4_Can (hcan1)
#define Arm_Motor5_Can (hcan1)
#define Arm_Motor6_Can (hcan1)

constexpr uint32_t Arm_Motor1_Master_Id = 0x03;
constexpr uint32_t Arm_Motor1_Slave_Id = 0x03;

constexpr uint32_t Arm_Motor2_Master_Id = 0x04;
constexpr uint32_t Arm_Motor2_Slave_Id = 0x04;

constexpr uint32_t Arm_Motor3_Master_Id = 0x05;
constexpr uint32_t Arm_Motor3_Slave_Id = 0x05;

constexpr uint32_t Arm_Motor4_Master_Id = 0x02;
constexpr uint32_t Arm_Motor4_Slave_Id = 0x02;

constexpr uint32_t Arm_Motor5_Master_Id = 0x01;
constexpr uint32_t Arm_Motor5_Slave_Id = 0x01;

constexpr uint32_t Arm_Motor6_Id = 2;

constexpr float Arm_Joint1_Min = -2.98f; //关节1最小角度
constexpr float Arm_Joint1_Max = 2.98f; //关节1最大角度

constexpr float Arm_Joint2_Min = 0.0f; //关节2最小角度
constexpr float Arm_Joint2_Max = 2.47f; //关节2最大角度

constexpr float Arm_Joint3_Min = -1.65f; //关节3最小角度
constexpr float Arm_Joint3_Max = 0.0f; //关节3最大角度

constexpr float Arm_Joint4_Min = -6.28f; //关节4最小角度
constexpr float Arm_Joint4_Max = 6.28f; //关节4最大角度

constexpr float Arm_Joint5_Min = -1.5707f; //关节5最小角度
constexpr float Arm_Joint5_Max = 1.5707f; //关节5最大角度

constexpr float Arm_Joint6_Min = -6.28f; //关节6最小角度
constexpr float Arm_Joint6_Max = 6.28f; //关节6最大角度

constexpr float Arm_Motor1_Pos_Max = Arm_Joint1_Max / (2 * PI); //电机1最大角度
constexpr float Arm_Motor1_Pos_Min = Arm_Joint1_Min / (2 * PI); //电机1最小角度

constexpr float Arm_Motor2_Pos_Max = Arm_Joint2_Max / (2 * PI) / 2.f * 3.f; //电机2最大角度
constexpr float Arm_Motor2_Pos_Min = Arm_Joint2_Min / (2 * PI) / 2.f * 3.f ; //电机2最小角度

constexpr float Arm_Motor3_Pos_Max = (130 / 360.f); //电机3最大角度
constexpr float Arm_Motor3_Pos_Min = (-40.f /360.f); //电机3最小角度

constexpr float Arm_Motor4_Pos_Max = Arm_Joint4_Max / (2 * PI); //电机4最大角度
constexpr float Arm_Motor4_Pos_Min = Arm_Joint4_Min / (2 * PI); //电机4最小角度

constexpr float Arm_Motor5_Pos_Max = Arm_Joint5_Max / (2 * PI); //电机5最大角度
constexpr float Arm_Motor5_Pos_Min = Arm_Joint5_Min / (2 * PI); //电机5最小角度

constexpr float Arm_Motor6_Pos_Max = Arm_Joint6_Max / (2 * PI) * (36.f * 2.f); //电机6最大角度
constexpr float Arm_Motor6_Pos_Min = Arm_Joint6_Min / (2 * PI) * (36.f * 2.f); //电机6最小角度


class arm_device {
  struct joint_t {
    float joint1; //关节1角度
    float joint2; //关节2角度
    float joint3; //关节3角度
    float joint4; //关节4角度
    float joint5; //关节5角度
    float joint6; //关节6角度
  };

  struct motor_t {
    float motor1; //电机1角度
    float motor2; //电机2角度
    float motor3; //电机3角度
    float motor4; //电机4角度
    float motor5; //电机5角度
    float motor6; //电机6角度
  };

  bool is_ctrl_enable = true;

  bool is_enable_last = false; //上一次是否使能，遥控器是否开启

  bool is_lost = false;
 public:
  arm_device();

  void init();

  void update_control(bool is_enable);

  void send_msg();

  void update_data();

  void set_joint1_target(float set);

  void set_joint2_target(float set);

  void set_joint3_target(float set);

  void set_joint4_target(float set);

  void set_joint5_target(float set);

  void set_joint6_target(float set);

  void set_joint1_compensation(float set);

  void set_joint2_compensation(float set);

  void set_joint3_compensation(float set);

  void set_joint4_compensation(float set);

  void set_joint5_compensation(float set);

  void set_joint6_compensation(float set);

  void set_arm_ctrl_enable(bool is_enable);

  void check_motor_loss();

  void limit_motor_pos();

  bool check_lost();

  struct {
    joint_t joint_states;
    joint_t joint_target; //关节目标角度
    joint_t joint_filtered_target; //关节滤波角度
    motor_t motor_offset;
    motor_t motor_pos_get;
    motor_t motor_pos_set;
    motor_t motor_torque_compensation; //电机力矩补偿
    struct {
      joint_t min;
      joint_t max; //关节角度限制
    } joint_limit;

    struct {
      motor_t min;
      motor_t max; //电机位置限制
    } motor_pos_limit;
  } data;

  MedianFilter joint1_filter;
  MedianFilter joint2_filter;
  MedianFilter joint3_filter;
  MedianFilter joint4_filter;
  MedianFilter joint5_filter;
  MedianFilter joint6_filter;
  struct {
    dm_motor_device motor1; //关节1电机
    dm_motor_device motor2; //关节2电机
    dm_motor_device motor3; //关节3电机
    dm_motor_device motor4; //关节4电机
    dm_motor_device motor5; //关节5电机
    dji_motor_device motor6; //关节6电机
  } motor;

};

#endif //DRV_ARM_H_
