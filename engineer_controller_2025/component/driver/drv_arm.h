/**
  ******************************************************************************
  * @file           : drv_arm.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-5
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

#include "drv_dji_motor.h"
#include "drv_lh_motor.h"

constexpr float Arm_Motor1_Offset = 0.7548f; //电机1偏置
constexpr float Arm_Motor2_Offset = 0.7938f; //电机2偏置
constexpr float Arm_Motor3_Offset = 0.3850f; //电机3偏置
constexpr float Arm_Motor4_Offset = 0.9986f; //电机4偏置
constexpr float Arm_Motor5_Offset = -0.4414f; //电机5偏置
constexpr float Arm_Motor6_Offset = 0.3184f; //电机6偏置

#define Arm_Motor1_Can (hfdcan1)

#define Arm_Motor2_Can (hfdcan2)
#define Arm_Motor3_Can (hfdcan2)
#define Arm_Motor4_Can (hfdcan2)
#define Arm_Motor5_Can (hfdcan2)
#define Arm_Motor6_Can (hfdcan2)

constexpr uint32_t Arm_Motor1_Id = 2;

constexpr uint32_t Arm_Motor2_Id = 0x01;

constexpr uint32_t Arm_Motor3_Id = 0x02;

constexpr uint32_t Arm_Motor4_Id = 0x03;

constexpr uint32_t Arm_Motor5_Id = 0x04;

constexpr uint32_t Arm_Motor6_Id = 0x05;


constexpr float Arm_Motor1_Torque_Compensation = 0.0f; //电机1力矩补偿`
constexpr float Arm_Motor2_Torque_Compensation = 0.0f; //电机2力矩补偿
constexpr float Arm_Motor3_Torque_Compensation = 0.0f; //电机3力矩补偿
constexpr float Arm_Motor4_Torque_Compensation = 0.0f; //电机4力矩补偿
constexpr float Arm_Motor5_Torque_Compensation = 0.0f; //电机5力矩补偿
constexpr float Arm_Motor6_Torque_Compensation = 0.0f; //电机6力矩补偿

constexpr float Arm_Motor1_Compensation_Angle_Offset = 0.0f; //电机1补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor2_Compensation_Angle_Offset = 0.0f; //电机2补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor3_Compensation_Angle_Offset = 0.0f; //电机3补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor4_Compensation_Angle_Offset = 0.0f; //电机4补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor5_Compensation_Angle_Offset = 0.0f; //电机5补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor6_Compensation_Angle_Offset = 0.0f; //电机6补偿角度偏置（即水平面角度差值）


#pragma pack(1)
struct arm_controller_tx_data_t{
  uint8_t is_data_valid; //数据是否有效
  float joint1; //关节1角度
  float joint2; //关节2角度
  float joint3; //关节3角度
  float joint4; //关节4角度
  float joint5; //关节5角度
  float joint6; //关节6角度
  float reserve_1; //保留
  uint8_t life_flag; //保留
};
#pragma pack()

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

  struct {
    dji_motor_device motor1; //关节1电机
    lh_motor_device motor2; //关节2电机
    lh_motor_device motor3; //关节3电机
    lh_motor_device motor4; //关节4电机
    lh_motor_device motor5; //关节5电机
    lh_motor_device motor6; //关节6电机
  } motor;

  arm_controller_tx_data_t controller_tx_data;

  bool is_ctrl_enable = true;

  bool is_enable_last = false; //上一次是否使能，遥控器是否开启
 public:
  arm_device() = default;

  void init();

  void update_control(bool is_enable);

  void send_msg();

  void update_data();


  void check_motor_loss();

  uint8_t * get_controller_tx_data() ;

  struct {
    joint_t joint_states;
    motor_t motor_offset;
    motor_t motor_pos_get;
    motor_t motor_pos_set;
    motor_t motor_torque_compensation; //电机力矩补偿
    motor_t motor_compensation_angle_offset; //电机补偿角度偏置（即水平面角度差值）
  } data;
};


#endif //DRV_ARM_H_
