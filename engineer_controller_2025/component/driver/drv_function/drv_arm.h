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
#include "drv_dji_motor.h"
#include "drv_dm_motor.h"

constexpr float Arm_Motor1_Offset = 0.9162f; //电机1偏置
constexpr float Arm_Motor2_Offset = -0.7075f; //电机2偏置
constexpr float Arm_Motor3_Offset = 0.3750f; //电机3偏置
constexpr float Arm_Motor4_Offset = 0.6902f; //电机4偏置
constexpr float Arm_Motor5_Offset = -0.7831f; //电机5偏置
constexpr float Arm_Motor6_Offset = 0.3184f; //电机6偏置

#define Arm_Motor1_Can (hcan1)
#define Arm_Motor2_Can (hcan1)
#define Arm_Motor3_Can (hcan1)

#define Arm_Motor4_Can (hcan2)
#define Arm_Motor5_Can (hcan2)
#define Arm_Motor6_Can (hcan2)

constexpr uint32_t Arm_Motor1_Id = 4;

constexpr uint32_t Arm_Motor2_Id = 1;

constexpr uint32_t Arm_Motor3_Id = 2;

constexpr uint32_t Arm_Motor4_Master_Id = 0x04;
constexpr uint32_t Arm_Motor4_Slave_Id = 0x03;

constexpr uint32_t Arm_Motor5_Master_Id = 0x06;
constexpr uint32_t Arm_Motor5_Slave_Id = 0x07;

constexpr uint32_t Arm_Motor6_Id = 1;

constexpr float Arm_Joint1_Min = -2.9110f; //关节1最小角度
constexpr float Arm_Joint1_Max = 2.9110f; //关节1最大角度

constexpr float Arm_Joint2_Min = 0.0f; //关节2最小角度
constexpr float Arm_Joint2_Max = 3.0635f; //关节2最大角度

constexpr float Arm_Joint3_Min = -2.15419f; //关节3最小角度
constexpr float Arm_Joint3_Max = 0.0f; //关节3最大角度

constexpr float Arm_Joint4_Min = -6.28f; //关节4最小角度
constexpr float Arm_Joint4_Max = 6.28f; //关节4最大角度

constexpr float Arm_Joint5_Min = -1.5707f; //关节5最小角度
constexpr float Arm_Joint5_Max = 1.5707f; //关节5最大角度

constexpr float Arm_Joint6_Min = -6.28f; //关节6最小角度
constexpr float Arm_Joint6_Max = 6.28f; //关节6最大角度

constexpr float Arm_Motor1_Torque_Compensation = 0.0f; //电机1力矩补偿`
constexpr float Arm_Motor2_Torque_Compensation = 0.03f; //电机2力矩补偿
constexpr float Arm_Motor3_Torque_Compensation = -0.1f; //电机3力矩补偿
constexpr float Arm_Motor4_Torque_Compensation = 0.0f; //电机4力矩补偿
constexpr float Arm_Motor5_Torque_Compensation = -0.01f; //电机5力矩补偿
constexpr float Arm_Motor6_Torque_Compensation = 0.0f; //电机6力矩补偿

constexpr float Arm_Motor1_Compensation_Angle_Offset = 0.0f; //电机1补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor2_Compensation_Angle_Offset = 0.0f; //电机2补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor3_Compensation_Angle_Offset = 0.0f; //电机3补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor4_Compensation_Angle_Offset = 0.0f; //电机4补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor5_Compensation_Angle_Offset = 0.0f; //电机5补偿角度偏置（即水平面角度差值）
constexpr float Arm_Motor6_Compensation_Angle_Offset = 0.0f; //电机6补偿角度偏置（即水平面角度差值）


#pragma pack(1)
struct arm_controller_tx_data_t {
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
    dji_motor_device motor2; //关节2电机
    dji_motor_device motor3; //关节3电机
    dm_motor_device motor4; //关节4电机
    dm_motor_device motor5; //关节5电机
    dji_motor_device motor6; //关节6电机
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

  void set_joint1_target(float set);

  void set_joint2_target(float set);

  void set_joint3_target(float set);

  void set_joint4_target(float set);

  void set_joint5_target(float set);

  void set_joint6_target(float set);

  void check_motor_loss();

  void update_tx_life_flag();

  uint8_t *get_controller_tx_data();

  struct {
    joint_t joint_states;
    joint_t joint_target; //关节目标角度
    motor_t motor_offset;
    motor_t motor_pos_get;
    motor_t motor_pos_set;
    motor_t motor_torque_compensation; //电机力矩补偿
    motor_t motor_compensation_angle_offset; //电机补偿角度偏置（即水平面角度差值）
    struct {
      joint_t min;
      joint_t max; //关节角度限制
    } joint_limit;
  } data;
};

#endif //DRV_ARM_H_
