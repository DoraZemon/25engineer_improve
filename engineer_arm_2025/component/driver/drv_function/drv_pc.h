/**
  ******************************************************************************
  * @file           : drv_pc.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#ifndef DRV_PC_H_
#define DRV_PC_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"
#include "drv_arm.h"
#include "drv_rc.h"
#include "drv_controller.h"
#include "drv_communicate.h"
//#include "drv_dm_Imu.h"
#include "drv_gimbal.h"
#include "drv_hi229um.h"

enum robot_error_type {
  remote = 0,
  pc,
  communicate,
  ctrl,
  arm_motor1,
  arm_motor2,
  arm_motor3,
  arm_motor4,
  arm_motor5,
  arm_motor6,
  imu,
  chassis_motor1,
  chassis_motor2,
  chassis_motor3,
  chassis_motor4,
  none,
};

enum robot_auto_situation_e{
  auto_none = 0, //无自动
  small_island, //小岛
    big_island, //大岛
    exchange,
    controller,
    ground_mine,
    arm_homing,
};


#pragma pack(1)
union robot_error_u {
  uint32_t code;
  struct {
    uint32_t remote: 1;
    uint32_t pc: 1;
    uint32_t communicate: 1;
    uint32_t controller: 1;
    uint32_t arm_motor1: 1;
    uint32_t arm_motor2: 1;
    uint32_t arm_motor3: 1;
    uint32_t arm_motor4: 1;
    uint32_t arm_motor5: 1;
    uint32_t arm_motor6: 1;
    uint32_t imu: 1;
    uint32_t chassis_motor1: 1;
    uint32_t chassis_motor2: 1;
    uint32_t chassis_motor3: 1;
    uint32_t chassis_motor4: 1;
    uint32_t reserve: 17; //保留位
  };
};

#pragma pack()

#pragma pack(1)
struct pc_normal_tx_data_t {
  uint8_t frame_head; //帧头
  uint8_t is_rc_online; //遥控器在线状态
  uint8_t is_from_dt7; //是否来自DT7
  uint8_t remote_ctrl[18]; //遥控器数据
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  float chassis_gyro_total_rounds;
  uint8_t is_arm_pump_holding_on;
  uint8_t is_left_pump_holding_on;
  uint8_t is_right_pump_holding_on;
  uint32_t error_code;
  uint8_t frame_tail; //帧尾
};

struct pc_controller_tx_data_t {
  uint8_t frame_head; //帧头
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  uint8_t button1; //按钮1
  uint8_t button2; //按钮2
  int16_t mouse_x; //鼠标X轴
  int16_t mouse_y; //鼠标Y轴
  int16_t mouse_z; //鼠标滚轮
  int8_t left_button_down; //左键按下
  int8_t right_button_down; //右键按下
  uint16_t keyboard_value;
  uint8_t is_controller_valid;
  float reserve_1; //保留
  float reserve_2; //保留
  float reserve_3; //保留
  uint16_t reserve_4; //保留
  uint32_t reserve_5; //保留
  uint8_t frame_tail; //帧尾
};

struct pc_rx_data_t {
  uint8_t frame_head; //帧头
  float joint1; //关节1
  float joint2; //关节2
  float joint3; //关节3
  float joint4; //关节4
  float joint5; //关节5
  float joint6; //关节6
  float joint1_compensation;// 关节1补偿力矩
  float joint2_compensation; //关节2补偿力矩
  float joint3_compensation; //关节3补偿力矩
  float joint4_compensation; //关节4补偿力矩
  float joint5_compensation; //关节5补偿力矩
  float joint6_compensation; //关节6补偿力矩
  uint8_t is_arm_pump_on; //机械臂泵是否开启
  uint8_t is_left_pump_on; //左泵是否开启
  uint8_t is_right_pump_on; //右泵是否开启
  int8_t chassis_spin;
  int16_t chassis_x;
  int16_t chassis_y;
  uint8_t arm_ctrl_enable; //机械臂使能
  uint8_t arm_reset_pitch_enable;
  uint8_t reserve;
  int16_t gimbal_pitch;
  uint8_t auto_situation; //自动情况
  uint8_t frame_tail; //帧尾
};
#pragma pack()

class pc_device {
 public:
  pc_device() = default;

  uint16_t normal_counter = 0;   // 100Hz发送计数器
  uint16_t controller_counter = 0; // 30Hz发送计数器
  pc_normal_tx_data_t normal_tx_data; //PC正常通信发送数据
  pc_controller_tx_data_t controller_tx_data; //PC控制器通信发送数据
  pc_rx_data_t rx_data; //PC接收数据

  robot_error_u error;
  robot_auto_situation_e auto_situation ;

  void update_data(rc_device &rc,
                   arm_device &arm,
                   controller_device &controller,
                   communicate_device &communicate,
                    gimbal_device &gimbal); //更新数据
  void transmit_data(rc_device &rc,
                     arm_device &arm,
                     controller_device &controller,
                     communicate_device &communicate,
                     hi229um_device &hi229, gimbal_device &gimbal); //发送数据到PC
  void set_lost();

  void set_connected();

  bool check_lost();

 private:
  bool is_lost = true;
};

#endif //DRV_PC_H_
