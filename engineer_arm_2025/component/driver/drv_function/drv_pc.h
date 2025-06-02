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
  float uplift_joint;
  float extend_joint;
  float pitch_joint;
  float chassis_gyro_total_rounds;
  uint8_t error_code;
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
  uint8_t is_arm_pump_holding_on;
  uint8_t is_left_pump_holding_on;
  uint8_t is_right_pump_holding_on;
  uint8_t is_excuting_small_island;
  float reserve_1; //保留
  float reserve_2; //保留
  float reserve_3; //保留
  float reserve_4; //保留
  float reserve_5; //保留
  uint8_t reserve_6; //保留
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
  uint8_t is_arm_pump_on; //机械臂泵是否开启
  uint8_t is_left_pump_on; //左泵是否开启
  uint8_t is_right_pump_on; //右泵是否开启
  uint8_t is_chassis_vel_control;
  int8_t chassis_spin;
  int16_t chassis_x;
  int16_t chassis_y;
  uint8_t camera_dir;
  uint8_t is_start_small_island; //是否开始小岛
  uint8_t reset_affiliates;
  uint8_t affiliates_high_mode;
  uint8_t magic_flag;
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

  void update_data(rc_device& rc,arm_device & arm,controller_device & controller); //更新数据
  void transmit_data(); //发送数据到PC
  void set_lost();
  void set_connected();
  bool check_lost();
 private:
  bool is_lost = true;
};

#endif //DRV_PC_H_
