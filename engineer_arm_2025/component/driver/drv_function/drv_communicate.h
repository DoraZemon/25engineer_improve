/**
  ******************************************************************************
  * @file           : drv_communicate.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-6
  ******************************************************************************
  */


#ifndef DRV_COMMUNICATE_H_
#define DRV_COMMUNICATE_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "bsp_can.h"
#include "drv_rc.h"
#include "drv_dji_motor.h"

#define COMMUNICATE_FRAME_HEAD 0x55 //通信帧头
#define COMMUNICATE_CAN         (hcan2)
#define COMMUNICATE_RX_ID 0x101
#define COMMUNICATE_TX_ID 0x100

#pragma pack(1)
struct communicate_rx_data_t {
  uint8_t frame_head; //帧头
  uint8_t is_arm_pump_holding: 1;
  uint8_t is_left_pump_holding: 1;
  uint8_t is_right_pump_holding: 1;
  uint8_t chassis_motor1_error: 1;
  uint8_t chassis_motor2_error: 1;
  uint8_t chassis_motor3_error: 1;
  uint8_t chassis_motor4_error: 1;
  uint8_t reserve_1: 1; //保留
  uint8_t reserve_2; //保留
  uint8_t reserve_3; //保留
  uint8_t reserve_4; //保留
  uint8_t reserve_5; //保留
  uint8_t reserve_6; //保留
  uint8_t reserve_7; //保留

};


struct communicate_tx_data_t {
  uint8_t frame_head; //帧头
  uint8_t is_arm_pump_open: 1;
  uint8_t is_left_pump_open: 1;
  uint8_t is_right_pump_open: 1;
  uint8_t is_rc_online: 1; //遥控器是否在线
  uint8_t reserve_1: 4; //保留
  int16_t speed_x;
  int16_t speed_y;
  int8_t speed_spin;
  uint8_t reserve_2; //保留
};

#pragma pack()


class communicate_device {
 public:
  communicate_tx_data_t tx_data;
  communicate_rx_data_t rx_data;

  can_device_t can_device; //CAN设备

  bool is_lost = true; //是否丢失
  uint32_t lost_num = 0;
  struct {
    bool is_arm_pump_open;
    bool is_left_pump_open;
    bool is_right_pump_open;
    bool is_rc_online; //遥控器是否在线
    int16_t speed_x; //速度X
    int16_t speed_y; //速度Y
    int8_t speed_spin; //旋转速度

    bool is_arm_pump_holding;
    bool is_left_pump_holding;
    bool is_right_pump_holding;
    bool is_chassis_motor1_error;
    bool is_chassis_motor2_error;
    bool is_chassis_motor3_error;
    bool is_chassis_motor4_error;

    bool chassis_error;
  } data;
  CAN_HandleTypeDef *hcan;

  void init(CAN_HandleTypeDef *hcan_, uint32_t tx_id, uint32_t rx_id, osSemaphoreId_t rxsem);

  void update(rc_device &rc);

  void check_for_loss();


  void send_msg();

  void update_rx_data(uint8_t *rx_data);

  void set_chassis_ctrl( int16_t speed_x, int16_t speed_y, int8_t speed_spin);

  void set_pump_ctrl(bool is_arm_pump_open, bool is_left_pump_open, bool is_right_pump_open);

  bool check_arm_pump_holding();

  bool check_left_pump_holding();

  bool check_right_pump_holding();

  bool check_chassis_error();

  bool check_lost();
};

#endif //DRV_COMMUNICATE_H_
