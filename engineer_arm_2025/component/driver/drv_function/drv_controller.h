/**
  ******************************************************************************
  * @file           : drv_controller.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#ifndef DRV_CONTROLLER_H_
#define DRV_CONTROLLER_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"
#include "rtos_inc.h"

#pragma pack(1)
typedef struct {
  uint8_t is_data_valid; //数据是否有效
  float joint1; //关节1角度
  float joint2; //关节2角度
  float joint3; //关节3角度
  float joint4; //关节4角度
  float joint5; //关节5角度
  float joint6; //关节6角度
  float reserve_1; //保留
  uint8_t life_flag;
} custom_judge_raw_msg;

#pragma pack()

struct controller_raw_data_t {
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  bool is_data_valid = false; //数据是否有效
  uint8_t life_flag = 0; //生命检测标志位
};


class controller_device {
 public:
  controller_raw_data_t raw_data;
  uint8_t last_life_flag = 0; //上一次生命检测标志位
  controller_raw_data_t last_valid_raw_data; //上一次接收到的数据
  bool is_lost = true;
  uint32_t lost_num = 0;

  bool judge_transfer_rx_callback(custom_judge_raw_msg *judge_raw_msg);

  void init();

  void update();

  void set_lost();

  void set_connected();

  bool check_lost() const;
};

#endif //DRV_CONTROLLER_H_
