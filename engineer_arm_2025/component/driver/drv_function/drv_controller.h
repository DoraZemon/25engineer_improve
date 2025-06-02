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
  float joint1;
  float joint2;
  float joint3;
  float joint4;
  float joint5;
  float joint6;
  float reserve_1;
  uint16_t reserve_2;
} custom_judge_raw_msg;

#pragma pack()

struct controller_raw_data_t{
    float joint1;
    float joint2;
    float joint3;
    float joint4;
    float joint5;
    float joint6;
};


class controller_device {
 public:
    controller_raw_data_t raw_data;
    bool is_lost = true;
  bool judge_transfer_rx_callback(custom_judge_raw_msg *judge_raw_msg);

  void init();
  void update();
  void set_lost();
  void set_connected();
  bool check_lost() const;
};

#endif //DRV_CONTROLLER_H_
