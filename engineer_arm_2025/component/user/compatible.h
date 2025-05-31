/**
  ******************************************************************************
  * @file           : compatible.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef COMPATIBLE_H_
#define COMPATIBLE_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "rtos_inc.h"

#pragma pack(1)
typedef struct {
  uint32_t id;
  uint8_t len;
  uint8_t *buf_data;
} can_device_transmit_member;
#pragma pack()

typedef struct {
  int16_t set_1000;
  uint8_t id;
} servo_ctrl_data_t;

void bsp_can_init();
#ifdef __cplusplus
}
#endif
//C++

#endif //COMPATIBLE_H_
