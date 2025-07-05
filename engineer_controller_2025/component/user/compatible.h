/**
  ******************************************************************************
  * @file           : compatible.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
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
  uint32_t len;
  uint8_t *buf_data;
} fdcan_device_transmit_member;
#pragma pack()

void bsp_fdcan_init();
#ifdef __cplusplus
}
#endif
//C++

#endif //COMPATIBLE_H_
