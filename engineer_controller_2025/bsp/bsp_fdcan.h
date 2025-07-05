/**
  ******************************************************************************
  * @file           : bsp_fdcan.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#ifndef BSP_FDCAN_H_
#define BSP_FDCAN_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "fdcan.h"
void FDCAN2Send_Task(void *argument);
void FDCAN1Send_Task(void *argument);
void FDCAN3Send_Task(void *argument);
void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#ifdef __cplusplus
}
#endif
//C++
#define MAX_CAN_DEVICE_NUM (8)//与cube 中fdcan的配置有关,fifo个数

#include <functional>
#include "compatible.h"
#include "rtos_inc.h"

class fdcan_device_t {
 public:
  static fdcan_device_t *can_device_list[3][MAX_CAN_DEVICE_NUM];
  static uint32_t can_device_num[3];
  static FDCAN_FilterTypeDef FDCAN_FilterInitStructure;

  std::function<void(uint8_t *)> rx_callback;

  void init_rx(FDCAN_HandleTypeDef *hfdcan_,
               uint32_t id,
               std::function<void(uint8_t *)> callback,
               osSemaphoreId_t rx_sem_);

  void init_tx(FDCAN_HandleTypeDef *hfdcan_, uint32_t len, uint32_t id, uint8_t *buff);

  void reset_tx(uint32_t len, uint32_t id);

  osStatus_t send_msg();

  uint32_t rx_id;
  FDCAN_HandleTypeDef *hfdcan;
  osSemaphoreId_t rx_sem;
  uint32_t index;
  fdcan_device_transmit_member tx_member;
};

#endif //BSP_FDCAN_H_
