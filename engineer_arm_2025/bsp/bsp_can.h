/**
  ******************************************************************************
  * @file           : bsp_can.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef BSP_CAN_H_
#define BSP_CAN_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "can.h"
void CAN2Send_Task(void *argument);
void CAN1Send_Task(void *argument);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
#ifdef __cplusplus
}
#endif
//C++
#define MAX_CAN_DEVICE_NUM (14 * 4)

#include <functional>
#include "compatible.h"
#include "rtos_inc.h"

class can_device_t {
 public:
  static can_device_t *can_device_list[2][MAX_CAN_DEVICE_NUM];
  static uint32_t can_filter[2][4];
  static uint32_t can_device_num[2];
  static CAN_FilterTypeDef CAN_FilterInitStructure;

  std::function<void(uint8_t *)> rx_callback;

  void init_rx(CAN_HandleTypeDef *hcan_, uint32_t id, std::function<void(uint8_t *)> callback, osSemaphoreId_t rx_sem_);

  void init_tx(CAN_HandleTypeDef *hcan_, uint8_t len, uint32_t id, uint8_t *buff);

  osStatus_t send_msg();

  uint32_t rx_id;
  CAN_HandleTypeDef *hcan;
  osSemaphoreId_t rx_sem;
  uint32_t index;
  can_device_transmit_member tx_member;
};

#endif //BSP_CAN_H_
