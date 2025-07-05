/**
  ******************************************************************************
  * @file           : compatible.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#include "compatible.h"
#include "fdcan.h"

void bsp_fdcan_init() {
    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_Start(&hfdcan2);
//    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, 0);
//    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO2_NEW_MESSAGE, 0);
}
