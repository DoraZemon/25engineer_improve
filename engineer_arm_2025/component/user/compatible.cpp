/**
  ******************************************************************************
  * @file           : compatible.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "compatible.h"
#include "can.h"

void bsp_can_init() {//记得放进main函数
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY);
}

//uint8_t imu_init(SPI_HandleTypeDef *hspi, uint8_t calibrate) {
//    return g_imu.init(hspi, calibrate);
//}