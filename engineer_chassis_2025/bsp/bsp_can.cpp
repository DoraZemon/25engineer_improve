/**
  ******************************************************************************
  * @file           : bsp_can.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "bsp_can.h"


#include <cstring>
#include <utility>
#include "bsp_can.h"
#include "GlobalCfg.h"

can_device_t *can_device_t::can_device_list[2][MAX_CAN_DEVICE_NUM] = {0};

CAN_FilterTypeDef can_device_t::CAN_FilterInitStructure = {
    .FilterMode = CAN_FILTERMODE_IDLIST,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterActivation = CAN_FILTER_ENABLE,
    .SlaveStartFilterBank  = 14
};

uint32_t can_device_t::can_filter[2][4] = {0};
uint32_t can_device_t::can_device_num[2] = {0};


void can_device_t::init_tx(CAN_HandleTypeDef *hcan_, uint8_t len, uint32_t id, uint8_t *buff) {
    hcan = hcan_;
    tx_member.len = len;
    tx_member.id = id;
    tx_member.buf_data = buff;
}

osStatus_t can_device_t::send_msg() {
    osStatus_t state;
    if (this->hcan == &hcan1) {
        state = osMessageQueuePut(CAN1SendQueueHandle, &this->tx_member, 0, 0);
    } else {
        state = osMessageQueuePut(CAN2SendQueueHandle, &this->tx_member, 0, 0);
    }
    return state;
}

#if CAN_SEND

void CAN1Send_Task(void *argument) {
    can_device_transmit_member msg;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for (;;) {
//        taskENTER_CRITICAL();
        osSemaphoreAcquire(CAN1CountingSemHandle, osWaitForever);
        osMessageQueueGet(CAN1SendQueueHandle, &msg, 0, osWaitForever);
        tx_header.StdId = msg.id;
        tx_header.DLC = msg.len;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg.buf_data, &tx_mailbox);
//        taskEXIT_CRITICAL();
    }
}
HAL_StatusTypeDef result;
uint32_t flag = 0;
void CAN2Send_Task(void *argument) {
    can_device_transmit_member msg;
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    for (;;) {
//        taskENTER_CRITICAL();
        osSemaphoreAcquire(CAN2CountingSemHandle, osWaitForever);
        osMessageQueueGet(CAN2SendQueueHandle, &msg, 0, osWaitForever);
        tx_header.StdId = msg.id;
        tx_header.DLC = msg.len;
        result =  HAL_CAN_AddTxMessage(&hcan2, &tx_header, msg.buf_data, &tx_mailbox);
        flag++;
//        taskEXIT_CRITICAL();
    }
}

#endif

void can_tx_complete_callback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        osSemaphoreRelease(CAN1CountingSemHandle);
    } else {
        osSemaphoreRelease(CAN2CountingSemHandle);
    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

/**************************can接收*****************************/


void can_device_t::init_rx(CAN_HandleTypeDef *hcan_,
                           uint32_t id,
                           std::function<void(uint8_t *)> callback,
                           osSemaphoreId_t rx_sem_) {
    taskENTER_CRITICAL();
    if (hcan && rx_id) {
        taskEXIT_CRITICAL();
        return;
    }
    hcan = hcan_;
    if (id) {
        rx_id = id;
    }
    if (callback) {
        this->rx_callback = std::move(callback);
    }
    if (rx_sem_) {
        this->rx_sem = rx_sem_;
    }
    uint8_t channel = 0;
    bool flag = false;
    if (hcan == &hcan1) {
        channel = 0;
        flag = true;
        CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    } else if (hcan == &hcan2) {
        channel = 1;
        flag = true;
        CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    }

    if (can_device_num[channel] < MAX_CAN_DEVICE_NUM && flag) {
        flag = false;
        index = can_device_num[channel];
        can_device_num[channel]++;
        CAN_FilterInitStructure.FilterBank = index / 4 + channel * 14;
        can_device_list[channel][index] = this;

        can_filter[channel][index % 4] = this->rx_id;

        CAN_FilterInitStructure.FilterIdLow = can_filter[channel][0] << 5;
        CAN_FilterInitStructure.FilterMaskIdLow = can_filter[channel][1] << 5;
        CAN_FilterInitStructure.FilterIdHigh = can_filter[channel][2] << 5;
        CAN_FilterInitStructure.FilterMaskIdHigh = can_filter[channel][3] << 5;

        HAL_CAN_ConfigFilter(this->hcan, &CAN_FilterInitStructure);
        if (this->index % 4 == 3) {
            memset(can_filter[channel], 0, sizeof(can_filter[channel]));
        }
    }
    taskEXIT_CRITICAL();

}

//CAN1 接收
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_t *device = can_device_t::can_device_list[0][index];
    if (device && device->rx_callback) {
        device->rx_callback(rx_data);
        osSemaphoreRelease(device->rx_sem);
    }
}

//CAN2 接收
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data);
    uint32_t index = rx_header.FilterMatchIndex;
    can_device_t *device = can_device_t::can_device_list[1][index];
    if (device && device->rx_callback) {
        device->rx_callback(rx_data);
        osSemaphoreRelease(device->rx_sem);
    }
}



