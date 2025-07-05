/**
  ******************************************************************************
  * @file           : bsp_fdcan.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#include "bsp_fdcan.h"
#include "rtos_inc.h"
#include "cmsis_os.h"

#define CAN_SEND 1

fdcan_device_t *fdcan_device_t::can_device_list[3][MAX_CAN_DEVICE_NUM] = {0};

FDCAN_FilterTypeDef fdcan_device_t::FDCAN_FilterInitStructure = {
    .FilterType = FDCAN_FILTER_MASK,
};

uint32_t fdcan_device_t::can_device_num[3] = {0};


void fdcan_device_t::init_tx(FDCAN_HandleTypeDef *hfdcan_, uint32_t len, uint32_t id, uint8_t *buff) {
    hfdcan = hfdcan_;
    tx_member.len = len;
    tx_member.id = id;
    tx_member.buf_data = buff;
}

void fdcan_device_t::reset_tx(uint32_t len, uint32_t id) {
    tx_member.len = len;
    tx_member.id = id;
}

osStatus_t fdcan_device_t::send_msg() {
    osStatus_t state;
    if (this->hfdcan == &hfdcan1) {
        state = osMessageQueuePut(FDCAN1SendQueueHandle, &this->tx_member, 0, 0);
    } else if (hfdcan == &hfdcan2) {
        state = osMessageQueuePut(FDCAN2SendQueueHandle, &this->tx_member, 0, 0);
    } else {
        state = osMessageQueuePut(FDCAN3SendQueueHandle, &this->tx_member, 0, 0);
    }
    return state;
}

#if CAN_SEND
// 兼容bxcan
void FDCAN1Send_Task(void *argument) {
    fdcan_device_transmit_member msg;
    uint32_t tx_mailbox;
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    for (;;) {
//        taskENTER_CRITICAL();
//        osSemaphoreAcquire(FDCAN1CountingSemHandle, osWaitForever);
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
            osDelay(1); // 等待FDCAN1的发送FIFO有空余空间
        }
        osMessageQueueGet(FDCAN1SendQueueHandle, &msg, 0, osWaitForever);
        tx_header.Identifier = msg.id;
        tx_header.DataLength = msg.len;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, msg.buf_data);//加入到tx_fifo准备发送
//        taskEXIT_CRITICAL();
    }
}

//扩展帧，仲裁域波特率：1Mbps 80%,数据与波特率：5Mbps 75%
void FDCAN2Send_Task(void *argument) {
    fdcan_device_transmit_member msg;
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    for (;;) {
//        taskENTER_CRITICAL();
//        osSemaphoreAcquire(FDCAN2CountingSemHandle, osWaitForever);
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0) {
            osDelay(1); // 等待FDCAN2的发送FIFO有空余空间
        }
        osMessageQueueGet(FDCAN2SendQueueHandle, &msg, 0, osWaitForever);
        tx_header.Identifier = msg.id;
        tx_header.DataLength = msg.len;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, msg.buf_data);//加入到tx_fifo准备发送
//        taskEXIT_CRITICAL();
    }
}

void FDCAN3Send_Task(void *argument) {
    fdcan_device_transmit_member msg;
    uint32_t tx_mailbox;
    FDCAN_TxHeaderTypeDef tx_header;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    for (;;) {
//        taskENTER_CRITICAL();
//        osSemaphoreAcquire(FDCAN3CountingSemHandle, osWaitForever);
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3) == 0) {
            osDelay(1); // 等待FDCAN3的发送FIFO有空余空间
        }
        osMessageQueueGet(FDCAN3SendQueueHandle, &msg, 0, osWaitForever);
        tx_header.Identifier = msg.id;
        tx_header.DataLength = msg.len;
        HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, msg.buf_data);//加入到tx_fifo准备发送
//        taskEXIT_CRITICAL();
    }
}

#endif

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {
    if (TxEventFifoITs & FDCAN_IT_TX_COMPLETE) {
        // 一帧 CAN 报文已经成功发送出去！
        // 可以设置标志位、发送下一帧、打印日志等
        if (hfdcan == &hfdcan1) {
            osSemaphoreRelease(FDCAN1CountingSemHandle);
        } else if (hfdcan == &hfdcan2) {
            osSemaphoreRelease(FDCAN2CountingSemHandle);
        } else if (hfdcan == &hfdcan3) {
            osSemaphoreRelease(FDCAN3CountingSemHandle);
        }
    }
}


/**************************can接收*****************************/


void fdcan_device_t::init_rx(FDCAN_HandleTypeDef *hfdcan_,
                             uint32_t id,
                             std::function<void(uint8_t *)> callback,
                             osSemaphoreId_t rx_sem_) {
    taskENTER_CRITICAL();
    if (hfdcan && rx_id) {
        taskEXIT_CRITICAL();
        return;
    }
    hfdcan = hfdcan_;
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
    if (hfdcan == &hfdcan1) {
        channel = 0;
        flag = true;
        FDCAN_FilterInitStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        FDCAN_FilterInitStructure.IdType = FDCAN_STANDARD_ID;
    } else if (hfdcan == &hfdcan2) {
        channel = 1;
        flag = true;
        FDCAN_FilterInitStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        FDCAN_FilterInitStructure.IdType = FDCAN_EXTENDED_ID;
    } else if (hfdcan == &hfdcan3) {
        channel = 2;
        flag = true;
        FDCAN_FilterInitStructure.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;// FDCAN3 只使用 RX FIFO1 todo
        FDCAN_FilterInitStructure.IdType = FDCAN_STANDARD_ID;
    }

    if (can_device_num[channel] < MAX_CAN_DEVICE_NUM && flag) {
        flag = false;
        index = can_device_num[channel];
        can_device_num[channel]++;
        can_device_list[channel][index] = this;

        FDCAN_FilterInitStructure.FilterIndex = index;
        if(FDCAN_FilterInitStructure.IdType == FDCAN_STANDARD_ID){
            FDCAN_FilterInitStructure.FilterID1 = rx_id;
            FDCAN_FilterInitStructure.FilterID2 = 0x7ff;
        }else if(FDCAN_FilterInitStructure.IdType == FDCAN_EXTENDED_ID){
            FDCAN_FilterInitStructure.FilterID1 = rx_id;
            FDCAN_FilterInitStructure.FilterID2 = 0x1FFFFFFF;
        }
//        FDCAN_FilterInitStructure.FilterID1 = this->rx_id << 5;//mask模式下只允许过滤这一个目标ID每一个过滤器
//        FDCAN_FilterInitStructure.FilterID1 = 0000;//mask模式下只允许过滤这一个目标ID每一个过滤器
//        FDCAN_FilterInitStructure.FilterID2 = 0000000;//mask模式下只允许过滤这一个目标ID每一个过滤器

        HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_FilterInitStructure);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE);

    }
    taskEXIT_CRITICAL();

}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

    if (hfdcan == &hfdcan1) {
        uint32_t index = rx_header.FilterIndex;
        fdcan_device_t *device = fdcan_device_t::can_device_list[0][index];
        if (device && device->rx_callback) {
            device->rx_callback(rx_data);
            osSemaphoreRelease(device->rx_sem);
        }
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);

    if (hfdcan == &hfdcan2) {

        uint32_t index = rx_header.FilterIndex;
        fdcan_device_t *device = fdcan_device_t::can_device_list[1][index];
        if (device && device->rx_callback) {
            device->rx_callback(rx_data);
            osSemaphoreRelease(device->rx_sem);
        }

    }

    if (hfdcan == &hfdcan3) {

        uint32_t index = rx_header.FilterIndex;
        fdcan_device_t *device = fdcan_device_t::can_device_list[2][index];
        if (device && device->rx_callback) {
            device->rx_callback(rx_data);
            osSemaphoreRelease(device->rx_sem);
        }

    }

}





