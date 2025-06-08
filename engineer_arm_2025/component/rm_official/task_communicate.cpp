//
// Created by 34147 on 2024/2/22.
//
#include "drv_controller.h"
#include "task_communicate.h"
//#include "drv_ui.h"
#include "GlobalCfg.h"

const osMutexAttr_t judge_power_rxdata_mutex_attr = {
    .name = "judge_power_rxdata_mutex"
};

const osMutexAttr_t judge_transfer_rxdata_mutex_attr = {
    .name = "judge_transfer_rxdata_mutex"
};

const osMutexAttr_t judge_power_txdata_mutex_attr = {
    .name = "judge_power_txdata_mutex"
};

const osMutexAttr_t judge_transfer_txdata_mutex_attr = {
    .name = "judge_transfer_txdata_mutex"
};

judgement_device g_judgement_power;//电管
judgement_device g_judgment_transfer;//图传
//ui_device g_ui;

extern controller_device g_controller;

//读+写（电管）
void judgeCtrl_task(void *argument) {
    uint32_t event;
    /* config & open judge uart receive and transmit it */
    g_judgement_power.init(&JUDGEMENT_POWER_UART,
                           refereeEventHandle,
                           &judge_power_rxdata_mutex_attr,
                           &judge_power_txdata_mutex_attr);
    /* 注册回调函数 */
    HAL_UART_RegisterCallback(g_judgement_power.usart.huart,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              judge_power_UartTxCpltCallBack);
    HAL_UART_RegisterRxEventCallback(g_judgement_power.usart.huart, judge_power_UartRxCallBack);
    osSemaphoreRelease(judgementInitBinarySemHandle);
    while (1) {
        event = osEventFlagsWait(refereeEventHandle,
                                 UART_TX_SIGNAL | UART_IDLE_SIGNAL,
                                 osFlagsWaitAny,
                                 osWaitForever);
        //receive judge data puts fifo
        if (event & UART_IDLE_SIGNAL) { //如果收到rx空闲信号,则开始处理数据
//            dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
//            dma_buffer_to_unpack_buffer(&judgement_usart.uart_dma_rxdata, UART_IDLE_IT);
            g_judgement_power.unpack_fifo_data(&g_judgement_power.judge_unpack_obj, DN_REG_ID);

            continue;
        }

        //send data to judge system for UI design
        if (event & UART_TX_SIGNAL) {   //如果收到可以发送的信号,则开始发送数据
            // UI 或者通信
            g_judgement_power.send_packed_fifo_data(&g_judgement_power.judge_txdata_fifo, DN_REG_ID);//把队列中存储的数据用串口发送
            continue;
            //机器人数据交互
//            robot_communicate_send_data(0x02F0,BLUE_INFANTRY4_ID,BLUE_HERO_ID,p_data,3);
            //雷达发送位置消息
            //lidar_send_init();
            //lidar_data_pack(0x0301,lidar_data,lidar_robot_interactive);
        }
    }
}

void judge_power_UartRxCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos) {
    g_judgement_power.usart_rx_processed(Pos);
}

void judge_power_UartTxCpltCallBack(UART_HandleTypeDef *huart) {
    g_judgement_power.usart.tx_finish_flag = 1;
}

#if DRAW_UI

//用来存储绘制UI数据
//void updateUIData_task(void *argument) {
//    osDelay(1000);
//    osSemaphoreAcquire(judgementInitBinarySemHandle, osWaitForever);//防止未初始化即进入
//    osDelay(2000);
//    g_ui.ptr_init(&g_robot, &g_judgement_power);
//    for (;;) {
//        g_ui.update_data_send();
//        osEventFlagsSet(refereeEventHandle, UART_TX_SIGNAL);
//        osDelay(20);
//    }
//}

#endif

void judge_transfer_UartRxCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos) {
    if (g_judgment_transfer.usart.uart_dma_rxdata.buff[0] == Trans_Remote_Header_First &&
        g_judgment_transfer.usart.uart_dma_rxdata.buff[1] == Trans_Remote_Header_Second) {
        return;
    }
    g_judgment_transfer.usart_rx_processed(Pos);

}

void judge_transfer_UartTxCpltCallBack(UART_HandleTypeDef *huart) {
    g_judgment_transfer.usart.tx_finish_flag = 1;
}

void receiveCtrl_task(void *argument) {
    uint32_t event;
    g_judgment_transfer.init(&JUDGEMENT_TRANSFER_UART,
                             RefereeEventHandle,
                             &judge_transfer_rxdata_mutex_attr,
                             &judge_transfer_txdata_mutex_attr);
    HAL_UART_RegisterCallback(g_judgment_transfer.usart.huart,
                              HAL_UART_TX_COMPLETE_CB_ID,
                              judge_transfer_UartTxCpltCallBack);
    HAL_UART_RegisterRxEventCallback(g_judgment_transfer.usart.huart, judge_transfer_UartRxCallBack);
    for (;;) {
        event = osEventFlagsWait(RefereeEventHandle,
                                 UART_IDLE_SIGNAL,
                                 osFlagsWaitAny,
                                 osWaitForever);
        if (event & UART_IDLE_SIGNAL) {//rx空闲中断,若无rx则不会进入此中断
            g_judgment_transfer.unpack_fifo_data(&g_judgment_transfer.judge_unpack_obj, DN_REG_ID);//获得数据
            if (g_controller.judge_transfer_rx_callback((custom_judge_raw_msg *) g_judgment_transfer.judge_rece_mesg.custom_interactive_data.data)) {//记录数据，判断数据是否有效
                osSemaphoreRelease(customRxBinarySemHandle);
            }
            continue;
        }
    }
}