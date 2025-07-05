//
// Created by 34147 on 2024/2/22.
//

#ifndef ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
#define ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "bsp_rm_usart.h"
void judgeCtrl_task(void *argument);
void refereeupdate_task(void *argument);
void judge_power_UartRxCallBack(struct __UART_HandleTypeDef *huart, uint16_t Pos);
void judge_power_UartTxCpltCallBack(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif
//C++
#include "drv_data_fifo.h"
#include "drv_judgement.h"
#include "judgement.h"

#define UART_TX_SIGNAL      ( 1 << 2 )
#define UART_IDLE_SIGNAL    ( 1 << 1 )

#define Trans_Remote_Header_First 0xA9
#define Trans_Remote_Header_Second 0x53

#endif //ENGINEER_CHASSIS_2024_TASK_COMMUNICATE_H
