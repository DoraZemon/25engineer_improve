/**
  ******************************************************************************
  * @file           : servo_ctrl_task.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-28
  ******************************************************************************
  */


#ifndef SERVO_CTRL_TASK_H_
#define SERVO_CTRL_TASK_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "usart.h"
void servo_ctrl_task(void *argument);
void servo_ctrl_UartTxCpltCallBack(UART_HandleTypeDef *huart);
#ifdef __cplusplus
}
#endif
//C++
#include "drv_serialservo.h"
#endif //SERVO_CTRL_TASK_H_
