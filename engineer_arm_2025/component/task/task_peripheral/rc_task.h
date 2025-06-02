/**
  ******************************************************************************
  * @file           : rc_task.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#ifndef RC_TASK_H_
#define RC_TASK_H_

#include "drv_rc.h"
#include"bsp_usart.h"

#ifdef __cplusplus
extern "C" {
#endif
//C
void rc_task(void *argument);
void rcRxCallBack(UART_HandleTypeDef *huart, uint16_t Size);
#ifdef __cplusplus
}
#endif
//C++
#endif //RC_TASK_H_
