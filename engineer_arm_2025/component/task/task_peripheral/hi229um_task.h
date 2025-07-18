//
// Created by 34147 on 2024/4/13.
//

#ifndef ENGINEER_CHASSIS_2024_COMPONENT_TASKS_TASKS_PERIPHERAL_HI229UM_TASK_H_
#define ENGINEER_CHASSIS_2024_COMPONENT_TASKS_TASKS_PERIPHERAL_HI229UM_TASK_H_
#include "drv_hi229um.h"
#include "GlobalCfg.h"
#ifdef __cplusplus
extern "C" {
#endif
//C
void hi229um_task(void *argument);
void hi229um_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size);
#ifdef __cplusplus
}
#endif
//C++

#endif //ENGINEER_CHASSIS_2024_COMPONENT_TASKS_TASKS_PERIPHERAL_HI229UM_TASK_H_
