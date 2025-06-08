/**
  ******************************************************************************
  * @file           : pc_task.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#ifndef PC_TASK_H_
#define PC_TASK_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
void pc_receive_task(void *argument);
void pc_transmit_task(void *argument);
#ifdef __cplusplus
}
#endif
//C++
#include "drv_pc.h"

#endif //PC_TASK_H_
