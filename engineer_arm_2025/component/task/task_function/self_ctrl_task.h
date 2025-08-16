/**
  ******************************************************************************
  * @file           : self_ctrl_task.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-8-1
  ******************************************************************************
  */


#ifndef SELF_CTRL_TASK_H_
#define SELF_CTRL_TASK_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
void self_ctrl_task(void *argument);
void self_kb_state_task(void *argument);
void self_kb_event_task(void *argument);
#ifdef __cplusplus
}
#endif
//C++
#include "drv_pc.h"
#include "drv_robot.h"
#endif //SELF_CTRL_TASK_H_
