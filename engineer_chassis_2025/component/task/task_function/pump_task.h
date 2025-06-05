/**
  ******************************************************************************
  * @file           : pump_task.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#ifndef PUMP_TASK_H_
#define PUMP_TASK_H_

#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif
//C
void pump_task(void *argument);
void adc_callback(ADC_HandleTypeDef *hadc);
#ifdef __cplusplus
}
#endif
//C++
#include "drv_pump.h"
#include "rtos_inc.h"

#endif //PUMP_TASK_H_
