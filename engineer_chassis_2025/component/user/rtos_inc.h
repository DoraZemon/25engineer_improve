/**
  ******************************************************************************
  * @file           : rtos_inc.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef RTOS_INC_H_
#define RTOS_INC_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "cmsis_os.h"

extern osMessageQueueId_t CAN1SendQueueHandle;
extern osMessageQueueId_t CAN2SendQueueHandle;
/* --------------信号量-------------------*/

extern osSemaphoreId_t CAN1CountingSemHandle;
extern osSemaphoreId_t CAN2CountingSemHandle;


extern osSemaphoreId_t adcUpdateBinarySemHandle;

extern osSemaphoreId_t ChassisMotor1UpdateBinarySemHandle;
extern osSemaphoreId_t ChassisMotor2UpdateBinarySemHandle;
extern osSemaphoreId_t ChassisMotor3UpdateBinarySemHandle;
extern osSemaphoreId_t ChassisMotor4UpdateBinarySemHandle;
extern osSemaphoreId_t IMUUpdateBinarySemHandle;
extern osSemaphoreId_t CommunicateUpdateBinarySemHandle;


#ifdef __cplusplus
}
#endif
//C++

#endif //RTOS_INC_H_
