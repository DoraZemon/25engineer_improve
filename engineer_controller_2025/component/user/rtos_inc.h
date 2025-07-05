/**
  ******************************************************************************
  * @file           : rtos_inc.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#ifndef RTOS_INC_H_
#define RTOS_INC_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "cmsis_os2.h"
extern osMessageQueueId_t FDCAN1SendQueueHandle;
extern osMessageQueueId_t FDCAN2SendQueueHandle;
extern osMessageQueueId_t FDCAN3SendQueueHandle;
/* --------------信号量-------------------*/

extern osSemaphoreId_t FDCAN1CountingSemHandle;//can的发送信号量可释放的个数与cube配置fdcan设置的tx_fifo大小一致
extern osSemaphoreId_t FDCAN2CountingSemHandle;
extern osSemaphoreId_t FDCAN3CountingSemHandle;
extern osSemaphoreId_t TestMotorBinarySemHandle;


extern osSemaphoreId_t ArmMotor1UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor2UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor3UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor4UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor5UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor6UpdateBinarySemHandle;

extern osSemaphoreId_t judgementInitBinarySemHandle;

extern osEventFlagsId_t refereeEventHandle;

#ifdef __cplusplus
}
#endif
//C++

#endif //RTOS_INC_H_
