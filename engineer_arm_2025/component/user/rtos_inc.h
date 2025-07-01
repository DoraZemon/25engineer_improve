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
extern osMessageQueueId_t ServoCtrlQueueHandle;
/* --------------信号量-------------------*/

extern osSemaphoreId_t CAN1CountingSemHandle;
extern osSemaphoreId_t CAN2CountingSemHandle;


extern osSemaphoreId_t ArmMotor1UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor2UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor3UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor4UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor5UpdateBinarySemHandle;
extern osSemaphoreId_t ArmMotor6UpdateBinarySemHandle;
extern osSemaphoreId_t RCUpdateBinarySemHandle;
extern osSemaphoreId_t PCUpdateBinarySemHandle;

extern osSemaphoreId_t CustomBinarySemHandle;
extern osSemaphoreId_t judgementInitBinarySemHandle;
extern osSemaphoreId_t customRxBinarySemHandle;//判断自定义控制器是否更新

extern osSemaphoreId_t IMUUpdateBinarySemHandle;
extern osSemaphoreId_t CommunicateUpdateBinarySemHandle;

extern osSemaphoreId_t servoctrlTxBinarySemHandle;


extern osEventFlagsId_t refereeEventHandle;
extern osEventFlagsId_t RefereeEventHandle;

#ifdef __cplusplus
}
#endif
//C++

#endif //RTOS_INC_H_
