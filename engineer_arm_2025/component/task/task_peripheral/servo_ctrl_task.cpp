/**
  ******************************************************************************
  * @file           : servo_ctrl_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-28
  ******************************************************************************
  */


#include "servo_ctrl_task.h"
#include "GlobalCfg.h"

#if SERVO
void servo_ctrl_task(void *argument) {
    HAL_UART_RegisterCallback(&SERVO_UART, HAL_UART_TX_COMPLETE_CB_ID, servo_ctrl_UartTxCpltCallBack);
    osSemaphoreRelease(servoctrlTxBinarySemHandle);
    for (;;) {
        servo_ctrl_data_t ctrl_data{};
        osSemaphoreAcquire(servoctrlTxBinarySemHandle, osWaitForever);
        osMessageQueueGet(ServoCtrlQueueHandle, &ctrl_data, 0, osWaitForever);
        LobotSerialServoMoveSet(&servo_device::servos, ctrl_data.id, ctrl_data.set_1000, 0);
    }
}

void servo_ctrl_UartTxCpltCallBack(UART_HandleTypeDef *huart) {
    osSemaphoreRelease(servoctrlTxBinarySemHandle);
}

#endif