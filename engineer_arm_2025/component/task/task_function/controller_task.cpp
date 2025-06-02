/**
  ******************************************************************************
  * @file           : controller_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "controller_task.h"

controller_device g_controller;

void controller_task(void *argument) {
    osDelay(200);
    static osStatus_t controller_status;
    g_controller.init();
    for (;;) {

        controller_status = osSemaphoreAcquire(customRxBinarySemHandle, 50);

        if (controller_status == osOK) {
            g_controller.set_connected();
            g_controller.update();
        } else {
            g_controller.set_lost();
        }
        osDelay(1);
    }
}