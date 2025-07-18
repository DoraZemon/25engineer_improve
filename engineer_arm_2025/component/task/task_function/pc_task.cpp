/**
  ******************************************************************************
  * @file           : pc_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "pc_task.h"


uint8_t pc_raw_data[64];
extern rc_device g_rc;
extern controller_device g_controller;
extern arm_device g_arm;
extern communicate_device g_communicate;
extern gimbal_device g_gimbal;
extern hi229um_device g_hi229um;

pc_device g_pc;
uint32_t flag;

void pc_receive_task(void *argument) {
    static osStatus_t pc_status;
    osSemaphoreAcquire(PCUpdateBinarySemHandle, 0);
    for (;;) {
        pc_status = osSemaphoreAcquire(PCUpdateBinarySemHandle, 10000);

        if (pc_status == osOK) {
            g_pc.set_connected();
            memcpy(&g_pc.rx_data, pc_raw_data, sizeof(g_pc.rx_data));
            g_pc.update_data(g_rc, g_arm, g_controller, g_communicate,g_gimbal);
            flag++;
        } else {
            g_pc.set_lost();
        }
        osDelay(1);
    }
}

void pc_transmit_task(void *argument) {
    osDelay(1000);
    for (;;) {
        g_pc.transmit_data(g_rc, g_arm, g_controller, g_communicate, g_hi229um,g_gimbal);
        osDelay(1); // Delay for demonstration purposes
    }
}