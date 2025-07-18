/**
  ******************************************************************************
  * @file           : communicate_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "communicate_task.h"
#include "drv_chassis.h"
#include "drv_pump.h"
#include "GlobalCfg.h"

extern chassis_device g_chassis;
extern pump_device g_pump;

communicate_device g_communicate;
void communicate_task(void *argument) {
    osDelay(200); //等待其他任务初始化完成
    g_communicate.init(&COMMUNICATE_CAN,COMMUNICATE_TX_ID,COMMUNICATE_RX_ID,CommunicateUpdateBinarySemHandle);
    for (;;) {
        g_communicate.update(g_chassis,g_pump);
        g_communicate.send_msg();
        osDelay(5);
    }
}