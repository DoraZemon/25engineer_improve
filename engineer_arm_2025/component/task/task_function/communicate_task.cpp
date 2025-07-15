/**
  ******************************************************************************
  * @file           : communicate_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-6
  ******************************************************************************
  */


#include "communicate_task.h"
#include "drv_rc.h"
#include "GlobalCfg.h"

extern rc_device g_rc;

communicate_device g_communicate;

void communicate_task(void *argument) {
    osDelay(200); //等待其他任务初始化完成
    g_communicate.init(&COMMUNICATE_CAN, COMMUNICATE_TX_ID, COMMUNICATE_RX_ID, CommunicateUpdateBinarySemHandle);
    for (;;) {
        g_communicate.update(g_rc);
#if NO_CHASSIS_COMMUNICATE
#else
        g_communicate.send_msg();
#endif
        osDelay(9);
    }
}