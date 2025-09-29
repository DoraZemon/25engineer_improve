/**
  ******************************************************************************
  * @file           : rc_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-1
  ******************************************************************************
  */


#include "rc_task.h"
#include "GlobalCfg.h"

rc_device g_rc(&RC_UART,&VT_RC_UART);

void rc_task(void *argument) {
    static osStatus_t s_stat;

    HAL_UART_RegisterRxEventCallback(g_rc.huart, rcRxCallBack);
    for (;;) {
        usart_start_receive_dma(g_rc.huart, g_rc.raw_data.buff, DR16_BUFF_SIZE);
        s_stat = osSemaphoreAcquire(RCUpdateBinarySemHandle, 20);
        if (s_stat == osOK) {
            g_rc.update_data();
            g_rc.update_event();
            g_rc.set_connect();

        } else {
            g_rc.set_lost();
        }
        g_rc.update_ready();
        osDelay(1);
    }
}

void rcRxCallBack(UART_HandleTypeDef *huart, uint16_t Size) {
    osSemaphoreRelease(RCUpdateBinarySemHandle);
}