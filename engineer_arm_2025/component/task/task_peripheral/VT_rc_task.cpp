//
// Created by 26034 on 2025/9/23.
//

#include "VT_rc_task.h"
#include "GlobalCfg.h"

extern rc_device g_rc;

void VT_rc_task(void *argument) {
    static osStatus_t s_stat;

    // HAL_UART_RegisterRxEventCallback(g_rc.vt_huart, vt_rcRxCallBack);
    for (;;)
    {
        //usart_start_receive_dma(g_rc.vt_huart, g_rc.vt_raw_data.buff, VT_RC_BUFF_SIZE);
        s_stat = osSemaphoreAcquire(VTRCUpdateBinarySemHandle, 42);
        if (s_stat == osOK && g_rc.get_dr_lost())
        {
            g_rc.vt_update_data();
            g_rc.update_event();
            g_rc.vt_set_connect();
        }else
        {
            g_rc.vt_set_lost();
        }
        g_rc.update_ready();
        osDelay(1);
    }
}

// void vt_rcRxCallBack(UART_HandleTypeDef *huart, uint16_t Size)
// {
//     osSemaphoreRelease(VTRCUpdateBinarySemHandle);
// }