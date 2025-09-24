//
// Created by 26034 on 2025/9/23.
//

#include "VT_rc_task.h"
#include "GlobalCfg.h"

rc_device vt_rc(&VT_RC_UART);

void VT_rc_task(void *argument) {
    static osStatus_t s_stat;

    HAL_UART_RegisterRxEventCallback(vt_rc.huart, vt_rcRxCallBack);
    for (;;)
    {
        usart_start_receive_dma(vt_rc.huart, vt_rc.vt_raw_data.buff, VT_RC_BUFF_SIZE);
        s_stat = osSemaphoreAcquire(VTRCUpdateBinarySemHandle, 42);
        if (s_stat == osOK)
        {
            //vt_rc.vt_update_data();
            return;
        }
    }
}

void vt_rcRxCallBack(UART_HandleTypeDef *huart, uint16_t Size)
{
    osSemaphoreRelease(VTRCUpdateBinarySemHandle);
}