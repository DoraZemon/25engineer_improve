//
// Created by 34147 on 2024/4/13.
//

#include "hi229um_task.h"

#if HI229UM
hi229um_device g_hi229um(&HI229UM_UART);
uint64_t lost_num = 0;

void hi229um_task(void *argument) {
    static osStatus_t s_stat;
//    g_hi229um.init();
    osSemaphoreAcquire(hi229umRxBinarySemHandle, 0);
    HAL_UART_RegisterRxEventCallback(&HI229UM_UART, hi229um_RxCallBack);
//    g_hi229um.set_nine_axis_mode();
    for (;;) {
//        if(g_hi229um.check_lost()){
//            g_hi229um.set_nine_axis_mode();
//        }
        g_hi229um.receive_dma();
        s_stat = osSemaphoreAcquire(hi229umRxBinarySemHandle, 20);
        if (s_stat == osOK) {
            g_hi229um.set_connect();
            if (g_hi229um.check_legal()) {
                g_hi229um.update_data();
            }
        } else {
            lost_num ++;
            g_hi229um.set_lost();
        }
        g_hi229um.update_ready();
        osDelay(1);
    }

}

void hi229um_RxCallBack(UART_HandleTypeDef *huart, uint16_t Size) {
    osSemaphoreRelease(hi229umRxBinarySemHandle);
}

#endif