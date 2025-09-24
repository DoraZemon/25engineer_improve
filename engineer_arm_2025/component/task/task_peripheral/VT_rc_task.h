//
// Created by 26034 on 2025/9/23.
//

#ifndef ENGINEER_ARM_2025_VT_RC_TASK_H
#define ENGINEER_ARM_2025_VT_RC_TASK_H

#include "drv_rc.h"
#include"bsp_usart.h"

#ifdef __cplusplus
extern "C" {
#endif
    //C
    void VT_rc_task(void *argument);
    void vt_rcRxCallBack(UART_HandleTypeDef *huart, uint16_t Size);
#ifdef __cplusplus
}
#endif
//C++
#endif //ENGINEER_ARM_2025_VT_RC_TASK_H