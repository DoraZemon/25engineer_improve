/**
  ******************************************************************************
  * @file           : gimbal_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-1
  ******************************************************************************
  */


#include "gimbal_task.h"
#include "drv_pc.h"
gimbal_device g_gimbal;
uint8_t zph = 0;
extern pc_device g_pc;
void gimbal_task(void *argument) {
    osDelay(200); //等待其他任务初始化完成
    for (;;) {
        if(!g_pc.check_lost()){
            g_gimbal.update_control();
        }
        osDelay(1);
        zph++;
    }
}
