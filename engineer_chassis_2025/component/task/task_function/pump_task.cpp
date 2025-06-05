/**
  ******************************************************************************
  * @file           : pump_task.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "pump_task.h"

pump_device g_pump(&hadc1);

void pump_task(void *argument) {
    HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, adc_callback);
    g_pump.init();
    g_pump.start_dma();
    osStatus_t s_stat;
    for (;;) {
        g_pump.update_control();
        s_stat = osSemaphoreAcquire(adcUpdateBinarySemHandle, 50);//todo 不知道会不会进回调
        if (s_stat == osOK) {
            g_pump.set_connected();
            g_pump.update_data();
        } else {
            g_pump.set_lost();
        }
        osDelay(2);
    }
}

void adc_callback(ADC_HandleTypeDef *hadc) {
    osSemaphoreRelease(adcUpdateBinarySemHandle);
}