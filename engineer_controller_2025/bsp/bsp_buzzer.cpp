/**
  ******************************************************************************
  * @file           : bsp_buzzer.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-25
  ******************************************************************************
  */


#include "bsp_buzzer.h"


void buzzer_on() {
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 500);
}

void buzzer_off() {
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
}