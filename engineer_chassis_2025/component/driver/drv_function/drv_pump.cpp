/**
  ******************************************************************************
  * @file           : drv_pump.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "drv_pump.h"
#include "GlobalCfg.h"

pump_device::pump_device(ADC_HandleTypeDef *_hadc)
    : hadc(_hadc), adc_buff{0, 0, 0}, last_buff{0, 0, 0}, filtered_buff{0, 0, 0} {
    arm_pump_judge_holding.block_current_min = ARM_SUCKER_BLOCK_CURRENT_MIN;
    arm_pump_judge_holding.block_current_max = ARM_SUCKER_BLOCK_CURRENT_MAX;
    left_pump_judge_holding.block_current_min = LEFT_SUCKER_BLOCK_CURRENT_MIN;
    left_pump_judge_holding.block_current_max = LEFT_SUCKER_BLOCK_CURRENT_MAX;
    right_pump_judge_holding.block_current_min = RIGHT_SUCKER_BLOCK_CURRENT_MIN;
    right_pump_judge_holding.block_current_max = RIGHT_SUCKER_BLOCK_CURRENT_MAX;
}

void pump_device::init() {
}


void pump_device::start_dma() {
    HAL_ADC_Start_DMA(this->hadc, (uint32_t *) this->adc_buff, 3);

}

void pump_device::update_data() {
    for (int i = 0; i < 3; i++) {
        filtered_buff[i] = uint16_t(float(adc_buff[i]) * 0.2f + float(last_buff[i]) * 0.8f);
        last_buff[i] = filtered_buff[i];
    }

    //todo 判断泵是否吸住
    judge_pump_holding(arm_pump_judge_holding, filtered_buff[2], data.pump_open_state.arm);
    judge_pump_holding(left_pump_judge_holding, filtered_buff[1], data.pump_open_state.left);
    judge_pump_holding(right_pump_judge_holding, filtered_buff[0], data.pump_open_state.right);

    data.pump_holding_state.arm = arm_pump_judge_holding.is_holding;
    data.pump_holding_state.left = left_pump_judge_holding.is_holding;
    data.pump_holding_state.right = right_pump_judge_holding.is_holding;
}

void pump_device::judge_pump_holding(pump_device::pump_judge_holding_t &pump_judge_holding,
                                     uint16_t current,
                                     bool is_pump_opened) {
    if (ABS(pump_judge_holding.measuring_current - current) > 0.001f) {
        pump_judge_holding.change_time = HAL_GetTick();
    }

    if (HAL_GetTick() > (pump_judge_holding.change_time + 3000)) {
    } else {
//可以在这里判断lost
        pump_judge_holding.measuring_current = current;

        //pump_On标志位值1距离泵开启还有一段时间，会误判，加以时间判断
        if (1000.0f < pump_judge_holding.measuring_current &&
            (pump_judge_holding.measuring_current < pump_judge_holding.block_current_min)) {
            if (!is_pump_opened) {
                pump_judge_holding.pump_on_time = HAL_GetTick();
            }
            if (is_pump_opened && (HAL_GetTick() > pump_judge_holding.pump_on_time + 500) &&
                HAL_GetTick() > (pump_judge_holding.holding_time + 100)) {
                pump_judge_holding.is_holding = true;
            }
        } else if (pump_judge_holding.measuring_current > pump_judge_holding.block_current_max ||
                   1000.0f > pump_judge_holding.measuring_current) {
            pump_judge_holding.holding_time = HAL_GetTick();
            pump_judge_holding.is_holding = false;
        }


    }
}


void pump_device::update_control() {
    if (this->data.pump_open_state.right) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    }


    if (this->data.pump_open_state.left) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }

    if (this->data.pump_open_state.arm) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    }
}


void pump_device::set_connected() {
    is_lost = false;
}

void pump_device::set_lost() {
    is_lost = true;
}

bool pump_device::check_lost() {
    return is_lost;
}


bool pump_device::check_arm_pump_holding() {
    return data.pump_holding_state.arm;
}

bool pump_device::check_left_pump_holding() {
    return data.pump_holding_state.left;
}

bool pump_device::check_right_pump_holding() {
    return data.pump_holding_state.right;
}


void pump_device::set_arm_pump_open_state(bool is_opened) {
    data.pump_open_state.arm = is_opened;
}

void pump_device::set_left_pump_open_state(bool is_opened) {
    data.pump_open_state.left = is_opened;
}

void pump_device::set_right_pump_open_state(bool is_opened) {
    data.pump_open_state.right = is_opened;
}