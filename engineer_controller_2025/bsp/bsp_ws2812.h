/**
  ******************************************************************************
  * @file           : bsp_ws2812.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-25
  ******************************************************************************
  */


#ifndef BSP_WS2812_H_
#define BSP_WS2812_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "stm32h7xx_hal.h"
#include "spi.h"
#include "cmsis_os2.h"
#define WS2812_SPI_UNIT     hspi6


#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void ws2812_flashing();
#ifdef __cplusplus
}
#endif
//C++


#endif //BSP_WS2812_H_
