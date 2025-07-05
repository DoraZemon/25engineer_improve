/**
  ******************************************************************************
  * @file           : bsp_ws2812.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-25
  ******************************************************************************
  */


#include "bsp_ws2812.h"


void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++) {
        txbuf[7 - i] = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++) {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

void ws2812_flashing() {
    static uint8_t r = 1, g = 1, b = 1;
    WS2812_Ctrl(r, g, b);
    r++;
    g += 5;
    b += 10;
    osDelay(1);
    r++;
    g++;
    b++;
    osDelay(100);
}