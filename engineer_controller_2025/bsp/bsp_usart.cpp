/**
  ******************************************************************************
  * @file           : bsp_usart.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "bsp_usart.h"


/**
 * @brief 串口发送
 * @param _huart
 * @param _buf
 * @param _len
 */
void usart_send_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len) {
    HAL_UART_Transmit(_huart, _buf, _len, 6);
    osDelay(1);
}

/**
 * @brief 串口通过DMA发送
 * @param _huart
 * @param _buf
 * @param _len
 */
void usart_send_buf_dma(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len) {
    HAL_UART_Transmit_DMA(_huart, _buf, _len);
    osDelay(1);
}

/**
 * @brief 串口通过DMA接受，关闭DMA半满中断
 * @param _huart
 * @param _buf
 * @param _size
 */
void usart_idle_receive_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _size) {
    HAL_UARTEx_ReceiveToIdle_IT(_huart, _buf, _size);
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);//关闭半满中断
}

void usart_start_receive_dma(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t size) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, buff, size);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
