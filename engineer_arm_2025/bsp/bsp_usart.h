/**
  ******************************************************************************
  * @file           : bsp_usart.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef BSP_USART_H_
#define BSP_USART_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "rtos_inc.h"
#include "usart.h"

void usart_send_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len);
void usart_send_buf_dma(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _len);
void usart_idle_receive_buf(UART_HandleTypeDef *_huart, uint8_t *_buf, uint8_t _size);
void usart_start_receive_dma(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t size);
#ifdef __cplusplus
}
#endif
//C++

#endif //BSP_USART_H_
