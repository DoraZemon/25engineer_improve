//
// Created by 34147 on 2024/2/21.
//

#ifndef ENGINEER_CHASSIS_2024_BSP_RM_USART_H
#define ENGINEER_CHASSIS_2024_BSP_RM_USART_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "usart.h"
#include "rtos_inc.h"
#include "drv_data_fifo.h"

typedef enum {
  UART_IDLE_IT = 0,
  UART_DMA_HALE_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct {
  fifo_s_t *data_fifo;
  uint16_t buff_size;
  uint8_t *buff;
  uint16_t read_index;
  uint16_t write_index;
  uart_it_type_e uart_it_type;
} uart_dma_rxdata_t;

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t tx_finish_flag;
  osEventFlagsId_t event;
  uart_dma_rxdata_t uart_dma_rxdata;
} usart_param_t;

void usart_communicate_config(usart_param_t *usart_param);

void usart_dma_send(usart_param_t *usart_param, const uint8_t *pData, uint32_t size);

#ifdef __cplusplus
}
#endif
//C++

#endif //ENGINEER_CHASSIS_2024_BSP_RM_USART_H
