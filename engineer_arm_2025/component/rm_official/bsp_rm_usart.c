//
// Created by 34147 on 2024/2/21.
//

#include "bsp_rm_usart.h"

//通信方式是串口，配置为波特率 115200，8 位数据位，1 位停止位，无硬件流控，无校验位

void usart_communicate_config(usart_param_t *usart_param) {
    HAL_UARTEx_ReceiveToIdle_DMA(usart_param->huart,
                                 usart_param->uart_dma_rxdata.buff,
                                 usart_param->uart_dma_rxdata.buff_size);
    __HAL_DMA_DISABLE_IT(usart_param->huart->hdmarx, DMA_IT_TC);
    __HAL_DMA_DISABLE_IT(usart_param->huart->hdmarx, DMA_IT_HT);
}

void usart_dma_send(usart_param_t *usart_param, const uint8_t *pData, uint32_t size) {
    if (usart_param->tx_finish_flag) {
        HAL_UART_Transmit_DMA(usart_param->huart, pData, size);
        __HAL_DMA_DISABLE_IT(usart_param->huart->hdmatx, DMA_IT_HT);
        usart_param->tx_finish_flag = 0;
    }
}