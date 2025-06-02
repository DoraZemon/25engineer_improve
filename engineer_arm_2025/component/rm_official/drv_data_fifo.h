//
// Created by 34147 on 2024/2/21.
//

#ifndef ENGINEER_CHASSIS_2024_DRV_DATA_FIFO_H
#define ENGINEER_CHASSIS_2024_DRV_DATA_FIFO_H
#ifdef __cplusplus
extern "C" {
#endif
//C

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#define ASSERT(x) do {while(!(x));} while(0)

#define MUTEX_WAIT() \
do {\
  osMutexWait(pfifo->mutex, osWaitForever);\
} while(0)\

#define MUTEX_RELEASE() \
do {\
  osMutexRelease(pfifo->mutex);\
} while(0)\

//! FIFO Memory Model (Single Byte Mode)
typedef struct {
  uint8_t *start_addr;                   //Start Address
  uint8_t *end_addr;                     //End Address
  uint32_t free;                         //The capacity of FIFO
  uint32_t buf_size;                     //Buffer size
  uint32_t used;                         //The number of elements in FIFO
  uint8_t read_index;                   //Read Index Pointer
  uint8_t write_index;                  //Write Index Pointer
  osMutexId_t mutex;
} fifo_s_t;

fifo_s_t *fifo_s_create(uint32_t unit_cnt, osMutexId mutex);
void fifo_s_destory(fifo_s_t *pfifo);

int32_t fifo_s_init(fifo_s_t *pfifo, void *base_addr, uint32_t unit_cnt, osMutexId mutex);

int32_t fifo_s_put(fifo_s_t *pfifo, uint8_t element);
int32_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint32_t number);
int32_t fifo_s_puts_no_mutex(fifo_s_t *pfifo, uint8_t *psource, uint32_t number);

uint8_t fifo_s_get(fifo_s_t *pfifo);
uint8_t fifo_s_get_no_mutex(fifo_s_t *pfifo);

uint16_t fifo_s_gets(fifo_s_t *pfifo, uint8_t *source, uint8_t len);
uint16_t fifo_s_gets_no_mutex(fifo_s_t *pfifo, uint8_t *source, uint8_t len);

uint8_t fifo_s_pre_read(fifo_s_t *pfifo, uint8_t offset);

uint8_t fifo_is_empty(fifo_s_t *pfifo);
uint8_t fifo_is_full(fifo_s_t *pfifo);
uint32_t fifo_used_count(fifo_s_t *pfifo);
uint32_t fifo_free_count(fifo_s_t *pfifo);
uint8_t fifo_flush(fifo_s_t *pfifo);
#ifdef __cplusplus
}
#endif
//C++

#endif //ENGINEER_CHASSIS_2024_DRV_DATA_FIFO_H
