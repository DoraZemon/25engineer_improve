//
// Created by 34147 on 2023/11/13.
//

#ifndef ENGINEER_CHASSIS_USER_LIB_H
#define ENGINEER_CHASSIS_USER_LIB_H

#ifdef __cplusplus
extern "C" {
#endif
//C
#include "stm32h7xx_hal.h"
#include "string.h"
#include <math.h>

typedef struct {
  GPIO_TypeDef *GPIOx;
  uint16_t GPIO_Pin;
} GPIO_t;

void val_limit(float *x, float min, float max);
void abs_limit(float *x, float abs_max);
uint32_t get_time_us();
uint32_t get_time_ms(void);
float get_time_ms_us();
uint16_t unsigned_16(uint8_t *p);
void low_pass(float &target, float current, float alpha);
#ifdef __cplusplus
}
#endif
//C++

#define PI 3.14159265358979f
#define DEC_CON (float) PI/180

#define ABS(x) ((x)>=0?(x):-(x))

#define HIGH_BYTE(x) ((uint8_t)(((x) & 0xff00) >> 8))
#define LOW_BYTE(x) ((uint8_t)((x) & 0x00ff))
#define MERGE_BYTES(high, low) (((uint16_t) (high) << 8) | (uint16_t)(low))

/**
 * @brief 将val限制在min和max之间
 */
#define VAL_LIMIT(val, min, max) \
    do                         \
    {                          \
        if ((val) <= (min))    \
        {                      \
            (val) =(min);      \
        }                      \
        else if((val)>=(max))  \
        {                      \
            (val)=(max);       \
        }                      \
                               \
    } while(0)

#define ABS_LIMIT(val, limit) \
  do                             \
  {                              \
    if ((val) <= -(limit))          \
    {                            \
      (val) = -(limit);             \
    }                            \
    else if ((val) >= (limit))     \
    {                            \
      (val) = (limit);             \
    }                            \
  } while (0)
/**
 * @brief 知道一个结构体的子变量的地址，这个结构体的名称和这个子变量在结构体的名称，可以返回结构体变量的基地址
 * @param 结构体的子变量的地址
 * @param 结构体的名称
 * @param 子变量在结构体的名称
 */
#define container_of(ptr, type, member) ({ \
    const typeof(((type *)0)->member) *__mptr = (ptr) ; \
    (type *)((char *)__mptr - offsetof(type,member)) ;})

float limited_val(float val, float min, float max);

#endif //ENGINEER_CHASSIS_USER_LIB_H
