//
// Created by 34147 on 2023/11/23.
//

#ifndef ENGINEER_CHASSIS_SPD_PLAN_H
#define ENGINEER_CHASSIS_SPD_PLAN_H
#ifdef __cplusplus
extern "C" {
#endif
#include "user_lib.h"
typedef struct {
  float out;//输出
  float acc;//加速增量
  float dec;//减速增量
  float out_max;//输出限幅
  float target;//目标值
  uint32_t current_time;
  uint32_t last_time;

} slope_speed_t;

typedef struct {
  float start_speed;
  float end_speed;
  uint16_t len;
  uint8_t flexible;
  uint32_t cnt;

  uint32_t current_time;
  uint32_t last_time;
} sigmoid_speed_t;

void slope_speed_init(slope_speed_t *slope_plan, float out, float acc, float dec, float out_max, float target);
float get_slope_speed(slope_speed_t *slope_plan);
void update_slope_spd(slope_speed_t *slope, float acc, float dec, float out_max);
void sigmoid_speed_init(sigmoid_speed_t *sigmoid_plan, uint16_t len, uint8_t flexible, uint32_t cnt);
float get_sigmoid_speed(sigmoid_speed_t *sigmoid_plan, uint32_t total_time_ms);

#ifdef __cplusplus
}
#endif
#endif //ENGINEER_CHASSIS_SPD_PLAN_H
