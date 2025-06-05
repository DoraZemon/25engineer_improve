/**
  ******************************************************************************
  * @file           : drv_pump.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#ifndef DRV_PUMP_H_
#define DRV_PUMP_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"
#include "adc.h"

class pump_device {

  struct pump_t {
    bool arm;
    bool left;
    bool right;
  };

  struct pump_judge_holding_t {
    uint32_t change_time;
    uint32_t pump_on_time; //泵开启时间
    uint32_t holding_time; //吸住时间
    float measuring_current; //测量电流
    float block_current_max;
    float block_current_min;
    bool is_holding; //是否处于吸住状态
  };
 private:
  uint16_t adc_buff[3];
  uint16_t last_buff[3];
  uint16_t filtered_buff[3];
  ADC_HandleTypeDef *hadc;
  struct {
    pump_t pump_open_state;
    pump_t pump_holding_state;
  } data;
  pump_judge_holding_t arm_pump_judge_holding;
  pump_judge_holding_t left_pump_judge_holding;
  pump_judge_holding_t right_pump_judge_holding;
  bool is_lost = true; //泵 adc 是否正常
 public:
  pump_device(ADC_HandleTypeDef *_hadc);

  void init();

  void start_dma();

  void set_connected();

  void set_lost();

  bool check_lost();

  void update_data();

  void update_control();

  bool check_arm_pump_holding();

  bool check_left_pump_holding();

  bool check_right_pump_holding();


  void set_arm_pump_open_state(bool is_opened);

  void set_left_pump_open_state(bool is_opened);

  void set_right_pump_open_state(bool is_opened);

  void judge_pump_holding(pump_judge_holding_t &pump_judge_holding, uint16_t current, bool is_pump_opened);
};

#endif //DRV_PUMP_H_
