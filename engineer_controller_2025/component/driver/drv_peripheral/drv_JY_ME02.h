/**
  ******************************************************************************
  * @file           : drv_JY_ME02.h
  * @author         : 34147
  * @brief          : 维特的JY-ME02-CAN编码器的底层
  * @attention      : None
  * @date           : 2024/7/1
  ******************************************************************************
  */



#ifndef ENGINEER_CUSTOM_CONTROLLER_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_JY_ME02_H_
#define ENGINEER_CUSTOM_CONTROLLER_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_JY_ME02_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "user_lib.h"
#include "bsp_can.h"
#define  ENCODER_SAMPLING_TIME  (0.1f)//s

#define UNIVERSAL_HEAD   (0x55)
#define ANGLE_HEAD       (0x55)
#define TEMP_HEAD        (0x56)


//#define SAVE  0x00
//#define RSW   0X02
//#define RRATE 0x03
//#define BAUD  0x04
//#define ENCODER_MODE    0x10
//#define ANG_VEL         0x11
//#define REVOLUTI        0x12
//#define ANGSPE_VAL      0x13
//#define TEMP            0x14
//#define SPIN_DIR        0x15
//#define ANGSPE_SAMPLING_TIME    0x17
//#define DEV_ADDR        0x1A
//#define READREG         0x27
//#define VERSIONL        0x2E
//#define VERSIONH        0x2F

struct JY_ME02_raw_data_t{
  float angle;
  int16_t round_cnt;
  float ang_vel;
  float temperature;
};



class JY_ME02_encoder_device{
 private:
  JY_ME02_raw_data_t raw_data;
  can_device_t can_device;
  bool lost_flag;
 public:
  JY_ME02_encoder_device();
  void init(CAN_HandleTypeDef *_hcan,
            uint32_t rx_id,
            osSemaphoreId_t rx_sem);
  void JY_ME02_rx_data_callback( uint8_t *rx_data);
  float get_angle() const;
  float get_total_angle() const;
  void check_for_loss();
  bool check_lost() const;
};
void JY_ME02_rx_data_callback(can_device_t *can_device, uint8_t *rx_data);
#endif //ENGINEER_CUSTOM_CONTROLLER_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_JY_ME02_H_
