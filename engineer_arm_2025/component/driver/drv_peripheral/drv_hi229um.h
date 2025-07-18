//
// Created by 34147 on 2024/4/13.
//

#ifndef ENGINEER_CHASSIS_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_HI229UM_H_
#define ENGINEER_CHASSIS_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_HI229UM_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "bsp_usart.h"
#include "crc.h"
#include "user_lib.h"
#ifdef __cplusplus
}
#endif
//C++
#define HI229UM_91           0
#define DRV_HI229UM_BUF_SIZE 90

#define CHSYNC1         (0x5A)        //帧头
#define CHSYNC2         (0xA5)
#define HI229UM_YAW_RANGE 180
#define HI229UM_PITCH_RANGE 90
#define HI229UM_ROLL_RANGE 180

#pragma pack(1)
union hi229um_raw {
#if HI229UM_91
  struct {
    struct {
      uint8_t head[2];
      uint16_t len;
      uint16_t crc;
    } frame_head;//6
    uint8_t tag;//1
    uint8_t user_id;//1
    uint16_t reserving_bytes;//2
    float pressure;//气压pa 4
    uint32_t time;//ms 4
    struct {
      float x;
      float y;
      float z;
    } acc;//加速度g 12
    struct {
      float x;
      float y;
      float z;
    } ang_v;//角速度deg/s 12
    struct {
      float x;
      float y;
      float z;
    } magnetic;//磁场uT 12
    struct {
      float roll;//for x
      float pitch;//for y
      float yaw;//for z
    } euler_ang;//欧拉角deg 12
    struct {
      float W;
      float X;
      float Y;
      float Z;
    } quaternion;//四元数 16
  };
  uint8_t buf[DRV_HI229UM_BUF_SIZE];
#else//90
  struct{
      struct {
          uint8_t head[2];
          uint16_t len;
          uint16_t crc;
      }frame_head;//6
      struct {
          uint8_t label;
          uint8_t id;
      }usr_id;//2
      struct {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }acc;//加速度0.001g 7
      struct {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }ang_v;//角速度0.1°/s 7
      struct {
          uint8_t label;
          int16_t x;
          int16_t y;
          int16_t z;
      }magnetic;//磁场强度0.001Gauss 7
      struct {
          uint8_t label;
          int16_t pitch;//for y
          int16_t roll;//for x
          int16_t yaw;//for z
      }euler_ang;//欧拉角0.01° or 0.1° 7
      struct {
          uint8_t label;
          float pressure;
      }air;//气压pa 5
  };
  uint8_t buf[DRV_HI229UM_BUF_SIZE];
#endif
};
#pragma pack()

typedef struct {
  float current_ang;
  float last_ang;
  float delta_ang;//每次角度差值
  int32_t round_cnt;//圈数计算
  float total_rounds;//总圈数
  float zero_offset_deg;
} euler_ang_t;

struct hi229um_data {
  struct {
    float x;
    float y;
    float z;
  } ang_v;//角速度,轴上速度
  struct {
    euler_ang_t pitch;
    euler_ang_t roll;
    euler_ang_t yaw;
  } euler;//欧拉角归一化
  struct {
    float x;
    float y;
    float z;
  } WCS_acc;//惯性系下加速度
  struct {
    float x;
    float y;
    float z;
  } WCS_vel;//惯性系下速度
  struct {
    float x;
    float y;
    float z;
  } WCS_pos;//惯性系下位置  WCS为世界坐标系,应该是世界坐标系更精准一点  inertial是惯性系
  uint32_t msg_cnt;
};

class hi229um_device {
 private:
  bool lost_flag;
  bool zero_offset_flag;
  bool ready_flag;
  UART_HandleTypeDef *huart;
  bool enable_flag;

  hi229um_raw raw_data;
  hi229um_data data;

  float current_time;
  float last_time;
  float delta_t;

 public:
  hi229um_device(UART_HandleTypeDef *huart);
  void init();
  void update_data();
  void update_euler();
  void set_connect();
  void set_lost();
  bool check_lost();
  void enable();
  void disable();
  bool check_legal();

  void set_current_as_offset();

  void update_ready();

  void receive_dma();

  float get_yaw_total_deg();
  bool check_enable();
  bool check_zero_offset();

  void set_nine_axis_mode();
 private:
  bool check_crc_passing(hi229um_raw *raw);
};

#endif //ENGINEER_CHASSIS_2024_COMPONENT_DRIVERS_DRV_PERIPHERAL_DRV_HI229UM_H_
