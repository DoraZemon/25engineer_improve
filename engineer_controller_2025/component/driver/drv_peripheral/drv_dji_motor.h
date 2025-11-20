/**
  ******************************************************************************
  * @file           : drv_dji_motor.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef DRV_DJI_MOTOR_H_
#define DRV_DJI_MOTOR_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "pid.h"
#include "lqr.h"
#include "bsp_can.h"

/*-----------------大疆电机的最大转动速度----------------------*/
#define DJI_MOTOR_MAX_SPEED_M3508 (9600.0f)
#define DJI_MOTOR_MAX_SPEED_M2006 (21000.0f)
#define DJI_MOTOR_MAX_SPEED_GM6020 (320.0f)
/*-----------------大疆电机的最大电流-------------------------*/
#define DJI_MOTOR_MAX_CURRENT_M3508 16384
#define DJI_MOTOR_MAX_CURRENT_M2006 10000
#define DJI_MOTOR_MAX_CURRENT_GM6020 16384

#define ENCODER_TO_ROUND (0.0001220703125f)         // 1/8192.0

enum DJI_MOTOR_TYPE {
  MOTOR_TYPE_NONE = 0,
  DJI_M3508,
  DJI_M2006,
  DJI_GM6020,
};

typedef struct {
  uint16_t encoder;
  int16_t speed_rpm;
  int16_t torque_current;
  uint8_t temperature;
} dji_motor_raw_data_t;

typedef struct {
  int64_t round_cnt;
  float total_rounds;
  float current_speed;//经过低通滤波
  float last_speed;
  float raw_current_speed;

  float current_round;
  float last_round;
  float zero_offset_round;

  int16_t torque_current;

  float stall_current_max;//检测堵转的电流，无归一化
  float stall_speed_min;//堵转是速度 归一化
  uint64_t stall_cnt;//堵转电流判断次数

  float offset_current;//补偿电流

  float *external_speed;  //外置速度  该速度需用户自己更新

} dji_motor_data_t;

class dji_motor_device {
 private:
  bool reverse_flag;
  uint32_t msg_cnt;
  uint32_t id;
  enum DJI_MOTOR_TYPE type;
  static uint8_t can1_tx_buff_0x200[8];
  static uint8_t can1_tx_buff_0x1ff[8];
  static uint8_t can1_tx_buff_0x2ff[8];
  static uint8_t can2_tx_buff_0x200[8];
  static uint8_t can2_tx_buff_0x1ff[8];
  static uint8_t can2_tx_buff_0x2ff[8];
 public:
  dji_motor_raw_data_t raw_data;
  dji_motor_data_t data;
  int16_t current_set;//最后设定的电流值
  pid velpid;
  pid pospid;
  SimpleLQR lqr;

  float low_pass_alpha = 1.0;//低通滤波器的系数

  bool ready_flag;
  bool is_zero_offset;
  bool lost_flag = true;
  bool stall_flag;
  uint32_t lost_num;
 protected:
  can_device_t can_device;
  bool is_using_external_speed;
 public:
  dji_motor_device();

  void init(CAN_HandleTypeDef *_hcan,
            bool reverse_flag,
            uint32_t _id,
            enum DJI_MOTOR_TYPE type,
            osSemaphoreId_t rx_sem, float stall_current = 1.0f, float stall_speed = 0.03f, float offset_current = 0.0f);

  void update_ready();

  bool check_ready() const;

  void set_free();

  void set_current_zero();

  void reset_total_rounds_zero_offset(float total_rounds);

  void set_reverse();

  void set_forward();

  void send_can_msg();

  void set_current_to_can_tx_buff() const;

  void set_pid(pid_param pospid, pid_param velpid);

  void change_speed_source(float *speed);

  void set_low_pass_alpha(float alpha);

  void update_data(uint8_t *rx_data);

  void check_motor_for_stall();//判断电机是否堵转

 public:
  float get_speed(); // 返回电机转子速度 rpm 实际值，不做归一化处理
  float get_total_rounds() const; // 返回电机转子的圈数 实际值，不做归一化处理
  float get_current_round();

  float get_speed_without_external();

  void set_current(float current); // 归一化
  void set_vel(float set_vel); // 归一化
  void set_pos(float set_pos);// 归一化
  bool check_lost() const;

  void check_motor_for_loss();

  bool check_reverse() const;

  bool check_stall() const;

  void set_stall_parameter(int16_t _stall_current_max, float _stall_speed_min);

  void set_offset_current(float offset_current);
};

#endif //DRV_DJI_MOTOR_H_
