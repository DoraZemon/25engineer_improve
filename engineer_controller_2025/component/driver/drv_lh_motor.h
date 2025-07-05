/**
  ******************************************************************************
  * @file           : drv_lh_motor.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-26
  ******************************************************************************
  */


#ifndef DRV_LH_MOTOR_H_
#define DRV_LH_MOTOR_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "bsp_fdcan.h"
#include "pid.h"
#include "lqr.h"

class lh_motor_device {
 public:
  enum param_data_type {
    INT = 0x01,
    FLOAT = 0x02,
  };

  enum cmd_id {
    CONTROL = 0x0000,
    REQUEST_FEEDBACK = 0x0200,
    FUNCTION_OPERATION = 0x0400,
    PARAMETER_READ = 0x0600,
    PARAMETER_WRITE = 0x0600,
    FEEDBACK_OFFSET = 0x0100,
    ADMINISTRATOR_OFFSET = 0x1000,
  };

  enum state_e {
    DISABLED = 0x00,
    ENABLED = 0x01,
  };

  enum control_mode_e {
    CURRENT = 0x02,
    TORQUE_POSITION = 0x03,
    VELOCITY = 0x04,
    POSITION = 0x05,
    DELTA_POSITION = 0x06,
  };

  typedef struct {
    int64_t round_cnt;
    float total_rounds;
    float total_rounds_without_offset;
    float current_round;
    float last_round;
    float zero_offset_round;

    float current_speed;//经过低通滤波
    float last_speed;//经过低通滤波
    float raw_current_speed;//直接归一化

    int8_t temperature;                 //电机温度℃

    float fb_torque_current;//电机反馈

    float fb_voltage;//电机反馈电压

    float offset_current;//补偿电流

  } lh_motor_data_t;

  struct lh_motor_ctrl_data_t {
    float pos;
    float vel;
    float current;
  };

#pragma pack(1)
  struct lh_motor_feedback_raw_data_t {
    uint8_t data_length; //数据长度
    uint8_t enable_flag; //使能标志位
    uint8_t control_mode; //控制模式
    float pos_ecd; //转子机械角度 编码器值
    float speed;
    float current;
    uint32_t error_code; //错误码
    uint16_t temperature; //电机温度
    uint16_t voltage; //电机反馈电压
    uint8_t limit_flag; //限位标志位
  };

  struct lh_motor_control_data_t {
    uint8_t data_length; //数据长度
    uint8_t enable_flag; //使能标志位
    uint8_t control_mode; //控制模式
    float pos; //目标位置
    float vel; //目标速度
    float current; //目标电流
  };
#pragma pack()

 private:
  uint32_t id;
  control_mode_e control_mode;
  FDCAN_HandleTypeDef *hfdcan;
  bool is_reverse;
  fdcan_device_t fdcan_device; //can数据,初始化

  fdcan_device_t parameter_read_device;

  lh_motor_data_t data;
  lh_motor_ctrl_data_t ctrl_data;

  uint8_t tx_buff[16] = {};

  state_e motor_state = DISABLED;

  lh_motor_feedback_raw_data_t raw_data = {}; //电机原始数据

  uint32_t msg_cnt;

 public:
  bool is_zero_offset = false;

      bool is_lost = true; //电机是否丢失

 bool is_ready = false; //电机是否准备就绪
  pid velpid;
  pid pospid;
  SimpleLQR lqr;



  void init(FDCAN_HandleTypeDef *hfdcan_,
            uint32_t id_,
            bool is_reverse_,
            control_mode_e control_mode,
            osSemaphoreId_t rx_sem_);

  void request_feedback();//请求电机反馈数据

  void send_control();//对电机进行控制

  void function_operation(uint8_t cmd_id);//函数操作

  void parameter_read(uint16_t para_add, float *data);//参数读取

  void parameter_read(uint16_t para_add, int32_t *data);//参数读取

  void parameter_write(uint16_t para_add, int32_t data);//参数写入

  void parameter_write(uint16_t para_add, float data);//参数写入


  void update_data(uint8_t *can_rx_data);//电机反馈数据处理

  void check_motor_for_loss();//检查电机是否丢失

  float get_current_round();

  float get_total_rounds() const;

  float get_speed();

  void reset_total_rounds_zero_offset(float total_rounds);

  void update_ready();

  bool check_ready() {
      return is_ready;
  }
  void set_offset_current(float current) {
      data.offset_current = current;
  }

  void set_current_zero() {
      ctrl_data.current = 0.0f;
  }

  void disable() {
      motor_state = DISABLED;
      ctrl_data.pos = 0.0f;
      ctrl_data.vel = 0.0f;
      ctrl_data.current = 0.0f;
      send_control();
  }

  void enable() {
      motor_state = ENABLED;
      ctrl_data.pos = 0.0f;
      ctrl_data.vel = 0.0f;
      ctrl_data.current = 0.0f;
      send_control();
  }

  void set_control_mode(control_mode_e mode) {
      this->control_mode = mode;
  }

  void set_vel(float set);

  void set_current(float set){
      ctrl_data.current = set + data.offset_current;
  }


 private:
  static void float_to_big_endian_bytes(float value, uint8_t *out);

  static void int32_to_big_endian(int32_t value, uint8_t *out);

  static int32_t big_endian_to_int32(uint8_t *data);

  static float big_endian_to_float(uint8_t *data);

  static uint16_t big_endian_to_uint16(uint8_t *data);

  static uint32_t big_endian_to_uint32(uint8_t *data);

};

#endif //DRV_LH_MOTOR_H_
