/**
  ******************************************************************************
  * @file           : drv_dm_motor.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 2025/4/21
  ******************************************************************************
  */


#ifndef AERIAL_GIMBAL_2025_COMPONENT_DRIVER_DRV_DM_MOTOR_H_
#define AERIAL_GIMBAL_2025_COMPONENT_DRIVER_DRV_DM_MOTOR_H_
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

enum DM_MOTOR_TYPE {
  DM_J3507_2EC = 0, //默认为
  DM_J4310_2EC = 1,
};

#define RAD2ROUND   (1.0f/(2.0f*PI))

///DM4310
const float DM_J4310_2EC_V_MAX = 30.0f;//速度
const float DM_J4310_2EC_P_MAX = 3.141593f;//位置
const float DM_J4310_2EC_T_MAX = 10.0f;//力矩

const float DM_J4310_2EC_KP_MIN = 0.0f;
const float DM_J4310_2EC_KP_MAX = 500.0f;
const float DM_J4310_2EC_KD_MIN = 0.0f;
const float DM_J4310_2EC_KD_MAX = 5.0f;

///DM3507
const float DM_J3507_2EC_V_MAX = 30.0f;//速度
const float DM_J3507_2EC_P_MAX = 3.1415f;//位置
const float DM_J3507_2EC_T_MAX = 10.0f;//力矩

const float DM_J3507_2EC_KP_MIN = 0.0f;
const float DM_J3507_2EC_KP_MAX = 500.0f;
const float DM_J3507_2EC_KD_MIN = 0.0f;
const float DM_J3507_2EC_KD_MAX = 5.0f;

#pragma pack(1)
typedef struct {
  uint16_t p_des: 16;
  uint16_t v_des: 12;
  uint16_t kp: 12;
  uint16_t kd: 12;
  uint16_t t_ff: 12;
} dm_motor_MIT_can_tx_buff_t;

typedef struct {
  float p_des;
  float v_des;
} dm_motor_PV_can_tx_buff_t;

typedef struct {
  float v_des;
} dm_motor_VO_can_tx_buff_t;
#pragma pack()

typedef struct {
  uint8_t id;
  uint8_t err;
  uint16_t pos_rad;       //转子机械角度 编码器值
  uint16_t vel_rad_s;       //转速以rad/s为单位
  uint16_t torque;    //转矩
  uint8_t t_mos;      //驱动上平均温度℃
  uint8_t t_rotor;    //电机线圈温度℃
} dm_motor_raw_data_t;

typedef struct {
  float p_des;//rad
  float v_des;//rad/s
  float kp;
  float kd;
  float torq;
} dm_motor_tx_t;

typedef enum {
  MOTOR_MODE_NONE = 0,
  DM_MIT,//MIT模式
  DM_PV,//速度位置模式
  DM_VO,//速度模式
} dm_motor_mode_e;

typedef enum {
  DM_MIT_Torque = 0U,    /*!< MIT模式下对力矩进行控制 */
  DM_MIT_Velocity = 1U,     /*!< MIT模式下对速度进行控制 */
  DM_MIT_Position = 2U   /*!< MIT模式下对位置进行控制 */
} dm_motor_mit_mode_e;

typedef enum {
  Disabled = 0U,             /*!< 失能电机，关闭电机 */
  Enabled = 1U,              /*!< 电机没有异常状态，已经使能并可以正常工作 */

  OverVoltage = 0x8,         /*!< 电机超压，电压高于32V，过压将退出“使能模式” */
  UnderVoltage = 0x9,        /*!< 电机欠压，电压低于15V，欠压将退出“使能模式” */
  OverCurrent = 0xA,         /*!< 电机过电流，电流高于9.8A，过流将退出“使能模式” */
  MosOverTemperature = 0xB,  /*!< MOS过温，温度大于120℃将退出“使能模式” */
  CoilOverTemperature = 0xC, /*!< 线圈过温，温度大于120℃将退出“使能模式” */
  CommunicationLos = 0xD,    /*!< 通讯丢失防护，设定周期内没有收到 CAN 指令将自动退出“使能模式” */
  Overload = 0xE             /*!< 电机过载 */
} dm_motor_state_enum;

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

//  float stall_current_max;//检测堵转的电流，无归一化
//  float stall_speed_min;//堵转是速度 归一化
//  uint64_t stall_cnt;//堵转电流判断次数

  float offset_current;//补偿电流

  float *external_speed;  //外置速度  该速度需用户自己更新

} dm_motor_data_t;

struct dm_motor_basic_info_t {
  float v_max;
  float p_max;
  float t_max;
  float kp_min;
  float kp_max;
  float kd_min;
  float kd_max;
};

class dm_motor_device {
 private:
  bool is_reverse;
  uint32_t msg_cnt = 0;
  uint32_t id;
  enum DM_MOTOR_TYPE type;

 public:
  float torque_set;//mit例举控制最后的力矩

  pid velpid;

  pid pospid;

  dm_motor_basic_info_t basic_info;

  float low_pass_alpha = 1.0;//低通滤波器的系数

  bool ready_flag;
  bool is_zero_offset = false;
  bool lost_flag;
  bool stall_flag;
  uint32_t lost_num;

  bool is_enable = false;
  uint32_t slave_id = 0;          //电机ID,slave_id tx
  uint32_t master_id = 0;         //电机ID,master_id rx

  fdcan_device_t fdcan_device;                    //can数据,初始化
  dm_motor_raw_data_t raw_data = {}; //电机原始数据
  dm_motor_tx_t ctrl_data = {}; //电机控制数据
  dm_motor_data_t data = {}; //电机解算数据
  uint8_t tx_buff[8] = {};

  //mode
  dm_motor_mode_e mode = DM_MIT;  //电机模式
  dm_motor_state_enum motor_state = Disabled;
  dm_motor_mit_mode_e mit_mode = DM_MIT_Torque;

  //stall
  uint32_t stall_cnt = 0;
  float stall_torque_max = 0.5f; // torque && speed
  float stall_speed_max = 0.03f; // torque && speed

  float offset_current = 0.0f;
 protected:
  bool is_using_external_speed;
 public:
  dm_motor_device() = default;

  void init(FDCAN_HandleTypeDef *hfdcan,
            uint32_t slaveId,
            uint16_t masterId,
            dm_motor_mode_e motor_mode,
            bool is_reverse,
            DM_MOTOR_TYPE type_,
            osSemaphoreId_t rxSem);

  void set_motor_enable();

  void set_motor_disable();

  void set_motor_clear_error();

  void set_motor_save_zero_offset();

  bool recover_the_motor();

  void set_ctrl_to_can_tx_buff();

  static float uint_to_float(int x_int, float x_min, float x_max, int bits);

  static int float_to_uint(float x, float x_min, float x_max, int bits);

  void MIT_inter_set_motor_round(float rounds);

  void MIT_inter_set_motor_speed(float speed);

  void MIT_inter_set_motor_normalization_torque(float torque);

  void MIT_outer_set_motor_total_rounds(float total_rounds);

  void MIT_outer_set_motor_speed(float speed);

  void MIT_ctrl_position_and_torque(float pos, float torque, float kp);

  void PV_inter_set_motor_total_rounds(float total_rounds, float speed);

  void VO_inter_set_motor_speed(float speed);

  void update_ready();

  bool check_ready() const;

  void set_offset_current(float current);

  void reset_total_rounds_zero_offset(float total_rounds);

  void set_reverse();

  void set_forward();

  void set_toggle();

  void send_can_msg();

  void change_speed_source(float *speed);

  void set_low_pass_alpha(float alpha);

  void update_data(uint8_t *rx_data);

  void set_free();

 public:
  float get_speed(); // 返回电机转子速度 rpm
  float get_speed_without_external();

  float get_total_rounds() const; // 返回电机转子的圈数 实际值
  float get_total_rounds_without_offset() const; // 返回电机转子的圈数 实际值
  float get_current_round();

  void set_vel(float set_vel); // 归一化
  void set_pos(float set_pos);// 归一化
  bool check_lost() const;

  void check_motor_for_loss();

  bool check_reverse() const;

  bool check_stall() const;

  void set_stall_parameter(float _stall_current_max, float _stall_speed_max);
};

#endif //AERIAL_GIMBAL_2025_COMPONENT_DRIVER_DRV_DM_MOTOR_H_
