/**
  ******************************************************************************
  * @file           : drv_dm_imu.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef DRV_DM_IMU_H_
#define DRV_DM_IMU_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++
#include "bsp_can.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN    (-58.8f)
#define GYRO_CAN_MAX    (34.88f)
#define GYRO_CAN_MIN    (-34.88f)
#define PITCH_CAN_MAX    (90.0f)
#define PITCH_CAN_MIN    (-90.0f)
#define ROLL_CAN_MAX    (180.0f)
#define ROLL_CAN_MIN    (-180.0f)
#define YAW_CAN_MAX        (180.0f)
#define YAW_CAN_MIN    (-180.0f)
#define TEMP_MIN            (0.0f)
#define TEMP_MAX            (60.0f)
#define Quaternion_MIN    (-1.0f)
#define Quaternion_MAX    (1.0f)

#define YAW_RANGE 180
#define PITCH_RANGE 90
#define ROLL_RANGE 180

typedef struct {
  float current_ang;
  float last_ang;
  float delta_ang;//每次角度差值
  int32_t round_cnt;//圈数计算
  float total_rounds;//总圈数
  float zero_offset_deg;
} euler_ang_t;

class dm_imu_device {
  enum imu_rx_data_frame_head {
    Accel = 1,
    Gyro = 2,
    Euler = 3,
    Quaternion = 4,
  };
  struct imu_data_t {
    float pitch;
    float roll;
    float yaw;

    struct {
      euler_ang_t pitch;
      euler_ang_t roll;
      euler_ang_t yaw;
    } euler;//欧拉角归一化

    float pitch_cnt;
    float yaw_cnt;
    float roll_cnt;

    float gyro[3];
    float accel[3];

    float q[4];

    float cur_temp;
  };
 public:
  CAN_HandleTypeDef *hcan;
  uint32_t can_id;
  uint32_t rx_id;
  uint8_t cmd[4];
  can_device_t can_device;
  bool is_using_quaternion;//是否使用四元数
  bool is_using_accel;
  imu_data_t data;
  bool is_lost;
  int64_t lost_num;

  bool is_zero_offset = false;
  uint32_t msg_cnt = 0;

  void update_data(uint8_t *rx_data);

  void init(CAN_HandleTypeDef *hcan_, uint32_t canid, uint32_t master_id, osSemaphoreId_t rxsem);

  void update_accel(uint8_t *pdata);

  void update_gyro(uint8_t *pdata);

  void update_euler(uint8_t *pdata);

  void update_euler_cnt();

  void update_quaternion(uint8_t *pdata);

  void request_imu_data();

  float get_pitch_raw();

  float get_roll();

  float get_yaw();

  float get_gyro_x();

  float get_gyro_y();

  float get_gyro_z();

  void check_imu_for_loss();

  bool check_lost();
};

#endif //DRV_DM_IMU_H_
