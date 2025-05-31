//
// Created by 34147 on 2024/1/26.
//

#ifndef ENGINEER_CHASSIS_2024_DRV_IMU_H
#define ENGINEER_CHASSIS_2024_DRV_IMU_H
#ifdef __cplusplus
extern "C" {
#endif
//C
#include "mytype.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "rtos_inc.h"
#include "globalcfg.h"
#include "pid.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#define MPU_DELAY(x) osDelay(x)

#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define IMU_YAW_RANGE 180
#define IMU_PITCH_RANGE 90
#define IMU_ROLL_RANGE 180

#define IMU_TEMP_MAX 41
#define IMU_TEMP_TARGET 40
#define IMU_TEMP_MIN 39

#define GxOFFSET 0.00456584f
#define GyOFFSET 0.0160367f
#define GzOFFSET (-0.00865563f)
#define gNORM 9.87805f

#define MPU6500_ACCEL_8G_SEN 0.002392578125
#define MPU6500_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define MPU6500_TEMP_FACTOR 0.0029951777638
#define MPU6500_TEMP_OFFSET  21

typedef struct {
  float accel[3];

  int16_t mx;
  int16_t my;
  int16_t mz;

  float temperature;
/*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
  float gyro[3];

  float gyro_offset[3];

  float gNorm;
  float TempWhenCali;
  float AccelScale;

  float roll;
  float pitch;
  float yaw;
} imu_data_t;

typedef struct {
  float current_deg;
  float last_deg;
  float round_cnt;
  float total_degree;
  float zero_offset_deg;
} euler_t_degree;
typedef struct {
  euler_t_degree Yaw;
  euler_t_degree Pitch;
  euler_t_degree Roll;
} euler_t;

uint8_t mpu_device_init();
void init_quaternion(imu_data_t *data);
void imu_ahrs_update(imu_data_t *data);
void imu_attitude_update(imu_data_t *data);
void mpu_get_offset(imu_data_t *data);
void mpu6500_get_data();
void ist8310_get_data();
void mpu_get_data(imu_data_t *data);

#ifdef __cplusplus
}
#endif
//C++






class imu_device {
 private:
  euler_t euler;
  imu_data_t data;
  uint8_t offset_cnt;
  bool lost_flag;
  bool zero_offset_flag;
  bool enable_flag;
  bool ready_flag;
  pid temppid;
 public:
  SPI_HandleTypeDef *hspi;
 public:
  uint8_t init(SPI_HandleTypeDef *hspi, uint8_t calibrate);
  void update_data();
  void update_temperature_control();
  void update_ready();
  void set_current_as_offset();
  void enable();
  void disable();
  void get_euler_whx();
  void ahrs_update();
  void attitude_update();
  void get_data();
  void set_lost();
  void set_connected();
  bool check_enable();
  bool check_lost();
  bool check_zero_offset();
  float get_yaw_total_rounds();
 private:
  void update_euler();

};

#endif //ENGINEER_CHASSIS_2024_DRV_IMU_H
