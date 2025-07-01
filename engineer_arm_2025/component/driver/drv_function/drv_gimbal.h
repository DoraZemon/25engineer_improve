/**
  ******************************************************************************
  * @file           : drv_gimbal.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-1
  ******************************************************************************
  */


#ifndef DRV_GIMBAL_H_
#define DRV_GIMBAL_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

#include "drv_serialservo.h"
#include "user_lib.h"
#include "GlobalCfg.h"


#define GIMBAL_PITCH_SERVO_ID                     (1)
#define GIMBAL_PITCH_1000_MAX                    (40.0f)
#define GIMBAL_PITCH_1000_MIN                    (0.0f)
#define GIMBAL_SERVO_PITCH_HORIZONTAL_1000  (940)

#define GIMBAL_SERVO_YAW_FORWARD_1000      (370)
#define GIMBAL_YAW_SERVO_ID                     (2)
#define GIMBAL_YAW_1000_MAX                    (60.0f)
#define GIMBAL_YAW_1000_MIN                    (00.0f)


class gimbal_device{
 private:
   struct {

     int16_t servo_set_pitch_1000;
     int16_t servo_set_yaw_1000;
   }data;


  servo_device pitch_servo; //云台俯仰舵机
    servo_device yaw_servo; //云台偏航舵机

  public:
    gimbal_device();

    void update_control();

    void set_pitch_target(int16_t set);
    void set_yaw_target(int16_t set);

    void send_ctrl();


};

#endif //DRV_GIMBAL_H_
