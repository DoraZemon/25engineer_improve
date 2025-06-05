/**
  ******************************************************************************
  * @file           : drv_chassis.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#ifndef DRV_CHASSIS_H_
#define DRV_CHASSIS_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

/*-------------------底盘坐标系的建立---------------------------*/
//                  x
//       LF----    |    ----  RF
//                 |
//                 |
//                 |
//                 |
//    y------------|--------------
//      LB                  RB
// 以图传正下方为坐标系原点，两后轮连线为y轴，从右指向左




#include "bsp_can.h"
#include "drv_dji_motor.h"


struct chassis_velocity_t {
  float speedX;
  float speedY;
  float speedSpin;
};

struct chassis_position_t {
  float positionX;
  float positionY;
  float positionSpin;//基本不用
};

enum chassis_control_type {
  Position = 1,
  Speed
};


class chassis_device {
 private:
  bool enable_flag;
  bool ready_flag;
  bool lost_flag;

  bool super_rotate_flag;

  enum chassis_control_type controlType;

  chassis_velocity_t velocity;
  chassis_position_t position;
  pid pos_rot_pid;

  float pos_yaw_angle;//位置环时的控制角度

  float chassis_yaw_total_angle; //底盘偏航角度
  float chassis_yaw_total_angle_offset; //底盘偏航角度偏置，每次刚开始位置环的时候记录

  CAN_HandleTypeDef *hcan;

 public:
  dji_motor_device wheel[4];
 public:
  chassis_device();

  void init(CAN_HandleTypeDef *hcan);//信号量不能再构造函数中使用

  __RAM_FUNC void update_position_control();

  __RAM_FUNC void update_speed_control();

  void update_data(float yaw_total);

  bool check_init_completely();

  bool check_can_use();

  void update_ready();

  bool check_ready() const;

  bool check_enable() const;

  void set_free();

  void set_speed_x(float x);

  void set_speed_y(float y);

  void set_speed_spin(float spin);

  void add_speed_x(float delta_x);

  void add_speed_y(float delta_y);

  void add_speed_spin(float delta_spin);


  void set_position_x(float x);

  void set_position_y(float y);

  void set_position_spin(float spin);

  void add_position_x(float delta_x);

  void add_position_y(float delta_y);

  void add_position_spin(float delta_spin);

  void disable();

  void enable();

  bool check_position_control();

  bool check_speed_control();

  void open_position_control();

  void open_speed_control();

  bool check_super_rotate() const;

  enum chassis_control_type check_control_type();

  void  check_motor_lost();

  void clean_speed_control();

  void clean_position_control();

  void close_yaw_spin();

  void can_set();
};


#endif //DRV_CHASSIS_H_
