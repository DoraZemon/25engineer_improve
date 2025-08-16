/**
  ******************************************************************************
  * @file           : drv_robot.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-8-1
  ******************************************************************************
  */


#ifndef DRV_ROBOT_H_
#define DRV_ROBOT_H_
#ifdef __cplusplus
extern "C" {
#endif
//C

#ifdef __cplusplus
}
#endif
//C++

#include "drv_pc.h"
#include "drv_rc.h"


class robot_device{
  bool is_ctrl_from_pc = true;

  struct{
    float vel_x;
    float vel_y;
    float vel_spin;
  }chassis_vel;


  bool is_arm_pump_on = true;

  arm_device::joint_t joint_set = {0,0,0,0,0,0};

  arm_device::joint_t filtered_joint_set = {0,0,0,0,0,0};

  bool is_arm_ctrl_enable = true;

 public:
  void update_control(pc_device & pc ,rc_device &rc,
                      arm_device &arm,
                      controller_device &controller,
                      communicate_device &communicate,
                      hi229um_device &hi229, gimbal_device &gimbal);

  void do_check_state(rc_device &rc);

  void do_check_event(rc_device &rc);


};

#endif //DRV_ROBOT_H_
