/**
  ******************************************************************************
  * @file           : drv_robot.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-8-1
  ******************************************************************************
  */


#include "drv_robot.h"


void robot_device::update_control(pc_device &pc,
                                  rc_device &rc,
                                  arm_device &arm,
                                  controller_device &controller,
                                  communicate_device &communicate,
                                  hi229um_device &hi229,
                                  gimbal_device &gimbal) {

    if(!controller.check_lost() && controller.raw_data.is_data_valid){
        joint_set.joint1 = controller.raw_data.joint1;
        joint_set.joint2 = controller.raw_data.joint2;
        joint_set.joint3 = controller.raw_data.joint3;
        joint_set.joint4 = controller.raw_data.joint4;
        joint_set.joint5 = controller.raw_data.joint5;
        joint_set.joint6 = controller.raw_data.joint6;
    }
    low_pass(filtered_joint_set.joint1,joint_set.joint1,0.01);
    low_pass(filtered_joint_set.joint2,joint_set.joint2,0.01);
    low_pass(filtered_joint_set.joint3,joint_set.joint3,0.01);
    low_pass(filtered_joint_set.joint4,joint_set.joint4,0.01);
    low_pass(filtered_joint_set.joint5,joint_set.joint5,0.01);
    low_pass(filtered_joint_set.joint6,joint_set.joint6,0.01);

    if(is_ctrl_from_pc){
        pc.enable_pc_ctrl();
    }else{
        pc.disable_pc_ctrl();

        arm.set_arm_ctrl_enable(true);

        gimbal.set_pitch_target(360);

        arm.set_joint1_target(filtered_joint_set.joint1);
        arm.set_joint2_target(filtered_joint_set.joint2);
        arm.set_joint3_target(filtered_joint_set.joint3);
        arm.set_joint4_target(filtered_joint_set.joint4);
        arm.set_joint5_target(filtered_joint_set.joint5);
        arm.set_joint6_target(filtered_joint_set.joint6);

        if(!rc.check_lost()){
            communicate.set_chassis_ctrl(chassis_vel.vel_x * powf(2.0,15.f),chassis_vel.vel_y* powf(2.0,15.f),chassis_vel.vel_spin* powf(2.0,7.f));
        }else{
            communicate.set_chassis_ctrl(0,0,0);
        }
//        communicate.set_chassis_ctrl(0,0,0);

        communicate.set_pump_ctrl(is_arm_pump_on,false,false);

    }
//    communicate.set_pump_ctrl(false,false,false);

}
uint16_t zsz = 0;
void robot_device::do_check_state(rc_device &rc) {
    if(rc.check_sw_state(RC_SW_R_DOWN) && rc.check_sw_state(RC_SW_L_DOWN)){
        is_ctrl_from_pc = false;

        zsz++;
        chassis_vel.vel_x = rc.get_left_rocker_y();
        chassis_vel.vel_y = -rc.get_left_rocker_x();
        chassis_vel.vel_spin = -0.3f*rc.get_right_rocker_x();

        if(rc.check_wheel_state(RC_WHEEL_UP)){
            is_arm_pump_on = false;
        }else if(rc.check_wheel_state(RC_WHEEL_DOWN)){
            is_arm_pump_on = true;
        }
    }else{
        is_ctrl_from_pc = true;
    }
}



void robot_device::do_check_event(rc_device &rc) {

}
