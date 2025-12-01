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
    if (is_ctrl_from_controller)
    {
        if(!controller.check_lost() && controller.raw_data.is_data_valid){
            joint_set.joint1 = controller.raw_data.joint1;
            joint_set.joint2 = controller.raw_data.joint2;
            joint_set.joint3 = controller.raw_data.joint3;
            joint_set.joint4 = controller.raw_data.joint4;
            joint_set.joint5 = controller.raw_data.joint5;
            joint_set.joint6 = controller.raw_data.joint6;
        }
        else
        {

        }
    }
    else if (is_ctrl_from_joint)
    {
        if (rc.check_ready())
        {
            if (rc.check_sw_state(RC_SW_L_UP))
            {
                joint_set.joint5 += rc.data.right_rocker.y * 0.001f * (-1.f);
                joint_set.joint6 += rc.data.right_rocker.x * 0.001f;
            }
            else if (rc.check_sw_state(RC_SW_L_MID))
            {
                joint_set.joint3 += rc.data.right_rocker.y * 0.001f * (-1.f);
                joint_set.joint4 += rc.data.right_rocker.x * 0.001f;
            }
            else if (rc.check_sw_state(RC_SW_L_DOWN))
            {
                joint_set.joint1 += rc.data.right_rocker.x * 0.001f * (-1.f);
                joint_set.joint2 += rc.data.right_rocker.y * 0.001f;
            }

        }
    }

    VAL_LIMIT(joint_set.joint1 , Arm_Joint1_Min , Arm_Joint1_Max);
    VAL_LIMIT(joint_set.joint2 , Arm_Joint2_Min , Arm_Joint2_Max);
    VAL_LIMIT(joint_set.joint3 , Arm_Joint3_Min , Arm_Joint3_Max);
    VAL_LIMIT(joint_set.joint4 , Arm_Joint4_Min , Arm_Joint4_Max);
    VAL_LIMIT(joint_set.joint5 , Arm_Joint5_Min , Arm_Joint5_Max);
    VAL_LIMIT(joint_set.joint6 , Arm_Joint6_Min , Arm_Joint6_Max);

    low_pass(filtered_joint_set.joint1,joint_set.joint1,0.01);
    low_pass(filtered_joint_set.joint2,joint_set.joint2,0.01);
    low_pass(filtered_joint_set.joint3,joint_set.joint3,0.01);
    low_pass(filtered_joint_set.joint4,joint_set.joint4,0.01);
    low_pass(filtered_joint_set.joint5,joint_set.joint5,0.01);
    low_pass(filtered_joint_set.joint6,joint_set.joint6,0.01);

    if(is_ctrl_from_pc){
        pc.enable_pc_ctrl();
    }else{
        pc.disable_pc_ctrl();//is_ctrl_from_pc = false

        arm.set_arm_ctrl_enable(true);//is_ctrl_enable_from_pc = true

        gimbal.set_pitch_target(360);

        arm.set_joint1_target(filtered_joint_set.joint1);//存数据进data.joint_target.jointn
        arm.set_joint2_target(filtered_joint_set.joint2);
        arm.set_joint3_target(filtered_joint_set.joint3);
        arm.set_joint4_target(filtered_joint_set.joint4);
        arm.set_joint5_target(filtered_joint_set.joint5);
        arm.set_joint6_target(filtered_joint_set.joint6);

        if(!rc.check_lost()){
            communicate.set_chassis_ctrl(chassis_vel.vel_x * powf(2.0,15.f),chassis_vel.vel_y* powf(2.0,15.f),chassis_vel.vel_spin* powf(2.0,7.f));
            communicate.set_pump_ctrl(is_arm_pump_on,false,false);
        }else{
            communicate.set_chassis_ctrl(0,0,0);
            communicate.set_pump_ctrl(false,false,false);
        }
//        communicate.set_chassis_ctrl(0,0,0);



    }
//    communicate.set_pump_ctrl(false,false,false);


}

void robot_device::do_check_state(rc_device &rc) {
    //控制来源切换
    if(rc.check_sw_state(RC_SW_R_DOWN))
    {
        is_ctrl_from_pc = false;
        is_ctrl_from_controller = true;
        is_ctrl_from_joint = false;

        //底盘运动控制
        chassis_vel.vel_x = rc.get_left_rocker_y();
        chassis_vel.vel_y = -rc.get_left_rocker_x();
        chassis_vel.vel_spin = -0.3f*rc.get_right_rocker_x();
        //气泵控制
        if(rc.check_sw_state(RC_SW_L_UP)){
            is_arm_pump_on = true;
        }else{
            is_arm_pump_on = false;
        }
    }
    else if (rc.check_sw_state(RC_SW_R_MID))
    {
        is_ctrl_from_pc = false;
        is_ctrl_from_controller = false;
        is_ctrl_from_joint = true;
        is_arm_pump_on = false;
    }
    else{
        is_ctrl_from_pc = false;
        is_arm_pump_on = false;
    }


}



void robot_device::do_check_event(rc_device &rc) {

}
