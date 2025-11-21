/**
  ******************************************************************************
  * @file           : drv_arm.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "drv_arm.h"
#include "arm_math.h"
#include "rtos_inc.h"
#include "GlobalCfg.h"
float k1 = 2.f;
arm_device::arm_device() : joint1_filter(5), joint2_filter(5), joint3_filter(5), joint4_filter(5), joint5_filter(5),
                           joint6_filter(5) {}

void arm_device::init() {
    motor.motor1.init(&Arm_Motor1_Can,
                      Arm_Motor1_Slave_Id,
                      Arm_Motor1_Master_Id,
                      DM_MIT,
                      false,
                      DM_J8009P_2EC,
                      ArmMotor1UpdateBinarySemHandle);
    motor.motor2.init(&Arm_Motor2_Can,
                      Arm_Motor2_Slave_Id,
                      Arm_Motor2_Master_Id,
                      DM_MIT,
                      true,
                      DM_J10010L_2EC,
                      ArmMotor2UpdateBinarySemHandle);
    motor.motor3.init(&Arm_Motor3_Can,
                      Arm_Motor3_Slave_Id,
                      Arm_Motor3_Master_Id,
                      DM_MIT,
                      true,
                      DM_J10010L_2EC,
                      ArmMotor3UpdateBinarySemHandle);
    motor.motor4.init(&Arm_Motor4_Can,
                      Arm_Motor4_Slave_Id,
                      Arm_Motor4_Master_Id,
                      DM_MIT,
                      false,
                      DM_J4310_2EC,
                      ArmMotor4UpdateBinarySemHandle);
    motor.motor5.init(&Arm_Motor5_Can,
                      Arm_Motor5_Slave_Id,
                      Arm_Motor5_Master_Id,
                      DM_MIT,
                      true,
                      DM_J4310_2EC,
                      ArmMotor5UpdateBinarySemHandle);
    motor.motor6.init(&Arm_Motor6_Can,
                      true,
                      Arm_Motor6_Id,
                      DJI_M2006,
                      ArmMotor6UpdateBinarySemHandle);

    data.motor_offset = {Arm_Motor1_Offset,
                         Arm_Motor2_Offset,
                         Arm_Motor3_Offset,
                         Arm_Motor4_Offset,
                         Arm_Motor5_Offset,
                         Arm_Motor6_Offset};

    motor.motor1.lqr.reset_lqr(20.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor2.lqr.reset_lqr(10.0, 0.0, 0.05, 0.0, 0.1, 1.0);
    motor.motor3.lqr.reset_lqr(10.0, 2.0, 0.05, 0.0, 0.1, 1.0);
    motor.motor4.lqr.reset_lqr(20.0, 3.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor5.lqr.reset_lqr(20.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor6.lqr.reset_lqr(0.5, 1.7, 0.0, 0.0, 0.0, 1.0);

    motor.motor1.set_low_pass_alpha(0.01);
    motor.motor2.set_low_pass_alpha(0.01);
    motor.motor3.set_low_pass_alpha(0.01);

    data.joint_limit = {
        {Arm_Joint1_Min, Arm_Joint2_Min, Arm_Joint3_Min, Arm_Joint4_Min, Arm_Joint5_Min, Arm_Joint6_Min},
        {Arm_Joint1_Max, Arm_Joint2_Max, Arm_Joint3_Max, Arm_Joint4_Max, Arm_Joint5_Max, Arm_Joint6_Max}};

    data.joint_torque_limit = {
        {Arm_Joint1_Torque_Min, Arm_Joint2_Torque_Min, Arm_Joint3_Torque_Min, Arm_Joint4_Torque_Min,
         Arm_Joint5_Torque_Min, Arm_Joint6_Torque_Min},
        {Arm_Joint1_Torque_Max, Arm_Joint2_Torque_Max, Arm_Joint3_Torque_Max, Arm_Joint4_Torque_Max,
         Arm_Joint5_Torque_Max, Arm_Joint6_Torque_Max}
    };

    data.motor_pos_limit = {
        {Arm_Motor1_Pos_Min, Arm_Motor2_Pos_Min, Arm_Motor3_Pos_Min, Arm_Motor4_Pos_Min, Arm_Motor5_Pos_Min,
         Arm_Motor6_Pos_Min},
        {Arm_Motor1_Pos_Max, Arm_Motor2_Pos_Max, Arm_Motor3_Pos_Max, Arm_Motor4_Pos_Max, Arm_Motor5_Pos_Max,
         Arm_Motor6_Pos_Max}
    };

    data.motor_torque_compensation = {
    };


    while (!motor.motor1.is_zero_offset || !motor.motor2.is_zero_offset || !motor.motor3.is_zero_offset ||
           !motor.motor4.is_zero_offset || !motor.motor5.is_zero_offset || !motor.motor6.is_zero_offset) {
        osDelay(10);
    }

    motor.motor1.reset_total_rounds_zero_offset(motor.motor1.get_current_round() - data.motor_offset.motor1);
    motor.motor2.reset_total_rounds_zero_offset(motor.motor2.get_current_round() - data.motor_offset.motor2);
    motor.motor3.reset_total_rounds_zero_offset(motor.motor3.get_current_round() - data.motor_offset.motor3);
    motor.motor4.reset_total_rounds_zero_offset(motor.motor4.get_current_round() - data.motor_offset.motor4);
    motor.motor5.reset_total_rounds_zero_offset(motor.motor5.get_current_round() - data.motor_offset.motor5);
    motor.motor6.reset_total_rounds_zero_offset(0.0f);


    is_initial = true;
}

void arm_device::set_joint1_target(float set) {
    data.joint_target.joint1 = set;
    VAL_LIMIT(data.joint_target.joint1, data.joint_limit.min.joint1, data.joint_limit.max.joint1);
}

void arm_device::set_joint2_target(float set) {
    data.joint_target.joint2 = set;
    VAL_LIMIT(data.joint_target.joint2, data.joint_limit.min.joint2, data.joint_limit.max.joint2);
}

void arm_device::set_joint3_target(float set) {
    data.joint_target.joint3 = set;
    VAL_LIMIT(data.joint_target.joint3, data.joint_limit.min.joint3, data.joint_limit.max.joint3);
}

void arm_device::set_joint4_target(float set) {
    data.joint_target.joint4 = set;
    VAL_LIMIT(data.joint_target.joint4, data.joint_limit.min.joint4, data.joint_limit.max.joint4);
}

void arm_device::set_joint5_target(float set) {
    data.joint_target.joint5 = set;
    VAL_LIMIT(data.joint_target.joint5, data.joint_limit.min.joint5, data.joint_limit.max.joint5);
}

void arm_device::set_joint6_target(float set) {
    data.joint_target.joint6 = set;
    VAL_LIMIT(data.joint_target.joint6, data.joint_limit.min.joint6, data.joint_limit.max.joint6);
}

void arm_device::set_joint1_target_torque(float set) {
    data.joint_target_torque.torque_joint1 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint1, data.joint_torque_limit.min.torque_joint1, data.joint_torque_limit.max.torque_joint1);
}

void arm_device::set_joint2_target_torque(float set) {
    data.joint_target_torque.torque_joint2 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint2, data.joint_torque_limit.min.torque_joint2, data.joint_torque_limit.max.torque_joint2);
}

void arm_device::set_joint3_target_torque(float set) {
    data.joint_target_torque.torque_joint3 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint3, data.joint_torque_limit.min.torque_joint3, data.joint_torque_limit.max.torque_joint3);
}

void arm_device::set_joint4_target_torque(float set) {
    data.joint_target_torque.torque_joint4 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint4, data.joint_torque_limit.min.torque_joint4, data.joint_torque_limit.max.torque_joint4);
}

void arm_device::set_joint5_target_torque(float set) {
    data.joint_target_torque.torque_joint5 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint5, data.joint_torque_limit.min.torque_joint5, data.joint_torque_limit.max.torque_joint5);
}

void arm_device::set_joint6_target_torque(float set) {
    data.joint_target_torque.torque_joint6 = set;
    VAL_LIMIT(data.joint_target_torque.torque_joint6, data.joint_torque_limit.min.torque_joint6, data.joint_torque_limit.max.torque_joint6);
}

void arm_device::set_joint1_compensation(float set) {
    data.motor_torque_compensation.motor1 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -50, 50);// 限制力矩补偿范围
}

void arm_device::set_joint2_compensation(float set) {
    data.motor_torque_compensation.motor2 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -50, 50);// 限制力矩补偿范围
}

void arm_device::set_joint3_compensation(float set) {
    data.motor_torque_compensation.motor3 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -50, 50);// 限制力矩补偿范围
}

void arm_device::set_joint4_compensation(float set) {
    data.motor_torque_compensation.motor4 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -5, 5);// 限制力矩补偿范围
}

void arm_device::set_joint5_compensation(float set) {
    data.motor_torque_compensation.motor5 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -5, 5);// 限制力矩补偿范围
}

void arm_device::set_joint6_compensation(float set) {
    data.motor_torque_compensation.motor6 = set;
    VAL_LIMIT(data.motor_torque_compensation.motor1, -5, 5);// 限制力矩补偿范围
}


void arm_device::set_arm_ctrl_enable(bool is_enable) {
    is_ctrl_enable_from_pc = is_enable;
}

void arm_device::update_data() {
    is_lost = motor.motor1.check_lost() || motor.motor2.check_lost() ||
              motor.motor3.check_lost() || motor.motor4.check_lost() ||
              motor.motor5.check_lost() || motor.motor6.check_lost();

    data.motor_pos_get.motor1 = motor.motor1.get_total_rounds();
    data.motor_pos_get.motor2 = motor.motor2.get_total_rounds();
    data.motor_pos_get.motor3 = motor.motor3.get_total_rounds();
    data.motor_pos_get.motor4 = motor.motor4.get_total_rounds();
    data.motor_pos_get.motor5 = motor.motor5.get_total_rounds();
    data.motor_pos_get.motor6 = motor.motor6.get_total_rounds();

    data.joint_states.joint1 = data.motor_pos_get.motor1 * 2 * PI;
    data.joint_states.joint2 = data.motor_pos_get.motor2 * 2 * PI / 3.f * 2.f;
    data.joint_states.joint3 = data.motor_pos_get.motor3 * 2 * PI - data.joint_states.joint2;
    data.joint_states.joint4 = data.motor_pos_get.motor4 * 2 * PI;
    data.joint_states.joint5 = data.motor_pos_get.motor5 * 2 * PI;
    data.joint_states.joint6 = data.motor_pos_get.motor6 * 2 * PI / (36.f * 2);

}

void arm_device::update_gravity_compensation()
{
    float theta1 = data.joint_states.joint2 - PI/2.f;
    float theta2 = data.joint_states.joint3 + PI + 0.28f;
    float theta3 = data.joint_states.joint5;
    float theta4 = data.joint_states.joint4;
    float theta5 = asinf(arm_sin_f32(theta3)*arm_sin_f32(theta4));

    float T_Compensation1_p = - k1*lm1*arm_sin_f32(theta1)*m1 - (l1*arm_sin_f32(theta1) + lm2*arm_sin_f32(theta1 + theta2))*m2 - (l1*arm_sin_f32(theta1) + l2*arm_sin_f32(theta1 + theta2) + lm3*arm_sin_f32(theta1 + theta2 + theta5))*m3;
    T_Compensation1_p *= g;

    float T_Compensation2_p = - lm2*m2*arm_sin_f32(theta1 + theta2) - (l2*arm_sin_f32(theta1 + theta2) + lm3*arm_sin_f32(theta1 + theta2 + theta5))*m3;
    T_Compensation2_p *= g;

    float T_Compensation3_p = - lm3*m3*arm_sin_f32(theta1 + theta2 + theta5)*arm_cos_f32(theta3)*arm_cos_f32(theta4)/sqrt(1 - arm_sin_f32(theta3)*arm_sin_f32(theta3)*arm_cos_f32(theta4)*arm_cos_f32(theta4));
    T_Compensation3_p *= g;

    float T_Compensation_r = - m3*lm3*arm_cos_f32(theta3)*arm_cos_f32(theta4)*arm_sin_f32(theta1 + theta2);
    T_Compensation_r *= g;

    data.motor_torque_compensation.motor1 = 0.f;
    data.motor_torque_compensation.motor2 = T_Compensation1_p;
    data.motor_torque_compensation.motor3 = T_Compensation2_p;
    data.motor_torque_compensation.motor4 = T_Compensation_r;
    data.motor_torque_compensation.motor5 = T_Compensation3_p;
    data.motor_torque_compensation.motor6 = 0.f;
}

void arm_device::update_control(bool is_enable) {

    update_data();//更新数据
    update_gravity_compensation();
    if (is_enable_last && !is_enable) {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置
    }

    is_enable_last = is_enable;

    data.joint_filtered_target.joint1 = joint1_filter.addData(data.joint_target.joint1);
    data.joint_filtered_target.joint2 = joint2_filter.addData(data.joint_target.joint2);
    data.joint_filtered_target.joint3 = joint3_filter.addData(data.joint_target.joint3);
    data.joint_filtered_target.joint4 = joint4_filter.addData(data.joint_target.joint4);
    data.joint_filtered_target.joint5 = joint5_filter.addData(data.joint_target.joint5);
    data.joint_filtered_target.joint6 = joint6_filter.addData(data.joint_target.joint6);

    if (is_enable) {//遥控器在线
#if ARM_DEBUG_MODE
#else
        data.motor_pos_set.motor1 = data.joint_filtered_target.joint1 / (2 * PI);// 关节值与电机位置值的换算关系
        data.motor_pos_set.motor2 = data.joint_filtered_target.joint2 / (2 * PI) / 2.f * 3.f;
        data.motor_pos_set.motor3 =
            data.joint_filtered_target.joint3 / (2 * PI) + data.joint_filtered_target.joint2 / (2 * PI);
        data.motor_pos_set.motor4 = data.joint_filtered_target.joint4 / (2 * PI);
        data.motor_pos_set.motor5 = data.joint_filtered_target.joint5 / (2 * PI);
        data.motor_pos_set.motor6 = data.joint_filtered_target.joint6 / (2 * PI) * (36.f * 2.f);
#endif
    }
#if ARM_REMOTE_CONTROL_PROTECT
    is_ctrl_enable = is_enable;
#endif

    limit_motor_pos();//限制电机位置

    if (is_ctrl_enable && is_ctrl_enable_from_pc && !is_ctrl_disable_to_reset_pitch) {

        motor.motor1.set_offset_current(data.motor_torque_compensation.motor1 / motor.motor1.basic_info.t_max);
        motor.motor2.set_offset_current(
            data.motor_torque_compensation.motor2 / motor.motor2.basic_info.t_max / 3.f * 2.f);
        motor.motor3.set_offset_current(data.motor_torque_compensation.motor3 / motor.motor3.basic_info.t_max);
        motor.motor4.set_offset_current(data.motor_torque_compensation.motor4 / motor.motor4.basic_info.t_max);
        motor.motor5.set_offset_current(data.motor_torque_compensation.motor5 / motor.motor5.basic_info.t_max);

        motor.motor1.MIT_inter_set_motor_normalization_torque(motor.motor1.lqr.calculate(data.motor_pos_get.motor1,
                                                                                         motor.motor1.get_speed(),
                                                                                         data.motor_pos_set.motor1,
                                                                                         0.002));
        motor.motor2.MIT_inter_set_motor_normalization_torque(motor.motor2.lqr.calculate(data.motor_pos_get.motor2,
                                                                                         motor.motor2.get_speed(),
                                                                                         data.motor_pos_set.motor2,
                                                                                         0.002));
        motor.motor3.MIT_inter_set_motor_normalization_torque(motor.motor3.lqr.calculate(data.motor_pos_get.motor3,
                                                                                         motor.motor3.get_speed(),
                                                                                         data.motor_pos_set.motor3,
                                                                                         0.002));
        motor.motor4.MIT_inter_set_motor_normalization_torque(motor.motor4.lqr.calculate(data.motor_pos_get.motor4,
                                                                                         motor.motor4.get_speed(),
                                                                                         data.motor_pos_set.motor4,
                                                                                         0.002));
        motor.motor5.MIT_inter_set_motor_normalization_torque(motor.motor5.lqr.calculate(data.motor_pos_get.motor5,
                                                                                         motor.motor5.get_speed(),
                                                                                         data.motor_pos_set.motor5,
                                                                                         0.002));
        motor.motor6.set_current(motor.motor6.lqr.calculate(data.motor_pos_get.motor6,
                                                            motor.motor6.get_speed(),
                                                            data.motor_pos_set.motor6,
                                                            0.002));

        //保护
        // motor.motor1.MIT_inter_set_motor_normalization_torque(0);
        // motor.motor2.MIT_inter_set_motor_normalization_torque(0);
        // motor.motor3.MIT_inter_set_motor_normalization_torque(0);
        // motor.motor4.MIT_inter_set_motor_normalization_torque(0);
        // motor.motor5.MIT_inter_set_motor_normalization_torque(0);
        // motor.motor6.set_current(0);


    } else if (is_ctrl_disable_to_reset_pitch) {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置

        motor.motor1.set_offset_current(0.0);
        motor.motor2.set_offset_current(Arm_Pitch_Reset_Motor2_Torque);
        motor.motor3.set_offset_current(Arm_Pitch_Reset_Motor3_Torque);
        motor.motor4.set_offset_current(0.0);
        motor.motor5.set_offset_current(0.0);
        motor.motor6.set_offset_current(0.0);

        motor.motor2.MIT_inter_set_motor_normalization_torque(0);
        motor.motor3.MIT_inter_set_motor_normalization_torque(0);

        motor.motor1.set_free();
        motor.motor4.set_free();
        motor.motor5.set_free();
        motor.motor6.set_free();
    } else {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置

        motor.motor1.set_offset_current(0.0);
        motor.motor2.set_offset_current(0.0);
        motor.motor3.set_offset_current(0.0);
        motor.motor4.set_offset_current(0.0);
        motor.motor5.set_offset_current(0.0);
        motor.motor6.set_offset_current(0.0);

        motor.motor1.set_free();
        motor.motor2.set_free();
        motor.motor3.set_free();
        motor.motor4.set_free();
        motor.motor5.set_free();
        motor.motor6.set_free();
    }
}


void arm_device::send_msg() {
    motor.motor1.send_can_msg();
    motor.motor2.send_can_msg();
    motor.motor3.send_can_msg();
    motor.motor4.send_can_msg();
    motor.motor5.send_can_msg();
    motor.motor6.send_can_msg();
}


void arm_device::check_motor_loss() {
    motor.motor1.check_motor_for_loss();
    motor.motor2.check_motor_for_loss();
    motor.motor3.check_motor_for_loss();
    motor.motor4.check_motor_for_loss();
    motor.motor5.check_motor_for_loss();
    motor.motor6.check_motor_for_loss();
}


void arm_device::recover_dm() {
    motor.motor1.recover_the_motor();
    motor.motor2.recover_the_motor();
    motor.motor3.recover_the_motor();
    motor.motor4.recover_the_motor();
    motor.motor5.recover_the_motor();

}


void arm_device::limit_motor_pos() {
    if (isnan(data.motor_pos_set.motor1)) {
        data.motor_pos_set.motor1 = data.motor_pos_get.motor1;
    }

    if (isnan(data.motor_pos_set.motor2)) {
        data.motor_pos_set.motor2 = data.motor_pos_get.motor2;
    }

    if (isnan(data.motor_pos_set.motor3)) {
        data.motor_pos_set.motor3 = data.motor_pos_get.motor3;
    }

    if (isnan(data.motor_pos_set.motor4)) {
        data.motor_pos_set.motor4 = data.motor_pos_get.motor4;
    }

    if (isnan(data.motor_pos_set.motor5)) {
        data.motor_pos_set.motor5 = data.motor_pos_get.motor5;
    }

    if (isnan(data.motor_pos_set.motor6)) {
        data.motor_pos_set.motor6 = data.motor_pos_get.motor6;
    }

    VAL_LIMIT(data.motor_pos_set.motor1, data.motor_pos_limit.min.motor1, data.motor_pos_limit.max.motor1);
    VAL_LIMIT(data.motor_pos_set.motor2, data.motor_pos_limit.min.motor2, data.motor_pos_limit.max.motor2);
    VAL_LIMIT(data.motor_pos_set.motor3, data.motor_pos_limit.min.motor3, data.motor_pos_limit.max.motor3);
    VAL_LIMIT(data.motor_pos_set.motor4, data.motor_pos_limit.min.motor4, data.motor_pos_limit.max.motor4);
    VAL_LIMIT(data.motor_pos_set.motor5, data.motor_pos_limit.min.motor5, data.motor_pos_limit.max.motor5);
    VAL_LIMIT(data.motor_pos_set.motor6, data.motor_pos_limit.min.motor6, data.motor_pos_limit.max.motor6);

    //todo pitch2电机限位
}


bool arm_device::check_lost() {
    return is_lost;
}

bool arm_device::check_initial() {
    return is_initial;
}

void arm_device::set_to_reset_pitch(bool is_enable) {
    is_to_reset_pitch = is_enable;
}

void arm_device::check_reset_pitch() {
    if (is_to_reset_pitch && !last_is_to_reset_pitch) {
        is_ctrl_disable_to_reset_pitch = true;
        osDelay(2000);
        motor.motor2.reset_total_rounds_zero_offset(-0.01f);
        motor.motor3.reset_total_rounds_zero_offset(0);
        osDelay(500);
        is_ctrl_disable_to_reset_pitch = false;
        is_to_reset_pitch = false;
        reset_pitch_num++;
    }

    last_is_to_reset_pitch = is_to_reset_pitch;
}