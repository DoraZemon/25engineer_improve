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
#include "rtos_inc.h"

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
                      false,
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
                      Arm_Motor6_Slave_Id,
                      Arm_Motor6_Master_Id,
                      DM_MIT,
                      false,
                      DM_J4310_2EC,
                      ArmMotor6UpdateBinarySemHandle);

    data.motor_offset = {Arm_Motor1_Offset,
                         Arm_Motor2_Offset,
                         Arm_Motor3_Offset,
                         Arm_Motor4_Offset,
                         Arm_Motor5_Offset,
                         Arm_Motor6_Offset};

    motor.motor1.lqr.reset_lqr(8.0, 3.2, 0.05, 0.0, 0.01, 1.0);
    motor.motor2.lqr.reset_lqr(8.0, 3.0, 0.05, 0.0, 0.01, 1.0);
    motor.motor3.lqr.reset_lqr(3.0, 3.0, 0.05, 0.0, 0.01, 1.0);
    motor.motor4.lqr.reset_lqr(15.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor5.lqr.reset_lqr(30.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor6.lqr.reset_lqr(30.0, 0.1, 0.0, 0.0, 0.0, 1.0);

    data.joint_limit = {
        {Arm_Joint1_Min, Arm_Joint2_Min, Arm_Joint3_Min, Arm_Joint4_Min, Arm_Joint5_Min, Arm_Joint6_Min},
        {Arm_Joint1_Max, Arm_Joint2_Max, Arm_Joint3_Max, Arm_Joint4_Max, Arm_Joint5_Max, Arm_Joint6_Max}};

    data.motor_torque_compensation = {
        Arm_Motor1_Torque_Compensation,
        Arm_Motor2_Torque_Compensation,
        Arm_Motor3_Torque_Compensation,
        Arm_Motor4_Torque_Compensation,
        Arm_Motor5_Torque_Compensation,
        Arm_Motor6_Torque_Compensation
    };

    data.motor_compensation_angle_offset = {
        Arm_Motor1_Compensation_Angle_Offset,
        Arm_Motor2_Compensation_Angle_Offset,
        Arm_Motor3_Compensation_Angle_Offset,
        Arm_Motor4_Compensation_Angle_Offset,
        Arm_Motor5_Compensation_Angle_Offset,
        Arm_Motor6_Compensation_Angle_Offset
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
    motor.motor6.reset_total_rounds_zero_offset(motor.motor6.get_current_round() - data.motor_offset.motor6);
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

void arm_device::update_data() {
    data.motor_pos_get.motor1 = motor.motor1.get_total_rounds();
    data.motor_pos_get.motor2 = motor.motor2.get_total_rounds();
    data.motor_pos_get.motor3 = motor.motor3.get_total_rounds();
    data.motor_pos_get.motor4 = motor.motor4.get_total_rounds();
    data.motor_pos_get.motor5 = motor.motor5.get_total_rounds();
    data.motor_pos_get.motor6 = motor.motor6.get_total_rounds();

    data.joint_states.joint1 = data.motor_pos_get.motor1 * 2 * PI;
    data.joint_states.joint2 = data.motor_pos_get.motor2 * 2 * PI;
    data.joint_states.joint3 = (data.motor_pos_get.motor3-data.motor_pos_get.motor2) * 2 * PI;
    data.joint_states.joint4 = data.motor_pos_get.motor4 * 2 * PI;
    data.joint_states.joint5 = data.motor_pos_get.motor5 * 2 * PI;
    data.joint_states.joint6 = data.motor_pos_get.motor6 * 2 * PI;

}

void arm_device::update_control(bool is_enable) {

    update_data();//更新数据

    if (is_enable_last && !is_enable) {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置
    }

    is_enable_last = is_enable;

    if (is_enable) {//遥控器在线
        data.motor_pos_set.motor1 = data.joint_target.joint1 / (2 * PI);// 关节值与电机位置值的换算关系
        data.motor_pos_set.motor2 = data.joint_target.joint2 / (2 * PI);
        data.motor_pos_set.motor3 = data.joint_target.joint3 / (2 * PI) + data.motor_pos_set.motor2;
        data.motor_pos_set.motor4 = data.joint_target.joint4 / (2 * PI);
        data.motor_pos_set.motor5 = data.joint_target.joint5 / (2 * PI);
        data.motor_pos_set.motor6 = data.joint_target.joint6 / (2 * PI);
    }
#if  ARM_REMOTE_CONTROL_PROTECT
    is_ctrl_enable = is_enable;
#endif
    if (is_ctrl_enable) {
        motor.motor1.set_offset_current(data.motor_torque_compensation.motor1);//力矩补偿可能与关节角度有关，补偿到每个关节再换算到每个电机
        motor.motor2.set_offset_current(data.motor_torque_compensation.motor2 * cosf(motor.motor2.get_total_rounds() * 2 * PI + data.motor_compensation_angle_offset.motor2/180.f * PI) + (data.motor_torque_compensation.motor3 * cosf(motor.motor3.get_total_rounds() * 2 * PI)) * (-1));
        motor.motor3.set_offset_current(data.motor_torque_compensation.motor3 * cosf(motor.motor3.get_total_rounds() * 2 * PI));
        motor.motor4.set_offset_current(data.motor_torque_compensation.motor4);
        motor.motor5.set_offset_current(data.motor_torque_compensation.motor5);
        motor.motor6.set_offset_current(data.motor_torque_compensation.motor6);

        motor.motor1.MIT_inter_set_motor_normalization_torque(motor.motor1.lqr.calculate(data.motor_pos_get.motor1,
                                                                                         motor.motor1.get_speed(),
                                                                                         data.motor_pos_set.motor1,
                                                                                         0.001));
        motor.motor2.MIT_inter_set_motor_normalization_torque(motor.motor2.lqr.calculate(data.motor_pos_get.motor2,
                                                                                            motor.motor2.get_speed(),
                                                                                            data.motor_pos_set.motor2,
                                                                                            0.001));
        motor.motor3.MIT_inter_set_motor_normalization_torque(motor.motor3.lqr.calculate(data.motor_pos_get.motor3,
                                                                                            motor.motor3.get_speed(),
                                                                                            data.motor_pos_set.motor3,
                                                                                            0.001));
        motor.motor4.MIT_inter_set_motor_normalization_torque(motor.motor4.lqr.calculate(data.motor_pos_get.motor4,
                                                                                            motor.motor4.get_speed(),
                                                                                            data.motor_pos_set.motor4,
                                                                                            0.001));
        motor.motor5.MIT_inter_set_motor_normalization_torque(motor.motor5.lqr.calculate(data.motor_pos_get.motor5,
                                                                                            motor.motor5.get_speed(),
                                                                                            data.motor_pos_set.motor5,
                                                                                            0.001));
        motor.motor6.MIT_inter_set_motor_normalization_torque(motor.motor6.lqr.calculate(data.motor_pos_get.motor6,
                                                                                            motor.motor6.get_speed(),
                                                                                            data.motor_pos_set.motor6,
                                                                                            0.001));


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