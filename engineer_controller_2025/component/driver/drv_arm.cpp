/**
  ******************************************************************************
  * @file           : drv_arm.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-5
  ******************************************************************************
  */


#include "drv_arm.h"


void arm_device::init() {
    motor.motor1.init(&Arm_Motor1_Can, false, Arm_Motor1_Id, DJI_GM6020, ArmMotor1UpdateBinarySemHandle);
    motor.motor2.init(&Arm_Motor2_Can, Arm_Motor2_Id, false, lh_motor_device::CURRENT, ArmMotor2UpdateBinarySemHandle);
    motor.motor3.init(&Arm_Motor3_Can, Arm_Motor3_Id, false, lh_motor_device::CURRENT, ArmMotor3UpdateBinarySemHandle);
    motor.motor4.init(&Arm_Motor4_Can, Arm_Motor4_Id, true, lh_motor_device::CURRENT, ArmMotor4UpdateBinarySemHandle);
    motor.motor5.init(&Arm_Motor5_Can, Arm_Motor5_Id, true, lh_motor_device::CURRENT, ArmMotor5UpdateBinarySemHandle);
    motor.motor6.init(&Arm_Motor6_Can, Arm_Motor6_Id, false, lh_motor_device::CURRENT, ArmMotor6UpdateBinarySemHandle);


    data.motor_offset = {Arm_Motor1_Offset,
                         Arm_Motor2_Offset,
                         Arm_Motor3_Offset,
                         Arm_Motor4_Offset,
                         Arm_Motor5_Offset,
                         Arm_Motor6_Offset};

//    motor.motor1.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
//    motor.motor2.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
//    motor.motor3.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
//    motor.motor4.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
//    motor.motor5.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
//    motor.motor6.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    motor.motor2.velpid.pid_reset(0.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    motor.motor3.velpid.pid_reset(1.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    motor.motor4.velpid.pid_reset(1.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    motor.motor5.velpid.pid_reset(1.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);
    motor.motor6.velpid.pid_reset(0.27f, 0.0f, -0.07f, 0.0f, 0.0f, 0.0f,0.0f,0.0f);


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

//    while (!motor.motor1.is_zero_offset || !motor.motor2.is_zero_offset || !motor.motor3.is_zero_offset ||
//           !motor.motor4.is_zero_offset || !motor.motor5.is_zero_offset || !motor.motor6.is_zero_offset) {
//        osDelay(10);
//    }

    motor.motor2.disable();
    motor.motor3.disable();
    motor.motor4.disable();
    motor.motor5.disable();
    motor.motor6.disable();

    for(int i = 0; i < 15; i++) {
        send_msg();
        osDelay(10);
    }

    motor.motor2.enable();
    motor.motor3.enable();
    motor.motor4.enable();
    motor.motor5.enable();
    motor.motor6.enable();

    for(int i = 0; i < 15; i++) {
        send_msg();
        osDelay(10);
    }

    motor.motor1.reset_total_rounds_zero_offset(motor.motor1.get_current_round() - data.motor_offset.motor1);
    motor.motor2.reset_total_rounds_zero_offset(motor.motor2.get_current_round() - data.motor_offset.motor2);
    motor.motor3.reset_total_rounds_zero_offset(motor.motor3.get_current_round() - data.motor_offset.motor3);
    motor.motor4.reset_total_rounds_zero_offset(motor.motor4.get_current_round() - data.motor_offset.motor4);
    motor.motor5.reset_total_rounds_zero_offset(motor.motor5.get_current_round() - data.motor_offset.motor5);
    motor.motor6.reset_total_rounds_zero_offset(0.f);

}


void arm_device::update_data() {
    data.motor_pos_get.motor1 = motor.motor1.get_total_rounds();
    data.motor_pos_get.motor2 = motor.motor2.get_total_rounds();
    data.motor_pos_get.motor3 = motor.motor3.get_total_rounds();
    data.motor_pos_get.motor4 = motor.motor4.get_total_rounds();//减速比
    data.motor_pos_get.motor5 = motor.motor5.get_total_rounds();
    data.motor_pos_get.motor6 = motor.motor6.get_total_rounds();

    data.joint_states.joint1 = data.motor_pos_get.motor1 * 2 * PI;
    data.joint_states.joint2 = data.motor_pos_get.motor2 * 2 * PI;
    data.joint_states.joint3 = data.motor_pos_get.motor3 * 2 * PI;
    data.joint_states.joint4 = data.motor_pos_get.motor4 * 2 * PI;
    data.joint_states.joint5 = data.motor_pos_get.motor5 * 2 * PI;
    data.joint_states.joint6 = data.motor_pos_get.motor6 * 2 * PI;

    motor.motor1.update_ready();
    motor.motor2.update_ready();
    motor.motor3.update_ready();
    motor.motor4.update_ready();
    motor.motor5.update_ready();
    motor.motor6.update_ready();

    controller_tx_data.joint1 = data.joint_states.joint1;
    controller_tx_data.joint2 = data.joint_states.joint2;
    controller_tx_data.joint3 = data.joint_states.joint3;
    controller_tx_data.joint4 = data.joint_states.joint4;
    controller_tx_data.joint5 = data.joint_states.joint5;
    controller_tx_data.joint6 = data.joint_states.joint6;
    controller_tx_data.is_data_valid = motor.motor1.check_ready() &&
                                       motor.motor2.check_ready() &&
                                       motor.motor3.check_ready() &&
                                       motor.motor4.check_ready() &&
                                       motor.motor5.check_ready() &&
                                       motor.motor6.check_ready();
    controller_tx_data.life_flag = (HAL_GetTick() / 10) % 10; //生命检测标志位，每10ms变化一次
}

void arm_device::update_control(bool is_enable) {

    update_data();//更新数据

    if (is_enable_last && !is_enable) {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置
    }

    is_enable_last = is_enable;


    if (is_ctrl_enable) {
        motor.motor1.set_offset_current(data.motor_torque_compensation.motor1);//力矩补偿可能与关节角度有关，补偿到每个关节再换算到每个电机
        motor.motor2.set_offset_current(data.motor_torque_compensation.motor2 * cosf(
            data.joint_states.joint2 + data.motor_compensation_angle_offset.motor2 / 180.f * PI) +
                                        (data.motor_torque_compensation.motor3 *
                                         cosf((data.joint_states.joint2 + data.joint_states.joint3) * 2 * PI)) * (-1));
        motor.motor3.set_offset_current(
            data.motor_torque_compensation.motor3 * cosf(motor.motor3.get_total_rounds() * 2 * PI));
        motor.motor4.set_offset_current(data.motor_torque_compensation.motor4);
        motor.motor5.set_offset_current(data.motor_torque_compensation.motor5);
        motor.motor6.set_offset_current(data.motor_torque_compensation.motor6);

        motor.motor1.set_current_zero();

        motor.motor2.set_vel(0.0f);
        motor.motor3.set_vel(0.0f);
        motor.motor4.set_vel(0.0f);
        motor.motor5.set_vel(0.0f);
        motor.motor6.set_vel(0.0f);


    } else {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置

        motor.motor1.set_offset_current(0.0);
        motor.motor2.set_offset_current(0.0);
        motor.motor3.set_offset_current(0.0);
        motor.motor4.set_offset_current(0.0);
        motor.motor5.set_offset_current(0.0);
        motor.motor6.set_offset_current(0.0);

        motor.motor1.set_current_zero();
        motor.motor2.set_current_zero();
        motor.motor3.set_current_zero();
        motor.motor4.set_current_zero();
        motor.motor5.set_current_zero();
        motor.motor6.set_current_zero();
    }


}

void arm_device::send_msg() {
//    motor.motor1.send_can_msg();


    static uint32_t hfdcan2_send_cnt = 0;

    static uint32_t hfdcan3_send_cnt = 0;

    if(hfdcan2_send_cnt % 3 == 0){
        motor.motor2.request_feedback();
        motor.motor2.send_control();
    }else if(hfdcan2_send_cnt % 3 == 1) {
        motor.motor3.request_feedback();
        motor.motor3.send_control();
    }else{
        motor.motor4.request_feedback();
        motor.motor4.send_control();
    }

    hfdcan2_send_cnt ++;
    hfdcan2_send_cnt = hfdcan2_send_cnt % 15; //每10次发送一次


    if(hfdcan3_send_cnt % 2 == 0){
        motor.motor5.send_control();
        motor.motor5.request_feedback();

    }else {
        motor.motor6.send_control();
        motor.motor6.request_feedback();
    }

    hfdcan3_send_cnt ++;
    hfdcan3_send_cnt = hfdcan2_send_cnt % 10; //每10次发送一次

}


void arm_device::check_motor_loss() {
    motor.motor1.check_motor_for_loss();
    motor.motor2.check_motor_for_loss();
    motor.motor3.check_motor_for_loss();
    motor.motor4.check_motor_for_loss();
    motor.motor5.check_motor_for_loss();
    motor.motor6.check_motor_for_loss();
}


uint8_t *arm_device::get_controller_tx_data() {
    return (uint8_t *) &controller_tx_data;
}