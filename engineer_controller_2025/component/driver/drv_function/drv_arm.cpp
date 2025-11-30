/**
  ******************************************************************************
  * @file           : drv_arm.cpp
  * @author         : 张澎皓
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#include "drv_arm.h"

#include "arm_math.h"
#include "rtos_inc.h"

float T_Compensation1_p;
float T_Compensation2_p;
float T_Compensation3_p;
float T_Compensation_r;
float k1 = 0.2f;//0.8
float k2 = 0.8f;//1.4
float k3 = 1.f;
float k4 = 1.f;
#if JY_ME02
float wrapTo180(float angle_deg) {
    // 保证角度在 [0, 360)
    angle_deg = fmodf(angle_deg, 360.0);
    if (angle_deg < 0) angle_deg += 360.0;

    // 映射到 [-180, 180)
    if (angle_deg >= 180.0)
        angle_deg -= 360.0;

    return angle_deg;
}


void arm_device::init() {
    encoder.encoder1.init(&Arm_Encoder1_Can,Arm_Encoder1_Id,ArmMotor1UpdateBinarySemHandle);
    encoder.encoder2.init(&Arm_Encoder2_Can,Arm_Encoder2_Id,ArmMotor2UpdateBinarySemHandle);
    encoder.encoder3.init(&Arm_Encoder3_Can,Arm_Encoder3_Id,ArmMotor3UpdateBinarySemHandle);
    encoder.encoder4.init(&Arm_Encoder4_Can,Arm_Encoder4_Id,ArmMotor4UpdateBinarySemHandle);
    encoder.encoder5.init(&Arm_Encoder5_Can,Arm_Encoder5_Id,ArmMotor5UpdateBinarySemHandle);
    encoder.encoder6.init(&Arm_Encoder6_Can,Arm_Encoder6_Id,ArmMotor6UpdateBinarySemHandle);

    data.encoder_offset = {Arm_Joint1_Offset,Arm_Joint2_Offset,Arm_Joint3_Offset,Arm_Joint4_Offset,Arm_Joint5_Offset,Arm_Joint6_Offset};

}

void arm_device::update_control(bool is_enable) {
    update_data();
}
void arm_device::update_data() {
    data.encoder_pos_get.encoder1 = wrapTo180(encoder.encoder1.get_angle());
    data.encoder_pos_get.encoder2 = wrapTo180(encoder.encoder2.get_angle());
    data.encoder_pos_get.encoder3 = wrapTo180(encoder.encoder3.get_angle());
    data.encoder_pos_get.encoder4 = wrapTo180(encoder.encoder4.get_angle()) ;//减速比
    data.encoder_pos_get.encoder5 = wrapTo180(encoder.encoder5.get_angle());
    data.encoder_pos_get.encoder6 = encoder.encoder6.get_total_angle();

    data.joint_states.joint1 = data.encoder_pos_get.encoder1 * 2 * PI / 360.f;
    data.joint_states.joint2 = data.encoder_pos_get.encoder2 * 2 * PI / 360.f;
    data.joint_states.joint3 = data.encoder_pos_get.encoder3 * 2 * PI / 360.f;
    data.joint_states.joint4 = data.encoder_pos_get.encoder4 * 2 * PI / 360.f;
    data.joint_states.joint5 = data.encoder_pos_get.encoder5 * 2 * PI / 360.f;
    data.joint_states.joint6 = data.encoder_pos_get.encoder6 * 2 * PI / 360.f;


    controller_tx_data.joint1 = data.joint_states.joint1;
    controller_tx_data.joint2 = data.joint_states.joint2;
    controller_tx_data.joint3 = data.joint_states.joint3;
    controller_tx_data.joint4 = data.joint_states.joint4;
    controller_tx_data.joint5 = data.joint_states.joint5;
    controller_tx_data.joint6 = data.joint_states.joint6;
    controller_tx_data.is_data_valid = !encoder.encoder1.check_lost() &&
                                       !encoder.encoder2.check_lost() &&
        !encoder.encoder3.check_lost() &&
        !encoder.encoder4.check_lost() &&
        !encoder.encoder5.check_lost() &&
        !encoder.encoder6.check_lost();
}

void arm_device::check_motor_loss() {
    encoder.encoder1.check_for_loss();
    encoder.encoder2.check_for_loss();
    encoder.encoder3.check_for_loss();
    encoder.encoder4.check_for_loss();
    encoder.encoder5.check_for_loss();
    encoder.encoder6.check_for_loss();

}

#else

void arm_device::init() {
    motor.motor1.init(&Arm_Motor1_Can, false, Arm_Motor1_Id, DJI_GM6020, ArmMotor1UpdateBinarySemHandle);
    motor.motor2.init(&Arm_Motor2_Can, true, Arm_Motor2_Id, DJI_GM6020, ArmMotor2UpdateBinarySemHandle);
    motor.motor3.init(&Arm_Motor3_Can, false, Arm_Motor3_Id, DJI_GM6020, ArmMotor3UpdateBinarySemHandle);
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
                      DM_J3507_2EC,
                      ArmMotor5UpdateBinarySemHandle);
    motor.motor6.init(&Arm_Motor6_Can, false, Arm_Motor6_Id, DJI_M2006, ArmMotor6UpdateBinarySemHandle);


    data.motor_offset = {Arm_Motor1_Offset,
                         Arm_Motor2_Offset,
                         Arm_Motor3_Offset,
                         Arm_Motor4_Offset,
                         Arm_Motor5_Offset,
                         Arm_Motor6_Offset};

    motor.motor1.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor2.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor3.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor4.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor5.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    motor.motor6.lqr.reset_lqr(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

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
    //2006随机编码器
    motor.motor4.reset_total_rounds_zero_offset(motor.motor4.get_current_round() - data.motor_offset.motor4);
    motor.motor5.reset_total_rounds_zero_offset(motor.motor5.get_current_round() - data.motor_offset.motor5);
    motor.motor6.reset_total_rounds_zero_offset(0);
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
    data.motor_pos_get.motor4 = motor.motor4.get_total_rounds() ;//减速比
    data.motor_pos_get.motor5 = motor.motor5.get_total_rounds() ;
    data.motor_pos_get.motor6 = motor.motor6.get_total_rounds() / 36.f;

    data.joint_states.joint1 = data.motor_pos_get.motor1 * 2 * PI;
    data.joint_states.joint2 = data.motor_pos_get.motor2 * 2 * PI;
    data.joint_states.joint3 = data.motor_pos_get.motor3 * 2 * PI - 0.67f;
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
//    controller_tx_data.is_data_valid = motor.motor1.check_ready() &&
//                                       motor.motor2.check_ready() &&
//                                       motor.motor3.check_ready() &&
//                                       motor.motor4.check_ready() ;
//    controller_tx_data.life_flag = (HAL_GetTick() / 10) % 10; //生命检测标志位，每10ms变化一次
}

void arm_device::update_control(bool is_enable) {

    update_data();//更新数据

    if (is_enable_last && !is_enable) {
        data.motor_pos_set = data.motor_pos_get;//如果遥控器断开，电机位置设为当前电机位置
    }

    is_enable_last = is_enable;

    update_gravity_compensation();

    if (is_enable) {//遥控器在线
        data.motor_pos_set.motor1 = data.joint_target.joint1 / (2 * PI);// 关节值与电机位置值的换算关系
        data.motor_pos_set.motor2 = data.joint_target.joint2 / (2 * PI);
        data.motor_pos_set.motor3 = data.joint_target.joint3 / (2 * PI);
        data.motor_pos_set.motor4 = data.joint_target.joint4 / (2 * PI);
        data.motor_pos_set.motor5 = data.joint_target.joint5 / (2 * PI);
        data.motor_pos_set.motor6 = data.joint_target.joint6 / (2 * PI);
    }

    if (is_ctrl_enable) {
        // motor.motor1.set_offset_current(data.motor_torque_compensation.motor1);//力矩补偿可能与关节角度有关，补偿到每个关节再换算到每个电机
        // motor.motor2.set_offset_current(data.motor_torque_compensation.motor2 * cosf(
        //     data.joint_states.joint2 + data.motor_compensation_angle_offset.motor2 / 180.f * PI) +
        //                                 (data.motor_torque_compensation.motor3 *
        //                                  cosf((-data.joint_states.joint2 - data.joint_states.joint3))) * (-1));
        // motor.motor3.set_offset_current(
        //     data.motor_torque_compensation.motor3 * cosf(-data.joint_states.joint2 - data.joint_states.joint3));
        // motor.motor4.set_offset_current(data.motor_torque_compensation.motor4);
        // motor.motor5.set_offset_current(data.motor_torque_compensation.motor5);
        // motor.motor6.set_offset_current(data.motor_torque_compensation.motor6);

        motor.motor1.set_offset_current(data.motor_torque_compensation.motor1);
        motor.motor2.set_offset_current(T_Compensation1_p/2.223f);
        motor.motor3.set_offset_current(T_Compensation2_p/2.223f);
        motor.motor4.set_offset_current(T_Compensation_r/DM_J4310_2EC_T_MAX);
        motor.motor5.set_offset_current(T_Compensation3_p/DM_J3507_2EC_T_MAX);
        motor.motor6.set_offset_current(data.motor_torque_compensation.motor6);

        // motor.motor1.set_current(motor.motor1.lqr.calculate(data.motor_pos_get.motor1,
        //                                                     motor.motor1.get_speed(),
        //                                                     data.motor_pos_set.motor1,
        //                                                     0.001));
        // motor.motor2.set_current(motor.motor2.lqr.calculate(data.motor_pos_get.motor2,
        //                                                     motor.motor2.get_speed(),
        //                                                     data.motor_pos_set.motor2,
        //                                                     0.001));
        // motor.motor3.set_current(motor.motor3.lqr.calculate(data.motor_pos_get.motor3,
        //                                                     motor.motor3.get_speed(),
        //                                                     data.motor_pos_set.motor3,
        //                                                     0.001));
        // motor.motor4.MIT_inter_set_motor_normalization_torque(motor.motor4.lqr.calculate(data.motor_pos_get.motor4,
        //                                                     motor.motor4.get_speed(),
        //                                                     data.motor_pos_set.motor4,
        //                                                     0.001));
        // motor.motor5.MIT_inter_set_motor_normalization_torque(motor.motor5.lqr.calculate(data.motor_pos_get.motor5,
        //                                                     motor.motor5.get_speed(),
        //                                                     data.motor_pos_set.motor5,
        //                                                     0.001));
        // motor.motor6.set_current(motor.motor6.lqr.calculate(data.motor_pos_get.motor6,
        //                                                     motor.motor6.get_speed() / 36.f,
        //                                                     data.motor_pos_set.motor6,
        //                                                     0.001));

        // motor.motor1.set_offset_current(0);
        // motor.motor2.set_offset_current(0);
        // motor.motor3.set_offset_current(0);
        // motor.motor4.set_offset_current(0);
        // motor.motor5.set_offset_current(0);
        // motor.motor6.set_offset_current(0);
        motor.motor1.set_current(0);
        motor.motor2.set_current(0);
        motor.motor3.set_current(0);
        motor.motor4.MIT_inter_set_motor_normalization_torque(0);
        motor.motor5.MIT_inter_set_motor_normalization_torque(0);
        motor.motor6.set_current(0);


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
        motor.motor4.set_free();
        motor.motor5.set_free();
        motor.motor6.set_current_zero();
    }


}

void arm_device::send_msg() {
    motor.motor1.send_can_msg();
    motor.motor2.send_can_msg();
    motor.motor3.send_can_msg();
    motor.motor4.send_can_msg();
    motor.motor5.send_can_msg();
    motor.motor6.send_can_msg();

//    motor.motor5.set_motor_enable();
//    motor.motor5.recover_the_motor();
}


void arm_device::check_motor_loss() {
    motor.motor1.check_motor_for_loss();
    motor.motor2.check_motor_for_loss();
    motor.motor3.check_motor_for_loss();
    motor.motor4.check_motor_for_loss();
    motor.motor5.check_motor_for_loss();
    motor.motor6.check_motor_for_loss();
}

#endif

uint8_t *arm_device::get_controller_tx_data() {
    return (uint8_t *) &controller_tx_data;
}

void arm_device::update_tx_life_flag() {
    controller_tx_data.life_flag = (HAL_GetTick() / 35) % 256; //生命检测标志位，每10ms变化一次
}

void arm_device::update_gravity_compensation()
{
    float theta1 = data.joint_states.joint2 - 1.36f;//PI/2
    float theta2 = data.joint_states.joint3 + PI;
    float theta3 = data.joint_states.joint5;
    float theta4 = data.joint_states.joint4;
    float theta5 = asinf(arm_sin_f32(theta3)*arm_cos_f32(theta4));
    // float Lm2 = lm2*k2;
    // float Lm3 = lm3*k3;
    T_Compensation1_p = - lm1*arm_sin_f32(theta1)*m1 - (l1*arm_sin_f32(theta1) + lm2*arm_sin_f32(theta1 + theta2))*m2 - (l1*arm_sin_f32(theta1) + l2*arm_sin_f32(theta1 + theta2) + lm3*arm_sin_f32(theta1 + theta2 + theta5))*m3;
    T_Compensation1_p *= g*k1;

    T_Compensation2_p = - k2*lm2*m2*arm_sin_f32(theta1 + theta2) - (l2*arm_sin_f32(theta1 + theta2) + lm3*arm_sin_f32(theta1 + theta2 + theta5))*m3;
    T_Compensation2_p *= g;

    T_Compensation3_p = - lm3*m3*arm_sin_f32(theta1 + theta2 + theta5)*arm_cos_f32(theta3)*arm_cos_f32(theta4)/sqrt(1 - arm_sin_f32(theta3)*arm_sin_f32(theta3)*arm_cos_f32(theta4)*arm_cos_f32(theta4));
    T_Compensation3_p *= g;

    T_Compensation_r = - m3*lm3*arm_cos_f32(theta3)*arm_cos_f32(theta4)*arm_sin_f32(theta1 + theta2);
    T_Compensation_r *= g;
}
