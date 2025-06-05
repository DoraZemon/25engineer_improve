/**
  ******************************************************************************
  * @file           : drv_chassis.cpp
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-6-5
  ******************************************************************************
  */


#include "drv_chassis.h"
#include "mecanum.h"
#include "GlobalCfg.h"


chassis_device::chassis_device()
    : enable_flag(true), lost_flag(true), ready_flag(false),
      controlType(Speed),
      velocity({0, 0, 0}), super_rotate_flag(
        false) {}

void chassis_device::init(CAN_HandleTypeDef *hcan) {
    this->hcan = hcan;

    wheel[0].init(hcan, false, 1, DJI_M3508, ChassisMotor1UpdateBinarySemHandle);
    wheel[1].init(hcan, false, 2, DJI_M3508, ChassisMotor2UpdateBinarySemHandle);
    wheel[2].init(hcan, true, 3, DJI_M3508, ChassisMotor3UpdateBinarySemHandle);
    wheel[3].init(hcan, true, 4, DJI_M3508, ChassisMotor4UpdateBinarySemHandle);

    wheel[0].velpid.pid_reset(0.95f, 0.06f, 10.5f, 0.002f, 4.0f, 0, 0, 0);
    wheel[1].velpid.pid_reset(0.95f, 0.06f, 9.7f, 0.002f, 5.0f, 0, 0, 0);
    wheel[2].velpid.pid_reset(0.95f, 0.06f, 11.0f, 0.002f, 5.0f, 0, 0, 0);
    wheel[3].velpid.pid_reset(0.95f, 0.06f, 10.3f, 0.002f, 4.0f, 0, 0, 0);

    wheel[0].pospid.pid_reset(0.9f, 0.3f, 0.1f, 0, 0.002f, 0, 0, 0);
    wheel[1].pospid.pid_reset(0.9f, 0.3f, 0.1f, 0, 0.002f, 0, 0, 0);
    wheel[2].pospid.pid_reset(0.9f, 0.3f, 0.1f, 0, 0.002f, 0, 0, 0);
    wheel[3].pospid.pid_reset(0.9f, 0.3f, 0.1f, 0, 0.002f, 0, 0, 0);

    this->pos_rot_pid.pid_reset(0.1f, 0, 0.01f, 0, 0, 0, 0, 0);

}


__RAM_FUNC void chassis_device::update_speed_control() {


    if (this->super_rotate_flag) {
        SuperChassisMotorSolverSet(this->wheel, velocity.speedX, velocity.speedY, velocity.speedSpin);
    } else {
        chassisMotorSolverSet(this->wheel, velocity.speedX, velocity.speedY, velocity.speedSpin);
    }

}


void chassis_device::update_data(float yaw_total) {
    chassis_yaw_total_angle = yaw_total;
}

bool chassis_device::check_init_completely() {
    if (this->wheel[0].is_zero_offset && this->wheel[1].is_zero_offset && this->wheel[2].is_zero_offset
        && this->wheel[3].is_zero_offset) {
        return true;
    } else {
        return false;
    }
}

bool chassis_device::check_can_use() {
//    if(!this->robot_set_easter_use_flag){
//        return false;
//    }

    if ((this->wheel[0].is_zero_offset || this->wheel[0].lost_flag) &&
        (this->wheel[1].is_zero_offset || this->wheel[1].lost_flag) &&
        (this->wheel[2].is_zero_offset || this->wheel[2].lost_flag)
        && (this->wheel[3].is_zero_offset || this->wheel[3].lost_flag)) {
        return true;
    } else {
        return false;
    }
}

void chassis_device::update_ready() {
    for (auto &i : this->wheel) {
        i.update_ready();
    }
    if (this->wheel[0].ready_flag && this->wheel[1].ready_flag && this->wheel[2].ready_flag &&
        this->wheel[3].ready_flag) {
        this->ready_flag = true;
    } else {
        this->ready_flag = false;
    }

    if (this->wheel[0].lost_flag || this->wheel[1].lost_flag || this->wheel[2].lost_flag || this->wheel[3].lost_flag) {
        this->lost_flag = true;
    } else {
        this->lost_flag = false;
    }

}

void chassis_device::close_yaw_spin() {
    this->velocity.speedSpin = 0.0f;
}

void chassis_device::set_free() {
    this->wheel[0].set_free();
    this->wheel[1].set_free();
    this->wheel[2].set_free();
    this->wheel[3].set_free();

    this->clean_speed_control();
    this->clean_position_control();
}

void chassis_device::set_speed_x(float x) {
    this->velocity.speedX = x;
    ABS_LIMIT(this->velocity.speedX, CHASSIS_VEL_MAX);
}

void chassis_device::set_speed_y(float y) {
    this->velocity.speedY = y;
    ABS_LIMIT(this->velocity.speedY, CHASSIS_VEL_MAX);
}

void chassis_device::set_speed_spin(float spin) {
    this->velocity.speedSpin = spin;
    ABS_LIMIT(this->velocity.speedSpin, CHASSIS_VEL_MAX * 0.8f);
}

void chassis_device::add_speed_x(float delta_x) {
    this->velocity.speedX += delta_x;
    ABS_LIMIT(this->velocity.speedX, CHASSIS_VEL_MAX);
}

void chassis_device::add_speed_y(float delta_y) {
    this->velocity.speedY += delta_y;
    ABS_LIMIT(this->velocity.speedX, CHASSIS_VEL_MAX);
}

void chassis_device::add_speed_spin(float delta_spin) {
    this->velocity.speedSpin += delta_spin;
    ABS_LIMIT(this->velocity.speedSpin, CHASSIS_VEL_MAX * 0.8f);
}


void chassis_device::set_position_x(float x) {
    this->position.positionX = x;
}

void chassis_device::set_position_y(float y) {
    this->position.positionY = y;
}

void chassis_device::set_position_spin(float spin) {
    this->position.positionSpin = spin;
}

void chassis_device::add_position_x(float delta_x) {
    this->position.positionX += delta_x;
}

void chassis_device::add_position_y(float delta_y) {
    this->position.positionY += delta_y;
}

void chassis_device::add_position_spin(float delta_spin) {
    this->position.positionSpin += delta_spin;
    VAL_LIMIT(this->position.positionSpin, -30, 30);
}


void chassis_device::clean_speed_control() {
    this->close_yaw_spin();

    this->velocity.speedX = 0;
    this->velocity.speedY = 0;
    this->velocity.speedSpin = 0;

    this->wheel[0].reset_total_rounds_zero_offset(0);
    this->wheel[1].reset_total_rounds_zero_offset(0);
    this->wheel[2].reset_total_rounds_zero_offset(0);
    this->wheel[3].reset_total_rounds_zero_offset(0);
}

void chassis_device::clean_position_control() {
    this->position.positionX = 0;
    this->position.positionY = 0;
    this->position.positionSpin = 0;
}

__RAM_FUNC  void chassis_device::update_position_control() {

    this->add_position_spin(2 * PI * this->pos_rot_pid.pid_calculate(this->pos_yaw_angle,
                                                                     chassis_yaw_total_angle -
                                                                     chassis_yaw_total_angle_offset));
    chassisMotorPositionSolverSet(this->wheel,
                                  this->position.positionX,
                                  this->position.positionY,
                                  this->position.positionSpin);
}


void chassis_device::disable() {
    this->enable_flag = false;
}

void chassis_device::enable() {
    this->enable_flag = true;
}

bool chassis_device::check_position_control() {
    return this->controlType == Position;
}

bool chassis_device::check_speed_control() {
    return this->controlType == Speed;
}

void chassis_device::open_speed_control() {
    if (!this->check_speed_control()) {
        this->controlType = Speed;
        this->clean_position_control();
    }
}

void chassis_device::open_position_control() {
    if (!this->check_position_control()) {
        this->controlType = Position;
        this->clean_speed_control();
        this->pos_yaw_angle = 0.f;
        chassis_yaw_total_angle_offset = chassis_yaw_total_angle;
    }
}

bool chassis_device::check_super_rotate() const {
    return this->super_rotate_flag;
}

bool chassis_device::check_ready() const {
    return this->ready_flag;
}

bool chassis_device::check_enable() const {
    return this->enable_flag;
}

enum chassis_control_type chassis_device::check_control_type() {
    return this->controlType;
}


void  chassis_device::check_motor_lost() {
    dji_motor_device *motor;
    for (auto &i : this->wheel) {
        motor = &i;
        motor->check_motor_for_loss();
    }
}


void chassis_device::can_set() {
    this->wheel[0].send_can_msg();
}
