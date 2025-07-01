/**
  ******************************************************************************
  * @file           : GlobalCfg.h
  * @author         : 34147
  * @brief          : None
  * @attention      : None
  * @date           : 25-5-31
  ******************************************************************************
  */


#ifndef GLOBALCFG_H_
#define GLOBALCFG_H_
#ifdef __cplusplus
extern "C" {
#endif
//C
#define CAN_SEND        1
#define DRAW_UI        1
#define DM_IMU         1
#define SERVO         1

#define ARM_REMOTE_CONTROL_PROTECT 1 //臂架遥控器保护
#define ARM_DEBUG_MODE 0
#define NO_CHASSIS_COMMUNICATE  0

#define RC_UART        huart1
#define JUDGEMENT_POWER_UART  huart3
#define JUDGEMENT_TRANSFER_UART huart6
#define SERVO_UART huart8

const uint8_t PC_Normal_Frame_Head = 0x55; //PC正常通信帧头
const uint8_t PC_Controller_Frame_Head = 0x66; //PC控制器通信帧头
const uint8_t PC_Rx_Frame_Head = 0x55; //PC接收数据帧头
const uint8_t PC_Frame_Tail = 0xAA; //PC正常通信帧尾



#ifdef __cplusplus
}
#endif
//C++

#endif //GLOBALCFG_H_
