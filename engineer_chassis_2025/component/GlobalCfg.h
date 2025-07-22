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

#define CHASSIS 1

#define CHASSIS_CAN         (hcan1)
#define IMU_CAN         (hcan2)

#define IMU_SLAVE_ID 0x20
#define IMU_MASTER_ID 0x30

#define COMMUNICATE_FRAME_HEAD 0x55 //通信帧头
#define COMMUNICATE_CAN         (hcan2)
#define COMMUNICATE_TX_ID 0x101
#define COMMUNICATE_RX_ID 0x100



#define ARM_SUCKER_BLOCK_CURRENT_MIN  1940.0f
#define ARM_SUCKER_BLOCK_CURRENT_MAX   2000.0f

#define LEFT_SUCKER_BLOCK_CURRENT_MIN   2200.0f
#define LEFT_SUCKER_BLOCK_CURRENT_MAX   2180.0f

#define RIGHT_SUCKER_BLOCK_CURRENT_MIN   2050.0f
#define RIGHT_SUCKER_BLOCK_CURRENT_MAX   2100.0f

#define CHASSIS_VEL_MAX 0.9f

#ifdef __cplusplus
}
#endif
//C++

#endif //GLOBALCFG_H_
