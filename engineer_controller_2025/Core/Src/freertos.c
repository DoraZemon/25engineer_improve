/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1_Send_Task */
osThreadId_t CAN1_Send_TaskHandle;
const osThreadAttr_t CAN1_Send_Task_attributes = {
  .name = "CAN1_Send_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2_Send_Task */
osThreadId_t CAN2_Send_TaskHandle;
const osThreadAttr_t CAN2_Send_Task_attributes = {
  .name = "CAN2_Send_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for rctask */
osThreadId_t rctaskHandle;
const osThreadAttr_t rctask_attributes = {
  .name = "rctask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbaltask */
osThreadId_t gimbaltaskHandle;
const osThreadAttr_t gimbaltask_attributes = {
  .name = "gimbaltask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for kb_eventtask */
osThreadId_t kb_eventtaskHandle;
const osThreadAttr_t kb_eventtask_attributes = {
  .name = "kb_eventtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for kb_statetask */
osThreadId_t kb_statetaskHandle;
const osThreadAttr_t kb_statetask_attributes = {
  .name = "kb_statetask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imutask */
osThreadId_t imutaskHandle;
const osThreadAttr_t imutask_attributes = {
  .name = "imutask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for frictiontask */
osThreadId_t frictiontaskHandle;
const osThreadAttr_t frictiontask_attributes = {
  .name = "frictiontask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bullet_feedtask */
osThreadId_t bullet_feedtaskHandle;
const osThreadAttr_t bullet_feedtask_attributes = {
  .name = "bullet_feedtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for shoot_counttask */
osThreadId_t shoot_counttaskHandle;
const osThreadAttr_t shoot_counttask_attributes = {
  .name = "shoot_counttask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for vision_transmittask */
osThreadId_t vision_transmittaskHandle;
const osThreadAttr_t vision_transmittask_attributes = {
  .name = "vision_transmittask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vison_ctrltask */
osThreadId_t vison_ctrltaskHandle;
const osThreadAttr_t vison_ctrltask_attributes = {
  .name = "vison_ctrltask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vision_receivetask */
osThreadId_t vision_receivetaskHandle;
const osThreadAttr_t vision_receivetask_attributes = {
  .name = "vision_receivetask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vision_calctask */
osThreadId_t vision_calctaskHandle;
const osThreadAttr_t vision_calctask_attributes = {
  .name = "vision_calctask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lost_checktask */
osThreadId_t lost_checktaskHandle;
const osThreadAttr_t lost_checktask_attributes = {
  .name = "lost_checktask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gyrotask */
osThreadId_t gyrotaskHandle;
const osThreadAttr_t gyrotask_attributes = {
  .name = "gyrotask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for updateUIDatatas */
osThreadId_t updateUIDatatasHandle;
const osThreadAttr_t updateUIDatatas_attributes = {
  .name = "updateUIDatatas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for judgeCtrltask */
osThreadId_t judgeCtrltaskHandle;
const osThreadAttr_t judgeCtrltask_attributes = {
  .name = "judgeCtrltask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for robottask */
osThreadId_t robottaskHandle;
const osThreadAttr_t robottask_attributes = {
  .name = "robottask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for navigationlamp_task */
osThreadId_t navigationlamp_taskHandle;
const osThreadAttr_t navigationlamp_task_attributes = {
  .name = "navigationlamp_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1SendQueueHandle */
osMessageQueueId_t CAN1SendQueueHandleHandle;
const osMessageQueueAttr_t CAN1SendQueueHandle_attributes = {
  .name = "CAN1SendQueueHandle"
};
/* Definitions for CAN2SendQueueHandle */
osMessageQueueId_t CAN2SendQueueHandleHandle;
const osMessageQueueAttr_t CAN2SendQueueHandle_attributes = {
  .name = "CAN2SendQueueHandle"
};
/* Definitions for YawMotorBinarySem */
osSemaphoreId_t YawMotorBinarySemHandle;
const osSemaphoreAttr_t YawMotorBinarySem_attributes = {
  .name = "YawMotorBinarySem"
};
/* Definitions for PitchMotorBinarySem */
osSemaphoreId_t PitchMotorBinarySemHandle;
const osSemaphoreAttr_t PitchMotorBinarySem_attributes = {
  .name = "PitchMotorBinarySem"
};
/* Definitions for ImuBinarySem */
osSemaphoreId_t ImuBinarySemHandle;
const osSemaphoreAttr_t ImuBinarySem_attributes = {
  .name = "ImuBinarySem"
};
/* Definitions for RCUpdateBinarySem */
osSemaphoreId_t RCUpdateBinarySemHandle;
const osSemaphoreAttr_t RCUpdateBinarySem_attributes = {
  .name = "RCUpdateBinarySem"
};
/* Definitions for FrictionLFMotorBinarySem */
osSemaphoreId_t FrictionLFMotorBinarySemHandle;
const osSemaphoreAttr_t FrictionLFMotorBinarySem_attributes = {
  .name = "FrictionLFMotorBinarySem"
};
/* Definitions for FrictionLBMotorBinarySem */
osSemaphoreId_t FrictionLBMotorBinarySemHandle;
const osSemaphoreAttr_t FrictionLBMotorBinarySem_attributes = {
  .name = "FrictionLBMotorBinarySem"
};
/* Definitions for FrictionRFMotorBinarySem */
osSemaphoreId_t FrictionRFMotorBinarySemHandle;
const osSemaphoreAttr_t FrictionRFMotorBinarySem_attributes = {
  .name = "FrictionRFMotorBinarySem"
};
/* Definitions for FrictionRBMotorBinarySem */
osSemaphoreId_t FrictionRBMotorBinarySemHandle;
const osSemaphoreAttr_t FrictionRBMotorBinarySem_attributes = {
  .name = "FrictionRBMotorBinarySem"
};
/* Definitions for BullerFeedMotorBinarySem */
osSemaphoreId_t BullerFeedMotorBinarySemHandle;
const osSemaphoreAttr_t BullerFeedMotorBinarySem_attributes = {
  .name = "BullerFeedMotorBinarySem"
};
/* Definitions for VisionBinarySem */
osSemaphoreId_t VisionBinarySemHandle;
const osSemaphoreAttr_t VisionBinarySem_attributes = {
  .name = "VisionBinarySem"
};
/* Definitions for judgementInitBinarySem */
osSemaphoreId_t judgementInitBinarySemHandle;
const osSemaphoreAttr_t judgementInitBinarySem_attributes = {
  .name = "judgementInitBinarySem"
};
/* Definitions for CAN1CountingSemHandle */
osSemaphoreId_t CAN1CountingSemHandleHandle;
const osSemaphoreAttr_t CAN1CountingSemHandle_attributes = {
  .name = "CAN1CountingSemHandle"
};
/* Definitions for CAN2CountingSemHandle */
osSemaphoreId_t CAN2CountingSemHandleHandle;
const osSemaphoreAttr_t CAN2CountingSemHandle_attributes = {
  .name = "CAN2CountingSemHandle"
};
/* Definitions for refereeEvent */
osEventFlagsId_t refereeEventHandle;
const osEventFlagsAttr_t refereeEvent_attributes = {
  .name = "refereeEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void rc_task(void *argument);
void gimbal_task(void *argument);
void kb_event_task(void *argument);
void kb_state_task(void *argument);
void imu_task(void *argument);
void friction_task(void *argument);
void bullet_feed_task(void *argument);
void shoot_count_task(void *argument);
void vision_transmit_task(void *argument);
void vision_ctrl_task(void *argument);
void vision_receive_task(void *argument);
void vision_calc_task(void *argument);
void lost_check_task(void *argument);
void gyro_task(void *argument);
void updateUIData_task(void *argument);
void judgeCtrl_task(void *argument);
void robot_task(void *argument);
void navigation_lamp_task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of YawMotorBinarySem */
  YawMotorBinarySemHandle = osSemaphoreNew(1, 0, &YawMotorBinarySem_attributes);

  /* creation of PitchMotorBinarySem */
  PitchMotorBinarySemHandle = osSemaphoreNew(1, 0, &PitchMotorBinarySem_attributes);

  /* creation of ImuBinarySem */
  ImuBinarySemHandle = osSemaphoreNew(1, 0, &ImuBinarySem_attributes);

  /* creation of RCUpdateBinarySem */
  RCUpdateBinarySemHandle = osSemaphoreNew(1, 0, &RCUpdateBinarySem_attributes);

  /* creation of FrictionLFMotorBinarySem */
  FrictionLFMotorBinarySemHandle = osSemaphoreNew(1, 0, &FrictionLFMotorBinarySem_attributes);

  /* creation of FrictionLBMotorBinarySem */
  FrictionLBMotorBinarySemHandle = osSemaphoreNew(1, 1, &FrictionLBMotorBinarySem_attributes);

  /* creation of FrictionRFMotorBinarySem */
  FrictionRFMotorBinarySemHandle = osSemaphoreNew(1, 1, &FrictionRFMotorBinarySem_attributes);

  /* creation of FrictionRBMotorBinarySem */
  FrictionRBMotorBinarySemHandle = osSemaphoreNew(1, 1, &FrictionRBMotorBinarySem_attributes);

  /* creation of BullerFeedMotorBinarySem */
  BullerFeedMotorBinarySemHandle = osSemaphoreNew(1, 0, &BullerFeedMotorBinarySem_attributes);

  /* creation of VisionBinarySem */
  VisionBinarySemHandle = osSemaphoreNew(1, 0, &VisionBinarySem_attributes);

  /* creation of judgementInitBinarySem */
  judgementInitBinarySemHandle = osSemaphoreNew(1, 0, &judgementInitBinarySem_attributes);

  /* creation of CAN1CountingSemHandle */
  CAN1CountingSemHandleHandle = osSemaphoreNew(3, 3, &CAN1CountingSemHandle_attributes);

  /* creation of CAN2CountingSemHandle */
  CAN2CountingSemHandleHandle = osSemaphoreNew(3, 3, &CAN2CountingSemHandle_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1SendQueueHandle */
  CAN1SendQueueHandleHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &CAN1SendQueueHandle_attributes);

  /* creation of CAN2SendQueueHandle */
  CAN2SendQueueHandleHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &CAN2SendQueueHandle_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN1_Send_Task */
  CAN1_Send_TaskHandle = osThreadNew(CAN1Send_Task, NULL, &CAN1_Send_Task_attributes);

  /* creation of CAN2_Send_Task */
  CAN2_Send_TaskHandle = osThreadNew(CAN2Send_Task, NULL, &CAN2_Send_Task_attributes);

  /* creation of rctask */
  rctaskHandle = osThreadNew(rc_task, NULL, &rctask_attributes);

  /* creation of gimbaltask */
  gimbaltaskHandle = osThreadNew(gimbal_task, NULL, &gimbaltask_attributes);

  /* creation of kb_eventtask */
  kb_eventtaskHandle = osThreadNew(kb_event_task, NULL, &kb_eventtask_attributes);

  /* creation of kb_statetask */
  kb_statetaskHandle = osThreadNew(kb_state_task, NULL, &kb_statetask_attributes);

  /* creation of imutask */
  imutaskHandle = osThreadNew(imu_task, NULL, &imutask_attributes);

  /* creation of frictiontask */
  frictiontaskHandle = osThreadNew(friction_task, NULL, &frictiontask_attributes);

  /* creation of bullet_feedtask */
  bullet_feedtaskHandle = osThreadNew(bullet_feed_task, NULL, &bullet_feedtask_attributes);

  /* creation of shoot_counttask */
  shoot_counttaskHandle = osThreadNew(shoot_count_task, NULL, &shoot_counttask_attributes);

  /* creation of vision_transmittask */
  vision_transmittaskHandle = osThreadNew(vision_transmit_task, NULL, &vision_transmittask_attributes);

  /* creation of vison_ctrltask */
  vison_ctrltaskHandle = osThreadNew(vision_ctrl_task, NULL, &vison_ctrltask_attributes);

  /* creation of vision_receivetask */
  vision_receivetaskHandle = osThreadNew(vision_receive_task, NULL, &vision_receivetask_attributes);

  /* creation of vision_calctask */
  vision_calctaskHandle = osThreadNew(vision_calc_task, NULL, &vision_calctask_attributes);

  /* creation of lost_checktask */
  lost_checktaskHandle = osThreadNew(lost_check_task, NULL, &lost_checktask_attributes);

  /* creation of gyrotask */
  gyrotaskHandle = osThreadNew(gyro_task, NULL, &gyrotask_attributes);

  /* creation of updateUIDatatas */
  updateUIDatatasHandle = osThreadNew(updateUIData_task, NULL, &updateUIDatatas_attributes);

  /* creation of judgeCtrltask */
  judgeCtrltaskHandle = osThreadNew(judgeCtrl_task, NULL, &judgeCtrltask_attributes);

  /* creation of robottask */
  robottaskHandle = osThreadNew(robot_task, NULL, &robottask_attributes);

  /* creation of navigationlamp_task */
  navigationlamp_taskHandle = osThreadNew(navigation_lamp_task, NULL, &navigationlamp_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of refereeEvent */
  refereeEventHandle = osEventFlagsNew(&refereeEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CAN1Send_Task */
/**
* @brief Function implementing the CAN1_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN1Send_Task */
__weak void CAN1Send_Task(void *argument)
{
  /* USER CODE BEGIN CAN1Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN1Send_Task */
}

/* USER CODE BEGIN Header_CAN2Send_Task */
/**
* @brief Function implementing the CAN2_Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN2Send_Task */
__weak void CAN2Send_Task(void *argument)
{
  /* USER CODE BEGIN CAN2Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN2Send_Task */
}

/* USER CODE BEGIN Header_rc_task */
/**
* @brief Function implementing the rctask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc_task */
__weak void rc_task(void *argument)
{
  /* USER CODE BEGIN rc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rc_task */
}

/* USER CODE BEGIN Header_gimbal_task */
/**
* @brief Function implementing the gimbaltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbal_task */
__weak void gimbal_task(void *argument)
{
  /* USER CODE BEGIN gimbal_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}

/* USER CODE BEGIN Header_kb_event_task */
/**
* @brief Function implementing the kb_eventtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kb_event_task */
__weak void kb_event_task(void *argument)
{
  /* USER CODE BEGIN kb_event_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END kb_event_task */
}

/* USER CODE BEGIN Header_kb_state_task */
/**
* @brief Function implementing the kb_statetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_kb_state_task */
__weak void kb_state_task(void *argument)
{
  /* USER CODE BEGIN kb_state_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END kb_state_task */
}

/* USER CODE BEGIN Header_imu_task */
/**
* @brief Function implementing the imutask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_task */
__weak void imu_task(void *argument)
{
  /* USER CODE BEGIN imu_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END imu_task */
}

/* USER CODE BEGIN Header_friction_task */
/**
* @brief Function implementing the frictiontask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_friction_task */
__weak void friction_task(void *argument)
{
  /* USER CODE BEGIN friction_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END friction_task */
}

/* USER CODE BEGIN Header_bullet_feed_task */
/**
* @brief Function implementing the bullet_feedtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_bullet_feed_task */
__weak void bullet_feed_task(void *argument)
{
  /* USER CODE BEGIN bullet_feed_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END bullet_feed_task */
}

/* USER CODE BEGIN Header_shoot_count_task */
/**
* @brief Function implementing the shoot_counttask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_shoot_count_task */
__weak void shoot_count_task(void *argument)
{
  /* USER CODE BEGIN shoot_count_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END shoot_count_task */
}

/* USER CODE BEGIN Header_vision_transmit_task */
/**
* @brief Function implementing the vision_transmittask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vision_transmit_task */
__weak void vision_transmit_task(void *argument)
{
  /* USER CODE BEGIN vision_transmit_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vision_transmit_task */
}

/* USER CODE BEGIN Header_vision_ctrl_task */
/**
* @brief Function implementing the vison_ctrltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vision_ctrl_task */
__weak void vision_ctrl_task(void *argument)
{
  /* USER CODE BEGIN vision_ctrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vision_ctrl_task */
}

/* USER CODE BEGIN Header_vision_receive_task */
/**
* @brief Function implementing the vision_receivetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vision_receive_task */
__weak void vision_receive_task(void *argument)
{
  /* USER CODE BEGIN vision_receive_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vision_receive_task */
}

/* USER CODE BEGIN Header_vision_calc_task */
/**
* @brief Function implementing the vision_calctask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vision_calc_task */
__weak void vision_calc_task(void *argument)
{
  /* USER CODE BEGIN vision_calc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END vision_calc_task */
}

/* USER CODE BEGIN Header_lost_check_task */
/**
* @brief Function implementing the lost_checktask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lost_check_task */
__weak void lost_check_task(void *argument)
{
  /* USER CODE BEGIN lost_check_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lost_check_task */
}

/* USER CODE BEGIN Header_gyro_task */
/**
* @brief Function implementing the gyrotask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyro_task */
__weak void gyro_task(void *argument)
{
  /* USER CODE BEGIN gyro_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gyro_task */
}

/* USER CODE BEGIN Header_updateUIData_task */
/**
* @brief Function implementing the updateUIDatatas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_updateUIData_task */
__weak void updateUIData_task(void *argument)
{
  /* USER CODE BEGIN updateUIData_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END updateUIData_task */
}

/* USER CODE BEGIN Header_judgeCtrl_task */
/**
* @brief Function implementing the judgeCtrltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_judgeCtrl_task */
__weak void judgeCtrl_task(void *argument)
{
  /* USER CODE BEGIN judgeCtrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END judgeCtrl_task */
}

/* USER CODE BEGIN Header_robot_task */
/**
* @brief Function implementing the robottask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_robot_task */
__weak void robot_task(void *argument)
{
  /* USER CODE BEGIN robot_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END robot_task */
}

/* USER CODE BEGIN Header_navigation_lamp_task */
/**
* @brief Function implementing the navigationlamp_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_navigation_lamp_task */
__weak void navigation_lamp_task(void *argument)
{
  /* USER CODE BEGIN navigation_lamp_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END navigation_lamp_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

