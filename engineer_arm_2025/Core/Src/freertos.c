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
#include "compatible.h"
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
extern IWDG_HandleTypeDef hiwdg;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1SendTask */
osThreadId_t CAN1SendTaskHandle;
const osThreadAttr_t CAN1SendTask_attributes = {
  .name = "CAN1SendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CAN2SendTask */
osThreadId_t CAN2SendTaskHandle;
const osThreadAttr_t CAN2SendTask_attributes = {
  .name = "CAN2SendTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for armtask */
osThreadId_t armtaskHandle;
const osThreadAttr_t armtask_attributes = {
  .name = "armtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lostchecktask */
osThreadId_t lostchecktaskHandle;
const osThreadAttr_t lostchecktask_attributes = {
  .name = "lostchecktask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rctask */
osThreadId_t rctaskHandle;
const osThreadAttr_t rctask_attributes = {
  .name = "rctask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pc_receivetask */
osThreadId_t pc_receivetaskHandle;
const osThreadAttr_t pc_receivetask_attributes = {
  .name = "pc_receivetask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for pc_transmittask */
osThreadId_t pc_transmittaskHandle;
const osThreadAttr_t pc_transmittask_attributes = {
  .name = "pc_transmittask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for receiveCtrltask */
osThreadId_t receiveCtrltaskHandle;
const osThreadAttr_t receiveCtrltask_attributes = {
  .name = "receiveCtrltask",
  .stack_size = 256 * 4,
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
/* Definitions for controllertask */
osThreadId_t controllertaskHandle;
const osThreadAttr_t controllertask_attributes = {
  .name = "controllertask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for imutask */
osThreadId_t imutaskHandle;
const osThreadAttr_t imutask_attributes = {
  .name = "imutask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for communicatetask */
osThreadId_t communicatetaskHandle;
const osThreadAttr_t communicatetask_attributes = {
  .name = "communicatetask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo_ctrltask */
osThreadId_t servo_ctrltaskHandle;
const osThreadAttr_t servo_ctrltask_attributes = {
  .name = "servo_ctrltask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbaltask */
osThreadId_t gimbaltaskHandle;
const osThreadAttr_t gimbaltask_attributes = {
  .name = "gimbaltask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hi229umtask */
osThreadId_t hi229umtaskHandle;
const osThreadAttr_t hi229umtask_attributes = {
  .name = "hi229umtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1SendQueue */
osMessageQueueId_t CAN1SendQueueHandle;
const osMessageQueueAttr_t CAN1SendQueue_attributes = {
  .name = "CAN1SendQueue"
};
/* Definitions for CAN2SendQueue */
osMessageQueueId_t CAN2SendQueueHandle;
const osMessageQueueAttr_t CAN2SendQueue_attributes = {
  .name = "CAN2SendQueue"
};
/* Definitions for ServoCtrlQueue */
osMessageQueueId_t ServoCtrlQueueHandle;
const osMessageQueueAttr_t ServoCtrlQueue_attributes = {
  .name = "ServoCtrlQueue"
};
/* Definitions for ArmMotor1UpdateBinarySem */
osSemaphoreId_t ArmMotor1UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor1UpdateBinarySem_attributes = {
  .name = "ArmMotor1UpdateBinarySem"
};
/* Definitions for ArmMotor2UpdateBinarySem */
osSemaphoreId_t ArmMotor2UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor2UpdateBinarySem_attributes = {
  .name = "ArmMotor2UpdateBinarySem"
};
/* Definitions for ArmMotor3UpdateBinarySem */
osSemaphoreId_t ArmMotor3UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor3UpdateBinarySem_attributes = {
  .name = "ArmMotor3UpdateBinarySem"
};
/* Definitions for ArmMotor4UpdateBinarySem */
osSemaphoreId_t ArmMotor4UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor4UpdateBinarySem_attributes = {
  .name = "ArmMotor4UpdateBinarySem"
};
/* Definitions for ArmMotor5UpdateBinarySem */
osSemaphoreId_t ArmMotor5UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor5UpdateBinarySem_attributes = {
  .name = "ArmMotor5UpdateBinarySem"
};
/* Definitions for ArmMotor6UpdateBinarySem */
osSemaphoreId_t ArmMotor6UpdateBinarySemHandle;
const osSemaphoreAttr_t ArmMotor6UpdateBinarySem_attributes = {
  .name = "ArmMotor6UpdateBinarySem"
};
/* Definitions for RCUpdateBinarySem */
osSemaphoreId_t RCUpdateBinarySemHandle;
const osSemaphoreAttr_t RCUpdateBinarySem_attributes = {
  .name = "RCUpdateBinarySem"
};
/* Definitions for PCUpdateBinarySem */
osSemaphoreId_t PCUpdateBinarySemHandle;
const osSemaphoreAttr_t PCUpdateBinarySem_attributes = {
  .name = "PCUpdateBinarySem"
};
/* Definitions for CustomBinarySem */
osSemaphoreId_t CustomBinarySemHandle;
const osSemaphoreAttr_t CustomBinarySem_attributes = {
  .name = "CustomBinarySem"
};
/* Definitions for judgementInitBinarySem */
osSemaphoreId_t judgementInitBinarySemHandle;
const osSemaphoreAttr_t judgementInitBinarySem_attributes = {
  .name = "judgementInitBinarySem"
};
/* Definitions for customRxBinarySem */
osSemaphoreId_t customRxBinarySemHandle;
const osSemaphoreAttr_t customRxBinarySem_attributes = {
  .name = "customRxBinarySem"
};
/* Definitions for CommunicateUpdateBinarySem */
osSemaphoreId_t CommunicateUpdateBinarySemHandle;
const osSemaphoreAttr_t CommunicateUpdateBinarySem_attributes = {
  .name = "CommunicateUpdateBinarySem"
};
/* Definitions for IMUUpdateBinarySem */
osSemaphoreId_t IMUUpdateBinarySemHandle;
const osSemaphoreAttr_t IMUUpdateBinarySem_attributes = {
  .name = "IMUUpdateBinarySem"
};
/* Definitions for servoctrlTxBinarySem */
osSemaphoreId_t servoctrlTxBinarySemHandle;
const osSemaphoreAttr_t servoctrlTxBinarySem_attributes = {
  .name = "servoctrlTxBinarySem"
};
/* Definitions for hi229umRxBinarySem */
osSemaphoreId_t hi229umRxBinarySemHandle;
const osSemaphoreAttr_t hi229umRxBinarySem_attributes = {
  .name = "hi229umRxBinarySem"
};
/* Definitions for CAN1CountingSem */
osSemaphoreId_t CAN1CountingSemHandle;
const osSemaphoreAttr_t CAN1CountingSem_attributes = {
  .name = "CAN1CountingSem"
};
/* Definitions for CAN2CountingSem */
osSemaphoreId_t CAN2CountingSemHandle;
const osSemaphoreAttr_t CAN2CountingSem_attributes = {
  .name = "CAN2CountingSem"
};
/* Definitions for refereeEvent */
osEventFlagsId_t refereeEventHandle;
const osEventFlagsAttr_t refereeEvent_attributes = {
  .name = "refereeEvent"
};
/* Definitions for RefereeEvent */
osEventFlagsId_t RefereeEventHandle;
const osEventFlagsAttr_t RefereeEvent_attributes = {
  .name = "RefereeEvent"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void arm_task(void *argument);
void lost_check_task(void *argument);
void rc_task(void *argument);
void pc_receive_task(void *argument);
void pc_transmit_task(void *argument);
void receiveCtrl_task(void *argument);
void updateUIData_task(void *argument);
void judgeCtrl_task(void *argument);
void controller_task(void *argument);
void imu_task(void *argument);
void communicate_task(void *argument);
void servo_ctrl_task(void *argument);
void gimbal_task(void *argument);
void hi229um_task(void *argument);

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
  /* creation of ArmMotor1UpdateBinarySem */
  ArmMotor1UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor1UpdateBinarySem_attributes);

  /* creation of ArmMotor2UpdateBinarySem */
  ArmMotor2UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor2UpdateBinarySem_attributes);

  /* creation of ArmMotor3UpdateBinarySem */
  ArmMotor3UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor3UpdateBinarySem_attributes);

  /* creation of ArmMotor4UpdateBinarySem */
  ArmMotor4UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor4UpdateBinarySem_attributes);

  /* creation of ArmMotor5UpdateBinarySem */
  ArmMotor5UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor5UpdateBinarySem_attributes);

  /* creation of ArmMotor6UpdateBinarySem */
  ArmMotor6UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmMotor6UpdateBinarySem_attributes);

  /* creation of RCUpdateBinarySem */
  RCUpdateBinarySemHandle = osSemaphoreNew(1, 0, &RCUpdateBinarySem_attributes);

  /* creation of PCUpdateBinarySem */
  PCUpdateBinarySemHandle = osSemaphoreNew(1, 0, &PCUpdateBinarySem_attributes);

  /* creation of CustomBinarySem */
  CustomBinarySemHandle = osSemaphoreNew(1, 0, &CustomBinarySem_attributes);

  /* creation of judgementInitBinarySem */
  judgementInitBinarySemHandle = osSemaphoreNew(1, 0, &judgementInitBinarySem_attributes);

  /* creation of customRxBinarySem */
  customRxBinarySemHandle = osSemaphoreNew(1, 0, &customRxBinarySem_attributes);

  /* creation of CommunicateUpdateBinarySem */
  CommunicateUpdateBinarySemHandle = osSemaphoreNew(1, 0, &CommunicateUpdateBinarySem_attributes);

  /* creation of IMUUpdateBinarySem */
  IMUUpdateBinarySemHandle = osSemaphoreNew(1, 0, &IMUUpdateBinarySem_attributes);

  /* creation of servoctrlTxBinarySem */
  servoctrlTxBinarySemHandle = osSemaphoreNew(1, 1, &servoctrlTxBinarySem_attributes);

  /* creation of hi229umRxBinarySem */
  hi229umRxBinarySemHandle = osSemaphoreNew(1, 0, &hi229umRxBinarySem_attributes);

  /* creation of CAN1CountingSem */
  CAN1CountingSemHandle = osSemaphoreNew(3, 3, &CAN1CountingSem_attributes);

  /* creation of CAN2CountingSem */
  CAN2CountingSemHandle = osSemaphoreNew(3, 3, &CAN2CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CAN1SendQueue */
  CAN1SendQueueHandle = osMessageQueueNew (32, sizeof(can_device_transmit_member), &CAN1SendQueue_attributes);

  /* creation of CAN2SendQueue */
  CAN2SendQueueHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &CAN2SendQueue_attributes);

  /* creation of ServoCtrlQueue */
  ServoCtrlQueueHandle = osMessageQueueNew (16, sizeof(servo_ctrl_data_t), &ServoCtrlQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN1SendTask */
  CAN1SendTaskHandle = osThreadNew(CAN1Send_Task, NULL, &CAN1SendTask_attributes);

  /* creation of CAN2SendTask */
  CAN2SendTaskHandle = osThreadNew(CAN2Send_Task, NULL, &CAN2SendTask_attributes);

  /* creation of armtask */
  armtaskHandle = osThreadNew(arm_task, NULL, &armtask_attributes);

  /* creation of lostchecktask */
  lostchecktaskHandle = osThreadNew(lost_check_task, NULL, &lostchecktask_attributes);

  /* creation of rctask */
  rctaskHandle = osThreadNew(rc_task, NULL, &rctask_attributes);

  /* creation of pc_receivetask */
  pc_receivetaskHandle = osThreadNew(pc_receive_task, NULL, &pc_receivetask_attributes);

  /* creation of pc_transmittask */
  pc_transmittaskHandle = osThreadNew(pc_transmit_task, NULL, &pc_transmittask_attributes);

  /* creation of receiveCtrltask */
  receiveCtrltaskHandle = osThreadNew(receiveCtrl_task, NULL, &receiveCtrltask_attributes);

  /* creation of updateUIDatatas */
  updateUIDatatasHandle = osThreadNew(updateUIData_task, NULL, &updateUIDatatas_attributes);

  /* creation of judgeCtrltask */
  judgeCtrltaskHandle = osThreadNew(judgeCtrl_task, NULL, &judgeCtrltask_attributes);

  /* creation of controllertask */
  controllertaskHandle = osThreadNew(controller_task, NULL, &controllertask_attributes);

  /* creation of imutask */
  imutaskHandle = osThreadNew(imu_task, NULL, &imutask_attributes);

  /* creation of communicatetask */
  communicatetaskHandle = osThreadNew(communicate_task, NULL, &communicatetask_attributes);

  /* creation of servo_ctrltask */
  servo_ctrltaskHandle = osThreadNew(servo_ctrl_task, NULL, &servo_ctrltask_attributes);

  /* creation of gimbaltask */
  gimbaltaskHandle = osThreadNew(gimbal_task, NULL, &gimbaltask_attributes);

  /* creation of hi229umtask */
  hi229umtaskHandle = osThreadNew(hi229um_task, NULL, &hi229umtask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of refereeEvent */
  refereeEventHandle = osEventFlagsNew(&refereeEvent_attributes);

  /* creation of RefereeEvent */
  RefereeEventHandle = osEventFlagsNew(&RefereeEvent_attributes);

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
      HAL_IWDG_Refresh(&hiwdg);
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
      osDelay(500);  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_CAN1Send_Task */
/**
* @brief Function implementing the CAN1SendTask thread.
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
* @brief Function implementing the CAN2SendTask thread.
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

/* USER CODE BEGIN Header_arm_task */
/**
* @brief Function implementing the armtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_arm_task */
__weak void arm_task(void *argument)
{
  /* USER CODE BEGIN arm_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END arm_task */
}

/* USER CODE BEGIN Header_lost_check_task */
/**
* @brief Function implementing the lostchecktask thread.
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

/* USER CODE BEGIN Header_pc_receive_task */
/**
* @brief Function implementing the pc_receivetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc_receive_task */
__weak void pc_receive_task(void *argument)
{
  /* USER CODE BEGIN pc_receive_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pc_receive_task */
}

/* USER CODE BEGIN Header_pc_transmit_task */
/**
* @brief Function implementing the pc_transmittask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pc_transmit_task */
__weak void pc_transmit_task(void *argument)
{
  /* USER CODE BEGIN pc_transmit_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pc_transmit_task */
}

/* USER CODE BEGIN Header_receiveCtrl_task */
/**
* @brief Function implementing the receiveCtrltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_receiveCtrl_task */
__weak void receiveCtrl_task(void *argument)
{
  /* USER CODE BEGIN receiveCtrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END receiveCtrl_task */
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

/* USER CODE BEGIN Header_controller_task */
/**
* @brief Function implementing the controllertask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_controller_task */
__weak void controller_task(void *argument)
{
  /* USER CODE BEGIN controller_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END controller_task */
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

/* USER CODE BEGIN Header_communicate_task */
/**
* @brief Function implementing the communicatetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_communicate_task */
__weak void communicate_task(void *argument)
{
  /* USER CODE BEGIN communicate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END communicate_task */
}

/* USER CODE BEGIN Header_servo_ctrl_task */
/**
* @brief Function implementing the servo_ctrltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_ctrl_task */
__weak void servo_ctrl_task(void *argument)
{
  /* USER CODE BEGIN servo_ctrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END servo_ctrl_task */
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

/* USER CODE BEGIN Header_hi229um_task */
/**
* @brief Function implementing the hi229umtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hi229um_task */
__weak void hi229um_task(void *argument)
{
  /* USER CODE BEGIN hi229um_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END hi229um_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

