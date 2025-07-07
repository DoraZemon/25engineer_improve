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
#include "bsp_ws2812.h"
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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FDCAN1_Send_Tas */
osThreadId_t FDCAN1_Send_TasHandle;
const osThreadAttr_t FDCAN1_Send_Tas_attributes = {
  .name = "FDCAN1_Send_Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FDCAN2_Send_Tas */
osThreadId_t FDCAN2_Send_TasHandle;
const osThreadAttr_t FDCAN2_Send_Tas_attributes = {
  .name = "FDCAN2_Send_Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FDCAN3_Send_Tas */
osThreadId_t FDCAN3_Send_TasHandle;
const osThreadAttr_t FDCAN3_Send_Tas_attributes = {
  .name = "FDCAN3_Send_Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for judgeCtrltask */
osThreadId_t judgeCtrltaskHandle;
const osThreadAttr_t judgeCtrltask_attributes = {
  .name = "judgeCtrltask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lostchecktask */
osThreadId_t lostchecktaskHandle;
const osThreadAttr_t lostchecktask_attributes = {
  .name = "lostchecktask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armtask */
osThreadId_t armtaskHandle;
const osThreadAttr_t armtask_attributes = {
  .name = "armtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for refereeupdateta */
osThreadId_t refereeupdatetaHandle;
const osThreadAttr_t refereeupdateta_attributes = {
  .name = "refereeupdateta",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FDCAN1SendQueue */
osMessageQueueId_t FDCAN1SendQueueHandle;
const osMessageQueueAttr_t FDCAN1SendQueue_attributes = {
  .name = "FDCAN1SendQueue"
};
/* Definitions for FDCAN2SendQueue */
osMessageQueueId_t FDCAN2SendQueueHandle;
const osMessageQueueAttr_t FDCAN2SendQueue_attributes = {
  .name = "FDCAN2SendQueue"
};
/* Definitions for FDCAN3SendQueue */
osMessageQueueId_t FDCAN3SendQueueHandle;
const osMessageQueueAttr_t FDCAN3SendQueue_attributes = {
  .name = "FDCAN3SendQueue"
};
/* Definitions for judgementInitBinarySem */
osSemaphoreId_t judgementInitBinarySemHandle;
const osSemaphoreAttr_t judgementInitBinarySem_attributes = {
  .name = "judgementInitBinarySem"
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
/* Definitions for FDCAN1CountingSem */
osSemaphoreId_t FDCAN1CountingSemHandle;
const osSemaphoreAttr_t FDCAN1CountingSem_attributes = {
  .name = "FDCAN1CountingSem"
};
/* Definitions for FDCAN2CountingSem */
osSemaphoreId_t FDCAN2CountingSemHandle;
const osSemaphoreAttr_t FDCAN2CountingSem_attributes = {
  .name = "FDCAN2CountingSem"
};
/* Definitions for FDCAN3CountingSem */
osSemaphoreId_t FDCAN3CountingSemHandle;
const osSemaphoreAttr_t FDCAN3CountingSem_attributes = {
  .name = "FDCAN3CountingSem"
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
void FDCAN1Send_Task(void *argument);
void FDCAN2Send_Task(void *argument);
void FDCAN3Send_Task(void *argument);
void judgeCtrl_task(void *argument);
void lost_check_task(void *argument);
void arm_task(void *argument);
void refereeupdate_task(void *argument);

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
  /* creation of judgementInitBinarySem */
  judgementInitBinarySemHandle = osSemaphoreNew(1, 0, &judgementInitBinarySem_attributes);

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

  /* creation of FDCAN1CountingSem */
  FDCAN1CountingSemHandle = osSemaphoreNew(8, 8, &FDCAN1CountingSem_attributes);

  /* creation of FDCAN2CountingSem */
  FDCAN2CountingSemHandle = osSemaphoreNew(8, 8, &FDCAN2CountingSem_attributes);

  /* creation of FDCAN3CountingSem */
  FDCAN3CountingSemHandle = osSemaphoreNew(8, 8, &FDCAN3CountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of FDCAN1SendQueue */
  FDCAN1SendQueueHandle = osMessageQueueNew (16, sizeof(fdcan_device_transmit_member), &FDCAN1SendQueue_attributes);

  /* creation of FDCAN2SendQueue */
  FDCAN2SendQueueHandle = osMessageQueueNew (16, sizeof(fdcan_device_transmit_member), &FDCAN2SendQueue_attributes);

  /* creation of FDCAN3SendQueue */
  FDCAN3SendQueueHandle = osMessageQueueNew (16, sizeof(fdcan_device_transmit_member), &FDCAN3SendQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of FDCAN1_Send_Tas */
  FDCAN1_Send_TasHandle = osThreadNew(FDCAN1Send_Task, NULL, &FDCAN1_Send_Tas_attributes);

  /* creation of FDCAN2_Send_Tas */
  FDCAN2_Send_TasHandle = osThreadNew(FDCAN2Send_Task, NULL, &FDCAN2_Send_Tas_attributes);

  /* creation of FDCAN3_Send_Tas */
  FDCAN3_Send_TasHandle = osThreadNew(FDCAN3Send_Task, NULL, &FDCAN3_Send_Tas_attributes);

  /* creation of judgeCtrltask */
  judgeCtrltaskHandle = osThreadNew(judgeCtrl_task, NULL, &judgeCtrltask_attributes);

  /* creation of lostchecktask */
  lostchecktaskHandle = osThreadNew(lost_check_task, NULL, &lostchecktask_attributes);

  /* creation of armtask */
  armtaskHandle = osThreadNew(arm_task, NULL, &armtask_attributes);

  /* creation of refereeupdateta */
  refereeupdatetaHandle = osThreadNew(refereeupdate_task, NULL, &refereeupdateta_attributes);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
      ws2812_flashing();
//    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_FDCAN1Send_Task */
/**
* @brief Function implementing th/e FDCAN1_Send_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FDCAN1Send_Task */
__weak void FDCAN1Send_Task(void *argument)
{
  /* USER CODE BEGIN FDCAN1Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FDCAN1Send_Task */
}

/* USER CODE BEGIN Header_FDCAN2Send_Task */
/**
* @brief Function implementing the FDCAN2_Send_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FDCAN2Send_Task */
__weak void FDCAN2Send_Task(void *argument)
{
  /* USER CODE BEGIN FDCAN2Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FDCAN2Send_Task */
}

/* USER CODE BEGIN Header_FDCAN3Send_Task */
/**
* @brief Function implementing the FDCAN3_Send_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FDCAN3Send_Task */
__weak void FDCAN3Send_Task(void *argument)
{
  /* USER CODE BEGIN FDCAN3Send_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END FDCAN3Send_Task */
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

/* USER CODE BEGIN Header_refereeupdate_task */
/**
* @brief Function implementing the refereeupdateta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_refereeupdate_task */
__weak void refereeupdate_task(void *argument)
{
  /* USER CODE BEGIN refereeupdate_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END refereeupdate_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

