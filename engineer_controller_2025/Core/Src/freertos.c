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
#include "iwdg.h"
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
/* Definitions for judgeCtrltask */
osThreadId_t judgeCtrltaskHandle;
const osThreadAttr_t judgeCtrltask_attributes = {
  .name = "judgeCtrltask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armtask */
osThreadId_t armtaskHandle;
const osThreadAttr_t armtask_attributes = {
  .name = "armtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for refereeupdatetask */
osThreadId_t refereeupdatetaskHandle;
const osThreadAttr_t refereeupdatetask_attributes = {
  .name = "refereeupdatetask",
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void judgeCtrl_task(void *argument);
void arm_task(void *argument);
void refereeupdate_task(void *argument);
void lost_check_task(void *argument);

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
  CAN1SendQueueHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &CAN1SendQueue_attributes);

  /* creation of CAN2SendQueue */
  CAN2SendQueueHandle = osMessageQueueNew (16, sizeof(can_device_transmit_member), &CAN2SendQueue_attributes);

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

  /* creation of judgeCtrltask */
  judgeCtrltaskHandle = osThreadNew(judgeCtrl_task, NULL, &judgeCtrltask_attributes);

  /* creation of armtask */
  armtaskHandle = osThreadNew(arm_task, NULL, &armtask_attributes);

  /* creation of refereeupdatetask */
  refereeupdatetaskHandle = osThreadNew(refereeupdate_task, NULL, &refereeupdatetask_attributes);

  /* creation of lostchecktask */
  lostchecktaskHandle = osThreadNew(lost_check_task, NULL, &lostchecktask_attributes);

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
      HAL_IWDG_Refresh(&hiwdg);//(625-1+1)*64/32000s
      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_R_Pin);
      osDelay(200);
//      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
//      osDelay(200);
      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_B_Pin);
      osDelay(200);
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
* @brief Function implementing the refereeupdatetask thread.
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

