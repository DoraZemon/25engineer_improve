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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pumptask */
osThreadId_t pumptaskHandle;
const osThreadAttr_t pumptask_attributes = {
  .name = "pumptask",
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
/* Definitions for chassistask */
osThreadId_t chassistaskHandle;
const osThreadAttr_t chassistask_attributes = {
  .name = "chassistask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lostcheck_task */
osThreadId_t lostcheck_taskHandle;
const osThreadAttr_t lostcheck_task_attributes = {
  .name = "lostcheck_task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for communicatetask */
osThreadId_t communicatetaskHandle;
const osThreadAttr_t communicatetask_attributes = {
  .name = "communicatetask",
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
/* Definitions for adcUpdateBinarySem */
osSemaphoreId_t adcUpdateBinarySemHandle;
const osSemaphoreAttr_t adcUpdateBinarySem_attributes = {
  .name = "adcUpdateBinarySem"
};
/* Definitions for ChassisMotor1UpdateBinarySem */
osSemaphoreId_t ChassisMotor1UpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisMotor1UpdateBinarySem_attributes = {
  .name = "ChassisMotor1UpdateBinarySem"
};
/* Definitions for ChassisMotor2UpdateBinarySem */
osSemaphoreId_t ChassisMotor2UpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisMotor2UpdateBinarySem_attributes = {
  .name = "ChassisMotor2UpdateBinarySem"
};
/* Definitions for ChassisMotor3UpdateBinarySem */
osSemaphoreId_t ChassisMotor3UpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisMotor3UpdateBinarySem_attributes = {
  .name = "ChassisMotor3UpdateBinarySem"
};
/* Definitions for ChassisMotor4UpdateBinarySem */
osSemaphoreId_t ChassisMotor4UpdateBinarySemHandle;
const osSemaphoreAttr_t ChassisMotor4UpdateBinarySem_attributes = {
  .name = "ChassisMotor4UpdateBinarySem"
};
/* Definitions for IMUUpdateBinarySem */
osSemaphoreId_t IMUUpdateBinarySemHandle;
const osSemaphoreAttr_t IMUUpdateBinarySem_attributes = {
  .name = "IMUUpdateBinarySem"
};
/* Definitions for CommunicateUpdateBinarySem */
osSemaphoreId_t CommunicateUpdateBinarySemHandle;
const osSemaphoreAttr_t CommunicateUpdateBinarySem_attributes = {
  .name = "CommunicateUpdateBinarySem"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void pump_task(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void chassis_task(void *argument);
void lost_check_task(void *argument);
void communicate_task(void *argument);

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
  /* creation of adcUpdateBinarySem */
  adcUpdateBinarySemHandle = osSemaphoreNew(1, 0, &adcUpdateBinarySem_attributes);

  /* creation of ChassisMotor1UpdateBinarySem */
  ChassisMotor1UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisMotor1UpdateBinarySem_attributes);

  /* creation of ChassisMotor2UpdateBinarySem */
  ChassisMotor2UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisMotor2UpdateBinarySem_attributes);

  /* creation of ChassisMotor3UpdateBinarySem */
  ChassisMotor3UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisMotor3UpdateBinarySem_attributes);

  /* creation of ChassisMotor4UpdateBinarySem */
  ChassisMotor4UpdateBinarySemHandle = osSemaphoreNew(1, 0, &ChassisMotor4UpdateBinarySem_attributes);

  /* creation of IMUUpdateBinarySem */
  IMUUpdateBinarySemHandle = osSemaphoreNew(1, 0, &IMUUpdateBinarySem_attributes);

  /* creation of CommunicateUpdateBinarySem */
  CommunicateUpdateBinarySemHandle = osSemaphoreNew(1, 0, &CommunicateUpdateBinarySem_attributes);

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

  /* creation of pumptask */
  pumptaskHandle = osThreadNew(pump_task, NULL, &pumptask_attributes);

  /* creation of CAN1SendTask */
  CAN1SendTaskHandle = osThreadNew(CAN1Send_Task, NULL, &CAN1SendTask_attributes);

  /* creation of CAN2SendTask */
  CAN2SendTaskHandle = osThreadNew(CAN2Send_Task, NULL, &CAN2SendTask_attributes);

  /* creation of chassistask */
  chassistaskHandle = osThreadNew(chassis_task, NULL, &chassistask_attributes);

  /* creation of lostcheck_task */
  lostcheck_taskHandle = osThreadNew(lost_check_task, NULL, &lostcheck_task_attributes);

  /* creation of communicatetask */
  communicatetaskHandle = osThreadNew(communicate_task, NULL, &communicatetask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_pump_task */
/**
* @brief Function implementing the pumptask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pump_task */
__weak void pump_task(void *argument)
{
  /* USER CODE BEGIN pump_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END pump_task */
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

/* USER CODE BEGIN Header_chassis_task */
/**
* @brief Function implementing the chassistask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_lost_check_task */
/**
* @brief Function implementing the lostcheck_task thread.
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

