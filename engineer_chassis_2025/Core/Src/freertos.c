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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adctask */
osThreadId_t adctaskHandle;
const osThreadAttr_t adctask_attributes = {
  .name = "adctask",
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
/* Definitions for lefttofTask */
osThreadId_t lefttofTaskHandle;
const osThreadAttr_t lefttofTask_attributes = {
  .name = "lefttofTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for righttofTask */
osThreadId_t righttofTaskHandle;
const osThreadAttr_t righttofTask_attributes = {
  .name = "righttofTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tofsendTask */
osThreadId_t tofsendTaskHandle;
const osThreadAttr_t tofsendTask_attributes = {
  .name = "tofsendTask",
  .stack_size = 128 * 4,
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
/* Definitions for LeftTofUpdateBinarySem */
osSemaphoreId_t LeftTofUpdateBinarySemHandle;
const osSemaphoreAttr_t LeftTofUpdateBinarySem_attributes = {
  .name = "LeftTofUpdateBinarySem"
};
/* Definitions for RightTofUpdateBinarySem */
osSemaphoreId_t RightTofUpdateBinarySemHandle;
const osSemaphoreAttr_t RightTofUpdateBinarySem_attributes = {
  .name = "RightTofUpdateBinarySem"
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
void adc_task(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void left_tof_task(void *argument);
void right_tof_task(void *argument);
void tof_send_task(void *argument);

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

  /* creation of LeftTofUpdateBinarySem */
  LeftTofUpdateBinarySemHandle = osSemaphoreNew(1, 0, &LeftTofUpdateBinarySem_attributes);

  /* creation of RightTofUpdateBinarySem */
  RightTofUpdateBinarySemHandle = osSemaphoreNew(1, 0, &RightTofUpdateBinarySem_attributes);

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

  /* creation of adctask */
  adctaskHandle = osThreadNew(adc_task, NULL, &adctask_attributes);

  /* creation of CAN1SendTask */
  CAN1SendTaskHandle = osThreadNew(CAN1Send_Task, NULL, &CAN1SendTask_attributes);

  /* creation of CAN2SendTask */
  CAN2SendTaskHandle = osThreadNew(CAN2Send_Task, NULL, &CAN2SendTask_attributes);

  /* creation of lefttofTask */
  lefttofTaskHandle = osThreadNew(left_tof_task, NULL, &lefttofTask_attributes);

  /* creation of righttofTask */
  righttofTaskHandle = osThreadNew(right_tof_task, NULL, &righttofTask_attributes);

  /* creation of tofsendTask */
  tofsendTaskHandle = osThreadNew(tof_send_task, NULL, &tofsendTask_attributes);

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

/* USER CODE BEGIN Header_adc_task */
/**
* @brief Function implementing the adctask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adc_task */
__weak void adc_task(void *argument)
{
  /* USER CODE BEGIN adc_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END adc_task */
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

/* USER CODE BEGIN Header_left_tof_task */
/**
* @brief Function implementing the lefttofTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_left_tof_task */
__weak void left_tof_task(void *argument)
{
  /* USER CODE BEGIN left_tof_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END left_tof_task */
}

/* USER CODE BEGIN Header_right_tof_task */
/**
* @brief Function implementing the righttofTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_right_tof_task */
__weak void right_tof_task(void *argument)
{
  /* USER CODE BEGIN right_tof_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END right_tof_task */
}

/* USER CODE BEGIN Header_tof_send_task */
/**
* @brief Function implementing the tofsendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_send_task */
__weak void tof_send_task(void *argument)
{
  /* USER CODE BEGIN tof_send_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tof_send_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

