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
/* Definitions for imutask */
osThreadId_t imutaskHandle;
const osThreadAttr_t imutask_attributes = {
  .name = "imutask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rctask */
osThreadId_t rctaskHandle;
const osThreadAttr_t rctask_attributes = {
  .name = "rctask",
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
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for armtask */
osThreadId_t armtaskHandle;
const osThreadAttr_t armtask_attributes = {
  .name = "armtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for absorbtask */
osThreadId_t absorbtaskHandle;
const osThreadAttr_t absorbtask_attributes = {
  .name = "absorbtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbalattitudet */
osThreadId_t gimbalattitudetHandle;
const osThreadAttr_t gimbalattitudet_attributes = {
  .name = "gimbalattitudet",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for infotask */
osThreadId_t infotaskHandle;
const osThreadAttr_t infotask_attributes = {
  .name = "infotask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for kb_eventtask */
osThreadId_t kb_eventtaskHandle;
const osThreadAttr_t kb_eventtask_attributes = {
  .name = "kb_eventtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for customEXtask */
osThreadId_t customEXtaskHandle;
const osThreadAttr_t customEXtask_attributes = {
  .name = "customEXtask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for autoControltask */
osThreadId_t autoControltaskHandle;
const osThreadAttr_t autoControltask_attributes = {
  .name = "autoControltask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for kb_statetask */
osThreadId_t kb_statetaskHandle;
const osThreadAttr_t kb_statetask_attributes = {
  .name = "kb_statetask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbtask */
osThreadId_t usbtaskHandle;
const osThreadAttr_t usbtask_attributes = {
  .name = "usbtask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_checktask */
osThreadId_t motor_checktaskHandle;
const osThreadAttr_t motor_checktask_attributes = {
  .name = "motor_checktask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for judgeCtrltask */
osThreadId_t judgeCtrltaskHandle;
const osThreadAttr_t judgeCtrltask_attributes = {
  .name = "judgeCtrltask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for updateUIDatatas */
osThreadId_t updateUIDatatasHandle;
const osThreadAttr_t updateUIDatatas_attributes = {
  .name = "updateUIDatatas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for receiveCtrltask */
osThreadId_t receiveCtrltaskHandle;
const osThreadAttr_t receiveCtrltask_attributes = {
  .name = "receiveCtrltask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Interpolator_ta */
osThreadId_t Interpolator_taHandle;
const osThreadAttr_t Interpolator_ta_attributes = {
  .name = "Interpolator_ta",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo_ctrltask */
osThreadId_t servo_ctrltaskHandle;
const osThreadAttr_t servo_ctrltask_attributes = {
  .name = "servo_ctrltask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for hi229umtask */
osThreadId_t hi229umtaskHandle;
const osThreadAttr_t hi229umtask_attributes = {
  .name = "hi229umtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for clawtask */
osThreadId_t clawtaskHandle;
const osThreadAttr_t clawtask_attributes = {
  .name = "clawtask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for gimbalslidetask */
osThreadId_t gimbalslidetaskHandle;
const osThreadAttr_t gimbalslidetask_attributes = {
  .name = "gimbalslidetask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for tof_checktask */
osThreadId_t tof_checktaskHandle;
const osThreadAttr_t tof_checktask_attributes = {
  .name = "tof_checktask",
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
/* Definitions for ServoCtrlQueue */
osMessageQueueId_t ServoCtrlQueueHandle;
const osMessageQueueAttr_t ServoCtrlQueue_attributes = {
  .name = "ServoCtrlQueue"
};
/* Definitions for RCUpdateBinarySem */
osSemaphoreId_t RCUpdateBinarySemHandle;
const osSemaphoreAttr_t RCUpdateBinarySem_attributes = {
  .name = "RCUpdateBinarySem"
};
/* Definitions for ImuDMABinarySem */
osSemaphoreId_t ImuDMABinarySemHandle;
const osSemaphoreAttr_t ImuDMABinarySem_attributes = {
  .name = "ImuDMABinarySem"
};
/* Definitions for ArmUpdateBinarySem */
osSemaphoreId_t ArmUpdateBinarySemHandle;
const osSemaphoreAttr_t ArmUpdateBinarySem_attributes = {
  .name = "ArmUpdateBinarySem"
};
/* Definitions for ArmResetInitBinarySem */
osSemaphoreId_t ArmResetInitBinarySemHandle;
const osSemaphoreAttr_t ArmResetInitBinarySem_attributes = {
  .name = "ArmResetInitBinarySem"
};
/* Definitions for CustomBinarySem */
osSemaphoreId_t CustomBinarySemHandle;
const osSemaphoreAttr_t CustomBinarySem_attributes = {
  .name = "CustomBinarySem"
};
/* Definitions for chassisLfUpdateBinarySem */
osSemaphoreId_t chassisLfUpdateBinarySemHandle;
const osSemaphoreAttr_t chassisLfUpdateBinarySem_attributes = {
  .name = "chassisLfUpdateBinarySem"
};
/* Definitions for chassisLbUpdateBinarySem */
osSemaphoreId_t chassisLbUpdateBinarySemHandle;
const osSemaphoreAttr_t chassisLbUpdateBinarySem_attributes = {
  .name = "chassisLbUpdateBinarySem"
};
/* Definitions for chassisRfUpdateBinarySem */
osSemaphoreId_t chassisRfUpdateBinarySemHandle;
const osSemaphoreAttr_t chassisRfUpdateBinarySem_attributes = {
  .name = "chassisRfUpdateBinarySem"
};
/* Definitions for chassisRbUpdateBinarySem */
osSemaphoreId_t chassisRbUpdateBinarySemHandle;
const osSemaphoreAttr_t chassisRbUpdateBinarySem_attributes = {
  .name = "chassisRbUpdateBinarySem"
};
/* Definitions for gimbalYawUpdateBinarySem */
osSemaphoreId_t gimbalYawUpdateBinarySemHandle;
const osSemaphoreAttr_t gimbalYawUpdateBinarySem_attributes = {
  .name = "gimbalYawUpdateBinarySem"
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
/* Definitions for AbsorbUpdateBinarySem */
osSemaphoreId_t AbsorbUpdateBinarySemHandle;
const osSemaphoreAttr_t AbsorbUpdateBinarySem_attributes = {
  .name = "AbsorbUpdateBinarySem"
};
/* Definitions for clawUpdateBinarySem */
osSemaphoreId_t clawUpdateBinarySemHandle;
const osSemaphoreAttr_t clawUpdateBinarySem_attributes = {
  .name = "clawUpdateBinarySem"
};
/* Definitions for gimbalSlideUpdateBinarySem */
osSemaphoreId_t gimbalSlideUpdateBinarySemHandle;
const osSemaphoreAttr_t gimbalSlideUpdateBinarySem_attributes = {
  .name = "gimbalSlideUpdateBinarySem"
};
/* Definitions for TofUpdateBinarySem */
osSemaphoreId_t TofUpdateBinarySemHandle;
const osSemaphoreAttr_t TofUpdateBinarySem_attributes = {
  .name = "TofUpdateBinarySem"
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
void imu_task(void *argument);
void rc_task(void *argument);
void CAN1Send_Task(void *argument);
void CAN2Send_Task(void *argument);
void chassis_task(void *argument);
void arm_task(void *argument);
void absorb_task(void *argument);
void gimbalattitude_task(void *argument);
void info_task(void *argument);
void kb_event_task(void *argument);
void customEX_task(void *argument);
void autoControl_task(void *argument);
void kb_state_task(void *argument);
void usb_task(void *argument);
void motor_check_task(void *argument);
void judgeCtrl_task(void *argument);
void updateUIData_task(void *argument);
void receiveCtrl_task(void *argument);
void customInterpolatorCtrl_task(void *argument);
void servo_ctrl_task(void *argument);
void hi229um_task(void *argument);
void claw_task(void *argument);
void gimbalslide_task(void *argument);
void tof_check_task(void *argument);

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
  /* creation of RCUpdateBinarySem */
  RCUpdateBinarySemHandle = osSemaphoreNew(1, 0, &RCUpdateBinarySem_attributes);

  /* creation of ImuDMABinarySem */
  ImuDMABinarySemHandle = osSemaphoreNew(1, 0, &ImuDMABinarySem_attributes);

  /* creation of ArmUpdateBinarySem */
  ArmUpdateBinarySemHandle = osSemaphoreNew(1, 0, &ArmUpdateBinarySem_attributes);

  /* creation of ArmResetInitBinarySem */
  ArmResetInitBinarySemHandle = osSemaphoreNew(1, 0, &ArmResetInitBinarySem_attributes);

  /* creation of CustomBinarySem */
  CustomBinarySemHandle = osSemaphoreNew(1, 0, &CustomBinarySem_attributes);

  /* creation of chassisLfUpdateBinarySem */
  chassisLfUpdateBinarySemHandle = osSemaphoreNew(1, 0, &chassisLfUpdateBinarySem_attributes);

  /* creation of chassisLbUpdateBinarySem */
  chassisLbUpdateBinarySemHandle = osSemaphoreNew(1, 0, &chassisLbUpdateBinarySem_attributes);

  /* creation of chassisRfUpdateBinarySem */
  chassisRfUpdateBinarySemHandle = osSemaphoreNew(1, 0, &chassisRfUpdateBinarySem_attributes);

  /* creation of chassisRbUpdateBinarySem */
  chassisRbUpdateBinarySemHandle = osSemaphoreNew(1, 0, &chassisRbUpdateBinarySem_attributes);

  /* creation of gimbalYawUpdateBinarySem */
  gimbalYawUpdateBinarySemHandle = osSemaphoreNew(1, 0, &gimbalYawUpdateBinarySem_attributes);

  /* creation of judgementInitBinarySem */
  judgementInitBinarySemHandle = osSemaphoreNew(1, 0, &judgementInitBinarySem_attributes);

  /* creation of customRxBinarySem */
  customRxBinarySemHandle = osSemaphoreNew(1, 0, &customRxBinarySem_attributes);

  /* creation of servoctrlTxBinarySem */
  servoctrlTxBinarySemHandle = osSemaphoreNew(1, 0, &servoctrlTxBinarySem_attributes);

  /* creation of hi229umRxBinarySem */
  hi229umRxBinarySemHandle = osSemaphoreNew(1, 0, &hi229umRxBinarySem_attributes);

  /* creation of AbsorbUpdateBinarySem */
  AbsorbUpdateBinarySemHandle = osSemaphoreNew(1, 0, &AbsorbUpdateBinarySem_attributes);

  /* creation of clawUpdateBinarySem */
  clawUpdateBinarySemHandle = osSemaphoreNew(1, 0, &clawUpdateBinarySem_attributes);

  /* creation of gimbalSlideUpdateBinarySem */
  gimbalSlideUpdateBinarySemHandle = osSemaphoreNew(1, 0, &gimbalSlideUpdateBinarySem_attributes);

  /* creation of TofUpdateBinarySem */
  TofUpdateBinarySemHandle = osSemaphoreNew(1, 0, &TofUpdateBinarySem_attributes);

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

  /* creation of imutask */
  imutaskHandle = osThreadNew(imu_task, NULL, &imutask_attributes);

  /* creation of rctask */
  rctaskHandle = osThreadNew(rc_task, NULL, &rctask_attributes);

  /* creation of CAN1SendTask */
  CAN1SendTaskHandle = osThreadNew(CAN1Send_Task, NULL, &CAN1SendTask_attributes);

  /* creation of CAN2SendTask */
  CAN2SendTaskHandle = osThreadNew(CAN2Send_Task, NULL, &CAN2SendTask_attributes);

  /* creation of chassistask */
  chassistaskHandle = osThreadNew(chassis_task, NULL, &chassistask_attributes);

  /* creation of armtask */
  armtaskHandle = osThreadNew(arm_task, NULL, &armtask_attributes);

  /* creation of absorbtask */
  absorbtaskHandle = osThreadNew(absorb_task, NULL, &absorbtask_attributes);

  /* creation of gimbalattitudet */
  gimbalattitudetHandle = osThreadNew(gimbalattitude_task, NULL, &gimbalattitudet_attributes);

  /* creation of infotask */
  infotaskHandle = osThreadNew(info_task, NULL, &infotask_attributes);

  /* creation of kb_eventtask */
  kb_eventtaskHandle = osThreadNew(kb_event_task, NULL, &kb_eventtask_attributes);

  /* creation of customEXtask */
  customEXtaskHandle = osThreadNew(customEX_task, NULL, &customEXtask_attributes);

  /* creation of autoControltask */
  autoControltaskHandle = osThreadNew(autoControl_task, NULL, &autoControltask_attributes);

  /* creation of kb_statetask */
  kb_statetaskHandle = osThreadNew(kb_state_task, NULL, &kb_statetask_attributes);

  /* creation of usbtask */
  usbtaskHandle = osThreadNew(usb_task, NULL, &usbtask_attributes);

  /* creation of motor_checktask */
  motor_checktaskHandle = osThreadNew(motor_check_task, NULL, &motor_checktask_attributes);

  /* creation of judgeCtrltask */
  judgeCtrltaskHandle = osThreadNew(judgeCtrl_task, NULL, &judgeCtrltask_attributes);

  /* creation of updateUIDatatas */
  updateUIDatatasHandle = osThreadNew(updateUIData_task, NULL, &updateUIDatatas_attributes);

  /* creation of receiveCtrltask */
  receiveCtrltaskHandle = osThreadNew(receiveCtrl_task, NULL, &receiveCtrltask_attributes);

  /* creation of Interpolator_ta */
  Interpolator_taHandle = osThreadNew(customInterpolatorCtrl_task, NULL, &Interpolator_ta_attributes);

  /* creation of servo_ctrltask */
  servo_ctrltaskHandle = osThreadNew(servo_ctrl_task, NULL, &servo_ctrltask_attributes);

  /* creation of hi229umtask */
  hi229umtaskHandle = osThreadNew(hi229um_task, NULL, &hi229umtask_attributes);

  /* creation of clawtask */
  clawtaskHandle = osThreadNew(claw_task, NULL, &clawtask_attributes);

  /* creation of gimbalslidetask */
  gimbalslidetaskHandle = osThreadNew(gimbalslide_task, NULL, &gimbalslidetask_attributes);

  /* creation of tof_checktask */
  tof_checktaskHandle = osThreadNew(tof_check_task, NULL, &tof_checktask_attributes);

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
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
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

/* USER CODE BEGIN Header_absorb_task */
/**
* @brief Function implementing the absorbtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_absorb_task */
__weak void absorb_task(void *argument)
{
  /* USER CODE BEGIN absorb_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END absorb_task */
}

/* USER CODE BEGIN Header_gimbalattitude_task */
/**
* @brief Function implementing the gimbalattitudet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbalattitude_task */
__weak void gimbalattitude_task(void *argument)
{
  /* USER CODE BEGIN gimbalattitude_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbalattitude_task */
}

/* USER CODE BEGIN Header_info_task */
/**
* @brief Function implementing the infotask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_info_task */
__weak void info_task(void *argument)
{
  /* USER CODE BEGIN info_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END info_task */
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

/* USER CODE BEGIN Header_customEX_task */
/**
* @brief Function implementing the customEXtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_customEX_task */
__weak void customEX_task(void *argument)
{
  /* USER CODE BEGIN customEX_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END customEX_task */
}

/* USER CODE BEGIN Header_autoControl_task */
/**
* @brief Function implementing the autoControltask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_autoControl_task */
__weak void autoControl_task(void *argument)
{
  /* USER CODE BEGIN autoControl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END autoControl_task */
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

/* USER CODE BEGIN Header_usb_task */
/**
* @brief Function implementing the usbtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usb_task */
__weak void usb_task(void *argument)
{
  /* USER CODE BEGIN usb_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usb_task */
}

/* USER CODE BEGIN Header_motor_check_task */
/**
* @brief Function implementing the motor_checktask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_check_task */
__weak void motor_check_task(void *argument)
{
  /* USER CODE BEGIN motor_check_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_check_task */
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

/* USER CODE BEGIN Header_customInterpolatorCtrl_task */
/**
* @brief Function implementing the Interpolator_ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_customInterpolatorCtrl_task */
__weak void customInterpolatorCtrl_task(void *argument)
{
  /* USER CODE BEGIN customInterpolatorCtrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END customInterpolatorCtrl_task */
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

/* USER CODE BEGIN Header_claw_task */
/**
* @brief Function implementing the clawtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_claw_task */
__weak void claw_task(void *argument)
{
  /* USER CODE BEGIN claw_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END claw_task */
}

/* USER CODE BEGIN Header_gimbalslide_task */
/**
* @brief Function implementing the gimbalslidetask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gimbalslide_task */
__weak void gimbalslide_task(void *argument)
{
  /* USER CODE BEGIN gimbalslide_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END gimbalslide_task */
}

/* USER CODE BEGIN Header_tof_check_task */
/**
* @brief Function implementing the tof_checktask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_tof_check_task */
__weak void tof_check_task(void *argument)
{
  /* USER CODE BEGIN tof_check_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END tof_check_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

