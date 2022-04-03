/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
osThreadId defaultTaskHandle;
osThreadId GimbalHandle;
osThreadId BusCommHandle;
osThreadId ChassisHandle;
osThreadId ShootHandle;
osThreadId MiniPCHandle;
osThreadId RefereeHandle;
osThreadId WatchDogHandle;
osThreadId ClientHandle;
osThreadId InsHandle;
osThreadId InitHandle;
osThreadId SuperCapHandle;
osMessageQId Key_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Gimbal_Task(void const * argument);
void BusComm_Task(void const * argument);
void Chassis_Task(void const * argument);
void Shoot_Task(void const * argument);
void MiniPC_Task(void const * argument);
void Referee_Task(void const * argument);
void WatchDog_Task(void const * argument);
void Client_Task(void const * argument);
void Ins_Task(void const * argument);
void Init_Task(void const * argument);
void SuperCap_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Key_Queue */
  osMessageQDef(Key_Queue, 16, uint16_t);
  Key_QueueHandle = osMessageCreate(osMessageQ(Key_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Gimbal */
  osThreadDef(Gimbal, Gimbal_Task, osPriorityHigh, 0, 128);
  GimbalHandle = osThreadCreate(osThread(Gimbal), NULL);

  /* definition and creation of BusComm */
  osThreadDef(BusComm, BusComm_Task, osPriorityRealtime, 0, 128);
  BusCommHandle = osThreadCreate(osThread(BusComm), NULL);

  /* definition and creation of Chassis */
  osThreadDef(Chassis, Chassis_Task, osPriorityHigh, 0, 128);
  ChassisHandle = osThreadCreate(osThread(Chassis), NULL);

  /* definition and creation of Shoot */
  osThreadDef(Shoot, Shoot_Task, osPriorityHigh, 0, 128);
  ShootHandle = osThreadCreate(osThread(Shoot), NULL);

  /* definition and creation of MiniPC */
  osThreadDef(MiniPC, MiniPC_Task, osPriorityNormal, 0, 128);
  MiniPCHandle = osThreadCreate(osThread(MiniPC), NULL);

  /* definition and creation of Referee */
  osThreadDef(Referee, Referee_Task, osPriorityHigh, 0, 128);
  RefereeHandle = osThreadCreate(osThread(Referee), NULL);

  /* definition and creation of WatchDog */
  osThreadDef(WatchDog, WatchDog_Task, osPriorityNormal, 0, 128);
  WatchDogHandle = osThreadCreate(osThread(WatchDog), NULL);

  /* definition and creation of Client */
  osThreadDef(Client, Client_Task, osPriorityNormal, 0, 128);
  ClientHandle = osThreadCreate(osThread(Client), NULL);

  /* definition and creation of Ins */
  osThreadDef(Ins, Ins_Task, osPriorityNormal, 0, 128);
  InsHandle = osThreadCreate(osThread(Ins), NULL);

  /* definition and creation of Init */
  osThreadDef(Init, Init_Task, osPriorityRealtime, 0, 128);
  InitHandle = osThreadCreate(osThread(Init), NULL);

  /* definition and creation of SuperCap */
  osThreadDef(SuperCap, SuperCap_Task, osPriorityHigh, 0, 128);
  SuperCapHandle = osThreadCreate(osThread(SuperCap), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_BusComm_Task */
/**
* @brief Function implementing the BusComm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BusComm_Task */
__weak void BusComm_Task(void const * argument)
{
  /* USER CODE BEGIN BusComm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END BusComm_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}

/* USER CODE BEGIN Header_MiniPC_Task */
/**
* @brief Function implementing the MiniPC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MiniPC_Task */
__weak void MiniPC_Task(void const * argument)
{
  /* USER CODE BEGIN MiniPC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MiniPC_Task */
}

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
__weak void Referee_Task(void const * argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
__weak void WatchDog_Task(void const * argument)
{
  /* USER CODE BEGIN WatchDog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WatchDog_Task */
}

/* USER CODE BEGIN Header_Client_Task */
/**
* @brief Function implementing the Client thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Client_Task */
__weak void Client_Task(void const * argument)
{
  /* USER CODE BEGIN Client_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Client_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_Init_Task */
/**
* @brief Function implementing the Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_SuperCap_Task */
/**
* @brief Function implementing the SuperCap thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SuperCap_Task */
__weak void SuperCap_Task(void const * argument)
{
  /* USER CODE BEGIN SuperCap_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SuperCap_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

