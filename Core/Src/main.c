/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "semphr.h"
#include "FreeRTOS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Servo_no_Base 	 	 1
#define Servo_no_1 		 2
#define Servo_no_2 		 3
#define Servo_no_Gripper 	 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* Definitions for Angle_Cal */
osThreadId_t Angle_CalHandle;
const osThreadAttr_t Angle_Cal_attributes = {
  .name = "Angle_Cal",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Servo_Base */
osThreadId_t Servo_BaseHandle;
const osThreadAttr_t Servo_Base_attributes = {
  .name = "Servo_Base",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 64 * 4
};
/* Definitions for Servo_Joint_1 */
osThreadId_t Servo_Joint_1Handle;
const osThreadAttr_t Servo_Joint_1_attributes = {
  .name = "Servo_Joint_1",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 64 * 4
};
/* Definitions for Servo_Gripper */
osThreadId_t Servo_GripperHandle;
const osThreadAttr_t Servo_Gripper_attributes = {
  .name = "Servo_Gripper",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 64 * 4
};
/* Definitions for Servo_Joint_2 */
osThreadId_t Servo_Joint_2Handle;
const osThreadAttr_t Servo_Joint_2_attributes = {
  .name = "Servo_Joint_2",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 64 * 4
};
/* Definitions for Base_Servo_Angele */
osMessageQueueId_t Base_Servo_AngeleHandle;
const osMessageQueueAttr_t Base_Servo_Angele_attributes = {
  .name = "Base_Servo_Angele"
};
/* Definitions for Joint1_Servo_Angle */
osMessageQueueId_t Joint1_Servo_AngleHandle;
const osMessageQueueAttr_t Joint1_Servo_Angle_attributes = {
  .name = "Joint1_Servo_Angle"
};
/* Definitions for Joint2_Servo_Angle */
osMessageQueueId_t Joint2_Servo_AngleHandle;
const osMessageQueueAttr_t Joint2_Servo_Angle_attributes = {
  .name = "Joint2_Servo_Angle"
};
/* Definitions for Gripper_Servo_Angle */
osMessageQueueId_t Gripper_Servo_AngleHandle;
const osMessageQueueAttr_t Gripper_Servo_Angle_attributes = {
  .name = "Gripper_Servo_Angle"
};
/* Definitions for Load_Mutex */
osMutexId_t Load_MutexHandle;
const osMutexAttr_t Load_Mutex_attributes = {
  .name = "Load_Mutex"
};
/* Definitions for Moving_Lock */
osSemaphoreId_t Moving_LockHandle;
const osSemaphoreAttr_t Moving_Lock_attributes = {
  .name = "Moving_Lock"
};
/* USER CODE BEGIN PV */
uint8_t Raw_VCP_Data[64];
uint8_t Vcp_Data_Available = 0;
uint8_t Servo_Angeles[4];
osStatus_t status_base,status_1,status_2,status_Gripper;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
void Angle_Cal_Task(void *argument);
void Servo_Base_Task(void *argument);
void Servo_Joint_1_Task(void *argument);
void Servo_Gripper_Task(void *argument);
void Servo_Joint_2_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of Load_Mutex */
  Load_MutexHandle = osMutexNew(&Load_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Moving_Lock */
  Moving_LockHandle = osSemaphoreNew(4, 4, &Moving_Lock_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Base_Servo_Angele */
  Base_Servo_AngeleHandle = osMessageQueueNew (1, sizeof(uint8_t), &Base_Servo_Angele_attributes);

  /* creation of Joint1_Servo_Angle */
  Joint1_Servo_AngleHandle = osMessageQueueNew (1, sizeof(uint8_t), &Joint1_Servo_Angle_attributes);

  /* creation of Joint2_Servo_Angle */
  Joint2_Servo_AngleHandle = osMessageQueueNew (1, sizeof(uint8_t), &Joint2_Servo_Angle_attributes);

  /* creation of Gripper_Servo_Angle */
  Gripper_Servo_AngleHandle = osMessageQueueNew (1, sizeof(uint8_t), &Gripper_Servo_Angle_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Angle_Cal */
  Angle_CalHandle = osThreadNew(Angle_Cal_Task, NULL, &Angle_Cal_attributes);

  /* creation of Servo_Base */
  Servo_BaseHandle = osThreadNew(Servo_Base_Task, NULL, &Servo_Base_attributes);

  /* creation of Servo_Joint_1 */
  Servo_Joint_1Handle = osThreadNew(Servo_Joint_1_Task, NULL, &Servo_Joint_1_attributes);

  /* creation of Servo_Gripper */
  Servo_GripperHandle = osThreadNew(Servo_Gripper_Task, NULL, &Servo_Gripper_attributes);

  /* creation of Servo_Joint_2 */
  Servo_Joint_2Handle = osThreadNew(Servo_Joint_2_Task, NULL, &Servo_Joint_2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 150-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Pwm_WriteServo(uint8_t Servo_NO,uint8_t Angle){
	    // Ensure position is between 0 and 180
	    if (Angle > 180) Angle = 180;

	    // Map position (0-180) to CCR value
	    uint32_t pulse_length = (499 + (Angle * 2000) / 180); // Pulse width in microseconds (500 to 2500 us)
        switch (Servo_NO) {
            case 1:
            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_length);
                break;
            case 2:
            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_length);
                break;
            case 3:
            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_length);
                break;
            case 4:
            	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pulse_length);
                break;
        }
}
void Servo_Ramp(uint8_t start_angle, uint8_t end_angle, uint16_t step_delay,uint8_t Servo_NO) {
    if (start_angle < end_angle) {
        for (uint8_t pos = start_angle; pos <= end_angle; pos++) {
        	Pwm_WriteServo(Servo_NO,pos);
            HAL_Delay(step_delay);
        }
    } else {
        for (uint8_t pos = start_angle; pos >= end_angle; pos--) {
        	Pwm_WriteServo(Servo_NO,pos);
            HAL_Delay(step_delay);
        }
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Angle_Cal_Task */
/**
  * @brief  Function implementing the Angle_Cal thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Angle_Cal_Task */
void Angle_Cal_Task(void *argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN 5 */
  CDC_Transmit_FS("VCP Started\n", strlen("VCP Started\n"));
  /* Infinite loop */
  for(;;)
  {
	  if(Vcp_Data_Available == 1){
		  Vcp_Data_Available =0;
		  //Data Format Vcp_Data_Available = 1&1&90&90&90
		  	  	  	  	   //mode&Base&Joint1&Joint2&Gripper
		  if(Raw_VCP_Data[0]==49)		  //Joint Angle Control Mode
		  {
			  CDC_Transmit_FS("Joint Angle Control Mode\n", strlen("Joint Angle Control Mode\n"));
			  char *token = strtok(Raw_VCP_Data, "&");
			  int i = 0 ;
			  while (token != NULL)
			      {
			          printf("%s\n", token);
			          token = strtok(NULL, "&");
			          osDelay(100);
			          //CDC_Transmit_FS(token, strlen(token));
			          int Angle = atoi(token);
			          Servo_Angeles[i]=Angle;
			          i++;
			      }
			  if (xSemaphoreTake(Moving_LockHandle, portMAX_DELAY) == pdTRUE) {
				  if (xSemaphoreTake(Load_MutexHandle, portMAX_DELAY) == pdTRUE) {
					  //Queue Load
					  uint8_t count = osMessageQueueGetCount(Base_Servo_AngeleHandle);
					  if(count==0){
					  status_base = osMessageQueuePut(Base_Servo_AngeleHandle, &Servo_Angeles[0],8, 100);
					  if (status_base != osOK) {
						  CDC_Transmit_FS("Sender Base Queue Error\n", strlen("Sender Base Queue Error\n"));
						 }
					  }
					  else{
						  CDC_Transmit_FS("Sender Base Queue Full\n", strlen("Sender Base Queue Full\n"));
					  }
					  count = osMessageQueueGetCount(Joint1_Servo_AngleHandle);
					  if(count==0){
						status_1 = osMessageQueuePut(Joint1_Servo_AngleHandle, &Servo_Angeles[1],8, 100);
					  if (status_1 != osOK) {
						CDC_Transmit_FS("Sender Joint1 Queue Error\n", strlen("Sender Base Queue Error\n"));
							}
						}
					  else{
						  CDC_Transmit_FS("Sender Joint1 Queue Full\n", strlen("Sender Base Queue Full\n"));
					  }
					  count = osMessageQueueGetCount(Joint2_Servo_AngleHandle);
					  if(count==0){
						status_2 = osMessageQueuePut(Joint2_Servo_AngleHandle, &Servo_Angeles[2],8, 100);
					  if (status_2 != osOK) {
						CDC_Transmit_FS("Sender Joint2 Queue Error\n", strlen("Sender Base Queue Error\n"));
							}
						}
					  else{
						  CDC_Transmit_FS("Sender Joint2 Queue Full\n", strlen("Sender Base Queue Full\n"));
					  }
					  count = osMessageQueueGetCount(Gripper_Servo_AngleHandle);
					  if(count==0){
						status_Gripper = osMessageQueuePut(Gripper_Servo_AngleHandle, &Servo_Angeles[3],8, 100);
					  if (status_Gripper != osOK) {
						CDC_Transmit_FS("Sender Gripper Queue Error\n", strlen("Sender Base Queue Error\n"));
							}
						  }
					  else{
					  CDC_Transmit_FS("Sender Gripper Queue Full\n", strlen("Sender Base Queue Full\n"));
					  }
					}
				  else {
					  CDC_Transmit_FS("Failed to take the mutex\n", strlen("Failed to take the mutex\n"));
				  }
				  xSemaphoreGive(Load_MutexHandle);
			  }
		  	}
		  else if(Raw_VCP_Data[0]==50)			//Coordinate mode
		  {
			  CDC_Transmit_FS("Coordinate mode\n", strlen("Coordinate mode\n"));
			  // This modul will be created with inverse kinematic function
		  }
		  else									//ERROR
	  	  {
			  CDC_Transmit_FS("Invalid Selection\n", strlen("Invalid Selection\n"));
	  	  }

	 }
	 osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Servo_Base_Task */
/**
* @brief Function implementing the Servo_Base thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Base_Task */
void Servo_Base_Task(void *argument)
{
  /* USER CODE BEGIN Servo_Base_Task */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    uint8_t startPos_Base = 60; // Starting Position
    int const step_delay_Base = 30; // Steps Delay
  /* Infinite loop */
  for(;;)
  {
	  uint8_t received_Angle_Base;// Target Postion
	  uint8_t count_Base = osMessageQueueGetCount(Base_Servo_AngeleHandle);
	  if(count_Base>0){
		  status_base = osMessageQueueGet(Base_Servo_AngeleHandle, &received_Angle_Base, 8, 100);
		  if (status_base == osOK) {
			  if(startPos_Base !=received_Angle_Base){
				  Servo_Ramp(startPos_Base,received_Angle_Base,step_delay_Base,Servo_no_Base);
			      }
		  startPos_Base =received_Angle_Base;
		  xSemaphoreGive(Moving_LockHandle);
		  	  }
		  }
	  else {
		  osDelay(100);
		  }
	  }
	  osDelay(100);
  /* USER CODE END Servo_Base_Task */
}

/* USER CODE BEGIN Header_Servo_Joint_1_Task */
/**
* @brief Function implementing the Servo_Joint_1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Joint_1_Task */
void Servo_Joint_1_Task(void *argument)
{
  /* USER CODE BEGIN Servo_Joint_1_Task */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    uint8_t startPos_1 = 60; // Starting Position
    uint8_t const step_delay_1 = 30; // Steps Delay
  /* Infinite loop */
  for(;;)
  {
	  uint8_t received_Angle_1;// Target Postion
	  uint8_t count_1 = osMessageQueueGetCount(Joint1_Servo_AngleHandle);
	  if(count_1>0){
		  status_1 = osMessageQueueGet(Joint1_Servo_AngleHandle, &received_Angle_1, 8, 100);
		  if (status_1 == osOK) {
			  if(startPos_1 !=received_Angle_1){
				  Servo_Ramp(startPos_1,received_Angle_1,step_delay_1,Servo_no_1);
		      }
		startPos_1 =received_Angle_1;
		xSemaphoreGive(Moving_LockHandle);
		  	  	  }
	  	  }
	  else {
		  osDelay(100);
	  	  }

	  osDelay(100);
  }
  /* USER CODE END Servo_Joint_1_Task */
}

/* USER CODE BEGIN Header_Servo_Gripper_Task */
/**
* @brief Function implementing the Servo_Gripper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Gripper_Task */
void Servo_Gripper_Task(void *argument)
{
  /* USER CODE BEGIN Servo_Gripper_Task */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    uint8_t startPos_Gripper = 60; // Starting Position
    uint8_t const step_delay_Gripper = 30; // Steps Delay
  /* Infinite loop */
  for(;;)
  {
	  uint8_t received_Angle_Gripper;// Target Postion
	  uint8_t count_Gripper = osMessageQueueGetCount(Gripper_Servo_AngleHandle);
	  if(count_Gripper>0){
		  status_Gripper = osMessageQueueGet(Gripper_Servo_AngleHandle, &received_Angle_Gripper, 8, 100);
		  if (status_Gripper == osOK) {
			  if(startPos_Gripper != received_Angle_Gripper){
				  Servo_Ramp(startPos_Gripper, received_Angle_Gripper,step_delay_Gripper,Servo_no_Gripper);
		      }
		startPos_Gripper = received_Angle_Gripper;
		xSemaphoreGive(Moving_LockHandle);
		  	  	  }
	  	  }
	  else {
		  osDelay(100);
	  	  }

	  osDelay(100);
  }
  /* USER CODE END Servo_Gripper_Task */
}

/* USER CODE BEGIN Header_Servo_Joint_2_Task */
/**
* @brief Function implementing the Servo_Joint_2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Servo_Joint_2_Task */
void Servo_Joint_2_Task(void *argument)
{
  /* USER CODE BEGIN Servo_Joint_2_Task */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    uint8_t startPos_2 = 60; // Starting Position
    uint8_t const step_delay_2 = 30; // Steps Delay
  /* Infinite loop */
  for(;;)
  {
	  uint8_t received_Angle_2;// Target Postion
	  uint8_t count_2 = osMessageQueueGetCount(Joint2_Servo_AngleHandle);
	  if(count_2>0){
		  status_2 = osMessageQueueGet(Joint2_Servo_AngleHandle, &received_Angle_2, 8, 100);
		  if (status_2 == osOK) {
			  if(startPos_2 !=received_Angle_2){
				  Servo_Ramp(startPos_2,received_Angle_2,step_delay_2,Servo_no_2);
		      }
		startPos_2=received_Angle_2;
		xSemaphoreGive(Moving_LockHandle);
		  	  	  }
	  	  }
	  else {
		  osDelay(100);
	  	  }
	  osDelay(100);
  }
  /* USER CODE END Servo_Joint_2_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
