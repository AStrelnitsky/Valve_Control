/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */ 
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include "rs485.h"
#include "servo.h"
#include "XM5-01_03_00_000_REV_A.h"
#include "semphr.h"
#define MAIN_THREAD_DELAY 1000/SYS_FREQ // period of main programm thread, ms
#define SERVO_THREAD_DELAY MAIN_THREAD_DELAY/NUMBER_OF_SERVOS // period of servo thread, ms
#define USER_LED_HALFPERIOD 100*MAIN_THREAD_DELAY // half period of blik LED cycle, ms

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
extern struct RS485 rs485;
extern struct SControl scontrol;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
osThreadId rs485TaskHandle;
osThreadId servoTaskHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void StartRS485Task(void const * argument);
void ServoTask(void const * argument);

 xSemaphoreHandle xBinarySemaphore;
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	
  /* USER CODE END RTOS_SEMAPHORES */
	vSemaphoreCreateBinary(xBinarySemaphore);
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(rs485Task, StartRS485Task, osPriorityNormal, 1, 128);
	rs485TaskHandle = osThreadCreate(osThread(rs485Task), NULL);
	osThreadDef(servoTask, ServoTask, osPriorityNormal, 2, 128);
	servoTaskHandle = osThreadCreate(osThread(servoTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	static uint16_t cnt1 = 0;
  /* Infinite loop */
  for(;;)
  {
    osDelay(MAIN_THREAD_DELAY);
		if (rs485.state == IDLE) 
		{
			rs485.state = WAIT_IT_RECEIVE;
			HAL_UART_Receive_IT(&huart3, rs485.rx_buffer, RS485_RX_SIZE);
		}
		++cnt1;
		if ( !(cnt1 % USER_LED_HALFPERIOD))
		{
			HAL_GPIO_TogglePin(GPIOA, USER_LED);
		}
		
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void  StartRS485Task (void const * argument)
{
	for(;;)
	{
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		ParseIncoming(&rs485);
	}
}
void ServoTask(void const * argument)
{
	for(;;)
	{
		osDelay(SERVO_THREAD_DELAY);
		if (scontrol.state == SERVO_IDLE) 
		{
			;
		}
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
