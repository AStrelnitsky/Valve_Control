/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */
#include "rs485.h"
#include "servo.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
extern struct RS485 rs485;
extern struct SControl scontrol;
extern xSemaphoreHandle xBinarySemaphore;
static portBASE_TYPE xHigherPriorityTaskWoken;
void HAL_UART_RxCallback(UART_HandleTypeDef*);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
	//static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */
//	rs485.state = WAIT_PARSE;
//	xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken);
//	if (xHigherPriorityTaskWoken == pdTRUE)
//	{
		//taskYIELD ();
//	}
  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */
	rs485.state = IDLE;
	
  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles I2C1 event interrupt.
*/
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
* @brief This function handles I2C1 error interrupt.
*/
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	HAL_UART_RxCallback(&huart3);
  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_DAC_IRQHandler(&hdac);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCallback(UART_HandleTypeDef *huart)
{
	
	static uint8_t byte_counter;
	uint8_t buff = huart->Instance->DR;
	
	switch (byte_counter)
	{
		case 0: 
			if (buff == STROB) ++byte_counter;
			else  
			{
				byte_counter = 0;
				huart->RxXferCount += 1;
			}
		break;
		case 1: 
			if (buff == HEADER) ++byte_counter;
		  else
			{
				byte_counter = 0;
				huart->RxXferCount += 2;
			}
		break;
		case 2: 
			if (buff == (rs485.address))
			{
				++byte_counter;
			}
			else 
			{
				byte_counter = 0;
				huart->RxXferCount += 3;				
			}
		break;
		case 3:
				byte_counter = 0;
				rs485.state = WAIT_DMA_RECEIVE;
				HAL_UART_Abort_IT(huart);
				rs485.rx_buffer_length = buff;
				HAL_UART_Receive_DMA(huart, rs485.rx_buffer, buff);
		break;
			
		default: break;
	}
	
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart->Instance) == USART2)
	{
			HAL_TIM_Base_Stop_IT(&htim2);
			scontrol.servo_rx_timer = RX_COMPLETE;
		
			if (scontrol.state == SERVO_WAIT_REG_WRITE_RECEIVE)
				{
						//scontrol.state = SERVO_READY_TO_ACTION;
						scontrol.servo_rx_timer = RX_CLEAR;
						scontrol.state = SERVO_WAIT_ACTION_TRANSMIT;
						scontrol.sdir = STX;
						SelectDaisyDirection(&scontrol);
						constructControlMessage(&scontrol, ACTION);
						HAL_UART_Transmit_IT(&huart2, (scontrol.tx_buffer), 10 + XL_ACTION_PARAMETR_SIZE);			
						HAL_TIM_Base_Start_IT(&htim2);
				
				}
			else if (scontrol.state == SERVO_WAIT_READ_RECEIVE)
			{
				scontrol.servos[scontrol.s_counter].mode = scontrol.rx_buffer[9];
				scontrol.state = SERVO_IDLE;
			}
			else
				{
					scontrol.state = SERVO_IDLE;
					
				}
	}
	else if ((huart->Instance) == USART3)
	{
		rs485.state = WAIT_PARSE;
		xSemaphoreGiveFromISR( xBinarySemaphore, &xHigherPriorityTaskWoken);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart->Instance) == USART2)
	{
		if (scontrol.state == SERVO_WAIT_IT_TRANSMIT)
		{
			scontrol.state = SERVO_IDLE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 14);
			
			htim2.Instance->ARR = SERVO_RX_TIMEOUT;
			htim2.Instance->CNT = 0;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		else if ( scontrol.state == SERVO_WAIT_REG_WRITE_TRANSMIT)
		{
			scontrol.state = SERVO_WAIT_REG_WRITE_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 11);
			
		}
		else if ( scontrol.state == SERVO_WAIT_ACTION_TRANSMIT)
		{
			scontrol.state = SERVO_WAIT_ACTION_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 11);
			
		}
		else if (scontrol.state == SERVO_WAIT_IT_RECEIVE)
		{
			//scontrol.state = SERVO_NEW_MESSAGE_IS_RECEIVED;
			scontrol.state = SERVO_IDLE;
		}
		else if (scontrol.state == SERVO_WAIT_READ_TRANSMIT)
		{
			
			scontrol.state = SERVO_WAIT_READ_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 15);
			
		}
		else if (scontrol.state == SERVO_WAIT_WRITE_MODE_TRANSMIT)
		{
			
			scontrol.state = SERVO_WAIT_WRITE_MODE_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 11);	
		}
		else if (scontrol.state == SERVO_WAIT_FACTORY_RESET_TRANSMIT)
		{
			scontrol.state = SERVO_WAIT_FACTORY_RESET_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 11);	
			
		}
		else if (scontrol.state == SERVO_WAIT_WRITE_ID_TRANSMIT)
		{
			scontrol.state = SERVO_WAIT_WRITE_ID_RECEIVE;
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 11);	
		}
		else
		{
			scontrol.sdir = SRX;
			SelectDaisyDirection(&scontrol);
			HAL_UART_Abort_IT(huart);
			HAL_UART_Receive_IT(huart, (scontrol.rx_buffer), 14);
			scontrol.state = SERVO_IDLE;
		}
			
}
		
	else if ((huart->Instance) == USART3)
	{
		
			rs485.dir = RX;
			SelectDirection(&rs485);
			
	}

}


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
