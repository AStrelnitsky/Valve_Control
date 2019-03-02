#include "stm32f4xx_hal.h"
#include "rs485.h"
#include "usart.h"
#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"
#define SDELAY 10000
struct RS485 rs485;

void SelectDirection(struct RS485* rs);
void ParseIncoming (struct RS485 *rs, struct SControl* sc);
void inline ParseMainMode (struct RS485 *rs, struct SControl* sc);
void inline ParseSettingMode (struct RS485 *rs, struct SControl* sc);
void  ParseBypass (struct RS485 *rs, struct SControl* sc);
void RsTransmit(struct RS485* rs);
uint16_t CalculateCRC ( uint8_t *data, uint8_t length, CRC_TYPE crc);
void ConstractSendMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code);
void inline ConstructBypassMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code);
void ConstructMainModeMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code);
void inline ConstructSettingModeMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code);

uint8_t CheckCRC(uint8_t * arr, uint8_t length);
static void simpleDelay(uint16_t delay);

void RS485_Init(void)
{
	rs485.address = DEFAULT_ADDRESS;
	rs485.dir = RX;
	rs485.crc_length = 1;
	rs485.crc_type = XOR_1;
	SelectDirection(&rs485);
}
void SelectDirection(struct RS485* rs)
{
	((rs->dir) == RX) ? HAL_GPIO_WritePin(DIR_RECEIVE) : HAL_GPIO_WritePin(DIR_TRANSMIT);
}
void ParseIncoming(struct RS485 *rs, struct SControl *sc)
{
	
	if ( CheckCRC((rs->rx_buffer), rs->rx_buffer_length))
	{
		if((rs->rx_buffer[FUNCTION_CODE]) == SERVO_BYPASS)
		{
			ParseBypass (rs, sc);
		}
		else if ((rs->rx_buffer[FUNCTION_CODE]) <= 0x0C)
		{
			ParseMainMode (rs, sc);
		}
		else if (((rs->rx_buffer[FUNCTION_CODE]) >= 0x41) && ((rs->rx_buffer[FUNCTION_CODE]) <= 0x60))
		{
			ParseSettingMode (rs, sc);
		}
  }
		else
	{
		ConstractSendMessage (rs, sc, ERROR_CRC);
		rs->state = IDLE;
	}
	memset((rs->rx_buffer), 0, RS485_RX_SIZE);
}
uint8_t CheckCRC(uint8_t *arr, uint8_t length)
{
	if (arr[length - 1] == 0xDC) // fake
	{
		return 1;
	}
	else return 0;
}
static void simpleDelay(uint16_t delay)
{
	int i = 0;
	for (i; i <= delay; ++i)
	{
		++i;
	}
}
void RsTransmit(struct RS485* rs)
{
	rs->dir = TX;
	SelectDirection(rs);
	rs->state = TRANSMITTING_DMA;
	HAL_UART_Transmit_DMA(&huart3, rs->tx_buffer, (rs->tx_buffer[LENGTH_BYTE] + 3));
}
void ConstractSendMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code)
{
	
		rs->tx_buffer[STROB_BYTE] = STROB;
		rs->tx_buffer[HEADER_BYTE] = HEADER;
		rs->tx_buffer[ADDRESS_BYTE] = rs->address;
		
		if ((rs->rx_buffer[FUNCTION_CODE]) == SERVO_BYPASS) // Subprotocol for direct control servos. Not work, developing/
		{
			
			ConstructBypassMessage (rs, sc, err_code);

		}
		else if ((rs->rx_buffer[FUNCTION_CODE]) <= 0x0C) // Main protocol
		{
			
			if ((rs->rx_buffer[FUNCTION_CODE]) != CONTROL_ONLY)
			{
				ConstructMainModeMessage (rs, sc, err_code);
			}
			
		}
		else if (((rs->rx_buffer[FUNCTION_CODE]) >= 0x41) && ((rs->rx_buffer[FUNCTION_CODE]) <= 0x60))
		{
			ConstructSettingModeMessage (rs, sc, err_code);
		}
		RsTransmit(rs);
}
void inline ParseBypass (struct RS485 *rs, struct SControl* sc) // Subprotocol for direct control servos. Not work, developing/
{
	uint8_t param = rs->rx_buffer[SERVO_PING_BYTE];
	if (param == PING_MODE)
			{
				//sc->servos[i].ping = 1;
			}
			else if ((param == WHEEL_MODE) || (param == JOINT_MODE))
			{
				sc->servos[0].new_mode = param / 0x0F - 1;
			}
			/*
			else if (param == MAIN_MODE)
			{
				sc->servo_control = 1;
				for (int i = 0; i < sc->number_of_servos; i++)
				{
					sc->servos[i].control[0] = rs->rx_buffer[SERVO_GOAL_ANGLE_0 + 2*i];
					sc->servos[i].control[1] = rs->rx_buffer[SERVO_GOAL_ANGLE_1 + 2*i];
				}
			} */
			
			else if (param == FACTORY_RESET)
			{
			//	sc->servos[i].freset = 1;
			}
			else if (param <= 5)
			{
//				sc->servos[i].new_ID = param;
			}
			
			ConstractSendMessage (rs, sc, ERROR_NO);
			rs->state = IDLE;
}
void inline ParseMainMode (struct RS485 *rs, struct SControl* sc) // Main protocol
{
		uint8_t err = ERROR_NO;
	

		if (rs->rx_buffer[LENGTH_BYTE] > (((sc->number_of_servos) << 1) + (rs->crc_length) + 1))
		{
			err = ERROR_LENGTH;
		}
		else
		{
			for (int i = 0; i < (sc->number_of_servos); i++)
				
					{
						sc->servos[i].control[0] = rs->rx_buffer[SERVO_GOAL_ANGLE_0_L_CM + 2*i];
						sc->servos[i].control[1] = rs->rx_buffer[SERVO_GOAL_ANGLE_0_H_CM + 2*i];
					}
		}
	
	ConstractSendMessage (rs, sc, err);
				
	rs->state = IDLE;
}
void inline ParseSettingMode (struct RS485 *rs, struct SControl* sc)
{
	;
}
void inline ConstructBypassMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code)
{
		rs->tx_buffer[LENGTH_BYTE] = NUMBER_OF_SERVOS + 2;
		for (int i = 0; i < NUMBER_OF_SERVOS; i++)
		{
			rs->tx_buffer[SEND_START_DATA_BYTE + i] = sc->IDs[i];
		}
		if (err_code == ERROR_NO)
		{
			rs->tx_buffer[FUNCTION_CODE] = rs->rx_buffer[FUNCTION_CODE];
		}
		else
		{
			rs->tx_buffer[FUNCTION_CODE] = (0x80 | (rs->rx_buffer[FUNCTION_CODE]));
			if ( err_code == ERROR_CRC)
			{
				rs->tx_buffer[FUNCTION_CODE + 1] = ERROR_CRC;
			}
		}
}
void ConstructMainModeMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code)
{
	uint16_t length,crc = 0;
	if (err_code == ERROR_NO) // if haven't errors, we are copying incoming function code to sending function code and constructing message accroding the STATUS byte
		{
			rs->tx_buffer[FUNCTION_CODE] = rs->rx_buffer[FUNCTION_CODE];
			
			if (rs->rx_buffer[FUNCTION_CODE] == CONTROL_WITH_RESPONSE ) // no sending data
			{
				rs->tx_buffer[LENGTH_BYTE] = 1 + rs485.crc_length;
			}
			
			else if (rs->rx_buffer[FUNCTION_CODE] == MAIN_MODE_1) // send servos angels and humidity
			{
				rs->tx_buffer[LENGTH_BYTE] = 3 + 2*(sc->number_of_servos) + rs485.crc_length;
				
				for (int i = 0; i < (sc->number_of_servos); ++i)
				{
					rs->tx_buffer[FUNCTION_CODE + 2*i + 1] = sc->servos[i].angle[0]; // current servo angles (10 bits)
					rs->tx_buffer[FUNCTION_CODE + 2*i + 2] = sc->servos[i].angle[1]; // 6 high bits of sc->servos[x].angle[1] - is error flags. There means will describe later
				}
					rs->tx_buffer[FUNCTION_CODE + 2*(sc->number_of_servos)] = 0; /// where will be write humidity data
					rs->tx_buffer[FUNCTION_CODE + 2*(sc->number_of_servos) + 1] = 0; /// where will be write current consumption data
			}
			else if (rs->rx_buffer[FUNCTION_CODE] == MAIN_MODE_2) // send servos angels and humidity
			{
				rs->tx_buffer[LENGTH_BYTE] = 1 + 2*(sc->number_of_servos) + rs485.crc_length;
				
				for (int i = 0; i < (sc->number_of_servos); ++i)
				{
					rs->tx_buffer[FUNCTION_CODE + 2*i + 1] = sc->servos[i].angle[0]; // current servo angles (10 bits)
					rs->tx_buffer[FUNCTION_CODE + 2*i + 2] = sc->servos[i].angle[1]; // 6 high bits of sc->servos[x].angle[1] - is error flags or current. There means will describe later
				}
			}
			else if (rs->rx_buffer[FUNCTION_CODE] == MAIN_MODE_3) // send servos angels and humidity
			{
				rs->tx_buffer[LENGTH_BYTE] = 3 +  rs485.crc_length;
				rs->tx_buffer[FUNCTION_CODE + 1] = 0; /// where will be write humidity data
				rs->tx_buffer[FUNCTION_CODE + 2] = 0; /// where will be write current consumption data
			}
			else if (rs->rx_buffer[FUNCTION_CODE] == MAIN_MODE_4) // send servos angels and humidity
			{
				rs->tx_buffer[LENGTH_BYTE] = 2 +  rs485.crc_length;
				rs->tx_buffer[FUNCTION_CODE + 1] = 0; /// where will be write status byte data
			}
		}
		
	else
		{
			((rs->tx_buffer[FUNCTION_CODE]) = 0x80 | (rs->rx_buffer[FUNCTION_CODE])); // if parsing procedure return error, we are modify function code in send message and write

				rs->tx_buffer[FUNCTION_CODE + 1] = err_code;
				rs->tx_buffer[LENGTH_BYTE] = 2 + rs485.crc_length;
			
		}
		
	////////adding CRC///////////////////////////////
	length = (rs->tx_buffer[LENGTH_BYTE]) + 3;
	crc = CalculateCRC ( rs->tx_buffer, length, rs->crc_type);
	rs->tx_buffer[length - rs485.crc_length] = (0x0F & crc);
	rs->tx_buffer[length - rs485.crc_length + 1] = ( crc >> 8);
	/////////////////////////////////////////////////
}
void inline ConstructSettingModeMessage (struct RS485* rs, struct SControl* sc, uint8_t err_code)
{
	;
}
uint16_t CalculateCRC ( uint8_t *data, uint8_t length, CRC_TYPE crc)
{
	uint16_t accum_2 = 0;
	uint8_t	accum_1 = 0;
	
 if (crc == XOR_2) // choise CRC type
	{
		for (int i = 3; i < length; ++i)
		{
			accum_2 = accum_2  ^= data[i];
		}
		return accum_2;
	}
	else if (crc == XOR_1)
	{
		for (int i = 3; i < length; ++i)
		{
			accum_1 = accum_1  ^= data[i];
		}
		return (uint16_t) accum_1;
	}
}
