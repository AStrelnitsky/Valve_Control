
#include "servo.h"
#include "XL320.h"
#include "tim.h"
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
struct SControl scontrol;
//struct SERVO servos[NUMBER_OF_SERVOS];
void servoInit(struct SControl*);
void constructControlMessage(struct SControl*, uint8_t param);
void parseMessage(struct SControl*);
uint8_t* addCRC( uint8_t*, uint8_t);
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size); //DXL's DEVELOPERS CODE
void SelectDaisyDirection(struct SControl* sc);
void simpleDelay( uint32_t delay);

void startServoControl(void)
{
	scontrol.sdir = SRX;
	SelectDaisyDirection(&scontrol);
	
	if (scontrol.s_counter < NUMBER_OF_SERVOS - 1)
	//if (scontrol.number_of_servos < scontrol.number_of_servos - 1)
	{
	
			++(scontrol.s_counter);
	}
	else 
	{
		scontrol.s_counter = 0;
		if (scontrol.send_counter < 1)
		{
			++(scontrol.send_counter);
		}
		else
		{
			scontrol.send_counter = 0;
		}

	}
	//scontrol.s_counter = 0;
	//scontrol.servo_rx_timer = RX_CLEAR;
	
	if ( scontrol.servos[scontrol.s_counter].ping == 1 )
	{
		scontrol.servos[scontrol.s_counter].ping = 0;
		servoPing(&scontrol, (scontrol.s_counter));
	}
	 if (scontrol.servos[scontrol.s_counter].mode != scontrol.servos[scontrol.s_counter].new_mode)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		scontrol.servo_rx_timer = RX_CLEAR;
		sendMessage(&scontrol, TOGGLE_MODE);
		scontrol.servos[scontrol.s_counter].mode = scontrol.servos[scontrol.s_counter].new_mode;
	}
	 if (scontrol.servos[scontrol.s_counter].ID != scontrol.servos[scontrol.s_counter].new_ID)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		scontrol.servo_rx_timer = RX_CLEAR;
		sendMessage(&scontrol, WRITE_ID);
		scontrol.servos[scontrol.s_counter].ID = scontrol.servos[scontrol.s_counter].new_ID;
	}
	 if (scontrol.servos[scontrol.s_counter].freset == 1)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		scontrol.servo_rx_timer = RX_CLEAR;
		sendMessage(&scontrol, FACT_RESET);
		scontrol.servos[scontrol.s_counter].freset = 0;
	}
	 if (scontrol.servo_control == 1 )
	{
		if(scontrol.send_counter == 0)
			{
				if ((scontrol.servos[scontrol.s_counter].mode) == 2)
				{
					sendMessage(&scontrol, READ_POSITION);
				}
				else
				{
					sendMessage(&scontrol, READ_SPEED);
				}
				
			}
		else
			{
				sendMessage(&scontrol, CONTROL);
				//scontrol.servo_control = 0;
			}
	}
	
}

void servoInit(struct SControl* sc)
{
	sc->send_counter = 0;
	sc->state = SERVO_IDLE;
	sc->bus_state = EMPTY;
	sc->sdir = STX;
	SelectDaisyDirection(sc);
	simpleDelay(20000000);
	sc->number_of_servos = 0;
	sc->servo_rx_timer = RX_CLEAR;
	HAL_UART_Abort_IT(&huart2);
	memset((sc->tx_buffer), 0, DAISY_TX_MAX_SIZE);
	
	for (uint32_t i = 0; i <= 4; i++)
	{
		if ((sc->servo_rx_timer) == RX_COMPLETE)
		{
			
			if (sc->rx_buffer[ID_BYTE])
			{
				sc->bus_state = LOAD;
				++(sc->number_of_servos);
				sc->IDs[(sc->number_of_servos) - 1] = (sc->rx_buffer[ID_BYTE]);
			}
				sc->servo_rx_timer = RX_CLEAR;
				memset((sc->rx_buffer), 0, DAISY_RX_MAX_SIZE);
				servoPing(sc, i);
		}
		else
		{
				sc->servo_rx_timer = RX_CLEAR;
				memset((sc->rx_buffer), 0, DAISY_RX_MAX_SIZE);
				servoPing(sc, i);
		}
			
	}
	
	sc->servo_rx_timer = RX_CLEAR;
	for (uint8_t i = 0; i < (sc->number_of_servos); ++i)
	{
		sc->s_counter = i;
		sc->servos[i].ID = sc->IDs[i];
		sendMessage(sc, READ_MODE);
		sc->servos[i].new_mode = (sc->servos[i].mode);
		scontrol.servos[i].new_ID = scontrol.servos[i].ID;
	}

}
void sendMessage(struct SControl *sc, enum TYPE_OF_MESSAGE type)
{
	//sc->state = SERVO_IDLE;
	HAL_UART_Abort_IT(&huart2);	
//if ((sc->state)	!= SERVO_READY_TO_ACTION)
{
	if(type == CONTROL)
	{
		//if(sc->servo_rx_timer == RX_CLEAR)
		{
			sc->state = SERVO_WAIT_REG_WRITE_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, REG_WRITE);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_GOAL_POSITION_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT;
			HAL_TIM_Base_Start_IT(&htim2);
		}
	
	}
	else if (type == READ_MODE)
	{
		if(sc->servo_rx_timer == RX_CLEAR)
		{
			sc->state = SERVO_WAIT_READ_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, READ);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_READ_MODE_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		sc->servo_rx_timer = RX_CLEAR;
	}
	else if (type == TOGGLE_MODE)
	{
		if(sc->servo_rx_timer == RX_CLEAR)
		{
			sc->state = SERVO_WAIT_WRITE_MODE_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, WRITE);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_WRITE_MODE_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT << 5;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		sc->servo_rx_timer = RX_CLEAR;
	}
	else if (type == FACT_RESET)
	{
		if(sc->servo_rx_timer == RX_CLEAR)
		{
			sc->state = SERVO_WAIT_FACTORY_RESET_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, F_RESET);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_FACTORY_RESET_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT << 5;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		
		while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		sc->servo_rx_timer = RX_CLEAR;
		
	}
	else if (type == WRITE_ID)
	{
		if(sc->servo_rx_timer == RX_CLEAR)
		{
			sc->state = SERVO_WAIT_WRITE_ID_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, ID_WRITE);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_WRITE_ID_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT << 5;
			HAL_TIM_Base_Start_IT(&htim2);
		}
		
		while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		sc->servo_rx_timer = RX_CLEAR;
	}
	else if (type == READ_POSITION)
{
	//if(sc->servo_rx_timer == RX_CLEAR)
		//{
			sc->state = SERVO_WAIT_READ_POSITION_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, POSITION_READ);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_READ_MODE_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT;
			HAL_TIM_Base_Start_IT(&htim2);
		//}
		
	//	while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		//sc->servo_rx_timer = RX_CLEAR;

}
else if (type == READ_SPEED)
{
	//if(sc->servo_rx_timer == RX_CLEAR)
		//{
			sc->state = SERVO_WAIT_READ_SPEED_TRANSMIT;
			sc->sdir = STX;
			SelectDaisyDirection(sc);
			constructControlMessage(sc, SPEED_READ);
			HAL_UART_Transmit_IT(&huart2, (sc->tx_buffer), 10 + XL_READ_MODE_PARAMETR_SIZE);
			htim2.Instance->ARR = SERVO_RX_TIMEOUT;
			HAL_TIM_Base_Start_IT(&htim2);
		//}
		
	//	while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW)))) {}
		//sc->servo_rx_timer = RX_CLEAR;

}
}

}
void constructControlMessage(struct SControl* sc, uint8_t param)
{
	unsigned short crc = 0;
	uint8_t num = sc->s_counter;
	if (param == REG_WRITE)
	{
		if (sc->servos[num].mode == 2)
		{
			uint16_t len = 3 + XL_GOAL_POSITION_PARAMETR_SIZE;
			sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
			sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
			sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
			sc->tx_buffer[RESERVED_BYTE]    =	  RESERVED;
			sc->tx_buffer[ID_BYTE] 			    =   (sc->servos[num].ID);
			sc->tx_buffer[LEN_L_BYTE]       =   (uint8_t) len;
			sc->tx_buffer[LEN_H_BYTE]       =   (uint8_t) (len >> 8);
			sc->tx_buffer[INSTRUCTION_BYTE] =   param;
			sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_GOAL_POSITION_ADDRESS_L;
			sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_GOAL_POSITION_ADDRESS_H;
			sc->tx_buffer[INSTRUCTION_BYTE + 3] = sc->servos[num].control[0];
			sc->tx_buffer[INSTRUCTION_BYTE + 4] = sc->servos[num].control[1];

	
			crc = update_crc(0, (sc->tx_buffer), 8 + XL_GOAL_POSITION_PARAMETR_SIZE );
    
			sc->tx_buffer[8 + XL_GOAL_POSITION_PARAMETR_SIZE] = crc & 0x00ff;
			sc->tx_buffer[9 + XL_GOAL_POSITION_PARAMETR_SIZE] = (crc>>8) & 0x00ff;
		}
		if (sc->servos[num].mode == 1)
		{
			uint16_t len = 3 + XL_GOAL_POSITION_PARAMETR_SIZE;
			sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
			sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
			sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
			sc->tx_buffer[RESERVED_BYTE]    =	  RESERVED;
			sc->tx_buffer[ID_BYTE] 			    =   (sc->servos[num].ID);
			sc->tx_buffer[LEN_L_BYTE]       =   (uint8_t) len;
			sc->tx_buffer[LEN_H_BYTE]       =   (uint8_t) (len >> 8);
			sc->tx_buffer[INSTRUCTION_BYTE] =   param;
			sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_GOAL_VELOCITY_ADDRESS_L;
			sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_GOAL_VELOCITY_ADDRESS_H;
			sc->tx_buffer[INSTRUCTION_BYTE + 3] = sc->servos[num].control[0];
			sc->tx_buffer[INSTRUCTION_BYTE + 4] = sc->servos[num].control[1];

	
			crc = update_crc(0, (sc->tx_buffer), 8 + XL_GOAL_VELOCITY_PARAMETR_SIZE );
    
			sc->tx_buffer[8 + XL_GOAL_VELOCITY_PARAMETR_SIZE] = crc & 0x00ff;
			sc->tx_buffer[9 + XL_GOAL_VELOCITY_PARAMETR_SIZE] = (crc>>8) & 0x00ff;
		}
	}
	else if (param == ACTION)
	{
		uint16_t len = 3 + XL_ACTION_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = BROADCAST_ID;//(sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = param;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_ACTION_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_ACTION_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_ACTION_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == READ_MODE)
	{
		uint16_t len = 3 + XL_READ_MODE_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = param;
		sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_CONTROL_MODE_ADDRESS_L;
		sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_CONTROL_MODE_ADDRESS_H;
		sc->tx_buffer[INSTRUCTION_BYTE + 3] = 0x04;
		sc->tx_buffer[INSTRUCTION_BYTE + 4] = 0x00;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_READ_MODE_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_READ_MODE_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_READ_MODE_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == POSITION_READ)
	{
		uint16_t len = 3 + XL_READ_MODE_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = READ;
		sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_CURRENT_POSITION_ADDRESS_L;
		sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_CURRENT_POSITION_ADDRESS_H;
		sc->tx_buffer[INSTRUCTION_BYTE + 3] = 0x02;
		sc->tx_buffer[INSTRUCTION_BYTE + 4] = 0x00;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_READ_MODE_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_READ_MODE_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_READ_MODE_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == SPEED_READ)
	{
		uint16_t len = 3 + XL_READ_MODE_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = READ;
		sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_CURRENT_SPEED_ADDRESS_L;
		sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_CURRENT_SPEED_ADDRESS_H;
		sc->tx_buffer[INSTRUCTION_BYTE + 3] = 0x02;
		sc->tx_buffer[INSTRUCTION_BYTE + 4] = 0x00;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_READ_MODE_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_READ_MODE_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_READ_MODE_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == WRITE)
	{
		uint16_t len = 3 + XL_WRITE_MODE_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = param;
		sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_CONTROL_MODE_ADDRESS_L;
		sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_CONTROL_MODE_ADDRESS_H;
		sc->tx_buffer[INSTRUCTION_BYTE + 3] = sc->servos[num].new_mode;//0x02;
		sc->tx_buffer[INSTRUCTION_BYTE + 4] = 0;//sc->servos[num].new_mode;//0x02;
		
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_WRITE_MODE_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_WRITE_MODE_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_WRITE_MODE_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == F_RESET)
	{
		uint16_t len = 3 + XL_FACTORY_RESET_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = param;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_FACTORY_RESET_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_FACTORY_RESET_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_FACTORY_RESET_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	else if (param == ID_WRITE)
	{
		uint16_t len = 3 + XL_WRITE_ID_PARAMETR_SIZE;
		sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
		sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
		sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
		sc->tx_buffer[RESERVED_BYTE]    = RESERVED;
		sc->tx_buffer[ID_BYTE] 			    = (sc->servos[num].ID);
		sc->tx_buffer[LEN_L_BYTE]       = (uint8_t) len;
		sc->tx_buffer[LEN_H_BYTE]       = (uint8_t) (len >> 8);
		sc->tx_buffer[INSTRUCTION_BYTE] = WRITE;
		sc->tx_buffer[INSTRUCTION_BYTE + 1] = XL_ID_ADDRESS_L;
		sc->tx_buffer[INSTRUCTION_BYTE + 2] = XL_ID_ADDRESS_H;
		sc->tx_buffer[INSTRUCTION_BYTE + 3] = sc->servos[num].new_ID;
		sc->tx_buffer[INSTRUCTION_BYTE + 4] = 0;//sc->servos[num].new_ID;
		
		crc = update_crc(0, (sc->tx_buffer), 8 + XL_WRITE_ID_PARAMETR_SIZE );
    
		sc->tx_buffer[8 + XL_WRITE_ID_PARAMETR_SIZE] = crc & 0xff;
		sc->tx_buffer[9 + XL_WRITE_ID_PARAMETR_SIZE] = (crc>>8) & 0xff;
	}
	
}

void servoPing(struct SControl* sc, uint8_t ID)
{
	uint8_t buff = 0;
	uint16_t len = 3;
	unsigned short crc = 0;
	
	sc->tx_buffer[HEADER1_BYTE]     = 	HEADER1;
	sc->tx_buffer[HEADER2_BYTE]     = 	HEADER2;
	sc->tx_buffer[HEADER3_BYTE]     = 	HEADER3;
	sc->tx_buffer[RESERVED_BYTE]    = 	RESERVED;
	sc->tx_buffer[ID_BYTE ] 				= 	ID;
	sc->tx_buffer[LEN_L_BYTE]       =   (uint8_t) len;
	sc->tx_buffer[LEN_H_BYTE]       =   (uint8_t) (len >> 8);
	sc->tx_buffer[INSTRUCTION_BYTE] =		PING;
	
	crc = update_crc(0, (sc->tx_buffer), 8 + XL_PING_PARAMETR_SIZE );
	
	sc->tx_buffer[8 + XL_PING_PARAMETR_SIZE] = crc & 0xff;
  sc->tx_buffer[9 + XL_PING_PARAMETR_SIZE] = (crc >> 8) & 0xff;
	
	sc->sdir = STX;
	SelectDaisyDirection(sc);
	sc->state = SERVO_WAIT_IT_TRANSMIT;
	
	HAL_UART_Transmit_IT(&huart2, sc->tx_buffer, 10 + XL_PING_PARAMETR_SIZE);
	
	while (!((sc->servo_rx_timer == RX_COMPLETE) || ((sc->servo_rx_timer == RX_OVERFLOW))))
 {
	 ;
 }
}
uint8_t* addCRC( uint8_t* buf, uint8_t length)
{
	unsigned short accum = 0;
	update_crc(accum, buf, length);
	buf[length - 2] =  (uint8_t) accum;
	buf[length - 1] = (uint8_t) (accum >> 8);
	return buf;
}
void parseMessage(struct SControl* sc)
{
	;
}
/////////////////////////////////DXL's DEVELOPERS CODE////////////////////////////////////////////
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
    unsigned short i, j;
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}
unsigned short crc_check(unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
	unsigned short crc_ac = 0;
	update_crc (crc_ac, data_blk_ptr, data_blk_size);
	if (crc_ac == (data_blk_ptr[11] + (data_blk_ptr[12] << 8)))
	{
		return 1;
	}
	else return 0;
}
void SelectDaisyDirection(struct SControl* sc)
{
	((sc->sdir) == SRX) ? HAL_GPIO_WritePin(DAISY_DIR_RECEIVE) : HAL_GPIO_WritePin(DAISY_DIR_TRANSMIT);
}
void simpleDelay( uint32_t delay)
{
	for (uint32_t c = 0; c < delay; c++){}
}
