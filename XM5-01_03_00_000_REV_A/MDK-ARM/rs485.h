#include "stm32f4xx_hal.h"
#include "usart.h"
#include "XM5-01_03_00_000_REV_A.h"

#define DIR_TRANSMIT GPIOC, GPIO_PIN_12, GPIO_PIN_SET
#define DIR_RECEIVE GPIOC, GPIO_PIN_12, GPIO_PIN_RESET
#define RS_RX_TIMEOUT 10000000

#define RS485_RX_SIZE 0x1F
#define RS485_TX_SIZE NUMBER_OF_SERVOS + 5

/*----------INDEPENDENT HEADERS -----------*/


#define STROB_BYTE   			0
#define HEADER_BYTE  			1
#define ADDRESS_BYTE 			2
#define LENGTH_BYTE  			3
#define FUNCTION_CODE 		0  // Number of "function code byte". Is a first byte, that wrote to rx_buffer, becouse it counted from zero

#define STROB							0xFF
#define HEADER 						0xFE
#define DEFAULT_ADDRESS 	0x7

/*-----------------------------------------*/

/*----------INCOMING MESSAGE STRUCTURE-----------*/

/////////DEFINES MAIN PROTOCOL ///////////////////

//       means function code for base mode (function codes from [0x01..0x0D])     //
//       see description in rs485.c, functions 'ParseIncoming' and 'ConstractSendMessage'

#define CONTROL_ONLY      						0x01 // accept control comands without answer
#define CONTROL_WITH_RESPONSE 				0x02 // accept control comands with answer
#define MAIN_MODE_1      							0x03 // accept control comands with return 4 angles ( per 2 bytes) and humidity (1 byte) and full current consumption (1 byte)
#define MAIN_MODE_2										0x04 // accept control comands with return 4 angles ( per 2 bytes) only
#define MAIN_MODE_3										0x05 // accept control comands with return humidity (1 byte) and full current consumption (1 byte) only
#define MAIN_MODE_4                   0x06 // accet control comands with return status byte. The status byte will coding system errors and wil describe later. Default value is zero.
#define SERVO_BYPASS 									0x0D // subprotocol for direct control servos. Not work, developing

//        means function code for connection setting mode (function codes from [0x21..0x40])     //
//////// 	developing //////////////////////////////////////////////////////////////////////////////

//       means function code for setting mode (function codes from [0x41..0x60])      //
//       see description in rs485.c, functions 'ParseIncoming' and 'ConstractSendMessage'

#define SETTING_ASK_DEVICE_TYPE 	 		0x41 // device type is 'VALVE_BOX_CONTROLLER'
#define SETTING_ASK_DECIMAL_NUMBER 		0x42 // decimal number is XM5-01_03_00_000_REVA
#define SETTING_ASK_SERIAL_NUMBER 		0x49 
#define SETTING_ASK_NUMBER_OF_SERVOS 	0x4A // returnes number of connected servos

//       means function code for service mode (function codes from [0x61..0x80])
//////// 	developing //////////////////////////////////////////////////////////////////////////////

//       addresing control values              //

#define SERVO_GOAL_ANGLE_0_L_CM FUNCTION_CODE + 1
#define SERVO_GOAL_ANGLE_0_H_CM SERVO_GOAL_ANGLE_0_L_CM + 1
#define SERVO_GOAL_ANGLE_1_L_CM SERVO_GOAL_ANGLE_0_H_CM + 1
#define SERVO_GOAL_ANGLE_1_H_CM SERVO_GOAL_ANGLE_1_L_CM + 1
#define SERVO_GOAL_ANGLE_2_L_CM SERVO_GOAL_ANGLE_1_H_CM + 1
#define SERVO_GOAL_ANGLE_2_H_CM SERVO_GOAL_ANGLE_2_L_CM + 1
#define SERVO_GOAL_ANGLE_3_L_CM SERVO_GOAL_ANGLE_2_H_CM + 1
#define SERVO_GOAL_ANGLE_3_H_CM SERVO_GOAL_ANGLE_3_L_CM + 1
#define STATUS SERVO_GOAL_ANGLE_3_H_CM + 1


//////////////////////////////////////////////////////////

/////////DEFINES FOR DIRECT SEVRO SUBPROTOCOL////////////////

#define SERVO_PING_BYTE FUNCTION_CODE + 1
#define SERVO_REG_ADDRESS_L SERVO_PING_BYTE + 1
#define SERVO_REG_ADDRESS_H SERVO_PING_BYTE + 2
#define SERVO_GOAL_ANGLE_0 SERVO_REG_ADDRESS_H + 1
#define SERVO_GOAL_ANGLE_1 SERVO_REG_ADDRESS_H + 2
#define SERVO_GOAL_ANGLE_2 SERVO_REG_ADDRESS_H + 3
#define SERVO_GOAL_ANGLE_3 SERVO_REG_ADDRESS_H + 4
#define WHEEL_MODE 				0x1F
#define JOINT_MODE 				0x2F
#define PING_MODE 				0xAC
#define FACTORY_RESET     0xFA

////////////////////////////////////////////////

/*---------SENDING MESSAGE STRUCTURE----------------*/
////////DEFINES FOR DIRECT SEVRO SUBPROTOCOL/////////

#define SEND_HEADER_1_BYTE 0
#define SEND_HEADER_2_BYTE 1
#define SEND_LENGTH_BYTE 2
#define SEND_START_DATA_BYTE 3

////////////////////////////////////////////////////


//////////DEFAULT VALUES////////////////////////

#define TRANSMIT_DELAY 		0xFFF

///////////////////////////////////////////////////

/////////////ERRORS///////////////////////////////

#define ERROR_NO				0x01
#define ERROR_CRC				0x02
#define ERROR_LENGTH    0x03

////////////////////////////////////////////////////

typedef enum 
{
	CRC_NO,
	XOR_1,
	XOR_2
} CRC_TYPE;
typedef enum 
{
	RS485_OK,
	RS485_ERR
} RS485_STATUS_TYPE;

struct RS485
{
	uint8_t address, crc_length;
	CRC_TYPE crc_type;
	enum 
{
	RX,
	TX
} dir;
	enum 
{
	IDLE,
	WAIT_IT_RECEIVE,
	WAIT_DMA_RECEIVE,
	WAIT_PARSE,
	TRANSMITTING_DMA
} state;

	uint8_t rx_buffer[RS485_RX_SIZE];
	uint8_t tx_buffer[RS485_TX_SIZE];
	int16_t servos[NUMBER_OF_SERVOS];
	uint8_t rx_buffer_length;
};

void RS485_Init(void);

