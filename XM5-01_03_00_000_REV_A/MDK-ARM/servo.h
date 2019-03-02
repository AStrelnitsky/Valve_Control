
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "XM5-01_03_00_000_REV_A.h"

#define DAISY_DIR_TRANSMIT GPIOE, GPIO_PIN_3, GPIO_PIN_SET
#define DAISY_DIR_RECEIVE GPIOE, GPIO_PIN_3, GPIO_PIN_RESET
#define SERVO_RX_TIMEOUT 1000000

#define DAISY_RX_MAX_SIZE 32
#define DAISY_TX_MAX_SIZE 24
#define TOTAL_NUMBERS_OF_SERVOS 0xFC
#define NUMBER_BYTES_OF_PING_TRANSMIT_MESSAGE 10
#define NUMBER_BYTES_OF_PING_RECEIVE_MESSAGE 10
#define MESSAGE_WAITING_TIMEOUT 1 //ms
//////////INCOMING MESSAGE STRUCTURE/////////////

#define HEADER1_BYTE   					   0
#define HEADER2_BYTE   					   1
#define HEADER3_BYTE   					 	 2
#define RESERVED_BYTE   				 	 3
#define ID_BYTE 			     				 4
#define LEN_L_BYTE 		             5
#define LEN_H_BYTE 		             6
#define INSTRUCTION_BYTE 		       7
#define CRC_L_BYTE DAISY_TX_SIZE + 7
#define CRC_H_BYTE CRC_L_BYTE + 	 1
////////////////////////////////////////////////

//////////SENDING MESSAGE STRUCTURE/////////////

////////////////////////////////////////////////

//////////DEFAULT VALUES////////////////////////

#define HEADER1       0xFF
#define HEADER2       0xFF
#define HEADER3       0xFD
#define RESERVED      0x0
#define BROADCAST_ID  0xFE
/////////////////////INSTRUCTIONS////////////////
#define PING				  0x1
#define READ          0x2
#define WRITE         0x3
#define REG_WRITE     0x4
#define ACTION 			  0x5
#define F_RESET 			0x6
#define ID_WRITE      0x7

#define XL_GOAL_VELOCITY_ADDRESS_L 32
#define XL_GOAL_VELOCITY_ADDRESS_H 0x00
#define XL_GOAL_POSITION_ADDRESS_L 30
#define XL_GOAL_POSITION_ADDRESS_H 0x00
#define XL_CONTROL_MODE_ADDRESS_L	 11
#define XL_CONTROL_MODE_ADDRESS_H  0
#define XL_ID_ADDRESS_L	 					 3
#define XL_ID_ADDRESS_H	 					 0
#define XL_GOAL_POSITION_PARAMETR_SIZE 4
#define XL_ACTION_PARAMETR_SIZE 0
#define XL_GOAL_VELOCITY_PARAMETR_SIZE 4
#define XL_READ_MODE_PARAMETR_SIZE 4
#define XL_WRITE_MODE_PARAMETR_SIZE 3
#define XL_PING_PARAMETR_SIZE 0
#define XL_FACTORY_RESET_PARAMETR_SIZE 0
#define XL_WRITE_ID_PARAMETR_SIZE 3
//#define XL_WRITE_ID_PARAMETR_SIZE 2
typedef enum 
{
	MSG_OK,
	MSG_BAD
} MSG_ST;
struct SERVO
{
	uint8_t ID;
	uint8_t new_ID;
	uint8_t angle [4];
	uint8_t control[4];
	int16_t torque;
	uint8_t ping;
	uint8_t new_mode;
	uint8_t mode;
	uint8_t freset;
};
enum TYPE_OF_MESSAGE
{
	CONTROL,
	SETTING,
	READ_MODE,
	TOGGLE_MODE,
	FACT_RESET,
	WRITE_ID,
	DATA_REQUEST
};
struct SControl
{
	uint8_t number_of_servos;
	uint8_t IDs[NUMBER_OF_SERVOS];
	struct SERVO servos [NUMBER_OF_SERVOS];//*servos;
	uint8_t s_counter;
	uint8_t servo_control;
enum 
{
	SERVO_IDLE,
	SERVO_WAIT_IT_TRANSMIT,
	SERVO_WAIT_IT_RECEIVE,
	SERVO_NEW_MESSAGE_IS_RECEIVED,
	SERVO_WAITE_DMA_TRANSMIT,
	SERVO_WAIT_DMA_RECEIVE,
	SERVO_TRANSMITTING_DMA,
	SERVO_WAIT_REG_WRITE_TRANSMIT,
	SERVO_WAIT_REG_WRITE_RECEIVE,
	SERVO_WAIT_ACTION_TRANSMIT,
	SERVO_WAIT_ACTION_RECEIVE,
	SERVO_WAIT_READ_RECEIVE,
	SERVO_WAIT_READ_TRANSMIT,
	SERVO_WAIT_WRITE_MODE_TRANSMIT,
	SERVO_WAIT_WRITE_MODE_RECEIVE,
	SERVO_WAIT_FACTORY_RESET_TRANSMIT,
	SERVO_WAIT_FACTORY_RESET_RECEIVE,
	SERVO_WAIT_WRITE_ID_TRANSMIT,
	SERVO_WAIT_WRITE_ID_RECEIVE,
	SERVO_READY_TO_ACTION
} state;
enum
{
	EMPTY,
	LOAD
} bus_state;
enum
{
	RX_CLEAR,
	RX_OVERFLOW,
	RX_COMPLETE
} servo_rx_timer;

uint8_t rx_buffer[DAISY_RX_MAX_SIZE];
uint8_t tx_buffer[DAISY_TX_MAX_SIZE];
enum
{
	SRX,
	STX
} sdir;
};
void servoInit(struct SControl*);
void startServoControl(void);
void sendMessage(struct SControl*, enum TYPE_OF_MESSAGE);
void servoPing(struct SControl*, uint8_t ID);
