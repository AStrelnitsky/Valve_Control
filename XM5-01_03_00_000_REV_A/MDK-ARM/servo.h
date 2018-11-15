
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "XM5-01_03_00_000_REV_A.h"

#define DAISY_DIR_TRANSMIT GPIOE, GPIO_PIN_3, GPIO_PIN_SET
#define DAISY_DIR_RECEIVE GPIOE, GPIO_PIN_3, GPIO_PIN_RESET

#define DAISY_RX_SIZE 32
#define DAISY_TX_SIZE 24

//////////INCOMING MESSAGE STRUCTURE/////////////

#define HEADER1_BYTE   					   0
#define HEADER2_BYTE   					   1
#define HEADER3_BYTE   					 	 2
#define RESERVED_BYTE   				 	 3
#define ID_BYTE 			     				 4
#define Len_L_BYTE 		             5
#define Len_H_BYTE 		             6
#define Instruction_BYTE 		       7
#define CRC_L_BYTE DAISY_TX_SIZE + 7
#define CRC_H_BYTE CRC_L_BYTE + 	 1
////////////////////////////////////////////////

//////////SENDING MESSAGE STRUCTURE/////////////

////////////////////////////////////////////////

//////////DEFAULT VALUES////////////////////////

#define HEADER1 0xFF
#define HEADER2 0xFF
#define HEADER3 0xFD
#define BROADCAST_ID 0xFE

struct SERVO
{
	uint8_t ID;
	int16_t angle;
	int16_t torque;
};
struct SControl
{
	uint8_t number_of_servos;
	struct SERVO *servos;
	uint8_t s_counter;
enum 
{
	SERVO_IDLE,
	SERVO_WAITE_DMA_TRANSMIT,
	SERVO_WAIT_DMA_RECEIVE,
	SERVO_TRANSMITTING_DMA
} state;

uint8_t rx_buffer[DAISY_RX_SIZE];
uint8_t tx_buffer[DAISY_TX_SIZE];

};
void ServoInit(void);
void startServoControl(void);
