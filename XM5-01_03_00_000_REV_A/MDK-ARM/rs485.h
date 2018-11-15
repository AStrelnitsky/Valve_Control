#include "stm32f4xx_hal.h"
#include "usart.h"
#include "XM5-01_03_00_000_REV_A.h"

#define DIR_TRANSMIT GPIOC, GPIO_PIN_12, GPIO_PIN_SET
#define DIR_RECEIVE GPIOC, GPIO_PIN_12, GPIO_PIN_RESET

#define RS485_RX_SIZE 4
#define RS485_TX_SIZE 3

//////////INCOMING MESSAGE STRUCTURE/////////////

#define PUMP_BYTE   0
#define HEADER_BYTE 1
#define ADDRESS_BYTE 2
////////////////////////////////////////////////

//////////SENDING MESSAGE STRUCTURE/////////////

////////////////////////////////////////////////

//////////DEFAULT VALUES////////////////////////

#define PUMP 0x55
#define HEADER 0xAA
#define DEFAULT_ADDRESS 0x7

struct RS485
{
	uint8_t address;
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
	TRANSMITTING_DMA
} state;

	uint8_t rx_buffer[RS485_RX_SIZE];
	uint8_t tx_buffer[RS485_TX_SIZE];
	int16_t servos[NUMBER_OF_SERVOS];
};

void RS485_Init(void);

