#include "stm32f4xx_hal.h"
#include "rs485.h"
#include "usart.h"
#include "servo.h"

struct RS485 rs485;

void SelectDirection(struct RS485* rs);
void ParseIncoming(struct RS485 *rs, struct SControl* sc);

void RS485_Init(void)
{
	rs485.address = DEFAULT_ADDRESS;
	rs485.dir = RX;
	SelectDirection(&rs485);
}
void SelectDirection(struct RS485* rs)
{
	((rs->dir) == RX) ? HAL_GPIO_WritePin(DIR_RECEIVE) : HAL_GPIO_WritePin(DIR_TRANSMIT);
}
void ParseIncoming(struct RS485 *rs, struct SControl* sc)
{
	for (int i = 0; i <= NUMBER_OF_SERVOS; ++i)
	{
		sc->servos[i].angle = (((int16_t)(rs->rx_buffer[2*i])) << 8) + ((int16_t)(rs->rx_buffer[2*i + 1]));
	}
}

