
#include "servo.h"

struct SControl scontrol;

void ServoInit(void)
{
	struct SERVO servos[NUMBER_OF_SERVOS];
	scontrol.servos = servos;
}
void startServoControl(void)
{
	
	scontrol.state = SERVO_WAITE_DMA_TRANSMIT;
}
