#include "ReceiverChannel.h"





void Receiver_Init (ChannelForFrequency_Data* ch, ChannelForDuty_Data chDuty[], int ne)
{
	int i = 0;

	ch->firstCaptured = 0;

	for(i = 0; i<ne; i++)
	{
		chDuty[i].firstCaptured = 0;
		chDuty[i].duty = 0;
	}

	ch->flagFirstFrequency = 1;
	ch->frequency = 0;
}



float correctThrottle (float throttle, float maxThrottleOff)
{
	if(throttle < maxThrottleOff)
		throttle = 0;
	else
		throttle -= maxThrottleOff;

	return throttle;
}



float correctAngle(float angle, float error)
{
	if(angle > -error && angle < error)
		angle = 0;

	return angle;
}






