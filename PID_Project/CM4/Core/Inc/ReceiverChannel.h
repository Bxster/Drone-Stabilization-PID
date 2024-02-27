#ifndef INC_RECEIVERCHANNEL_H_
#define INC_RECEIVERCHANNEL_H_


#include "main.h"


#define LEVEL1_DUTY 7
#define LEVEL2_DUTY 10
#define LEVEL3_DUTY 14
#define error_level 1

#define NUMBER_CHANNELS 5
#define IC_CHANNEL1 0 //tim2 ch1
#define IC_CHANNEL2 1 //tim2 ch2
#define IC_CHANNEL3 2 //tim2 ch3
#define IC_CHANNEL4 3 //tim2 ch4
#define IC_CHANNEL5 4 //tim1 ch2
#define IC_CHANNEL6 5 //not used

#define MIN_RANGE_DUTY 6.2
#define MAX_RANGE_DUTY 15.4



#define ROLL_ERROR_CALIBRATION 0.2
#define MAX_ERROR_STABILIZATION 0.35
#define MAX_ERROR_YAW 0.2



typedef struct ChannelForDuty_Data{
	uint32_t val;
	int firstCaptured;

	float usWidth;
	float duty;

}ChannelForDuty_Data;



typedef struct ChannelForFrequency_Data{
	uint32_t val;
	int firstCaptured;
	int flagFirstFrequency;

	float frequency;
}ChannelForFrequency_Data;







void Receiver_Init (ChannelForFrequency_Data* ch, ChannelForDuty_Data chDuty[], int ne);

float correctThrottle (float throttle, float maxThrottleOff);

float correctAngle(float angle, float error);



#endif
