/*
 * PID.h
 *
 *  Created on: Dec 9, 2022
 *      Author: PAVILION15CW1012
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct /*PID_Config*/{
	float kp; //coefficienti di proporzionalità, derivabilità e integrabilità
	float kd;
	float ki;
	float dt; //intervallo per il controllo discreto
	float lastError; //errore nell'istante t-1
	float Iterm; //termine integrale
	float outMax;
	float outMin;
} PID;


void PID_Init( PID*, float, float, float, float, float, float);

float PID_Compute(float, float, PID*);

float* SpeedCompute(float vI[]);


#endif /* INC_PID_H_ */
