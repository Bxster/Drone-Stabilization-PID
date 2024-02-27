/*
 * parametri.h
 *
 *  Created on: 6 ott 2023
 *      Author: PAVILION15CW1012
 */

#ifndef INC_PARAMETRI_H_
#define INC_PARAMETRI_H_


#define MOTOR_MIN_UP 1240 //minimo valore del duty cycle- 6% Duty (minima velocità)
#define MOTOR_MAX_UP 2000 //massimo valore del duty cycle - 10% Duty (massima velocità)
//#define MOTOR_MAX_SPEED_3 1220.6926 //sqrt(1/(4*B_3)+1/ (2*L*B_3) +1/(4*D))
//#define MOTOR_MAX_SPEED_4 1220.2435 // sqrt(1/(4*B_4)+1/ (2*L*B_4) +1/(4*D))
#define MOTOR_MAX_SPEED_4 1412.95

#define b 0.00001163  //0.000001163 // coefficiente di spinta di batteria 4-celle
#define l 0.335 //distanza tra il motore e il centro del drone
#define d 0.00008   //0.08 // coefficiente di resistenza aereodinamica
#define MaxSpeed 665.44
#define MinSpeed 477.34
#define MinDuty 6.2
#define MaxDuty 6.6

#define ENCODER_PPR = 2048;
#define GEARBOX_RATIO = 1;
#define ENCODER_COUNTING_MODE =4;

//PID
#define Kp 0.05
#define Kd 0.03
#define Ki 0
#define dtempo 0.01



#endif /* INC_PARAMETRI_H_ */
