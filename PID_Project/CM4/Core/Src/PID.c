/*
 * PID.c
 *
 *  Created on: Dec 9, 2022
 *      Author: PAVILION15CW1012
 */
#include<math.h>
#include<PID.h>
#include<map.h>
#include<parametri.h>


void PID_Init( PID* conf, float kp, float kd, float ki, float dt, float outMin, float outMax)
{
	conf->kp = kp;
	conf->kd = kd;
	conf->ki = ki;
	conf->dt = dt;
	conf->Iterm = 0;
	conf->lastError = 0;
	conf->outMax = outMax,
	conf->outMin = outMin;
}

float PID_Compute(float input, float setPoint, PID* conf)
{
	float error = setPoint - input;
	//calcolo della variabile di errore

	conf->Iterm += (error * conf->dt) * conf->ki;
	/*calcola l'integrale dell'errore (aggiungendo il nuovo errore). Noto il valore
	 * precedente dell'integrale dell'errore aggiungiamo l'errore nel nuovo passaggio
	 * quindi otteniamo l'area sotto la funzione di errore passo dopo passo*/

	if((conf->Iterm) > conf->outMax) conf->Iterm = conf->outMax;
	else if((conf->Iterm) < conf->outMin) conf->Iterm = conf->outMin;

	float error2 = error;
	float lastError = conf->lastError;

	if (error2 <=0 && lastError <=0) {
		error2 = -error2;
		lastError = -lastError;
	}

	float dInput = (error2 - lastError) / conf->dt;
	//derivata del valore di uscita

	if (dInput > 0) {
		if (error < 0) dInput=-dInput;
	}
	else {
		if (error < 0) dInput=-dInput;
	}

	float output = (conf->kp * error) + (conf->Iterm) + (conf->kd * dInput);
    //calcola l'output del PID sommando tutti e tre gli output
	//printf("%.4f, %.4f, %.4f\r\n", input, conf->kp*error, conf->kd * dInput);

	if(output > conf->outMax) output = conf->outMax;
	else if(output < conf->outMin) output = conf->outMin;

	conf->lastError = error;
	//ricorda la variabile per il prossimo ciclo

	return output;
}

float* SpeedCompute(float virtualInputs[])
{
	static float Speeds_quad[4];
	static float Speeds[4];

	Speeds_quad[0] = (1/(4*b))*virtualInputs[0] + (1/(2*l*b))*virtualInputs[2] - (1/(4*d))*virtualInputs[3];
	Speeds_quad[1] = (1/(4*b))*virtualInputs[0] - (1/(2*l*b))*virtualInputs[1] + (1/(4*d))*virtualInputs[3];
	Speeds_quad[2] = (1/(4*b))*virtualInputs[0] - (1/(2*l*b))*virtualInputs[2] - (1/(4*d))*virtualInputs[3];
	Speeds_quad[3] = (1/(4*b))*virtualInputs[0] + (1/(2*l*b))*virtualInputs[1] + (1/(4*d))*virtualInputs[3];
    /*
     * Calcoliamo le velocità dei motori al quadrato, poichè non possono essere negative. Partendo dal
     * valore di throttle e seguendo le matrici di controllo dei droni andiamo a sommare e sottrarre le
     * variabili date tramite il pid per il controllo delle velocità.
     */

	Speeds[0]= sqrt(Speeds_quad[0]);
	Speeds[1]= sqrt(Speeds_quad[1]);
	Speeds[2]= sqrt(Speeds_quad[2]);
	Speeds[3]= sqrt(Speeds_quad[3]);
    //Una volta calcolata la velocità dei motori al quadrato, viene eseguita la radice

    return Speeds;
}
