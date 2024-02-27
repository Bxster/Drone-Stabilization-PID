/*
 * map.c
 *
 *  Created on: Dec 9, 2022
 *      Author: PAVILION15CW1012
 */

#include<map.h>
#include<parametri.h>


float map(float val, float from_src, float to_src, float from_dst, float to_dst)
{
	//val: valore della velocità da convertire in pwm;
	// from_src: minimo valore di velocità dei motori;
	// to_src: massimo valore di velocità dei motori;
	//from_dst: minimo valore di pwm;
	// to_dst: massimo valore di pwm;

	float duty=0;

	duty = (((to_dst - from_dst)/(to_src - from_src))*(val - from_src)) + from_dst;

	if(duty< (MinDuty/100)*20000){
		return (MinDuty/100)*20000;
	}

	if(duty> (MaxDuty/100)*20000){
		return (MaxDuty/100)*20000;
	}
	//questi if non fanno spegnere il motore in caso di uscita dal range di pwm

	return duty;
}
