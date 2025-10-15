/* 
 * File:   PWM.h
 * Author: E306-PC2
 *
 * Created on 6 octobre 2025, 11:54
 */

#ifndef PWM_H
#define	PWM_H
#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1


void InitPWM();
void PWMSetSpeed(float vitesseEnPourcents, float moteur);
void PWMUpdateSpeed();
void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur);
#endif


