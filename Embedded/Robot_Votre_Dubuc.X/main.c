#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "timer.h"
#include "IO.h"
#include "PWM.h"
#include "Robot.h"
#include "ADC.h"

unsigned int ADCValue0;
unsigned int ADCValue1;
unsigned int ADCValue2;

int main(void) {
    InitOscillator();
    InitIO();
    
    //Initialisation oscillateur
    InitTimer1();
    InitTimer23();
    InitPWM();
    InitADC1();
    //PWMSetSpeed(20,0);

    //PWMUpdateSpeed();
    PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);

    //Configuration des input et output (IO)

    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;

    LED_BLANCHE_2 = 1;
    LED_BLEUE_2 = 1;
    LED_ORANGE_2 = 1;
    LED_ROUGE_2 = 1;
    LED_VERTE_2 = 1;
    //Boucle Principale
    //unsigned int * result = ADCGetResult();

    while (1) {
        unsigned int * result = ADCGetResult();
        if (ADCIsConversionFinished()) {
            ADCClearConversionFinishedFlag();
            result = ADCGetResult();
            ADCValue0 = result[0];
            ADCValue1 = result[1];
            ADCValue2 = result[2];
        }
    }
}