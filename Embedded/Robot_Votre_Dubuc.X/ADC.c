#include <xc.h>
#include "adc.h"
#include "Robot.h" 
#include "main.h"
#include "PWM.h"

unsigned char ADCResultIndex = 0;
static unsigned int ADCResult[5];
unsigned char ADCConversionFinishedFlag;
/****************************************************************************************************/
// Configuration ADC

/****************************************************************************************************/
void InitADC1(void) {
    //cf. ADC Reference Manual page 47
    //Configuration en mode 12 bits mono canal ADC avec conversions successives sur 4 éentres
    /************************************************************/
    //AD1CON1 Configuration de base de l?ADC
    /************************************************************/
    AD1CON1bits.ADON = 0; // ADC module OFF - pendant la config
    AD1CON1bits.AD12B = 1; // 0 : 10bits - 1 : 12bits
    AD1CON1bits.FORM = 0b00; // 00 = Integer (DOUT = 0000 dddd dddd dddd)
    AD1CON1bits.ASAM = 0; // 0 = Sampling begins when SAMP bit is set
    AD1CON1bits.SSRC = 0b111; // 111 = Internal counter ends sampling and starts conversion (auto-convert)
    /************************************************************/
    //AD1CON2 Configuration du multiplexer et balayage
    /************************************************************/
    AD1CON2bits.VCFG = 0b000; // 000 : Voltage Reference = AVDD AVss
    AD1CON2bits.CSCNA = 1; // 1 : Enable Channel Scanning
    AD1CON2bits.CHPS = 0b00; // Converts CH0 only
    AD1CON2bits.SMPI = 4; // 4+1 conversions successives avant interrupt
    AD1CON2bits.ALTS = 0;
    AD1CON2bits.BUFM = 0;
    /************************************************************/
    //AD1CON3 Horloge de conversion
    /************************************************************/
    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 8; // ADC Conversion Clock TAD = TCY * (ADCS + 1)
    AD1CON3bits.SAMC = 2; // Auto Sample Time
    /************************************************************/
    //AD1CON4 DMA (non utilisé ici)
    /************************************************************/
    AD1CON4bits.ADDMAEN = 0; // DMA is not used
    /************************************************************/
    //Configuration des ports analogiques
    /************************************************************/
    //ADC éutiliss : 8(B8)-9(B9)-10(B10)
    ANSELBbits.ANSB8 = 1; //entrée
    ANSELBbits.ANSB9 = 1;
    ANSELBbits.ANSB10 = 1;
    
    ANSELBbits.ANSB11 = 1; // AN11
    ANSELBbits.ANSB0 = 1; // AN12
    
    AD1CSSLbits.CSS8 = 1; // Enable AN8 for scan
    AD1CSSLbits.CSS9 = 1; // Enable AN9 for scan
    AD1CSSLbits.CSS10 = 1; // Enable AN10 for scan
    
    AD1CSSLbits.CSS11 = 1;
    AD1CSSLbits.CSS0 = 1;
    
    /* Assign MUXA inputs */
    AD1CHS0bits.CH0SA = 0; // CH0SA bits ignored for CH0 +ve input selection
    AD1CHS0bits.CH0NA = 0; // Select VREF- for CH0 -ve inpu
    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1; // Enable A/D interrupt
    AD1CON1bits.ADON = 1; // Turn on the A/D converter
}




void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void) {
    IFS0bits.AD1IF = 0;

    // 1. Lecture directe des valeurs
    unsigned int val_gauche_gauche = ADC1BUF0; // AN8
    unsigned int val_gauche        = ADC1BUF1; // AN9
    unsigned int val_centre        = ADC1BUF2; // AN10
    unsigned int val_droit         = ADC1BUF3; // AN11
    unsigned int val_droit_droit   = ADC1BUF4; // AN0

    ADCResult[0] = val_gauche_gauche;
    ADCResult[1] = val_gauche;
    ADCResult[2] = val_centre;
    ADCResult[3] = val_droit;
    ADCResult[4] = val_droit_droit;

    // 2. Conversion mise à jour de l'état
    // Formule simplifiée dérivée de votre doc : Dist = 42200 / ADC - 5
    if(val_centre > 100) robotState.distanceTelemetreCentre = (42200 / val_centre) - 5;
    else robotState.distanceTelemetreCentre = 80.0; 

    if(val_gauche > 100) robotState.distanceTelemetreGauche = (42200 / val_gauche) - 5;
    else robotState.distanceTelemetreGauche = 80.0;

    if(val_droit > 100) robotState.distanceTelemetreDroit = (42200 / val_droit) - 5;
    else robotState.distanceTelemetreDroit = 80.0;
    
    if(val_gauche_gauche > 100) robotState.distanceTelemetreGaucheGauche = (42200 / val_gauche_gauche) - 5;
    else robotState.distanceTelemetreGaucheGauche = 80.0;
    
    if(val_droit_droit > 100) robotState.distanceTelemetreDroiteDroite = (42200 / val_droit_droit) - 5;
    else robotState.distanceTelemetreDroiteDroite = 80.0;


    ADCConversionFinishedFlag = 1;
    
    OperatingSystemLoop();
    PWMUpdateSpeed(); 
}


void ADC1StartConversionSequence() {
    AD1CON1bits.SAMP = 1; //Lance une acquisition ADC
}

unsigned int * ADCGetResult(void) {
    return ADCResult;
}


unsigned char ADCIsConversionFinished(void) {
    return ADCConversionFinishedFlag;
}

void ADCClearConversionFinishedFlag(void) {
    ADCConversionFinishedFlag = 0;
}
