/*
 * File:   IO.c
 */

#include <xc.h>
#include "IO.h"

void InitIO()
{
    //****************************************************************************************************/
    // Declaration des pin Analogiques
    //****************************************************************************************************/
    ANSELA=0;             //Desactivation de toutes entree analogique
    ANSELB=0;             //Desactivation de toutes entree analogique
    ANSELD=0;             //Desactivation de toutes entree analogique
    ANSELC=0;             //Desactivation de toutes entree analogique
    ANSELE=0;             //Desactivation de toutes entree analogique
    ANSELG=0;             //Desactivation de toutes entree analogique

    // Configuration des sorties

    //******* LED ***************************
    _TRISJ6 = 0;  // LED Orange
    _TRISJ5 = 0; //LED Blanche
    _TRISJ4 = 0; // LED Bleue
    _TRISJ11 = 0; // LED Rouge
    _TRISH10 = 0; // LED Verte 
    //LED 2
    _TRISK15 = 0;  // LED Orange
    _TRISA0 = 0; //LED Blanche
    _TRISA9 = 0; // LED Bleue
    _TRISA10 = 0; // LED Rouge
    _TRISH3 = 0; // LED Verte 
    
    _TRISJ10 = 0; // EN_PWM 
    //****** Moteurs ************************

    // Configuration des entrées
    //Bouton 
    _TRISH1 = 0;     // Configure RG6 en entrée
 
    

    /****************************************************************************************************/
    // Gestion des pin remappables
    /****************************************************************************************************/
    UnlockIO(); // On unlock les registres d'entrées/sorties, ainsi que les registres des PPS
    
    //Assignation des remappable pins
        
    LockIO(); // On lock les registres d'entrées/sorties, ainsi que les registres des PPS
}


void LockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
                "mov #0x46, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2,[w1] \n"
                "mov.b w3,[w1] \n"
                "bset OSCCON, #6":: : "w1", "w2", "w3");
}

void UnlockIO() {
    asm volatile ("mov #OSCCON,w1 \n"
                "mov #0x46, w2 \n"
                "mov #0x57, w3 \n"
                "mov.b w2,[w1] \n"
                "mov.b w3,[w1] \n"
                "bclr OSCCON, #6":: : "w1", "w2", "w3");
}
