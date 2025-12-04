#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "timer.h"
#include "IO.h"
#include "PWM.h"
#include "Robot.h"
#include "ADC.h"
#include "main.h"



unsigned char DIST_OBSTACLE_DETECTE;
unsigned char DIST_OBSTACLE_DETECTE1;
unsigned char DIST_OBSTACLE_DETECTE2;

extern unsigned long timestamp;
extern unsigned long t1;
unsigned char stateRobot;
const unsigned long T_60_SECONDS_TICKS = 60000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0;

// Variables globaless
unsigned char vitesse_avance;
LogicFunction_t pfn_SetNextRobotState = NULL;

unsigned char finishing_turn = 0; 


// --- FONCTIONS UTILITAIRES ---

void StopRobotCompletely(void) {
    PWMSetSpeedConsigne(0, MOTEUR_DROIT);
    PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
    EN_PWM = 0; 
    LED_BLANCHE_1 = 1; LED_BLEUE_1 = 1; LED_ORANGE_1 = 1; LED_ROUGE_1 = 1; LED_VERTE_1 = 1;
}


int IsPathClear(void) {
    // Il faut que TOUT soit supérieur au seuil "VOIE_LIBRE".
    if (robotState.distanceTelemetreCentre > DIST_VOIE_LIBRE &&
        robotState.distanceTelemetreGauche > DIST_VOIE_LIBRE &&
        robotState.distanceTelemetreDroit  > DIST_VOIE_LIBRE) 
    {
        return 1; // C'est tout bon, on peut y aller
    }
    return 0; // Il y a encore un obstacle quelque part devant
}


// --- MAIN ---
int main(void) {
    InitOscillator();
    InitIO();
    InitTimer1(); InitTimer23(); InitTimer4();
    InitPWM();
    InitADC1();
    
    // Init LEDs
    LED_BLANCHE_1 = 0; LED_BLEUE_1 = 0; LED_ORANGE_1 = 0; LED_ROUGE_1 = 0; LED_VERTE_1 = 0;
    EN_PWM = 0; 
    stateRobot = STATE_ATTENTE; 
    pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
    
    // BOUCLE D'ATTENTE DEPART
    while(!robot_is_running){
        
        if (_RH1 == 1) { 
            start_time_ticks = t1;
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;
            LED_ROUGE_2 = 1;              
            vitesse_avance=35;
            pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
            DIST_OBSTACLE_DETECTE=40.0;
            DIST_OBSTACLE_DETECTE1=30.0;
            DIST_OBSTACLE_DETECTE2=28.0;
        }
        // MODE 2 : LABYRINTHE / MAZE (Bouton RH2)
        else if (_RH2 == 1) { 
            start_time_ticks = t1;
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;
            LED_VERTE_2 = 1;
            vitesse_avance = 30; 
            pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
            DIST_OBSTACLE_DETECTE=30.0;
            DIST_OBSTACLE_DETECTE1=20.0;
            DIST_OBSTACLE_DETECTE2=15.0;
        }     
    }
    
    while (robot_is_running==1) {
        
        // 1. Arrêt au bout de 60s
        if (t1 - start_time_ticks >= T_60_SECONDS_TICKS) {
            robot_is_running = 0;
            StopRobotCompletely();
            break; 
        }

        if (ADCIsConversionFinished() == 1) {
             ADCClearConversionFinishedFlag();
            LED_ORANGE_1 = (robotState.distanceTelemetreCentre < DIST_OBSTACLE_DETECTE);
            LED_BLEUE_1  = (robotState.distanceTelemetreGauche < DIST_OBSTACLE_DETECTE1);
            LED_ROUGE_1  = (robotState.distanceTelemetreDroit  < DIST_OBSTACLE_DETECTE1);
            LED_VERTE_1  = (robotState.distanceTelemetreDroiteDroite  < DIST_OBSTACLE_DETECTE2);
            LED_BLANCHE_1  = (robotState.distanceTelemetreGaucheGauche  < DIST_OBSTACLE_DETECTE2);
        }
    }
        
        
    return 0;
}   

void OperatingSystemLoop(void)
{
    switch (stateRobot)
    {
    case STATE_ATTENTE:
        PWMSetSpeedConsigne(0, MOTEUR_DROIT);
        PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
        stateRobot = STATE_ATTENTE_EN_COURS;
        break; 

    case STATE_ATTENTE_EN_COURS:
        if (timestamp > 1000) stateRobot = STATE_AVANCE;
        break;

    // --- AVANCE ---
    case STATE_AVANCE:
        PWMSetSpeedConsigne(vitesse_avance, MOTEUR_DROIT);
        PWMSetSpeedConsigne(vitesse_avance, MOTEUR_GAUCHE);
        stateRobot = STATE_AVANCE_EN_COURS;
        break;

    case STATE_AVANCE_EN_COURS:
        pfn_SetNextRobotState(); // On vérifie les obstacles en continu
        break;

    // --- ROTATION GAUCHE ---
    case STATE_TOURNE_GAUCHE:
        PWMSetSpeedConsigne(22, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-1, MOTEUR_GAUCHE);
        timestamp = 0; 
        finishing_turn = 0; // Reset du flag
        stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
        break;

    case STATE_TOURNE_GAUCHE_EN_COURS:
        // Sécurité : Si on tourne depuis trop longtemps, on est coincé
        if (timestamp > TIMEOUT_BLOCAGE) {
            stateRobot = STATE_RECULE;
            return;
        }

        // Vérification de la sortie
        if (finishing_turn == 0) {
            if (IsPathClear()) {
                finishing_turn = 1;
                timestamp = 0;      // On reset le temps
            }
        }
        // Application de la marge de sécurité (hystérésis)
        else {
            // On continue de tourner un tout petit peu
            if (timestamp > MARGE_SECURITE_ROTATION) {
                stateRobot = STATE_AVANCE; 
            }
        }
        break;

    // --- ROTATION DROITE ---
    case STATE_TOURNE_DROITE:
        PWMSetSpeedConsigne(-1, MOTEUR_DROIT);
        PWMSetSpeedConsigne(22, MOTEUR_GAUCHE);
        timestamp = 0; 
        finishing_turn = 0;
        stateRobot = STATE_TOURNE_DROITE_EN_COURS;
        break;

    case STATE_TOURNE_DROITE_EN_COURS:
        if (timestamp > TIMEOUT_BLOCAGE) {
            stateRobot = STATE_RECULE;
            return;
        }

        if (finishing_turn == 0) {
            if (IsPathClear()) {
                finishing_turn = 1;
                timestamp = 0;
            }
        } else {
            if (timestamp > MARGE_SECURITE_ROTATION) {
                stateRobot = STATE_AVANCE;
            }
        }
        break;


    case STATE_TOURNE_SUR_PLACE_GAUCHE:
        PWMSetSpeedConsigne(20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        finishing_turn = 0;
        stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
        break;

    case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
        if (IsPathClear()) { 
             stateRobot = STATE_AVANCE;
        }
        if (timestamp > TIMEOUT_BLOCAGE) stateRobot = STATE_RECULE;
        break;
        
    case STATE_TOURNE_SUR_PLACE_DROITE:
        PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
        timestamp = 0; 
        finishing_turn = 0;
        stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
        break;

    case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
        if (IsPathClear()) {
             stateRobot = STATE_AVANCE;
        }
        if (timestamp > TIMEOUT_BLOCAGE) stateRobot = STATE_RECULE;
        break;

    // --- GESTION DU BLOCAGE ---
    case STATE_RECULE:
        PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        stateRobot = STATE_RECULE_EN_COURS;
        break;

    case STATE_RECULE_EN_COURS:
        // Recul sur temps fixe
        if (timestamp > 15) { 
            stateRobot = STATE_DEMI_TOUR; 
        }
        break; 

    case STATE_DEMI_TOUR:
        PWMSetSpeedConsigne(20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        stateRobot = STATE_DEMI_TOUR_EN_COURS;
        break;

    case STATE_DEMI_TOUR_EN_COURS:
        // Demi-tour sur temps fixe ~180°
        if (timestamp > 15) {
            stateRobot = STATE_AVANCE;
        }
        break;

    default :
        stateRobot = STATE_ATTENTE;
        break;
    }
}

unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode(void)
{
    // Acquisition des distances
    float d_TC  = robotState.distanceTelemetreCentre;
    float d_TG  = robotState.distanceTelemetreGauche;
    float d_TD  = robotState.distanceTelemetreDroit;
    float d_TGG = robotState.distanceTelemetreGaucheGauche; 
    float d_TDD = robotState.distanceTelemetreDroiteDroite;

    // Masque Binaire
    // 1 = Obstacle, 0 = Libre
    unsigned char obstacleMask = 0;
    
    if (d_TGG < DIST_OBSTACLE_DETECTE2) obstacleMask |= MASK_TGG;
    if (d_TG  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TG;
    if (d_TC  < DIST_OBSTACLE_DETECTE) obstacleMask |= MASK_TC;
    if (d_TD  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TD;
    if (d_TDD < DIST_OBSTACLE_DETECTE2) obstacleMask |= MASK_TDD;

    // Sécurité (CRITIQUE)
    // Priorité absolue
    if (d_TC < DIST_CRITIQUE || d_TG < DIST_CRITIQUE1 || d_TD < DIST_CRITIQUE1) {
        // Logique : Fuir le coté le plus bloqué
        if ((d_TG + d_TGG) < (d_TD + d_TDD)) {
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE; // Trop bloqué à gauche
        } else {
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Trop bloqué à droite ou face
        }
    }
    // 4. Logique 32 cas
    else {
        switch (obstacleMask) {
            // --- CAS 0 : VOIE LIBRE ---
            case 0b00000: // libre
                nextStateRobot = STATE_AVANCE;
                break;

            // --- CAS OBSTACLES LATÉRAUX SEULS (Suivi de mur/Centrage) ---
            case 0b10000: // très à gauche
            case 0b01000: // à gauche
            case 0b11000: // large à gauche
                nextStateRobot = STATE_TOURNE_DROITE; 
                break;

            case 0b00001: // très à droite
            case 0b00010: // à droite
            case 0b00011: // large à droite
                nextStateRobot = STATE_TOURNE_GAUCHE; 
                break;

            // --- CAS OBSTACLES FRONTAUX SIMPLES ---
            case 0b00100: // centre
                // Aller vers le coté le plus ouvert
                if (d_TGG > d_TDD) nextStateRobot = STATE_TOURNE_GAUCHE;
                else               nextStateRobot = STATE_TOURNE_DROITE;
                break;

            // --- CAS ENTONNOIRS / COINS (Frontal + Latéral) ---
            case 0b11100: // Gauche + Centre
            case 0b01100: // Gauche + Centre
            case 0b10100: // Ext Gauche + Centre
                nextStateRobot = STATE_TOURNE_DROITE; 
                break;

            case 0b00111: // Droite + Centre
            case 0b00110: // Droite + Centre
            case 0b00101: // Ext Droite + Centre
                nextStateRobot = STATE_TOURNE_GAUCHE; 
                break;

            // --- CAS COMPLEXES / TROP D'OBSTACLES ---
            case 0b11110: // Tout sauf ext droite
            case 0b01111: // Tout sauf ext gauche
            case 0b11111: // Cul de sac total
            case 0b10101: // Obstacles éparpillés  
                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;  
                break;

            case 0b10001:
                nextStateRobot = STATE_AVANCE; 
                break;
                
            case 0b01010: //Murs G et D, Centre libre 
                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;  
                break;
                
            // --- Combinaisons rares ---
            default:
                // Si le centre est pris, on tourne. Sinon on avance prudemment.
                if (obstacleMask & MASK_TC) nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                else                        nextStateRobot = STATE_AVANCE;
                break;
        }
    }

    // Gestion des transitions
    // On vérifie si l'état change et si on n'est pas déjà en train d'attendre une transition (state - 1)
    if (nextStateRobot != stateRobot && nextStateRobot != (stateRobot - 1)) {
        stateRobot = nextStateRobot;
    }
}
