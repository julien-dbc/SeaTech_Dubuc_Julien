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

extern unsigned long timestamp;
extern unsigned long t1;
unsigned char stateRobot;
const unsigned long T_60_SECONDS_TICKS = 480000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0;

// Variables globales
unsigned char vitesse_avance = 35;
LogicFunction_t pfn_SetNextRobotState = NULL;

// Indicateur pour la marge de sécurité (Hystérésis temporelle)
unsigned char finishing_turn = 0; 


// --- FONCTIONS UTILITAIRES ---

void StopRobotCompletely(void) {
    PWMSetSpeedConsigne(0, MOTEUR_DROIT);
    PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
    EN_PWM = 0; 
    // Allumer toutes les LEDs pour signaler l'arrêt
    LED_BLANCHE_1 = 1; LED_BLEUE_1 = 1; LED_ORANGE_1 = 1; LED_ROUGE_1 = 1; LED_VERTE_1 = 1;
}

/**
 * @brief Vérifie si l'avant du robot est suffisamment dégagé pour avancer.
 * @return 1 si libre, 0 si obstacle.
 */
int IsPathClear(void) {
    // On vérifie le Centre, la Gauche (TG) et la Droite (TD).
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
        }
    }
    
    // BOUCLE PRINCIPALE
    while (robot_is_running==1) {
        
        // 1. Arrêt au bout de 60s
        if (t1 - start_time_ticks >= T_60_SECONDS_TICKS) {
            robot_is_running = 0;
            StopRobotCompletely();
            break; 
        }

        // 2. Conversion ADC (Capteurs)
        if (ADCIsConversionFinished() == 1) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            float volts;
            
            // Conversion brute (adapter la formule selon vos capteurs si besoin)
            // Ajout d'une sécurité: si volts trop faible, distance = max (80cm)
            volts = ((float) result[0]) * 3.3 / 4096; robotState.distanceTelemetreGaucheGauche = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[1]) * 3.3 / 4096; robotState.distanceTelemetreGauche       = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[2]) * 3.3 / 4096; robotState.distanceTelemetreCentre       = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[3]) * 3.3 / 4096; robotState.distanceTelemetreDroit        = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[4]) * 3.3 / 4096; robotState.distanceTelemetreDroiteDroite = (volts > 0.2) ? (34 / volts - 5) : 80.0;
        }

        // 3. Debug LEDs (Allumées si obstacle détecté)
        LED_ORANGE_1 = (robotState.distanceTelemetreCentre < DIST_OBSTACLE_DETECTE);
        LED_BLEUE_1  = (robotState.distanceTelemetreGauche < DIST_OBSTACLE_DETECTE);
        LED_ROUGE_1  = (robotState.distanceTelemetreDroit  < DIST_OBSTACLE_DETECTE);
    }
    return 0;
}   

// --- LOGIQUE D'ÉTAT (OS) ---

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

    // --- ROTATION GAUCHE (Boucle Fermée) ---
    case STATE_TOURNE_GAUCHE:
        PWMSetSpeedConsigne(25, MOTEUR_DROIT);
        PWMSetSpeedConsigne(0, MOTEUR_GAUCHE); // Pivot sur roue
        timestamp = 0; 
        finishing_turn = 0; // Reset du flag de fin
        stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
        break;

    case STATE_TOURNE_GAUCHE_EN_COURS:
        // 1. Sécurité Timeout : Si on tourne depuis trop longtemps, on est coincé
        if (timestamp > TIMEOUT_BLOCAGE) {
            stateRobot = STATE_RECULE;
            return;
        }

        // 2. Vérification de la sortie (Si on n'est pas déjà en train de finir)
        if (finishing_turn == 0) {
            if (IsPathClear()) {
                finishing_turn = 1; // On a vu la sortie !
                timestamp = 0;      // On reset le temps pour la marge de sécurité
            }
        }
        // 3. Application de la marge de sécurité (hystérésis)
        else {
            // On continue de tourner un tout petit peu (MARGE_SECURITE_ROTATION)
            if (timestamp > MARGE_SECURITE_ROTATION) {
                stateRobot = STATE_AVANCE; // FINI ! On repart tout droit
            }
        }
        break;

    // --- ROTATION DROITE (Boucle Fermée) ---
    case STATE_TOURNE_DROITE:
        PWMSetSpeedConsigne(0, MOTEUR_DROIT); // Pivot sur roue
        PWMSetSpeedConsigne(25, MOTEUR_GAUCHE);
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

    // --- ROTATION SUR PLACE (Pour urgence ou coincé) ---
    // Ici on peut garder un timer fixe OU utiliser la même logique. 
    // Pour la sécurité, on utilise souvent un timer fixe pour faire un vrai 90°
    // Mais appliquons la logique capteur pour être cohérent.
    case STATE_TOURNE_SUR_PLACE_GAUCHE:
        PWMSetSpeedConsigne(20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        finishing_turn = 0;
        stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
        break;

    case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
        // On impose quand même une rotation minimale de 100ms pour se dégager d'un mur frontal
        if (timestamp > 1 && IsPathClear()) { 
             stateRobot = STATE_AVANCE;
        }
        // Timeout si bloqué
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
        if (timestamp > 1 && IsPathClear()) {
             stateRobot = STATE_AVANCE;
        }
        if (timestamp > TIMEOUT_BLOCAGE) stateRobot = STATE_RECULE;
        break;

    // --- GESTION DU BLOCAGE (RECUL) ---
    case STATE_RECULE:
        PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        stateRobot = STATE_RECULE_EN_COURS;
        break;

    case STATE_RECULE_EN_COURS:
        // Recul sur temps fixe (obligatoire car pas d'yeux derrière)
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
        // Demi-tour sur temps fixe (pour faire ~180°)
        if (timestamp > 15) {
            stateRobot = STATE_AVANCE;
        }
        break;

    default :
        stateRobot = STATE_ATTENTE;
        break;
    }
}


// ============================================================================
// CHOIX DE LA DIRECTION (INITIATION DU MOUVEMENT)
// ============================================================================
unsigned char nextStateRobot = 0;
void SetNextRobotStateInAutomaticMode(void)
{
    float d_TC  = robotState.distanceTelemetreCentre;
    float d_TG  = robotState.distanceTelemetreGauche;
    float d_TD  = robotState.distanceTelemetreDroit;
    float d_TGG = robotState.distanceTelemetreGaucheGauche;
    float d_TDD = robotState.distanceTelemetreDroiteDroite;

    // 1. URGENCE (Trop près < 10cm) -> Pivot sur place
    if (d_TC < DIST_CRITIQUE || d_TG < DIST_CRITIQUE || d_TD < DIST_CRITIQUE) {
        // On tourne à l'opposé du danger latéral
        if (d_TG < d_TD) nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
        else             nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    }
    
    // 2. OBSTACLE FRO<NTAL OU "TUNNEL" -> On pivote
    else if (d_TC < DIST_OBSTACLE_DETECTE) {
        if (d_TG >= d_TD) nextStateRobot = STATE_TOURNE_GAUCHE; // Gauche plus libre
        else             nextStateRobot = STATE_TOURNE_DROITE; // Droite plus libre
    }
    
    // 3. OBSTACLE LATÉRAL -> On tourne simplement
    else if (d_TG < DIST_OBSTACLE_DETECTE || d_TGG < DIST_OBSTACLE_DETECTE) {
        nextStateRobot = STATE_TOURNE_DROITE;
    }
    else if (d_TD < DIST_OBSTACLE_DETECTE || d_TDD < DIST_OBSTACLE_DETECTE) {
        nextStateRobot = STATE_TOURNE_GAUCHE;
    }
    
    // 4. SINON -> Tout droit
    else {
        nextStateRobot = STATE_AVANCE;
    }

    // Application de l'état
    if (nextStateRobot != stateRobot - 1 && nextStateRobot != stateRobot) {
        stateRobot = nextStateRobot;
    }
}