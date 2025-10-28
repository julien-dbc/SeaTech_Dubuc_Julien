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
const unsigned long T_60_SECONDS_TICKS = 60000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0; // 0: Arrêté / 1: En cours d'exécution


void StopRobotCompletely(void) {
    // Arrête les moteurs
    PWMSetSpeedConsigne(0, MOTEUR_DROIT);
    PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
    
    // Met les LEDs dans un état 'Arrêté' (ex: toutes allumées)
    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;
    
    // Désactive l'alimentation des moteurs pour plus de sécurité
    EN_PWM = 0; 
}




int main(void) {
    // --- Initialisation ---
    InitOscillator();
    InitIO();
    InitTimer1();
    InitTimer23();
    InitTimer4();
    InitPWM();
    InitADC1();
    
    // Configuration initiale des LEDs (toutes éteintes au départ)
    LED_BLANCHE_1 = 0; LED_BLEUE_1 = 0; LED_ORANGE_1 = 0; LED_ROUGE_1 = 0; LED_VERTE_1 = 0;
    LED_BLANCHE_2 = 1; LED_BLEUE_2 = 1; LED_ORANGE_2 = 1; LED_ROUGE_2 = 0; LED_VERTE_2 = 1;
    
    EN_PWM = 0; // Moteurs désactivés en attente
    stateRobot = STATE_ATTENTE; // Initialisation de l'état robot
    
    // --- BOUCLE D'ATTENTE AVANT DÉPART ---
    // Le robot attend ici l'appui sur le bouton.
    while(!robot_is_running){
        if (_RH1 == 1) { 
            start_time_ticks = t1; // ? Enregistre le temps de départ
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;// Autorise l'exécution
            LED_ROUGE_2 = 1;              // Signal visuel de démarrage
        }
    }

    
    // --- BOUCLE PRINCIPALE (Course de 60 secondes) ---
    while (robot_is_running==1) {
        
        // 1. **GESTION DU TEMPS ET ARRÊT**
        if (t1 - start_time_ticks >= T_60_SECONDS_TICKS) {
            robot_is_running = 0; // ? Condition de sortie du while
            StopRobotCompletely();
            break; // Sort immédiatement du 'while' pour s'arrêter
        }

        // 2. **GESTION DE L'ADC (Télémètres)**
        if (ADCIsConversionFinished() == 1) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            
            // Calculs et mise à jour de robotState (comme dans votre code original)
            float volts;
            volts = ((float) result[0]) * 3.3 / 4096; robotState.distanceTelemetreGaucheGauche = 34 / volts - 5;
            volts = ((float) result[1]) * 3.3 / 4096; robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result[2]) * 3.3 / 4096; robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result[3]) * 3.3 / 4096; robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result[4]) * 3.3 / 4096; robotState.distanceTelemetreDroiteDroite = 34 / volts - 5;
        }

        // 3. **LOGIQUE DE CONTRÔLE ET AFFICHAGE (LEDs)**
        
        // Mise à jour de l'état du robot
        OperatingSystemLoop(); 
        
        // Affichage des LEDs (comme dans votre code original)
        LED_ORANGE_1 = (robotState.distanceTelemetreCentre > 30);
        LED_BLEUE_1 = (robotState.distanceTelemetreGauche > 30);
        LED_BLANCHE_1 = (robotState.distanceTelemetreGaucheGauche > 30);
        LED_ROUGE_1 = (robotState.distanceTelemetreDroit > 30);
        LED_VERTE_1 = (robotState.distanceTelemetreDroiteDroite > 30);
    }
    
    // --- ARRÊT DU SYSTÈME ---
    // Le code atteint ce point après 60 secondes.
    // Il reste ici, le robot est arrêté.
    while (1) {
        // Le robot ne fait plus rien.
        // On pourrait ajouter un clignotement de LED pour signaler la fin.
        // ex: LED_ROUGE_1 = !LED_ROUGE_1; Wait_ms(500);
    }

}   
    




void OperatingSystemLoop(void)
{
switch (stateRobot)
{
case STATE_ATTENTE:
timestamp = 0;
PWMSetSpeedConsigne(0, MOTEUR_DROIT);
PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
stateRobot = STATE_ATTENTE_EN_COURS;

case STATE_ATTENTE_EN_COURS:
if (timestamp > 1000)
stateRobot = STATE_AVANCE;
break;

case STATE_AVANCE:
PWMSetSpeedConsigne(30, MOTEUR_DROIT);
PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
stateRobot = STATE_AVANCE_EN_COURS;
break;

case STATE_AVANCE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_GAUCHE:
PWMSetSpeedConsigne(30, MOTEUR_DROIT);
PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
break;

case STATE_TOURNE_LEGER_GAUCHE:
PWMSetSpeedConsigne(30, MOTEUR_DROIT);
PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
break;

case STATE_TOURNE_GAUCHE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_LEGER_DROITE:
PWMSetSpeedConsigne(20, MOTEUR_DROIT);
PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_DROITE_EN_COURS;
break;

case STATE_TOURNE_DROITE:
PWMSetSpeedConsigne(0, MOTEUR_DROIT);
PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_DROITE_EN_COURS;
break;

case STATE_TOURNE_DROITE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_SUR_PLACE_GAUCHE:
PWMSetSpeedConsigne(10, MOTEUR_DROIT);
PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
break;

case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_SUR_PLACE_DROITE:
PWMSetSpeedConsigne(-15, MOTEUR_DROIT);
PWMSetSpeedConsigne(15, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
break;

case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;
default :
stateRobot = STATE_ATTENTE;
break;
}
}


unsigned char nextStateRobot=0;


void SetNextRobotStateInAutomaticMode()
{
    unsigned char positionObstacle = PAS_D_OBSTACLE;
    
    // Détermination de la position des obstacles
    if (robotState.distanceTelemetreCentre < 30) {
        positionObstacle = OBSTACLE_EN_FACE;
    }
    else if (robotState.distanceTelemetreGauche < 30) {
        positionObstacle = OBSTACLE_A_GAUCHE;
    }
    else if (robotState.distanceTelemetreGaucheGauche < 30) {
        positionObstacle = OBSTACLE_LEGER_GAUCHE;
    }
    else if (robotState.distanceTelemetreDroit < 30) {
        positionObstacle = OBSTACLE_A_DROITE;
    }
    else if (robotState.distanceTelemetreDroiteDroite < 30) {
        positionObstacle = OBSTACLE_LEGER_DROITE;
    }
    else {
        positionObstacle = PAS_D_OBSTACLE;
    }

    // Détermination de l'état à venir du robot
    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;
    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_GAUCHE;
    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_DROITE;
    
    else if (positionObstacle == OBSTACLE_LEGER_DROITE)
        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
    else if (positionObstacle == OBSTACLE_LEGER_GAUCHE)
        nextStateRobot = STATE_TOURNE_LEGER_DROITE;
    
    else if (positionObstacle == OBSTACLE_EN_FACE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
        
    // Si l'on n'est pas dans la transition de l'étape en cours
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}









//unsigned char stateRobot;
//void OperatingSystemLoop(void)
//{
//switch (stateRobot)
//{
//case STATE_ATTENTE:
//timestamp = 0;
//PWMSetSpeedConsigne(0, MOTEUR_DROIT);
//PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
//stateRobot = STATE_ATTENTE_EN_COURS;
//case STATE_ATTENTE_EN_COURS:
//if (timestamp > 1000)
//stateRobot = STATE_AVANCE;
//break;
//case STATE_AVANCE:
//PWMSetSpeedConsigne(30, MOTEUR_DROIT);
//PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
//stateRobot = STATE_AVANCE_EN_COURS;
//break;
//case STATE_AVANCE_EN_COURS:
//SetNextRobotStateInAutomaticMode();
//break;
//case STATE_TOURNE_GAUCHE:
//PWMSetSpeedConsigne(30, MOTEUR_DROIT);
//PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
//stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
//break;
//case STATE_TOURNE_GAUCHE_EN_COURS:
//SetNextRobotStateInAutomaticMode();
//break;
//case STATE_TOURNE_DROITE:
//PWMSetSpeedConsigne(0, MOTEUR_DROIT);
//PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
//stateRobot = STATE_TOURNE_DROITE_EN_COURS;
//break;
//case STATE_TOURNE_DROITE_EN_COURS:
//SetNextRobotStateInAutomaticMode();
//break;
//case STATE_TOURNE_SUR_PLACE_GAUCHE:
//PWMSetSpeedConsigne(15, MOTEUR_DROIT);
//PWMSetSpeedConsigne(-15, MOTEUR_GAUCHE);
//stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
//break;
//case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
//SetNextRobotStateInAutomaticMode();
//break;
//case STATE_TOURNE_SUR_PLACE_DROITE:
//PWMSetSpeedConsigne(-15, MOTEUR_DROIT);
//PWMSetSpeedConsigne(15, MOTEUR_GAUCHE);
//stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
//break;
//case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
//SetNextRobotStateInAutomaticMode();
//break;
//default :
//stateRobot = STATE_ATTENTE;
//break;
//}
//}
//unsigned char nextStateRobot=0;
//void SetNextRobotStateInAutomaticMode()
//{
//unsigned char positionObstacle = PAS_D_OBSTACLE;
////éDtermination de la position des obstacles en fonction des ééètlmtres
//if ( robotState.distanceTelemetreDroit < 30 &&
//robotState.distanceTelemetreCentre > 20 &&
//robotState.distanceTelemetreGauche > 30) //Obstacle àdroite
//positionObstacle = OBSTACLE_A_DROITE;
//else if(robotState.distanceTelemetreDroit > 30 &&
//robotState.distanceTelemetreCentre > 20 &&
//robotState.distanceTelemetreGauche < 30) //Obstacle àgauche
//positionObstacle = OBSTACLE_A_GAUCHE;
//else if(robotState.distanceTelemetreCentre < 20) //Obstacle en face
//positionObstacle = OBSTACLE_EN_FACE;
//else if(robotState.distanceTelemetreDroit > 30 &&
//robotState.distanceTelemetreCentre > 20 &&
//robotState.distanceTelemetreGauche > 30) //pas d?obstacle
//positionObstacle = PAS_D_OBSTACLE;
////éDtermination de lé?tat àvenir du robot
//if (positionObstacle == PAS_D_OBSTACLE)
//nextStateRobot = STATE_AVANCE;
//else if (positionObstacle == OBSTACLE_A_DROITE)
//nextStateRobot = STATE_TOURNE_GAUCHE;
//else if (positionObstacle == OBSTACLE_A_GAUCHE)
//nextStateRobot = STATE_TOURNE_DROITE;
//else if (positionObstacle == OBSTACLE_EN_FACE)
//nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
////Si l?on n?est pas dans la transition de lé?tape en cours
//if (nextStateRobot != stateRobot - 1)
//stateRobot = nextStateRobot;
//}
//

