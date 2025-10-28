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
const unsigned long T_60_SECONDS_TICKS = 116000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0; // 0: Arrêté / 1: En cours d'exécution

unsigned char obstacle_map = 0;

// 0 = Dégagé, 1 = Lointain, 2 = Critique
unsigned char etatTGG = 0;
unsigned char etatTG = 0;
unsigned char etatTC = 0;
unsigned char etatTD = 0;
unsigned char etatTDD = 0;


unsigned char ConvertDistanceToState(float distance)
{
    // Gère les valeurs négatives ou 'inf' de la formule (34/V - 5)
    // si V est élevé (proche) ou V est faible (loin)
    if (distance < 0.0) return 2; // Très proche, pic de tension
    
    if (distance < SEUIL_ETAT_2_CRITIQUE) {
        return 2; // Critique
    } else if (distance < SEUIL_ETAT_1_LOINTAIN) {
        return 1; // Lointain
    } else {
        return 0; // Dégagé
    }
}




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

            
            //Conversion des distances en états 0, 1, 2
            etatTGG = ConvertDistanceToState(robotState.distanceTelemetreGaucheGauche);
            etatTG = ConvertDistanceToState(robotState.distanceTelemetreGauche);
            etatTC = ConvertDistanceToState(robotState.distanceTelemetreCentre);
            etatTD = ConvertDistanceToState(robotState.distanceTelemetreDroit);
            etatTDD = ConvertDistanceToState(robotState.distanceTelemetreDroiteDroite);
           
        }

        // 3. **LOGIQUE DE CONTRÔLE ET AFFICHAGE (LEDs)**
        
        // Mise à jour de l'état du robot
        OperatingSystemLoop(); 
        
        
        // Affichage des LEDs (comme dans votre code original)

//        LED_ORANGE_1 = (robotState.distanceTelemetreCentre > dist);
//
//        LED_BLEUE_1 = (robotState.distanceTelemetreGauche > dist);
//
//        LED_BLANCHE_1 = (robotState.distanceTelemetreGaucheGauche > dist);
//
//        LED_ROUGE_1 = (robotState.distanceTelemetreDroit > dist);
//
//        LED_VERTE_1 = (robotState.distanceTelemetreDroiteDroite > dist);
        
        LED_ORANGE_1 = (etatTC > 0); 
        // Bleue (Gauche): Allumée si obstacle (1 ou 2)
        LED_BLEUE_1 = (etatTG > 0); 
        // Blanche (Ext G): Allumée si obstacle (1 ou 2)
        LED_BLANCHE_1 = (etatTGG > 0);
        // Rouge (Droite): Allumée si obstacle (1 ou 2)
        LED_ROUGE_1 = (etatTD > 0);
        // Verte (Ext D): Allumée si obstacle (1 ou 2)
        LED_VERTE_1 = (etatTDD > 0);
        
    }
    
    while (1) {

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
PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
break;

case STATE_TOURNE_LEGER_GAUCHE:
PWMSetSpeedConsigne(30, MOTEUR_DROIT);
PWMSetSpeedConsigne(25, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_LEGER_GAUCHE_EN_COURS;
break;

 case STATE_TOURNE_LEGER_GAUCHE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_GAUCHE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_LEGER_DROITE:
PWMSetSpeedConsigne(25, MOTEUR_DROIT);
PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_LEGER_DROITE_EN_COURS;
break;

 case STATE_TOURNE_LEGER_DROITE_EN_COURS:
 SetNextRobotStateInAutomaticMode();
 break;

case STATE_TOURNE_DROITE:
PWMSetSpeedConsigne(20, MOTEUR_DROIT);
PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
stateRobot = STATE_TOURNE_DROITE_EN_COURS;
break;

case STATE_TOURNE_DROITE_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;

case STATE_TOURNE_SUR_PLACE_GAUCHE:
PWMSetSpeedConsigne(15, MOTEUR_DROIT);
PWMSetSpeedConsigne(-15, MOTEUR_GAUCHE);
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

case STATE_ARRET:
PWMSetSpeedConsigne(0, MOTEUR_DROIT);
PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
stateRobot = STATE_ARRET_EN_COURS;
break;

case STATE_ARRET_EN_COURS:
SetNextRobotStateInAutomaticMode();
break;
             
case STATE_RECULE:
PWMSetSpeedConsigne(-30, MOTEUR_DROIT);
PWMSetSpeedConsigne(-30, MOTEUR_GAUCHE);
stateRobot = STATE_RECULE_EN_COURS;
break;

case STATE_RECULE_EN_COURS:
// Après avoir reculé, on force une rotation pour se dégager
stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; 
break;

default :
stateRobot = STATE_ATTENTE;
break;
}
}


unsigned char nextStateRobot=0;

/**
 * @brief Détermine l'état suivant du robot basé sur la logique hiérarchisée (0, 1, 2).
 * @brief 0=Dégagé, 1=Lointain, 2=Critique
 * @brief La fonction lit les variables globales etatTC, etatTG, etatTD, etc.
 */
void SetNextRobotStateInAutomaticMode()
{
    // Initialisation par défaut : avancer
    nextStateRobot = STATE_AVANCE;

    // --- LOGIQUE DE DÉCISION HIÉRARCHISÉE (Généralisée) ---

    // PRIORITÉ 1: DANGER CENTRE (État 2)
    // Gère [xx2xx] - (ex: [00200], [11200], [22212], ...)
    if (etatTC == 2) {
        nextStateRobot = STATE_RECULE;
    }
    
    // PRIORITÉ 2: DANGER CENTRE (État 1)
    // Gère [xx1xx] (sauf si Prio 1 déjà prise)
    else if (etatTC == 1) {
        // Tourner du côté le plus libre (État 0)
        if (etatTG == 0 && etatTD > 0) {
            // G Gêné, D Libre -> Tourne D
            nextStateRobot = STATE_TOURNE_GAUCHE; // Correction: TG est libre (0), TD est bloqué (>0) -> Tourner GAUCHE
        } else if (etatTD == 0 && etatTG > 0) {
            // G Bloqué, D Libre -> Tourne D
            nextStateRobot = STATE_TOURNE_DROITE;
        } else {
            // Les deux côtés sont 0 (libres) ou les deux sont > 0 (bloqués)
            // On choisit par défaut de tourner à gauche (ou droite)
            nextStateRobot = STATE_TOURNE_GAUCHE;
        }
    }
    
    // PRIORITÉ 3: CENTRE DÉGAGÉ (État 0), DANGER LATÉRAL CRITIQUE (État 2)
    // Gère [x202x], [x200x], [x002x]
    else if (etatTG == 2 || etatTD == 2) {
        if (etatTG == 2 && etatTD == 2) {
            // Impasse étroite (ex: [02020])
            nextStateRobot = STATE_RECULE;
        } else if (etatTG == 2) {
            // Danger critique à gauche -> Tourne sur place à DROITE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
        } else { // etatTD == 2
            // Danger critique à droite -> Tourne sur place à GAUCHE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
        }
    }
    
    // PRIORITÉ 4: CENTRE DÉGAGÉ (État 0), DANGER LATÉRAL LOINTAIN (État 1)
    // Gère [x101x], [x100x], [x001x] (sauf si Prio 3 déjà prise)
    else if (etatTG == 1 || etatTD == 1) {
        if (etatTG == 1 && etatTD == 1) {
             // Couloir large, on continue
             nextStateRobot = STATE_AVANCE;
        } else if (etatTG == 1) {
            // Obstacle lointain à gauche -> Tourne DROITE
            nextStateRobot = STATE_TOURNE_DROITE;
        } else { // etatTD == 1
            // Obstacle lointain à droite -> Tourne GAUCHE
            nextStateRobot = STATE_TOURNE_GAUCHE;
        }
    }
    
    // PRIORITÉ 5: CENTRE DÉGAGÉ (État 0), DANGERS EXTÉRIEURS (TGG, TDD)
    // Gère [20002], [10001], [10000] etc. (Votre exemple)
    else if (etatTDD > 0 || etatTGG > 0) {
         if (etatTDD > 0 && etatTGG > 0) {
             // Passage large ([10001] ou [20002]), on continue
             nextStateRobot = STATE_AVANCE;
         } else if (etatTDD > 0) { // Uniquement extrême droite
            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
         } else { // etatTGG > 0 (Uniquement extrême gauche)
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
         }
    }
    
    // PRIORITÉ 6: TOUT DÉGAGÉ
    // Gère [00000]
    else {
        nextStateRobot = STATE_AVANCE;
    }

    // Transition vers le nouvel état
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}




//
//void SetNextRobotStateInAutomaticMode()
//{
//    // 'obstacle_map' est calculée dans la boucle main()
//    
//    switch (obstacle_map)
//    {
//        // -----------------------------------------------------------
//        // 0. AVANCER (Rien, ou obstacles très larges non menaçants)
//        // -----------------------------------------------------------
//        case 0b00000: // [00000] - Rien
//        case 0b10001: // [10001] - TGG + TDD (Passage large)
//            nextStateRobot = STATE_AVANCE;
//            break;
//            
//        // -----------------------------------------------------------
//        // 1. OBSTACLES CÔTÉ DROIT (TD, TDD) -> Tourner GAUCHE
//        // -----------------------------------------------------------
//        case 0b00001: // [00001] - TDD (Extrême droite)
//            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
//            break;
//            
//        case 0b00010: // [00010] - TD (Droite)
//        case 0b00011: // [00011] - TD + TDD (Mur à droite)
//            nextStateRobot = STATE_TOURNE_GAUCHE;
//            break;
//
//        // -----------------------------------------------------------
//        // 2. OBSTACLES CÔTÉ GAUCHE (TG, TGG) -> Tourner DROITE
//        // -----------------------------------------------------------
//        case 0b10000: // [10000] - TGG (Extrême gauche)
//            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
//            break;
//            
//        case 0b01000: // [01000] - TG (Gauche)
//        case 0b11000: // [11000] - TGG + TG (Mur à gauche)
//            nextStateRobot = STATE_TOURNE_DROITE;
//            break;
//
//        // -----------------------------------------------------------
//        // 3. BLOCAGE CENTRAL (TC impliqué) -> Rotation sur place
//        // -----------------------------------------------------------
//        case 0b00100: // [00100] - TC (Obstacle pile en face)
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Choix arbitraire
//            break;
//
//        case 0b00101: // [00101] - TC + TDD
//        case 0b00110: // [00110] - TC + TD
//        case 0b00111: // [00111] - Blocage Centre-Droit
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Dégager vers la gauche
//            break;
//
//        case 0b01100: // [01100] - TG + TC
//        case 0b10100: // [10100] - TGG + TC
//        case 0b11100: // [11100] - Blocage Centre-Gauche
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE; // Dégager vers la droite
//            break;
//
//        // -----------------------------------------------------------
//        // 4. BLOCAGES COMPLEXES / MULTIPLES
//        // -----------------------------------------------------------
//
//        // Obstacles symétriques (TG+TD ou TGG+TC+TDD) -> Avancer si centre libre
//        case 0b01010: // [01010] - TG + TD (Porte)
//        case 0b10101: // [10101] - TGG + TC + TDD (Cas étrange)
//            nextStateRobot = STATE_AVANCE; // Tente de passer au centre
//            break;
//            
//        // Combinaisons Gauche/Droite sans Centre
//        case 0b10010: // TGG + TD
//        case 0b01001: // TG + TDD
//        case 0b11010: // TGG+TG + TD
//        case 0b01011: // TG + TD+TDD
//            nextStateRobot = STATE_AVANCE; // Tente de passer au centre
//            break;
//            
//        // Combinaisons bloquant tout le passage
//        case 0b01110: // [01110] - TG + TC + TD (Mur en U)
//        case 0b01111: // [01111]
//        case 0b11110: // [11110]
//        case 0b10110: // [10110]
//        case 0b01101: // [01101]
//        case 0b11011: // [11011]
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Blocage frontal étendu
//            break;
//
//        // -----------------------------------------------------------
//        // 5. BLOCAGE TOTAL -> Reculer
//        // -----------------------------------------------------------
//        case 0b11111: // [11111] - Tout est bloqué
//            nextStateRobot = STATE_RECULE;
//            break;
//
//        // Autres cas (par défaut, pour les combinaisons restantes)
//        default:
//            nextStateRobot = STATE_AVANCE;
//            break;
//    }
//    
//    // Si l'on n'est pas dans la transition de l'étape en cours
//    // (C'est-à-dire si on ne passe pas de STATE_XXX_EN_COURS à STATE_XXX)
//    if (nextStateRobot != stateRobot - 1)
//        stateRobot = nextStateRobot;
//}
//
//
//
