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
unsigned char robot_is_running = 0; // 0: Arr�t� / 1: En cours d'ex�cution

unsigned char obstacle_map = 0;

// 0 = D�gag�, 1 = Lointain, 2 = Critique
unsigned char etatTGG = 0;
unsigned char etatTG = 0;
unsigned char etatTC = 0;
unsigned char etatTD = 0;
unsigned char etatTDD = 0;


unsigned char ConvertDistanceToState(float distance)
{
    // G�re les valeurs n�gatives ou 'inf' de la formule (34/V - 5)
    // si V est �lev� (proche) ou V est faible (loin)
    if (distance < 0.0) return 2; // Tr�s proche, pic de tension
    
    if (distance < SEUIL_ETAT_2_CRITIQUE) {
        return 2; // Critique
    } else if (distance < SEUIL_ETAT_1_LOINTAIN) {
        return 1; // Lointain
    } else {
        return 0; // D�gag�
    }
}




void StopRobotCompletely(void) {
    // Arr�te les moteurs
    PWMSetSpeedConsigne(0, MOTEUR_DROIT);
    PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
    
    // Met les LEDs dans un �tat 'Arr�t�' (ex: toutes allum�es)
    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;
    
    // D�sactive l'alimentation des moteurs pour plus de s�curit�
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
    
    // Configuration initiale des LEDs (toutes �teintes au d�part)
    LED_BLANCHE_1 = 0; LED_BLEUE_1 = 0; LED_ORANGE_1 = 0; LED_ROUGE_1 = 0; LED_VERTE_1 = 0;
    LED_BLANCHE_2 = 1; LED_BLEUE_2 = 1; LED_ORANGE_2 = 1; LED_ROUGE_2 = 0; LED_VERTE_2 = 1;
    
    EN_PWM = 0; // Moteurs d�sactiv�s en attente
    stateRobot = STATE_ATTENTE; // Initialisation de l'�tat robot
    
    // --- BOUCLE D'ATTENTE AVANT D�PART ---
    // Le robot attend ici l'appui sur le bouton.
    while(!robot_is_running){
        if (_RH1 == 1) { 
            start_time_ticks = t1; // ? Enregistre le temps de d�part
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;// Autorise l'ex�cution
            LED_ROUGE_2 = 1;              // Signal visuel de d�marrage
        }
    }

    
    // --- BOUCLE PRINCIPALE (Course de 60 secondes) ---
    while (robot_is_running==1) {
        
        // 1. **GESTION DU TEMPS ET ARR�T**
        if (t1 - start_time_ticks >= T_60_SECONDS_TICKS) {
            robot_is_running = 0; // ? Condition de sortie du while
            StopRobotCompletely();
            break; // Sort imm�diatement du 'while' pour s'arr�ter
        }

        // 2. **GESTION DE L'ADC (T�l�m�tres)**
        if (ADCIsConversionFinished() == 1) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            
            // Calculs et mise � jour de robotState (comme dans votre code original)
            float volts;
            volts = ((float) result[0]) * 3.3 / 4096; robotState.distanceTelemetreGaucheGauche = 34 / volts - 5;
            volts = ((float) result[1]) * 3.3 / 4096; robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result[2]) * 3.3 / 4096; robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result[3]) * 3.3 / 4096; robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result[4]) * 3.3 / 4096; robotState.distanceTelemetreDroiteDroite = 34 / volts - 5;

            
            //Conversion des distances en �tats 0, 1, 2
            etatTGG = ConvertDistanceToState(robotState.distanceTelemetreGaucheGauche);
            etatTG = ConvertDistanceToState(robotState.distanceTelemetreGauche);
            etatTC = ConvertDistanceToState(robotState.distanceTelemetreCentre);
            etatTD = ConvertDistanceToState(robotState.distanceTelemetreDroit);
            etatTDD = ConvertDistanceToState(robotState.distanceTelemetreDroiteDroite);
           
        }

        // 3. **LOGIQUE DE CONTR�LE ET AFFICHAGE (LEDs)**
        
        // Mise � jour de l'�tat du robot
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
        // Bleue (Gauche): Allum�e si obstacle (1 ou 2)
        LED_BLEUE_1 = (etatTG > 0); 
        // Blanche (Ext G): Allum�e si obstacle (1 ou 2)
        LED_BLANCHE_1 = (etatTGG > 0);
        // Rouge (Droite): Allum�e si obstacle (1 ou 2)
        LED_ROUGE_1 = (etatTD > 0);
        // Verte (Ext D): Allum�e si obstacle (1 ou 2)
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
// Apr�s avoir recul�, on force une rotation pour se d�gager
stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; 
break;

default :
stateRobot = STATE_ATTENTE;
break;
}
}


unsigned char nextStateRobot=0;

/**
 * @brief D�termine l'�tat suivant du robot bas� sur la logique hi�rarchis�e (0, 1, 2).
 * @brief 0=D�gag�, 1=Lointain, 2=Critique
 * @brief La fonction lit les variables globales etatTC, etatTG, etatTD, etc.
 */
void SetNextRobotStateInAutomaticMode()
{
    // Initialisation par d�faut : avancer
    nextStateRobot = STATE_AVANCE;

    // --- LOGIQUE DE D�CISION HI�RARCHIS�E (G�n�ralis�e) ---

    // PRIORIT� 1: DANGER CENTRE (�tat 2)
    // G�re [xx2xx] - (ex: [00200], [11200], [22212], ...)
    if (etatTC == 2) {
        nextStateRobot = STATE_RECULE;
    }
    
    // PRIORIT� 2: DANGER CENTRE (�tat 1)
    // G�re [xx1xx] (sauf si Prio 1 d�j� prise)
    else if (etatTC == 1) {
        // Tourner du c�t� le plus libre (�tat 0)
        if (etatTG == 0 && etatTD > 0) {
            // G G�n�, D Libre -> Tourne D
            nextStateRobot = STATE_TOURNE_GAUCHE; // Correction: TG est libre (0), TD est bloqu� (>0) -> Tourner GAUCHE
        } else if (etatTD == 0 && etatTG > 0) {
            // G Bloqu�, D Libre -> Tourne D
            nextStateRobot = STATE_TOURNE_DROITE;
        } else {
            // Les deux c�t�s sont 0 (libres) ou les deux sont > 0 (bloqu�s)
            // On choisit par d�faut de tourner � gauche (ou droite)
            nextStateRobot = STATE_TOURNE_GAUCHE;
        }
    }
    
    // PRIORIT� 3: CENTRE D�GAG� (�tat 0), DANGER LAT�RAL CRITIQUE (�tat 2)
    // G�re [x202x], [x200x], [x002x]
    else if (etatTG == 2 || etatTD == 2) {
        if (etatTG == 2 && etatTD == 2) {
            // Impasse �troite (ex: [02020])
            nextStateRobot = STATE_RECULE;
        } else if (etatTG == 2) {
            // Danger critique � gauche -> Tourne sur place � DROITE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
        } else { // etatTD == 2
            // Danger critique � droite -> Tourne sur place � GAUCHE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
        }
    }
    
    // PRIORIT� 4: CENTRE D�GAG� (�tat 0), DANGER LAT�RAL LOINTAIN (�tat 1)
    // G�re [x101x], [x100x], [x001x] (sauf si Prio 3 d�j� prise)
    else if (etatTG == 1 || etatTD == 1) {
        if (etatTG == 1 && etatTD == 1) {
             // Couloir large, on continue
             nextStateRobot = STATE_AVANCE;
        } else if (etatTG == 1) {
            // Obstacle lointain � gauche -> Tourne DROITE
            nextStateRobot = STATE_TOURNE_DROITE;
        } else { // etatTD == 1
            // Obstacle lointain � droite -> Tourne GAUCHE
            nextStateRobot = STATE_TOURNE_GAUCHE;
        }
    }
    
    // PRIORIT� 5: CENTRE D�GAG� (�tat 0), DANGERS EXT�RIEURS (TGG, TDD)
    // G�re [20002], [10001], [10000] etc. (Votre exemple)
    else if (etatTDD > 0 || etatTGG > 0) {
         if (etatTDD > 0 && etatTGG > 0) {
             // Passage large ([10001] ou [20002]), on continue
             nextStateRobot = STATE_AVANCE;
         } else if (etatTDD > 0) { // Uniquement extr�me droite
            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
         } else { // etatTGG > 0 (Uniquement extr�me gauche)
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
         }
    }
    
    // PRIORIT� 6: TOUT D�GAG�
    // G�re [00000]
    else {
        nextStateRobot = STATE_AVANCE;
    }

    // Transition vers le nouvel �tat
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}




//
//void SetNextRobotStateInAutomaticMode()
//{
//    // 'obstacle_map' est calcul�e dans la boucle main()
//    
//    switch (obstacle_map)
//    {
//        // -----------------------------------------------------------
//        // 0. AVANCER (Rien, ou obstacles tr�s larges non mena�ants)
//        // -----------------------------------------------------------
//        case 0b00000: // [00000] - Rien
//        case 0b10001: // [10001] - TGG + TDD (Passage large)
//            nextStateRobot = STATE_AVANCE;
//            break;
//            
//        // -----------------------------------------------------------
//        // 1. OBSTACLES C�T� DROIT (TD, TDD) -> Tourner GAUCHE
//        // -----------------------------------------------------------
//        case 0b00001: // [00001] - TDD (Extr�me droite)
//            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
//            break;
//            
//        case 0b00010: // [00010] - TD (Droite)
//        case 0b00011: // [00011] - TD + TDD (Mur � droite)
//            nextStateRobot = STATE_TOURNE_GAUCHE;
//            break;
//
//        // -----------------------------------------------------------
//        // 2. OBSTACLES C�T� GAUCHE (TG, TGG) -> Tourner DROITE
//        // -----------------------------------------------------------
//        case 0b10000: // [10000] - TGG (Extr�me gauche)
//            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
//            break;
//            
//        case 0b01000: // [01000] - TG (Gauche)
//        case 0b11000: // [11000] - TGG + TG (Mur � gauche)
//            nextStateRobot = STATE_TOURNE_DROITE;
//            break;
//
//        // -----------------------------------------------------------
//        // 3. BLOCAGE CENTRAL (TC impliqu�) -> Rotation sur place
//        // -----------------------------------------------------------
//        case 0b00100: // [00100] - TC (Obstacle pile en face)
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Choix arbitraire
//            break;
//
//        case 0b00101: // [00101] - TC + TDD
//        case 0b00110: // [00110] - TC + TD
//        case 0b00111: // [00111] - Blocage Centre-Droit
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // D�gager vers la gauche
//            break;
//
//        case 0b01100: // [01100] - TG + TC
//        case 0b10100: // [10100] - TGG + TC
//        case 0b11100: // [11100] - Blocage Centre-Gauche
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE; // D�gager vers la droite
//            break;
//
//        // -----------------------------------------------------------
//        // 4. BLOCAGES COMPLEXES / MULTIPLES
//        // -----------------------------------------------------------
//
//        // Obstacles sym�triques (TG+TD ou TGG+TC+TDD) -> Avancer si centre libre
//        case 0b01010: // [01010] - TG + TD (Porte)
//        case 0b10101: // [10101] - TGG + TC + TDD (Cas �trange)
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
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Blocage frontal �tendu
//            break;
//
//        // -----------------------------------------------------------
//        // 5. BLOCAGE TOTAL -> Reculer
//        // -----------------------------------------------------------
//        case 0b11111: // [11111] - Tout est bloqu�
//            nextStateRobot = STATE_RECULE;
//            break;
//
//        // Autres cas (par d�faut, pour les combinaisons restantes)
//        default:
//            nextStateRobot = STATE_AVANCE;
//            break;
//    }
//    
//    // Si l'on n'est pas dans la transition de l'�tape en cours
//    // (C'est-�-dire si on ne passe pas de STATE_XXX_EN_COURS � STATE_XXX)
//    if (nextStateRobot != stateRobot - 1)
//        stateRobot = nextStateRobot;
//}
//
//
//
