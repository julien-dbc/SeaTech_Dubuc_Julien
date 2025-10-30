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
const unsigned long T_60_SECONDS_TICKS = 56000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0; // 0: Arr�t� / 1: En cours d'ex�cution

unsigned char obstacle_map = 0;

unsigned char recul_counter =0;
unsigned char consecutive_left_turns = 0;
unsigned char consecutive_right_turns = 0;

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
            obstacle_map = 0;
            if (robotState.distanceTelemetreGaucheGauche < OBSTACLE_THRESHOLD2) obstacle_map |= (1 << 4);
            if (robotState.distanceTelemetreGauche < OBSTACLE_THRESHOLD)       obstacle_map |= (1 << 3);
            if (robotState.distanceTelemetreCentre < OBSTACLE_THRESHOLD1)       obstacle_map |= (1 << 2);
            if (robotState.distanceTelemetreDroit < OBSTACLE_THRESHOLD)        obstacle_map |= (1 << 1);
            if (robotState.distanceTelemetreDroiteDroite < OBSTACLE_THRESHOLD2) obstacle_map |= (1 << 0);
           
        }

        // 3. **LOGIQUE DE CONTR�LE ET AFFICHAGE (LEDs)**
        
        // Mise � jour de l'�tat du robot
        OperatingSystemLoop(); 
        
        
        // Affichage des LEDs (comme dans votre code original)

        LED_ORANGE_1 = (robotState.distanceTelemetreCentre > OBSTACLE_THRESHOLD1);

        LED_BLEUE_1 = (robotState.distanceTelemetreGauche > OBSTACLE_THRESHOLD);

        LED_BLANCHE_1 = (robotState.distanceTelemetreGaucheGauche > OBSTACLE_THRESHOLD2);

        LED_ROUGE_1 = (robotState.distanceTelemetreDroit > OBSTACLE_THRESHOLD);

        LED_VERTE_1 = (robotState.distanceTelemetreDroiteDroite > OBSTACLE_THRESHOLD2);
        

    }
    
//    while (1) {
//
//    }

}   
    




void OperatingSystemLoop(void)
{
    // Dur�es minimales (en ticks/ms) pour les actions.
    // (J'utilise les nouvelles valeurs que vous avez fournies)
    const unsigned int DUREE_RECUL = 40;
    const unsigned int DUREE_ROTATION_LEGERE = 15;
    const unsigned int DUREE_ROTATION_PRONONCEE = 15;
    const unsigned int DUREE_ROTATION_SUR_PLACE = 25;
    const unsigned int DUREE_DEMI_TOUR = 160; 

    switch (stateRobot)
    {
    case STATE_ATTENTE:
        timestamp = 0;
        PWMSetSpeedConsigne(0, MOTEUR_DROIT);
        PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
        stateRobot = STATE_ATTENTE_EN_COURS;
        break; 

    case STATE_ATTENTE_EN_COURS:
        if (timestamp > 1000)
            stateRobot = STATE_AVANCE;
        break;

    case STATE_AVANCE:
        PWMSetSpeedConsigne(35, MOTEUR_DROIT);
        PWMSetSpeedConsigne(35, MOTEUR_GAUCHE);
        stateRobot = STATE_AVANCE_EN_COURS;
        // <--- Logique anti-toupis: reset des compteurs
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;
        break;

    case STATE_AVANCE_EN_COURS:
        SetNextRobotStateInAutomaticMode();
        break;

    case STATE_AVANCE_PEU:
        PWMSetSpeedConsigne(20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
        stateRobot = STATE_AVANCE_PEU_COURS;
        // <--- Logique anti-toupis: reset des compteurs
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;       
        break;

    case STATE_AVANCE_PEU_COURS:
        SetNextRobotStateInAutomaticMode();
        break;

    // --- ROTATIONS GAUCHE (AVEC ANTI-TOUPIS) ---
        
        
        
       
//case STATE_TOURNE_GAUCHE:
//        consecutive_left_turns++;
//        consecutive_right_turns = 0;
//        
//        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
//            stateRobot = STATE_DEMI_TOUR_DROITE; 
//            consecutive_left_turns = 0; 
//        } else {
//            
//            const float MAX_DIST_DETECT = 150.0;
//            const float VITESSE_AJUSTEMENT = 20.0; // Intensit� de la rotation
//            const float VITESSE_BASE = 30.0;       // Vitesse d'avanc�e de base
//            
//            float distance = robotState.distanceTelemetreDroit;
//            if (distance < 0.0) {
//                distance = 0.0;
//            }
//            float i = distance / MAX_DIST_DETECT; 
//            float j = (1.0 - i) * VITESSE_AJUSTEMENT; 
//            
//
//            PWMSetSpeedConsigne(VITESSE_BASE + j, MOTEUR_DROIT); 
//            PWMSetSpeedConsigne(VITESSE_BASE - j, MOTEUR_GAUCHE); 
//            
//            timestamp = 0; 
//            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
//        }
//        break;    
        
        
        
        
        
        
        
    case STATE_TOURNE_GAUCHE:
        consecutive_left_turns++;
        //consecutive_right_turns = 0;
        
        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'�vasion
            consecutive_left_turns = 0; // On r�initialise
        } else {
            PWMSetSpeedConsigne(20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(5, MOTEUR_GAUCHE); 
            timestamp = 0; 
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
        }
        break;
        
        
        
        
    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            SetNextRobotStateInAutomaticMode(); // On r��value APRES avoir tourn�
        }
        break;

        
        
//        case STATE_TOURNE_LEGER_GAUCHE:
//        consecutive_left_turns++;
//        consecutive_right_turns = 0;
//        
//        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
//            stateRobot = STATE_DEMI_TOUR_DROITE; 
//            consecutive_left_turns = 0;
//        } else {
//            
//            const float MAX_DIST_DETECT = 150.0;
//            const float VITESSE_AJUSTEMENT = 20.0; // Intensit� de la rotation
//            const float VITESSE_BASE = 15.0;       // Vitesse d'avanc�e de base
//            
//            float distance = robotState.distanceTelemetreDroiteDroite;
//            if (distance < 0.0) {
//                distance = 0.0;
//            }
//            float i = distance / MAX_DIST_DETECT; 
//            float j = (1.0 - i) * VITESSE_AJUSTEMENT; 
//            
//
//            PWMSetSpeedConsigne(VITESSE_BASE - j, MOTEUR_DROIT); 
//            PWMSetSpeedConsigne(VITESSE_BASE + j, MOTEUR_GAUCHE); 
//            
//            timestamp = 0; 
//            stateRobot = STATE_TOURNE_LEGER_GAUCHE;
//        }
//        break; 
        
   
        
        
    case STATE_TOURNE_LEGER_GAUCHE:
        consecutive_left_turns++;
        //consecutive_right_turns = 0;
        
        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; 
            consecutive_left_turns = 0;
        } else {
            PWMSetSpeedConsigne(30, MOTEUR_DROIT);
            PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_LEGER_GAUCHE_EN_COURS;
        }
        break;

        
        
        
        
    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_LEGER_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_LEGERE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;

    case STATE_TOURNE_SUR_PLACE_GAUCHE:
        consecutive_left_turns++;
        //consecutive_right_turns = 0;
        
        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; 
            consecutive_left_turns = 0;
        } else {
            PWMSetSpeedConsigne(15, MOTEUR_DROIT);
            PWMSetSpeedConsigne(-15, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
        }
        break;

    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_SUR_PLACE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;

    // --- ROTATIONS DROITE (AVEC ANTI-TOUPIS) ---

//    case STATE_TOURNE_LEGER_DROITE:
//        consecutive_right_turns++;
//        consecutive_left_turns = 0;
//        
//        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
//            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'�vasion
//            consecutive_right_turns = 0; 
//        } else {
//            
//            const float MAX_DIST_DETECT = 150.0;
//            const float VITESSE_AJUSTEMENT = 20.0; // Intensit� de la rotation
//            const float VITESSE_BASE = 15.0;       // Vitesse d'avanc�e de base
//            
//            float distance = robotState.distanceTelemetreGaucheGauche;
//            if (distance < 0.0) {
//                distance = 0.0;
//            }
//            float i = distance / MAX_DIST_DETECT; 
//            float j = (1.0 - i) * VITESSE_AJUSTEMENT; 
//            
//
//            PWMSetSpeedConsigne(VITESSE_BASE - j, MOTEUR_DROIT); 
//            PWMSetSpeedConsigne(VITESSE_BASE + j, MOTEUR_GAUCHE); 
//            
//            timestamp = 0; 
//            stateRobot = STATE_TOURNE_LEGER_DROITE_EN_COURS;
//        }
//        break;        
        
        
    case STATE_TOURNE_LEGER_DROITE:
        consecutive_right_turns++;
        //consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'�vasion
            consecutive_right_turns = 0; 
        } else {
            PWMSetSpeedConsigne(20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_LEGER_DROITE_EN_COURS;
        }
        break;

    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_LEGER_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_LEGERE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;

        
        
//    case STATE_TOURNE_DROITE:
//        consecutive_right_turns++;
//        consecutive_left_turns = 0;
//        
//        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
//            stateRobot = STATE_DEMI_TOUR_DROITE;
//            consecutive_right_turns = 0;
//        } else {
//            
//            const float MAX_DIST_DETECT = 150.0;
//            const float VITESSE_AJUSTEMENT = 15.0; // Intensit� de la rotation
//            const float VITESSE_BASE = 30.0;       // Vitesse d'avanc�e de base
//            
//            float distance = robotState.distanceTelemetreGauche;
//            if (distance < 0.0) {
//                distance = 0.0;
//            }
//            float i = distance / MAX_DIST_DETECT; 
//            float j = (1.0 - i) * VITESSE_AJUSTEMENT; 
//            
//
//            PWMSetSpeedConsigne(VITESSE_BASE - j, MOTEUR_DROIT); 
//            PWMSetSpeedConsigne(VITESSE_BASE + j, MOTEUR_GAUCHE); 
//            
//            timestamp = 0; 
//            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
//        }
//        break;         
        

    case STATE_TOURNE_DROITE:
        consecutive_right_turns++;
        //consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE;
            consecutive_right_turns = 0;
        } else {
            PWMSetSpeedConsigne(5, MOTEUR_DROIT);
            PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
        }
        break;
        
        
        
        
        
    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;

    case STATE_TOURNE_SUR_PLACE_DROITE:
        consecutive_right_turns++;
        //consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE;
            consecutive_right_turns = 0;
        } else {
            PWMSetSpeedConsigne(-15, MOTEUR_DROIT);
            PWMSetSpeedConsigne(15, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
        }
        break;

    // <--- CORRECTION: �tat d'attente r�-int�gr�
    case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_SUR_PLACE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;
        
    // --- AUTRES ETATS ---

    case STATE_ARRET:
        PWMSetSpeedConsigne(-3, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-3, MOTEUR_GAUCHE);
        stateRobot = STATE_ARRET_EN_COURS;
        break;

    case STATE_ARRET_EN_COURS:
        SetNextRobotStateInAutomaticMode();
        break;

    case STATE_RECULE:
        PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-20, MOTEUR_GAUCHE);
        timestamp = 0; 
        recul_counter++; 
        // <--- Logique anti-toupis: reset des compteurs
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;
        stateRobot = STATE_RECULE_EN_COURS;
        break;

    case STATE_RECULE_EN_COURS:
        // Attendre d'avoir physiquement recul� avant de tourner
        if (timestamp > DUREE_RECUL) 
        {
            // Logique 1-sur-2
            if (recul_counter % 3 == 0) 
            {
                stateRobot = STATE_DEMI_TOUR_DROITE; 
            }
            else 
            {
                if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) {
                    stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                } else {
                    stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
                }
            }
        }
        break; 

    case STATE_DEMI_TOUR_DROITE:
//        consecutive_left_turns = 0;  
//        consecutive_right_turns = 0;
        PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
        PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
        timestamp = 0; 
        stateRobot = STATE_DEMI_TOUR_DROITE_EN_COURS;
        break;

    case STATE_DEMI_TOUR_DROITE_EN_COURS:
        if (timestamp > DUREE_DEMI_TOUR) 
        {
            stateRobot = STATE_AVANCE;
        }
        break;

    default :
        stateRobot = STATE_ATTENTE;
        break;
    }
}

unsigned char nextStateRobot=0;


//void SetNextRobotStateInAutomaticMode()
//{
//    // Initialisation par d�faut : avancer
//    nextStateRobot = STATE_AVANCE;
//
//    // --- LOGIQUE DE D�CISION HI�RARCHIS�E (G�n�ralis�e) ---
//
//    // PRIORIT� 1: DANGER CENTRE (�tat 2)
//    // G�re [xx2xx] - (ex: [00200], [11200], [22212], ...)
//    if (etatTC == 2) {
//        nextStateRobot = STATE_RECULE;
//    }
//    
//    // PRIORIT� 2: DANGER CENTRE (�tat 1)
//    // G�re [xx1xx] (sauf si Prio 1 d�j� prise)
//    else if (etatTC == 1) {
//        // Tourner du c�t� le plus libre (�tat 0)
//        if (etatTG == 0 && etatTD > 0) {
//            // Gauche est libre (0), Droite est bloqu�e (>0) -> Tourner GAUCHE
//            nextStateRobot = STATE_TOURNE_GAUCHE;
//        } else if (etatTD == 0 && etatTG > 0) {
//            // Droite est libre (0), Gauche est bloqu�e (>0) -> Tourner DROITE
//            nextStateRobot = STATE_TOURNE_DROITE;
//        } else if (etatTD == 0 && etatTG == 0) { // Les deux sont libres
//            // PR�F�RENCE: Le user pr�f�re la droite si elle est libre
//            nextStateRobot = STATE_TOURNE_DROITE;
//        }
//        else {
//            // Les deux c�t�s sont > 0 (bloqu�s)
//            // -> On utilise la distance REELLE pour choisir
//            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) {
//                nextStateRobot = STATE_TOURNE_GAUCHE; // Plus d'espace � gauche
//            } else {
//                nextStateRobot = STATE_TOURNE_DROITE; // Plus d'espace � droite (ou �galit�)
//            }
//        }
//    }
//    
//    // PRIORIT� 3: CENTRE D�GAG� (�tat 0), DANGER LAT�RAL CRITIQUE (�tat 2)
//    // G�re [x202x], [x200x], [x002x]
//    else if (etatTG == 2 || etatTD == 2) {
//        if (etatTG == 2 && etatTD == 2) {
//            // Impasse �troite (ex: [02020])
//            nextStateRobot = STATE_RECULE;
//        } else if (etatTG == 2) {
//            // Danger critique � gauche -> Tourne sur place � DROITE
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//        } else { // etatTD == 2
//            // Danger critique � droite -> Tourne sur place � GAUCHE
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//        }
//    }
//    
//    // --- RESTRUCTURATION DE PRIORIT� 4 & 5 (Pour g�rer [00011]) ---
//    // Le centre (TC) est 0. Les c�t�s (TG, TD) ne sont pas 2.
//
//    // PRIORIT� 4: MURS LOINTAINS (�tat 1 ou 2) - G�re [00011], [00012], [00022]
//    // G�re [11000] et [00011]
//    // On v�rifie qu'il s'agit bien d'un "mur" (deux capteurs) et que l'autre c�t� est d�gag�
//    else if ((etatTG > 0 && etatTGG > 0) && (etatTD == 0 && etatTDD == 0)) {
//        // [11000] ou [21000] ou [22000] -> Mur � gauche
//        nextStateRobot = STATE_TOURNE_DROITE;
//    }
//    else if ((etatTD > 0 && etatTDD > 0) && (etatTG == 0 && etatTGG == 0)) {
//        // [00011] ou [00012] ou [00022] -> Mur � droite
//        nextStateRobot = STATE_TOURNE_GAUCHE; // Action forte
//    }
//    
//    // PRIORIT� 5: OBSTACLES LOINTAINS ISOL�S (�tat 1)
//    // G�re [01010], [01000], [00010] (cas non-murs)
//    else if (etatTG == 1 || etatTD == 1) {
//        if (etatTG == 1 && etatTD == 1) {
//             // Couloir large [x101x]
//             nextStateRobot = STATE_AVANCE;
//        } else if (etatTG == 1) {
//            // Obstacle lointain isol� � gauche [x100x]
//            nextStateRobot = STATE_TOURNE_DROITE;
//        } else { // etatTD == 1
//            // Obstacle lointain isol� � droite [x001x]
//            nextStateRobot = STATE_TOURNE_GAUCHE;
//        }
//    }
//    
//    // CORRIG�: PRIORIT� 6 MANQUANTE (Logique pour obstacles ext�rieurs seuls)
//    // G�re [10001], [20002], [10000], [00002]
//    else if (etatTGG > 0 || etatTDD > 0)
//    {
//        if (etatTGG > 0 && etatTDD > 0) {
//             // Passage large ([10001] ou [20002]), on continue
//             nextStateRobot = STATE_AVANCE;
//        } else if (etatTGG > 0) { // Uniquement extr�me gauche
//           nextStateRobot = STATE_TOURNE_LEGER_DROITE;
//        } else { // etatTDD > 0 (Uniquement extr�me droite)
//           nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
//        }
//    }
//    
//    // CORRIG�: PRIORIT� 7 MANQUANTE (Cas [00000])
//    else {
//        // Tout est d�gag�
//        nextStateRobot = STATE_AVANCE;
//    }
//    
//
//    // Transition vers le nouvel �tat
//    if (nextStateRobot != stateRobot - 1){
//        stateRobot = nextStateRobot;
//    }
//}







void SetNextRobotStateInAutomaticMode()
{
    // 'obstacle_map' est calcul�e dans la boucle main()
    
    switch (obstacle_map)
    {
        // -----------------------------------------------------------
        // 0. AVANCER (Rien, ou obstacles tr�s larges non mena�ants)
        // -----------------------------------------------------------
        case 0b00000: // [00000] - Rien
            nextStateRobot = STATE_AVANCE;
            break;
        case 0b10001: // [10001] - TGG + TDD (Passage large)
            nextStateRobot = STATE_AVANCE_PEU;
            break;
            
        // -----------------------------------------------------------
        // 1. OBSTACLES C�T� DROIT (TD, TDD) -> Tourner GAUCHE
        // -----------------------------------------------------------
        case 0b00001: // [00001] - TDD (Extr�me droite)
            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
            break;
            
        case 0b00010: // [00010] - TD (Droite)
        case 0b00011: // [00011] - TD + TDD (Mur � droite)
            nextStateRobot = STATE_TOURNE_GAUCHE;
            break;

        // -----------------------------------------------------------
        // 2. OBSTACLES C�T� GAUCHE (TG, TGG) -> Tourner DROITE
        // -----------------------------------------------------------
        case 0b10000: // [10000] - TGG (Extr�me gauche)
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
            break;
            
        case 0b01000: // [01000] - TG (Gauche)
        case 0b11000: // [11000] - TGG + TG (Mur � gauche)
            nextStateRobot = STATE_TOURNE_DROITE;
            break;

        // -----------------------------------------------------------
        // 3. BLOCAGE CENTRAL (TC impliqu�) -> Rotation sur place
        // -----------------------------------------------------------
        case 0b00100: // [00100] - TC (Obstacle pile en face)
        
        // Les deux c�t�s sont "libres" (obstacle_map == 0)
        // On v�rifie quel c�t� a LE PLUS d'espace r�el.
        if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
        {
            // Plus d'espace � gauche -> On tourne � GAUCHE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
        } 
        else if (robotState.distanceTelemetreGauche < robotState.distanceTelemetreDroit)  
        {
            // Plus d'espace � droite (ou �galit�) -> On tourne � DROITE
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
        }else
        {
            nextStateRobot = STATE_DEMI_TOUR_DROITE;

        }
        break;

        case 0b00101: // [00101] - TC + TDD
        case 0b00110: // [00110] - TC + TD
        case 0b00111: // [00111] - Blocage Centre-Droit
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // D�gager vers la gauche
            break;

        case 0b01100: // [01100] - TG + TC
        case 0b10100: // [10100] - TGG + TC
        case 0b11100: // [11100] - Blocage Centre-Gauche
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE; // D�gager vers la droite
            break;

        // -----------------------------------------------------------
        // 4. BLOCAGES COMPLEXES / MULTIPLES
        // -----------------------------------------------------------

        // Obstacles sym�triques (TG+TD ou TGG+TC+TDD) -> Avancer si centre libre
        case 0b01010: // [01010] - TG + TD (Porte)
        case 0b10101: // [10101] - TGG + TC + TDD (Cas �trange)
            nextStateRobot = STATE_RECULE; //STATE_DEMI_TOUR_DROITE; // Tente de passer au centre
            break;
            
        // Combinaisons Gauche/Droite sans Centre
        case 0b10010: // TGG + TD
        case 0b01001: // TG + TDD
        case 0b11010: // TGG+TG + TD
        case 0b01011: // TG + TD+TDD
            nextStateRobot = STATE_RECULE; //STATE_DEMI_TOUR_DROITE; // Tente de passer au centre
            break;
            
        // Combinaisons bloquant tout le passage
        case 0b01110: // [01110] - TG + TC + TD (Mur en U)
        case 0b01111: // [01111]
        case 0b11110: // [11110]
        // CORRIG�: Suppression de la faute de frappe 'case 0s.'
        case 0b10110: // [10110]
        case 0b01101: // [01101]
        case 0b11011: // [11011]
            nextStateRobot = STATE_RECULE; // Blocage frontal �tendu
            break;

        // -----------------------------------------------------------
        // 5. BLOCAGE TOTAL -> Reculer
        // -----------------------------------------------------------
        case 0b11111: // [11111] - Tout est bloqu�
            nextStateRobot = STATE_RECULE;
            break;

        // Autres cas (par d�faut, pour les combinaisons restantes)
        default:
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
            break;
    }
    
    // Si l'on n'est pas dans la transition de l'�tape en cours
    // (C'est-�-dire si on ne passe pas de STATE_XXX_EN_COURS � STATE_XXX)
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}
