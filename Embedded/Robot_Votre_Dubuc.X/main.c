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
unsigned char robot_is_running = 0; // 0: Arrêté / 1: En cours d'exécution

unsigned char obstacle_map = 0;

unsigned char recul_counter =0;
unsigned char consecutive_left_turns = 0;
unsigned char consecutive_right_turns = 0;

unsigned char vitesse_avance;
LogicFunction_t pfn_SetNextRobotState;




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



LogicFunction_t pfn_SetNextRobotState = NULL;
int main(void) {
    // --- Initialisation ---
    InitOscillator();
    InitIO();
    InitTimer1();
    InitTimer23();
    InitTimer4();
    InitPWM();
    InitADC1();
    unsigned char OBSTACLE_THRESHOLD;
    unsigned char OBSTACLE_THRESHOLD1;
    unsigned char OBSTACLE_THRESHOLD2;
    pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
    
    
    // Configuration initiale des LEDs (toutes éteintes au départ)
    LED_BLANCHE_1 = 0; LED_BLEUE_1 = 0; LED_ORANGE_1 = 0; LED_ROUGE_1 = 0; LED_VERTE_1 = 0;
    LED_BLANCHE_2 = 1; LED_BLEUE_2 = 1; LED_ORANGE_2 = 1; LED_ROUGE_2 = 0; LED_VERTE_2 = 0;
    
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
            OBSTACLE_THRESHOLD=30;
            OBSTACLE_THRESHOLD1=45;
            OBSTACLE_THRESHOLD2=20;
            vitesse_avance=35;
            pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
        }
        if (_RH2 == 1) { 
            start_time_ticks = t1; // ? Enregistre le temps de départ
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;// Autorise l'exécution
            LED_VERTE_2 = 1;              // Signal visuel de démarrage
            OBSTACLE_THRESHOLD=25;
            OBSTACLE_THRESHOLD1=35;
            OBSTACLE_THRESHOLD2=18;
            vitesse_avance=20;
            pfn_SetNextRobotState = &SetNextRobotStateInMazeMode;
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
            obstacle_map = 0;
            if (robotState.distanceTelemetreGaucheGauche < OBSTACLE_THRESHOLD2) obstacle_map |= (1 << 4);
            if (robotState.distanceTelemetreGauche < OBSTACLE_THRESHOLD)       obstacle_map |= (1 << 3);
            if (robotState.distanceTelemetreCentre < OBSTACLE_THRESHOLD1)       obstacle_map |= (1 << 2);
            if (robotState.distanceTelemetreDroit < OBSTACLE_THRESHOLD)        obstacle_map |= (1 << 1);
            if (robotState.distanceTelemetreDroiteDroite < OBSTACLE_THRESHOLD2) obstacle_map |= (1 << 0);
           
        }

        // 3. **LOGIQUE DE CONTRÔLE ET AFFICHAGE (LEDs)**
        
        // Mise à jour de l'état du robot
//        OperatingSystemLoop(); 
        
        
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
    



unsigned char nextStateRobot=0;
void OperatingSystemLoop(void)
{
    // Durées minimales (en ticks/ms) pour les actions.
    // (J'utilise les nouvelles valeurs que vous avez fournies)
    const unsigned int DUREE_RECUL = 10;
    const unsigned int DUREE_ROTATION_LEGERE = 5;     // Ancien: 15
    const unsigned int DUREE_ROTATION_PRONONCEE = 7;  // Ancien: 15
    const unsigned int DUREE_ROTATION_SUR_PLACE = 12; // Ancien: 25
    const unsigned int DUREE_DEMI_TOUR = 15; 

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
        PWMSetSpeedConsigne(vitesse_avance, MOTEUR_DROIT);
        PWMSetSpeedConsigne(vitesse_avance, MOTEUR_GAUCHE);
        stateRobot = STATE_AVANCE_EN_COURS;
        // <--- Logique anti-toupis: reset des compteurs
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;
        break;

    case STATE_AVANCE_EN_COURS:
        //SetNextRobotStateInAutomaticMode();
        pfn_SetNextRobotState();
        break;

    case STATE_AVANCE_PEU:
        PWMSetSpeedConsigne(15, MOTEUR_DROIT);
        PWMSetSpeedConsigne(15, MOTEUR_GAUCHE);
        stateRobot = STATE_AVANCE_PEU_COURS;
        // <--- Logique anti-toupis: reset des compteurs
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;       
        break;

    case STATE_AVANCE_PEU_COURS:
        //SetNextRobotStateInAutomaticMode();
        pfn_SetNextRobotState();
        break;

 
        
    case STATE_TOURNE_GAUCHE:
        consecutive_left_turns++;
        consecutive_right_turns = 0;
        
        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'évasion
            consecutive_left_turns = 0; // On réinitialise
        } else {
            PWMSetSpeedConsigne(30, MOTEUR_DROIT);
            PWMSetSpeedConsigne(10, MOTEUR_GAUCHE); 
            timestamp = 0; 
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
        }
        break;
        
        
        
        
    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            SetNextRobotStateInAutomaticMode(); // On réévalue APRES avoir tourné
            //stateRobot = STATE_AVANCE; //pfn_SetNextRobotState();
        }
        break;

        
        
   
        
        
    case STATE_TOURNE_LEGER_GAUCHE:
        consecutive_left_turns++;
        consecutive_right_turns = 0;
        
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

        
        
        
        
    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_LEGER_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_LEGERE)
        {
            //SetNextRobotStateInAutomaticMode();
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;

    case STATE_TOURNE_SUR_PLACE_GAUCHE:
        consecutive_left_turns++;
        consecutive_right_turns = 0;
        
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

    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_SUR_PLACE)
        {
            //SetNextRobotStateInAutomaticMode();
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;

        
        
    case STATE_TOURNE_LEGER_DROITE:
        consecutive_right_turns++;
        consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'évasion
            consecutive_right_turns = 0; 
        } else {
            PWMSetSpeedConsigne(20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_LEGER_DROITE_EN_COURS;
        }
        break;

    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_LEGER_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_LEGERE)
        {
            //SetNextRobotStateInAutomaticMode();
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;

        
        

    case STATE_TOURNE_DROITE:
        consecutive_right_turns++;
        consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE;
            consecutive_right_turns = 0;
        } else {
            PWMSetSpeedConsigne(10, MOTEUR_DROIT);
            PWMSetSpeedConsigne(30, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
        }
        break;
        
        
        
        
        
    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            //SetNextRobotStateInAutomaticMode();
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;

    case STATE_TOURNE_SUR_PLACE_DROITE:
        consecutive_right_turns++;
        consecutive_left_turns = 0;
        
        if (consecutive_right_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE;
            consecutive_right_turns = 0;
        } else {
            PWMSetSpeedConsigne(-20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
            timestamp = 0; 
            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
        }
        break;

    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_SUR_PLACE)
        {
            //SetNextRobotStateInAutomaticMode();
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;
        
    // --- AUTRES ETATS ---

//    case STATE_ARRET:
//        PWMSetSpeedConsigne(-5, MOTEUR_DROIT);
//        PWMSetSpeedConsigne(-5, MOTEUR_GAUCHE);
//        stateRobot = STATE_ARRET_EN_COURS;
//        break;
//
//    case STATE_ARRET_EN_COURS:
//        //SetNextRobotStateInAutomaticMode();
//        pfn_SetNextRobotState();
//        break;

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
        // Attendre d'avoir physiquement reculé avant de tourner
        if (timestamp > DUREE_RECUL) 
        {
            pfn_SetNextRobotState();
//            // Logique 1-sur-2
//            if (recul_counter % 3 == 0) 
//            {
//                stateRobot = STATE_DEMI_TOUR_DROITE; 
//                recul_counter=0; 
//            }
//            else 
//            {
//                if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) {
//                    stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//                } else {
//                    stateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//                }
//            }
        }
        break; 

    case STATE_DEMI_TOUR_DROITE:
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;
        PWMSetSpeedConsigne(25, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-25, MOTEUR_GAUCHE);
        timestamp = 0; 
        stateRobot = STATE_DEMI_TOUR_DROITE_EN_COURS;
        break;

    case STATE_DEMI_TOUR_DROITE_EN_COURS:
        if (timestamp > DUREE_DEMI_TOUR) 
        {
            pfn_SetNextRobotState();
            //stateRobot = STATE_AVANCE;
        }
        break;

    default :
        stateRobot = STATE_ATTENTE;
        break;
    }
}



//void SetNextRobotStateInAutomaticMode()
//{
//    // --- MEMOIRE DU PIÈGE (Ajout) ---
//    static int stuck_score = 0; 
//
//    // --- LOGIQUE ORIGINALE (Stable) ---
//    // TGG | TG | TC | TD | TDD
//    //  4  |  3 |  2 |  1 |  0
//
//    // 1. PRIORITÉ 1: DANGER CENTRAL
//    if (obstacle_map & (1 << 2)) // Si (TC == 1)
//    {
//        // Cas 1A: Impasse / Mur frontal
//        if ((obstacle_map & (1 << 3)) || (obstacle_map & (1 << 1))) 
//        {
//            nextStateRobot = STATE_RECULE;
//        }
//        // Cas 1B: Obstacle "poteau"
//        else 
//        {
//            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//            else 
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//        }
//    }
//
//    // 2. PRIORITÉ 2: DANGER LATÉRAL (Centre libre)
//    // Cas 2A: Obstacles des deux côtés ("Porte")
//    else if ((obstacle_map & (1 << 3)) && (obstacle_map & (1 << 1)))
//    {
//        nextStateRobot = STATE_AVANCE_PEU; 
//    }
//    // Cas 2B: Obstacle à GAUCHE
//    else if (obstacle_map & (1 << 3)) // TG
//    {
//        nextStateRobot = STATE_TOURNE_DROITE; 
//    }
//    else if (obstacle_map & (1 << 4)) // TGG
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_DROITE; 
//    }
//    // Cas 2C: Obstacle à DROITE
//    else if (obstacle_map & (1 << 1)) // TD
//    {
//        nextStateRobot = STATE_TOURNE_GAUCHE; 
//    }
//    else if (obstacle_map & (1 << 0)) // TDD
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; 
//    }
//
//    // 3. PRIORITÉ 3: TOUT EST DÉGAGÉ
//    else 
//    {
//        nextStateRobot = STATE_AVANCE;
//    }
//
//
//    // --- SURVEILLANCE DU CUL-DE-SAC (Ajout Intelligent) ---
//    // Cette partie s'exécute APRES avoir choisi l'état, pour le corriger si besoin.
//
//    if (nextStateRobot == STATE_RECULE) 
//    {
//        // Si le robot décide de reculer, c'est grave. On augmente le score fortement.
//        stuck_score += 10; 
//    }
//    else if (nextStateRobot == STATE_AVANCE || nextStateRobot == STATE_AVANCE_PEU) 
//    {
//        // Si le robot avance, on diminue le score DOUCEMENT.
//        // Cela permet de se "souvenir" qu'on a reculé il y a peu de temps.
//        if (stuck_score > 0) stuck_score -= 1; 
//    }
//    // Note : Les états de rotation (TOURNE_XXX) ne changent pas le score.
//
//    // SEUIL DE DÉCLENCHEMENT
//    // Si stuck_score dépasse 30 (ex: 3 reculs rapprochés), on force le demi-tour.
//    if (stuck_score >= 50)
//    {
//        nextStateRobot = STATE_DEMI_TOUR_DROITE;
//        stuck_score = 0; // On remet à zéro une fois la décision prise
//    }
//
//
//    // --- APPLICATION ---
//    if (nextStateRobot != stateRobot - 1)
//    {
//        stateRobot = nextStateRobot;
//    }
//}
//



void SetNextRobotStateInAutomaticMode()
{
    // --- MEMOIRE DU PIÈGE ---
    static int stuck_score = 0; 
    
    // Distance de sécurité absolue (si on est plus près que ça, on pivote sur place)
    const float DISTANCE_TROP_PRES = 5.0; 

    // -------------------------------------------------------------
    // 1. PRIORITÉ 1: DANGER CENTRAL (Collision frontale)
    // -------------------------------------------------------------
    if (obstacle_map & (1 << 2)) // Si (TC == 1)
    {
        // Si on est bloqué devant ET sur les côtés -> Recul
        if ((obstacle_map & (1 << 3)) || (obstacle_map & (1 << 1))) 
        {
            nextStateRobot = STATE_RECULE;
        }
        // Sinon, on pivote du côté le plus libre
        else 
        {
            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
            else 
                nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
        }
    }

    // -------------------------------------------------------------
    // 2. PRIORITÉ 2: SAUVEGARDE LATÉRALE (Mur très proche)
    // Si un mur est vraiment collé au robot, on interdit d'avancer.
    // -------------------------------------------------------------
    else if (robotState.distanceTelemetreGauche < DISTANCE_TROP_PRES || 
             robotState.distanceTelemetreGaucheGauche < DISTANCE_TROP_PRES)
    {
        // Trop près à gauche -> On pivote sur place à droite pour dégager l'avant
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    }
    else if (robotState.distanceTelemetreDroit < DISTANCE_TROP_PRES || 
             robotState.distanceTelemetreDroiteDroite < DISTANCE_TROP_PRES)
    {
        // Trop près à droite -> On pivote sur place à gauche
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    }

    // -------------------------------------------------------------
    // 3. PRIORITÉ 3: GESTION DU PASSAGE (La correction majeure est ici)
    // Si on voit gauche (TG) ET droite (TD), on ne fonce pas bêtement.
    // -------------------------------------------------------------
    else if ((obstacle_map & (1 << 3)) && (obstacle_map & (1 << 1)))
    {
        // On compare les distances réelles pour se centrer
        float diff = robotState.distanceTelemetreGauche - robotState.distanceTelemetreDroit;

        if (diff > 5.0) {
            // Beaucoup plus de place à gauche -> On corrige vers la gauche
            nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; 
        }
        else if (diff < -5.0) {
            // Beaucoup plus de place à droite -> On corrige vers la droite
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
        }
        else {
            // On est à peu près centré -> On peut avancer doucement
            nextStateRobot = STATE_AVANCE_PEU; 
        }
    }

    // -------------------------------------------------------------
    // 4. PRIORITÉ 4: EVITEMENT CLASSIQUE (Un seul côté détecté)
    // -------------------------------------------------------------
    else if (obstacle_map & (1 << 3)) // Obstacle à GAUCHE (TG)
    {
        nextStateRobot = STATE_TOURNE_DROITE; 
    }
    else if (obstacle_map & (1 << 4)) // Obstacle loin à GAUCHE (TGG)
    {
        nextStateRobot = STATE_TOURNE_LEGER_DROITE; 
    }
    else if (obstacle_map & (1 << 1)) // Obstacle à DROITE (TD)
    {
        nextStateRobot = STATE_TOURNE_GAUCHE; 
    }
    else if (obstacle_map & (1 << 0)) // Obstacle loin à DROITE (TDD)
    {
        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; 
    }

    // -------------------------------------------------------------
    // 5. VOIE LIBRE
    // -------------------------------------------------------------
    else 
    {
        nextStateRobot = STATE_AVANCE;
    }

    // --- SURVEILLANCE DU CUL-DE-SAC ---
    if (nextStateRobot == STATE_RECULE) stuck_score += 10; 
    else if (nextStateRobot == STATE_AVANCE) { if (stuck_score > 0) stuck_score -= 1; }

    if (stuck_score >= 50) {
        nextStateRobot = STATE_DEMI_TOUR_DROITE;
        stuck_score = 0; 
    }

    // Application
    if (nextStateRobot != stateRobot - 1) stateRobot = nextStateRobot;
}




//void SetNextRobotStateInAutomaticMode()
//{
//    // --- MEMOIRE DU PIÈGE ---
//    static int stuck_score = 0; 
//    
//    // Définition d'une distance critique (en cm)
//    // Si un obstacle est plus proche que ça, on abandonne toute manoeuvre douce.
//    const float DISTANCE_CRITIQUE = 6.0; 
//
//    // -------------------------------------------------------------
//    // 1. PRIORITÉ 1: DANGER CENTRAL (Inchangé - Anti-collision frontale)
//    // -------------------------------------------------------------
//    if (obstacle_map & (1 << 2)) // Si (TC == 1)
//    {
//        // Cas 1A: Impasse / Mur frontal
//        if ((obstacle_map & (1 << 3)) || (obstacle_map & (1 << 1))) 
//        {
//            nextStateRobot = STATE_RECULE;
//        }
//        // Cas 1B: Obstacle "poteau"
//        else 
//        {
//            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//            else 
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//        }
//    }
//
//    // -------------------------------------------------------------
//    // 1.5. PRIORITÉ CRITIQUE : SAUVEGARDE LATÉRALE (NOUVEAU)
//    // Ici, on regarde la distance BRUTE. Si on est trop près (< 10cm),
//    // on interdit le "Forçage de passage" et on tourne franchement.
//    // -------------------------------------------------------------
//    else if (robotState.distanceTelemetreGaucheGauche < DISTANCE_CRITIQUE)
//    {
//        // TGG détecte un mur TRÈS proche -> On tourne à DROITE (Pas léger, normal)
//        nextStateRobot = STATE_TOURNE_DROITE;
//    }
//    else if (robotState.distanceTelemetreDroiteDroite < DISTANCE_CRITIQUE)
//    {
//        // TDD détecte un mur TRÈS proche -> On tourne à GAUCHE (Pas léger, normal)
//        nextStateRobot = STATE_TOURNE_GAUCHE;
//    }
//
//    // -------------------------------------------------------------
//    // 2. PRIORITÉ 2: DANGER LATÉRAL (Centre libre)
//    // Maintenant, on peut forcer le passage car on sait qu'on n'est pas
//    // en zone "Critique".
//    // -------------------------------------------------------------
//    
//    // Cas 2A: Obstacles des deux côtés ("Porte")
//    else if ((obstacle_map & (1 << 3)) && (obstacle_map & (1 << 1)))
//    {
//        nextStateRobot = STATE_AVANCE_PEU; 
//    }
//    // Cas 2B: Obstacle à GAUCHE (Detection standard > 10cm mais < Threshold)
//    else if (obstacle_map & (1 << 3)) // TG
//    {
//        nextStateRobot = STATE_TOURNE_DROITE; 
//    }
//    else if (obstacle_map & (1 << 4)) // TGG (Loin)
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_DROITE; 
//    }
//    // Cas 2C: Obstacle à DROITE (Detection standard > 10cm mais < Threshold)
//    else if (obstacle_map & (1 << 1)) // TD
//    {
//        nextStateRobot = STATE_TOURNE_GAUCHE; 
//    }
//    else if (obstacle_map & (1 << 0)) // TDD (Loin)
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; 
//    }
//
//    // -------------------------------------------------------------
//    // 3. PRIORITÉ 3: TOUT EST DÉGAGÉ
//    // -------------------------------------------------------------
//    else 
//    {
//        nextStateRobot = STATE_AVANCE;
//    }
//
//
//    // --- SURVEILLANCE DU CUL-DE-SAC (Code existant) ---
//    if (nextStateRobot == STATE_RECULE) 
//    {
//        stuck_score += 10; 
//    }
//    else if (nextStateRobot == STATE_AVANCE || nextStateRobot == STATE_AVANCE_PEU) 
//    {
//        if (stuck_score > 0) stuck_score -= 1; 
//    }
//
//    if (stuck_score >= 60)
//    {
//        nextStateRobot = STATE_DEMI_TOUR_DROITE;
//        stuck_score = 0; 
//    }
//
//    // --- APPLICATION ---
//    if (nextStateRobot != stateRobot - 1)
//    {
//        stateRobot = nextStateRobot;
//    }
//}
//
//






//===============================================SetNextRobotStateInMazeMode()=======================================

/**
 * @brief Définit l'état suivant du robot en mode Labyrinthe (Wall Follower).
 *
 * Cette fonction implémente la "règle de la main gauche".
 * Le robot essaie de toujours garder un mur sur sa gauche.
 * * Hiérarchie de décision :
 * P1 : Gérer les culs-de-sac (blocage total).
 * P2 : Gérer les coins extérieurs (quand le mur gauche disparaît).
 * P3 : Gérer les coins intérieurs (quand un mur apparaît en face).
 * P4 : Gérer le suivi de couloir (cas normal, avancer tout droit).
 *
 * Utilise 'obstacle_map' (variable globale) :
 * TGG(4) | TG(3) | TC(2) | TD(1) | TDD(0)
 */
void SetNextRobotStateInMazeMode(void)
{
    // PRIORITÉ 1: CUL-DE-SAC
    // Si l'avant (TC), la gauche (TG) ET la droite (TD) sont bloqués.
    // C'est une impasse.
    if ( (obstacle_map & (1 << 2)) && // Si TC == 1
         (obstacle_map & (1 << 3)) && // ET TG == 1
         (obstacle_map & (1 << 1)) )  // ET TD == 1
    {
        // Seule solution : faire demi-tour
        nextStateRobot = STATE_DEMI_TOUR_DROITE;
    }

    // PRIORITÉ 2: COIN EXTÉRIEUR (Perte du mur gauche)
    // S'il n'y a PAS de mur à gauche (TG == 0), le robot doit
    // tourner à gauche pour "retrouver" un mur à suivre.
    else if ( !(obstacle_map & (1 << 3)) ) // Si TG == 0
    {
        // On tourne sur place à gauche (virage ~90°)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    }
    
    // PRIORITÉ 3: COIN INTÉRIEUR (Mur en face)
    // Si on arrive ici, c'est que TG == 1 (on a un mur à gauche).
    // Si on voit un mur en face (TC == 1), c'est un coin intérieur.
    else if ( (obstacle_map & (1 << 2)) ) // Si TC == 1
    {
        // On tourne sur place à droite (virage ~90°)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    }

    // PRIORITÉ 4: SUIVI DE MUR (Couloir normal)
    // On arrive ici si :
    // - TG == 1 (on a un mur à gauche)
    // - TC == 0 (l'avant est dégagé)
    // C'est le cas normal : on longe le mur.
    else 
    {
        // On affine la trajectoire pour rester parallèle au mur gauche.
        
        // Si TGG == 1 (on est TROP PRÈS du mur gauche)
        if ( (obstacle_map & (1 << 4)) ) 
        {
            // On se décale légèrement à droite pour s'éloigner
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
        }
        // Si TGG == 0 (on est à bonne distance)
        else
        {
            // On continue tout droit
            nextStateRobot = STATE_AVANCE;
        }
    }

    // --- Transition d'état ---
    // (Logique inchangée de votre fonction d'origine)
    // Si le nouvel état est différent de l'état "en cours" actuel
    if (nextStateRobot != stateRobot - 1)
    {
        stateRobot = nextStateRobot;
    }
}