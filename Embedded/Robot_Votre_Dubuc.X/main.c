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
const unsigned long T_60_SECONDS_TICKS = 6000000;

unsigned long start_time_ticks = 0;
unsigned char robot_is_running = 0; // 0: Arrêté / 1: En cours d'exécution

unsigned char obstacle_map = 0;

unsigned char recul_counter =0;
unsigned char consecutive_left_turns = 0;
unsigned char consecutive_right_turns = 0;

unsigned char vitesse_avance;

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
    unsigned char OBSTACLE_THRESHOLD;
    unsigned char OBSTACLE_THRESHOLD1;
    unsigned char OBSTACLE_THRESHOLD2;
    
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
        }
        if (_RH2 == 1) { 
            start_time_ticks = t1; // ? Enregistre le temps de départ
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;// Autorise l'exécution
            LED_VERTE_2 = 1;              // Signal visuel de démarrage
            OBSTACLE_THRESHOLD=20;
            OBSTACLE_THRESHOLD1=30;
            OBSTACLE_THRESHOLD2=15;
            vitesse_avance=20;
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
    // Durées minimales (en ticks/ms) pour les actions.
    // (J'utilise les nouvelles valeurs que vous avez fournies)
    const unsigned int DUREE_RECUL = 20;
    const unsigned int DUREE_ROTATION_LEGERE = 5;     // Ancien: 15
    const unsigned int DUREE_ROTATION_PRONONCEE = 7;  // Ancien: 15
    const unsigned int DUREE_ROTATION_SUR_PLACE = 10; // Ancien: 25
    const unsigned int DUREE_DEMI_TOUR = 40; 

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

 
        
    case STATE_TOURNE_GAUCHE:
        consecutive_left_turns++;
        consecutive_right_turns = 0;
        
        if (consecutive_left_turns > MAX_CONSECUTIVE_TURNS) {
            stateRobot = STATE_DEMI_TOUR_DROITE; // Man?uvre d'évasion
            consecutive_left_turns = 0; // On réinitialise
        } else {
            PWMSetSpeedConsigne(20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(5, MOTEUR_GAUCHE); 
            timestamp = 0; 
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
        }
        break;
        
        
        
        
    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_GAUCHE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            SetNextRobotStateInAutomaticMode(); // On réévalue APRES avoir tourné
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
            SetNextRobotStateInAutomaticMode();
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
            SetNextRobotStateInAutomaticMode();
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
            SetNextRobotStateInAutomaticMode();
        }
        break;

        
        

    case STATE_TOURNE_DROITE:
        consecutive_right_turns++;
        consecutive_left_turns = 0;
        
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
        
        
        
        
        
    // <--- CORRECTION: État d'attente ré-intégré
    case STATE_TOURNE_DROITE_EN_COURS:
        if (timestamp > DUREE_ROTATION_PRONONCEE)
        {
            SetNextRobotStateInAutomaticMode();
        }
        break;

    case STATE_TOURNE_SUR_PLACE_DROITE:
        consecutive_right_turns++;
        consecutive_left_turns = 0;
        
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

    // <--- CORRECTION: État d'attente ré-intégré
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
        // Attendre d'avoir physiquement reculé avant de tourner
        if (timestamp > DUREE_RECUL) 
        {
            // Logique 1-sur-2
            if (recul_counter % 5 == 0) 
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
        consecutive_left_turns = 0;  
        consecutive_right_turns = 0;
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
//    // 'obstacle_map' est calculée dans la boucle main()
//    
//    switch (obstacle_map)
//    {
//        // -----------------------------------------------------------
//        // 0. AVANCER (Rien, ou obstacles très larges non menaçants)
//        // -----------------------------------------------------------
//        case 0b00000: // [00000] - Rien
//            nextStateRobot = STATE_AVANCE;
//            break;
//        case 0b10001: // [10001] - TGG + TDD (Passage large)
//            nextStateRobot = STATE_AVANCE_PEU;
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
//        
//        // Les deux côtés sont "libres" (obstacle_map == 0)
//        // On vérifie quel côté a LE PLUS d'espace réel.
//        if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
//        {
//            // Plus d'espace à gauche -> On tourne à GAUCHE
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//        } 
//        else if (robotState.distanceTelemetreGauche < robotState.distanceTelemetreDroit)  
//        {
//            // Plus d'espace à droite (ou égalité) -> On tourne à DROITE
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//        }else
//        {
//            nextStateRobot = STATE_DEMI_TOUR_DROITE;
//
//        }
//        break;
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
//            nextStateRobot = STATE_RECULE; //STATE_DEMI_TOUR_DROITE; // Tente de passer au centre
//            break;
//            
//        // Combinaisons Gauche/Droite sans Centre
//        case 0b10010: // TGG + TD
//        case 0b01001: // TG + TDD
//        case 0b11010: // TGG+TG + TD
//        case 0b01011: // TG + TD+TDD
//            nextStateRobot = STATE_RECULE; //STATE_DEMI_TOUR_DROITE; // Tente de passer au centre
//            break;
//            
//        // Combinaisons bloquant tout le passage
//        case 0b01110: // [01110] - TG + TC + TD (Mur en U)
//        case 0b01111: // [01111]
//        case 0b11110: // [11110]
//        // CORRIGÉ: Suppression de la faute de frappe 'case 0s.'
//        case 0b10110: // [10110]
//        case 0b01101: // [01101]
//        case 0b11011: // [11011]
//            nextStateRobot = STATE_RECULE; // Blocage frontal étendu
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
//            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//            break;
//    }
//    
//    // Si l'on n'est pas dans la transition de l'étape en cours
//    // (C'est-à-dire si on ne passe pas de STATE_XXX_EN_COURS à STATE_XXX)
//    if (nextStateRobot != stateRobot - 1)
//        stateRobot = nextStateRobot;
//
//}





// Où : main.c
// Remplacez toute la fonction

void SetNextRobotStateInAutomaticMode()
{
    // TGG(4) | TG(3) | TC(2) | TD(1) | TDD(0)

    // -----------------------------------------------------------------
    // PRIORITÉ 1: DANGER CENTRAL (Bit 2 = 1)
    // (Logique inchangée, elle est correcte)
    // -----------------------------------------------------------------
    if (obstacle_map & (1 << 2)) // Si (TC == 1)
    {
        // Cas 1A: Impasse / Mur frontal (ex: [x111x], [01110], [11100])
        if ((obstacle_map & (1 << 3)) || (obstacle_map & (1 << 1))) 
        {
            nextStateRobot = STATE_RECULE;//STATE_TOURNE_SUR_PLACE_GAUCHE;
        }
        
        // Cas 1B: Obstacle "poteau" (ex: [00100], [10101])
        else 
        {
            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
                nextStateRobot = STATE_TOURNE_GAUCHE;
            else 
                nextStateRobot = STATE_TOURNE_DROITE;
        }
    }

    // -----------------------------------------------------------------
    // PRIORITÉ 2: GESTION DES PASSAGES (Centre libre, TC == 0)
    // NOUS VÉRIFIONS LES CAS SPÉCIFIQUES D'ABORD
    // -----------------------------------------------------------------
    
    // Cas [01010] (Porte TG+TD) -> Problème 1
    else if (obstacle_map == 0b01010)
    {
        nextStateRobot = STATE_AVANCE_PEU; // On force le passage
    }
    
    // Cas [10001] (Porte large TGG+TDD) -> Problème 2
    else if (obstacle_map == 0b10001)
    {
        nextStateRobot = STATE_AVANCE; // Passage large
    }

    // Cas [01001] (Offset TG+TDD) -> Problème 1
    else if (obstacle_map == 0b01001)
    {
        // Obstacle gauche (TG) et lointain droit (TDD)
        // On se décale légèrement à droite pour centrer
        nextStateRobot = STATE_TOURNE_LEGER_DROITE; 
    }
    
    // Cas [10010] (Offset TGG+TD)
    else if (obstacle_map == 0b10010)
    {
        // Obstacle lointain gauche (TGG) et droit (TD)
        // On se décale légèrement à gauche pour centrer
        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE;
    }
    
    // -----------------------------------------------------------------
    // PRIORITÉ 3: MURS LATÉRAUX (Centre libre, non-passage)
    // Maintenant que les passages sont gérés, on gère les murs.
    // -----------------------------------------------------------------
    
    // Cas: Obstacle à GAUCHE (TG ou TGG)
    else if (obstacle_map & (1 << 3)) // [x100x] - Si (TG == 1)
    {
        nextStateRobot = STATE_TOURNE_DROITE; // Virage prononcé
    }
    else if (obstacle_map & (1 << 4)) // [1000x] - Si (TGG == 1)
    {
        // On arrive ici SEULEMENT si TGG=1 mais PAS dans un cas "passage"
        nextStateRobot = STATE_TOURNE_LEGER_DROITE; // Virage léger
    }

    // Cas: Obstacle à DROITE (TD ou TDD)
    else if (obstacle_map & (1 << 1)) // [xx01x] - Si (TD == 1)
    {
        nextStateRobot = STATE_TOURNE_GAUCHE; // Virage prononcé
    }
    else if (obstacle_map & (1 << 0)) // [xx001] - Si (TDD == 1)
    {
        // On arrive ici SEULEMENT si TDD=1 mais PAS dans un cas "passage"
        // (ex: [00001] pur). Le bug [10001] est corrigé.
        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; // Virage léger
    }

    // -----------------------------------------------------------------
    // PRIORITÉ 4: TOUT EST DÉGAGÉ
    // -----------------------------------------------------------------
    else // obstacle_map == 0b00000
    {
        nextStateRobot = STATE_AVANCE;
    }


    // Transition vers le nouvel état
    if (nextStateRobot != stateRobot - 1)
    {
        stateRobot = nextStateRobot;
    }
}





//void SetNextRobotStateInAutomaticMode()
//{
//    // 'obstacle_map' est calculée dans la boucle main()
//    // TGG | TG | TC | TD | TDD
//    //  4  |  3 |  2 |  1 |  0
//
//    // Logique hiérarchique :
//    // 1. Gérer les dangers immédiats (Centre bloqué)
//    // 2. Gérer les obstacles latéraux (Murs)
//    // 3. Gérer les corrections de trajectoire (Murs lointains)
//    // 4. Avancer
//    
//
//    // -----------------------------------------------------------------
//    // PRIORITÉ 1: DANGER CENTRAL (Bit 2 = 1)
//    // Le capteur central (TC) est le plus critique.
//    // -----------------------------------------------------------------
//    if (obstacle_map & (1 << 2)) // Si (TC == 1)
//    {
//        // Cas 1A: Impasse / Mur frontal (ex: [x111x], [01110], [11100])
//        // Si le centre ET un côté sont bloqués, c'est une impasse.
//        if ((obstacle_map & (1 << 3)) || (obstacle_map & (1 << 1))) 
//        {
//            nextStateRobot = STATE_RECULE;
//        }
//        
//        // Cas 1B: Obstacle "poteau" (ex: [00100], [10101], [10100])
//        // Le centre est bloqué, mais les côtés (TG, TD) sont libres.
//        // On tourne sur place du côté où il y a le PLUS d'espace.
//        else 
//        {
//            if (robotState.distanceTelemetreGauche > robotState.distanceTelemetreDroit) 
//            {
//                // Plus d'espace à gauche -> On tourne à GAUCHE
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
//            } 
//            else 
//            {
//                // Plus d'espace à droite (ou égalité) -> On tourne à DROITE
//                nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
//            }
//        }
//    }
//
//    // -----------------------------------------------------------------
//    // PRIORITÉ 2: DANGER LATÉRAL (Centre libre)
//    // (TC == 0)
//    // -----------------------------------------------------------------
//    
//    // Cas 2A: Obstacles des deux côtés (ex: [01010] "Porte")
//    else if ((obstacle_map & (1 << 3)) && (obstacle_map & (1 << 1)))
//    {
//        // Le centre est libre, mais TG et TD détectent.
//        // C'est un passage étroit ou une "porte".
//        nextStateRobot = STATE_AVANCE_PEU; // On tente de passer doucement
//    }
//    
//    // Cas 2B: Obstacle à GAUCHE (TG ou TGG)
//    else if (obstacle_map & (1 << 3)) // [x100x] - Si (TG == 1)
//    {
//        nextStateRobot = STATE_TOURNE_DROITE; // Virage prononcé
//    }
//    else if (obstacle_map & (1 << 4)) // [1000x] - Si (TGG == 1)
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_DROITE; // Virage léger
//    }
//
//    // Cas 2C: Obstacle à DROITE (TD ou TDD)
//    else if (obstacle_map & (1 << 1)) // [xx01x] - Si (TD == 1)
//    {
//        nextStateRobot = STATE_TOURNE_GAUCHE; // Virage prononcé
//    }
//    else if (obstacle_map & (1 << 0)) // [xx001] - Si (TDD == 1)
//    {
//        nextStateRobot = STATE_TOURNE_LEGER_GAUCHE; // Virage léger
//    }
//
//    // -----------------------------------------------------------------
//    // PRIORITÉ 3: TOUT EST DÉGAGÉ
//    // (TC=0, TG=0, TD=0, TGG=0, TDD=0)
//    // -----------------------------------------------------------------
//    else // obstacle_map == 0b00000
//    {
//        nextStateRobot = STATE_AVANCE;
//    }
//
//
//    // Transition vers le nouvel état (logique inchangée)
//    if (nextStateRobot != stateRobot - 1)
//    {
//        stateRobot = nextStateRobot;
//    }
//}

