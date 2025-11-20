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
            pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
        }
        // MODE 2 : LABYRINTHE / MAZE (Bouton RH2)
        else if (_RH2 == 1) { 
            start_time_ticks = t1;
            robot_is_running = 1; 
            EN_PWM = 1;                   
            stateRobot = STATE_AVANCE;
            LED_VERTE_2 = 1;             // LED Verte pour mode Labyrinthe
            
            // Paramètres spécifiques Maze (souvent plus lent pour être précis)
            vitesse_avance = 25; 
            
            // ASSIGNATION DE L'INTELLIGENCE LABYRINTHE
            pfn_SetNextRobotState = &SetNextRobotStateInMazeMode;
        }
        
        
    }
    
    
//        // --- BOUCLE D'ATTENTE AVANT DÉPART ---
//    // Le robot attend ici l'appui sur le bouton.
//    while(!robot_is_running){
//        if (_RH1 == 1) { 
//            start_time_ticks = t1; // ? Enregistre le temps de départ
//            robot_is_running = 1; 
//            EN_PWM = 1;                   
//            stateRobot = STATE_AVANCE;// Autorise l'exécution
//            LED_ROUGE_2 = 1;              // Signal visuel de démarrage
//            OBSTACLE_THRESHOLD=30;
//            OBSTACLE_THRESHOLD1=45;
//            OBSTACLE_THRESHOLD2=20;
//            vitesse_avance=35;
//            pfn_SetNextRobotState = &SetNextRobotStateInAutomaticMode;
//        }
//        if (_RH2 == 1) { 
//            start_time_ticks = t1; // ? Enregistre le temps de départ
//            robot_is_running = 1; 
//            EN_PWM = 1;                   
//            stateRobot = STATE_AVANCE;// Autorise l'exécution
//            LED_VERTE_2 = 1;              // Signal visuel de démarrage
//            OBSTACLE_THRESHOLD=25;
//            OBSTACLE_THRESHOLD1=35;
//            OBSTACLE_THRESHOLD2=18;
//            vitesse_avance=20;
//            pfn_SetNextRobotState = &SetNextRobotStateInMazeMode;
//        }
//        
//    }
//    
    
    
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
            volts = ((float) result[0]) * 3.3 / 4096;if (volts < 0.4f) volts = 0.4f; robotState.distanceTelemetreGaucheGauche = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[1]) * 3.3 / 4096;if (volts < 0.4f) volts = 0.4f; robotState.distanceTelemetreGauche       = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[2]) * 3.3 / 4096;if (volts < 0.4f) volts = 0.4f; robotState.distanceTelemetreCentre       = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[3]) * 3.3 / 4096;if (volts < 0.4f) volts = 0.4f; robotState.distanceTelemetreDroit        = (volts > 0.2) ? (34 / volts - 5) : 80.0;
            volts = ((float) result[4]) * 3.3 / 4096;if (volts < 0.4f) volts = 0.4f; robotState.distanceTelemetreDroiteDroite = (volts > 0.2) ? (34 / volts - 5) : 80.0;
        }

        // 3. Debug LEDs (Allumées si obstacle détecté)
        LED_ORANGE_1 = (robotState.distanceTelemetreCentre < DIST_OBSTACLE_DETECTE);
        LED_BLEUE_1  = (robotState.distanceTelemetreGauche < DIST_OBSTACLE_DETECTE1);
        LED_ROUGE_1  = (robotState.distanceTelemetreDroit  < DIST_OBSTACLE_DETECTE1);
        LED_VERTE_1  = (robotState.distanceTelemetreDroiteDroite  < DIST_OBSTACLE_DETECTE2);
        LED_BLANCHE_1  = (robotState.distanceTelemetreGaucheGauche  < DIST_OBSTACLE_DETECTE2);
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
        PWMSetSpeedConsigne(22, MOTEUR_DROIT);
        PWMSetSpeedConsigne(-1, MOTEUR_GAUCHE); // Pivot sur roue
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
        PWMSetSpeedConsigne(-1, MOTEUR_DROIT); // Pivot sur roue
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



unsigned char nextStateRobot = 0;

// Fonction utilitaire pour l'hystérésis (Robustesse Signal)
//unsigned char IsObstacle(float distance, float threshold) {
//    // Si on détecte, on reste à 1 tant qu'on est pas "largement" sorti (threshold + hysteresis)
//    // Note: Ici je simplifie, une vraie hystérésis nécessiterait une variable d'état statique par capteur.
//    return (distance < threshold) ? 1 : 0;
//}

void SetNextRobotStateInAutomaticMode(void)
{
    // 1. Acquisition des distances (Lecture depuis la struct globale)
    float d_TC  = robotState.distanceTelemetreCentre;
    float d_TG  = robotState.distanceTelemetreGauche;
    float d_TD  = robotState.distanceTelemetreDroit;
    // Assumons que ces champs ont été ajoutés à la struct ROBOT_STATE_BITS [cite: 401]
    float d_TGG = robotState.distanceTelemetreGaucheGauche; 
    float d_TDD = robotState.distanceTelemetreDroiteDroite;

    // 2. Création du Masque Binaire (Obstacle Map)
    // 1 = Obstacle, 0 = Libre
    unsigned char obstacleMask = 0;
    
    if (d_TGG < DIST_OBSTACLE_DETECTE2) obstacleMask |= MASK_TGG;
    if (d_TG  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TG;
    if (d_TC  < DIST_OBSTACLE_DETECTE) obstacleMask |= MASK_TC;
    if (d_TD  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TD;
    if (d_TDD < DIST_OBSTACLE_DETECTE2) obstacleMask |= MASK_TDD;

    // 3. Vérification de Sécurité (CRITIQUE)
    // Priorité absolue : Si trop près, on ignore le masque standard pour pivoter d'urgence
    if (d_TC < DIST_CRITIQUE || d_TG < DIST_CRITIQUE1 || d_TD < DIST_CRITIQUE1) {
        // Logique d'urgence : fuir le coté le plus bloqué
        if ((d_TG + d_TGG) < (d_TD + d_TDD)) {
            nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE; // Trop bloqué à gauche
        } else {
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE; // Trop bloqué à droite ou face
        }
    }
    // 4. Logique Standard via Switch (Les 32 cas)
    else {
        switch (obstacleMask) {
            // --- CAS 0 : VOIE LIBRE ---
            case 0b00000: // 0: Tout libre
                nextStateRobot = STATE_AVANCE;
                break;

            // --- CAS OBSTACLES LATÉRAUX SEULS (Suivi de mur/Centrage) ---
            case 0b10000: // 16: Mur très à gauche
            case 0b01000: // 8:  Mur à gauche
            case 0b11000: // 24: Mur large à gauche
                nextStateRobot = STATE_TOURNE_DROITE; // Correction légère
                break;

            case 0b00001: // 1:  Mur très à droite
            case 0b00010: // 2:  Mur à droite
            case 0b00011: // 3:  Mur large à droite
                nextStateRobot = STATE_TOURNE_GAUCHE; // Correction légère
                break;

            // --- CAS OBSTACLES FRONTAUX SIMPLES ---
            case 0b00100: // 4:  Poteau au centre
                // Choix intelligent: aller vers le coté le plus "ouvert" (TGG vs TDD)
                if (d_TGG > d_TDD) nextStateRobot = STATE_TOURNE_GAUCHE;
                else               nextStateRobot = STATE_TOURNE_DROITE;
                break;

            // --- CAS ENTONNOIRS / COINS (Frontal + Latéral) ---
            case 0b11100: // 28: Bloqué Gauche + Centre
            case 0b01100: // 12: Bloqué Gauche + Centre
            case 0b10100: // 20: Bloqué Ext Gauche + Centre
                nextStateRobot = STATE_TOURNE_DROITE; // Fuite vers la droite
                break;

            case 0b00111: // 7:  Bloqué Droite + Centre
            case 0b00110: // 6:  Bloqué Droite + Centre
            case 0b00101: // 5:  Bloqué Ext Droite + Centre
                nextStateRobot = STATE_TOURNE_GAUCHE; // Fuite vers la gauche
                break;

            // --- CAS COMPLEXES / TROP D'OBSTACLES (Sac de noeuds) ---
            case 0b11110: // 30: Tout sauf ext droite
            case 0b01111: // 15: Tout sauf ext gauche
            case 0b11111: // 31: Cul de sac total
            case 0b10101: // 21: Obstacles éparpillés  
                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;  // Fuite vers la gauche
                break;
                
                
                
            case 0b10001:
                
                // Cul de sac ou blocage massif -> Demi-tour sur place
                nextStateRobot = STATE_AVANCE; 
                break;
                
                
                
            
            case 0b01010: // 10: Tunnel étroit (Murs G et D, Centre libre 
                nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;  // Fuite vers la gauche
                break;
                

            // --- CAS PAR DÉFAUT (Combinaisons rares) ---
            default:
                // Si le centre est pris, on tourne. Sinon on avance prudemment.
                if (obstacleMask & MASK_TC) nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
                else                        nextStateRobot = STATE_AVANCE;
                break;
        }
    }

    // 5. Application de l'état (Gestion des transitions)
    // On vérifie si l'état change et si on n'est pas déjà en train d'attendre une transition (state - 1)
    if (nextStateRobot != stateRobot && nextStateRobot != (stateRobot - 1)) {
        stateRobot = nextStateRobot;
    }
}

//
////===============================================SetNextRobotStateInMazeMode()=======================================
//
///**
// * @brief Définit l'état suivant du robot en mode Labyrinthe (Wall Follower).
// *
// * Cette fonction implémente la "règle de la main gauche".
// * Le robot essaie de toujours garder un mur sur sa gauche.
// * * Hiérarchie de décision :
// * P1 : Gérer les culs-de-sac (blocage total).
// * P2 : Gérer les coins extérieurs (quand le mur gauche disparaît).
// * P3 : Gérer les coins intérieurs (quand un mur apparaît en face).
// * P4 : Gérer le suivi de couloir (cas normal, avancer tout droit).
// *
// * Utilise 'obstacle_map' (variable globale) :
// * TGG(4) | TG(3) | TC(2) | TD(1) | TDD(0)
// */
/**
 * @brief Mode Labyrinthe : Règle de la main gauche (Wall Follower).
 */
void SetNextRobotStateInMazeMode(void)
{
    // 1. Acquisition des distances (Même méthode que le mode Auto)
    float d_TC  = robotState.distanceTelemetreCentre;
    float d_TG  = robotState.distanceTelemetreGauche;
    float d_TD  = robotState.distanceTelemetreDroit;
    float d_TGG = robotState.distanceTelemetreGaucheGauche; 
    
    // 2. Création du Masque Binaire (Mise à jour pour ce mode)
    unsigned char obstacleMask = 0;
    
    // Note : On peut ajuster les seuils pour le labyrinthe si nécessaire
    if (d_TGG < DIST_OBSTACLE_DETECTE2) obstacleMask |= MASK_TGG;
    if (d_TG  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TG;
    if (d_TC  < DIST_OBSTACLE_DETECTE)  obstacleMask |= MASK_TC;
    if (d_TD  < DIST_OBSTACLE_DETECTE1) obstacleMask |= MASK_TD;

    nextStateRobot = stateRobot; // Par défaut, on garde l'état

    // --- LOGIQUE DE PRIORITÉ (Main Gauche) ---

    // CAS 1 : CUL-DE-SAC (Bloqué Devant + Gauche + Droite)
    if ((obstacleMask & MASK_TC) && (obstacleMask & MASK_TG) && (obstacleMask & MASK_TD)) {
        nextStateRobot = STATE_DEMI_TOUR;
    }
    
    // CAS 2 : COIN EXTÉRIEUR (Perte du mur gauche)
    // Si on avançait et qu'on ne voit plus de mur à gauche, il faut tourner pour le suivre.
    else if (!(obstacleMask & MASK_TG)) {
        // On vérifie qu'on n'est pas juste en train de passer une petite ouverture
        // Si on ne voit rien à gauche, on tourne à gauche pour recoller au mur
        nextStateRobot = STATE_TOURNE_GAUCHE; 
        // Ou STATE_TOURNE_SUR_PLACE_GAUCHE si le virage doit être serré
    }
    
    // CAS 3 : COIN INTÉRIEUR (Mur en face)
    // On a un mur à gauche (TG détecté), mais on est bloqué devant.
    else if (obstacleMask & MASK_TC) {
        // On tourne à droite pour éviter le mur en face tout en gardant le mur gauche comme pivot
        nextStateRobot = STATE_TOURNE_SUR_PLACE_DROITE;
    }
    
    // CAS 4 : SUIVI DE MUR (Couloir normal)
    // Mur à gauche (TG ok), Voie libre devant (TC ok).
    else {
        // Régulation fine pour rester parallèle
        if (obstacleMask & MASK_TGG) {
            // On est TROP PRÈS du mur gauche (capteur TGG déclenché)
            nextStateRobot = STATE_TOURNE_LEGER_DROITE;
        }
        else {
            // Distance correcte
            nextStateRobot = STATE_AVANCE;
        }
    }

    // 3. Application de l'état (Gestion des transitions)
    // On évite de relancer l'état s'il est déjà en cours (ex: STATE_AVANCE vs STATE_AVANCE_EN_COURS)
    if (nextStateRobot != stateRobot && nextStateRobot != (stateRobot - 1)) {
        stateRobot = nextStateRobot;
    }
}