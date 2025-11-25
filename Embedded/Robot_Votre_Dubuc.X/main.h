/* * File:   main.h
 * Author: E306-PC2
 *
 * Created on 15 octobre 2025, 11:25
 */

#ifndef MAIN_H
#define	MAIN_H

// États principaux
#define STATE_ATTENTE 0
#define STATE_ATTENTE_EN_COURS 1
#define STATE_AVANCE 2
#define STATE_AVANCE_EN_COURS 3

#define STATE_AVANCE_PEU 4
#define STATE_AVANCE_PEU_COURS 5

// États de rotation à gauche
#define STATE_TOURNE_LEGER_GAUCHE 6
#define STATE_TOURNE_LEGER_GAUCHE_EN_COURS 7

#define STATE_TOURNE_GAUCHE 8
#define STATE_TOURNE_GAUCHE_EN_COURS 9

// États de rotation à droite
#define STATE_TOURNE_LEGER_DROITE 10
#define STATE_TOURNE_LEGER_DROITE_EN_COURS 11

#define STATE_TOURNE_DROITE 12
#define STATE_TOURNE_DROITE_EN_COURS 13

// États de rotation sur place
#define STATE_TOURNE_SUR_PLACE_GAUCHE 14
#define STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS 15

#define STATE_TOURNE_SUR_PLACE_DROITE 16
#define STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS 17

// États d'arrêt ou de recul (optionnels)
#define STATE_ARRET 18
#define STATE_ARRET_EN_COURS 19
#define STATE_RECULE 20
#define STATE_RECULE_EN_COURS 21
#define STATE_DEMI_TOUR 22
#define STATE_DEMI_TOUR_EN_COURS 23

// Prototypes
void OperatingSystemLoop(void);
void SetNextRobotStateInAutomaticMode(void);
void StopRobotCompletely(void);
void SetNextRobotStateInMazeMode(void);



    
#define OBSTACLE_THRESHOLD3 170.0f
#define MAX_CONSECUTIVE_TURNS 50


typedef void (*LogicFunction_t)(void);


//#define DIST_OBSTACLE_DETECTE   35.0f  
//#define DIST_OBSTACLE_DETECTE1   25.0f  
//#define DIST_OBSTACLE_DETECTE2   23.0f  
// Distance pour ARRÊTER de tourner (Voie libre)
// Doit être supérieur à DIST_OBSTACLE_DETECTE pour éviter l'effet "Mite autour de la lampe"
#define DIST_VOIE_LIBRE         40.0f  
// Distance critique (Arrêt d'urgence / Pivot sur place obligatoire)
#define DIST_CRITIQUE           33.0f
#define DIST_CRITIQUE1           28.0f



// Seuils binaires pour le masque (Bitmask)
#define MASK_TGG  0x10  // 10000 - Gauche Gauche
#define MASK_TG   0x08  // 01000 - Gauche
#define MASK_TC   0x04  // 00100 - Centre
#define MASK_TD   0x02  // 00010 - Droite
#define MASK_TDD  0x01  // 00001 - Droite Droite



// --- 2. TIMERS (TICKS) ---
// Hystérésis temporelle : Une fois que le capteur dit "C'est libre", 
// on continue de tourner un tout petit peu pour dégager l'arrière du robot.
#define MARGE_SECURITE_ROTATION  5   // ~5ms de rotation supplémentaire
#define TIMEOUT_BLOCAGE          2000 // Si on tourne > 2sec, on considère qu'on est coincé
#endif	/* MAIN_H */
