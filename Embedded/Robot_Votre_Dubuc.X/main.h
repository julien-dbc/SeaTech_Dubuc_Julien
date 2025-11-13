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
#define STATE_DEMI_TOUR_DROITE 22
#define STATE_DEMI_TOUR_DROITE_EN_COURS 23

// Types d'obstacles détectés (Obsolète avec le système 5-bits, mais gardé pour référence)
#define PAS_D_OBSTACLE 0
#define OBSTACLE_A_GAUCHE 1
#define OBSTACLE_LEGER_GAUCHE 2
#define OBSTACLE_A_DROITE 3
#define OBSTACLE_LEGER_DROITE 4
#define OBSTACLE_EN_FACE 5


// Assurez-vous que ces valeurs ne chevauchent pas les états existants.
// Prototypes
void OperatingSystemLoop(void);
void SetNextRobotStateInAutomaticMode(void);
void StopRobotCompletely(void);
void SetNextRobotStateInMazeMode(void);


//#define OBSTACLE_THRESHOLD //30.0f
//#define OBSTACLE_THRESHOLD1 //45.0f
//#define OBSTACLE_THRESHOLD2 //20.0f
    
#define OBSTACLE_THRESHOLD3 170.0f

#define MAX_CONSECUTIVE_TURNS 10


typedef void (*LogicFunction_t)(void);


#endif	/* MAIN_H */