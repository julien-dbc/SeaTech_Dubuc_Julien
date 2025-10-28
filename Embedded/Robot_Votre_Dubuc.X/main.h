/* * File:   main.h
 * Author: E306-PC2
 *
 * Created on 15 octobre 2025, 11:25
 */

#ifndef MAIN_H
#define	MAIN_H

// �tats principaux
#define STATE_ATTENTE 0
#define STATE_ATTENTE_EN_COURS 1
#define STATE_AVANCE 2
#define STATE_AVANCE_EN_COURS 3

// �tats de rotation � gauche
#define STATE_TOURNE_LEGER_GAUCHE 4
#define STATE_TOURNE_LEGER_GAUCHE_EN_COURS 5

#define STATE_TOURNE_GAUCHE 6
#define STATE_TOURNE_GAUCHE_EN_COURS 7

// �tats de rotation � droite
#define STATE_TOURNE_LEGER_DROITE 8
#define STATE_TOURNE_LEGER_DROITE_EN_COURS 9

#define STATE_TOURNE_DROITE 10
#define STATE_TOURNE_DROITE_EN_COURS 11

// �tats de rotation sur place
#define STATE_TOURNE_SUR_PLACE_GAUCHE 12
#define STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS 13

#define STATE_TOURNE_SUR_PLACE_DROITE 14
#define STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS 15

// �tats d'arr�t ou de recul (optionnels)
#define STATE_ARRET 16
#define STATE_ARRET_EN_COURS 17
#define STATE_RECULE 18
#define STATE_RECULE_EN_COURS 19

// Types d'obstacles d�tect�s (Obsol�te avec le syst�me 5-bits, mais gard� pour r�f�rence)
#define PAS_D_OBSTACLE 0
#define OBSTACLE_A_GAUCHE 1
#define OBSTACLE_LEGER_GAUCHE 2
#define OBSTACLE_A_DROITE 3
#define OBSTACLE_LEGER_DROITE 4
#define OBSTACLE_EN_FACE 5


// Assurez-vous que ces valeurs ne chevauchent pas les �tats existants.
// Prototypes
void OperatingSystemLoop(void);
void SetNextRobotStateInAutomaticMode(void);
void StopRobotCompletely(void);


#define DANGER_CRITIQUE_CENTRE 25 // Obstacle tr�s proche, action d'urgence
#define DANGER_CRITIQUE_COTE 25

#define DANGER_PROCHE_CENTRE 35   // Obstacle proche, rotation forte
#define DANGER_PROCHE_COTE 35

#define DANGER_LOINTAIN_CENTRE 45 // Obstacle lointain, l�ger ajustement
#define DANGER_LOINTAIN_COTE_EXT 25

#define dist 30


#define SEUIL_ETAT_2_CRITIQUE 20 // En dessous = �tat 2 (Critique)
#define SEUIL_ETAT_1_LOINTAIN 35 // En dessous = �tat 1 (Lointain), Au dessus = �tat 0 (D�gag�)

#endif	/* MAIN_H */