/*
 * vehicle_control.h
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef INC_VEHICLE_CONTROL_H_
#define INC_VEHICLE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "app_types.h"

/*
 * Interface publique du module vehicle_control.
 *
 * Ce module reçoit les commandes de la manette, reçoit les données capteurs,
 * maintient l'état courant du véhicule et calcule la commande moteur à appliquer.
 *
 * Aucun accès matériel direct n'est fait ici.
 */

/*============================================================================
 * PUBLIC TYPES
 *===========================================================================*/

typedef enum
{
    VEHICLE_STATE_IDLE = 0,
    VEHICLE_STATE_MANUAL,
    VEHICLE_STATE_LINE_FOLLOW,
    VEHICLE_STATE_OBSTACLE_AVOID,
    VEHICLE_STATE_DIAGNOSTIC,
    VEHICLE_STATE_FAILSAFE

} vehicle_state_t;


/*============================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

/*
 * Initialise la machine à états du véhicule et remet les modes automatiques à zéro.
 */
void VehicleControl_Init(void);

/*
 * Analyse une trame Bluetooth reçue et remplit une structure control_cmd_t.
 */
bool VehicleControl_ParseFrame(const char *frame, control_cmd_t *cmd);

/*
 * Traite une commande valide provenant de la manette.
 * Met à jour l'état courant du véhicule et les modes START/STOP automatiques.
 */
void VehicleControl_ProcessCommand(const control_cmd_t *cmd);

/*
 * Force le véhicule en FAILSAFE lors d'une perte de communication Bluetooth.
 */
void VehicleControl_OnTimeout(void);

/*
 * Calcule la commande moteur à appliquer selon l'état courant du véhicule.
 */
void VehicleControl_GetMotorCommand(motor_cmd_t *mcmd);

/*
 * Retourne l'état courant du véhicule.
 */
vehicle_state_t VehicleControl_GetState(void);

/*
 * Retourne la dernière commande reçue de la manette.
 */
void VehicleControl_GetLastCommand(control_cmd_t *cmd);

/*
 * Fournit au contrôle véhicule l'état actuel du capteur de ligne.
 */
void VehicleControl_SetLineState(line_state_t line_state);

/*
 * Fournit au contrôle véhicule l'erreur de ligne calculée par le module capteur.
 */
void VehicleControl_SetLineError(int line_error);

/*
 * Fournit au contrôle véhicule les distances des capteurs de proximité.
 */
void VehicleControl_SetProximityData(const proximity_sensor_data_t *prox);

/*
 * Indique si le suivi de ligne est armé par START.
 */
bool VehicleControl_IsLineFollowEnabled(void);

/*
 * Indique si l'évitement d'obstacle est armé par START.
 */
bool VehicleControl_IsObstacleAvoidEnabled(void);

#endif /* INC_VEHICLE_CONTROL_H_ */

