/*
 * vehicle_motors.h
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Interface publique du module vehicle_motors.
 *
 * Ce module masque les détails matériels des quatre contrôleurs DRV8234.
 * Il permet au reste du projet de commander :
 * - les deux côtés du véhicule;
 * - un seul moteur en mode diagnostic;
 * - la mise en roue libre;
 * - la lecture nFAULT et Ripple Counting.
 */

#ifndef BSP_INC_VEHICLE_MOTORS_H_
#define BSP_INC_VEHICLE_MOTORS_H_

#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"
#include "FreeRTOS.h"
#include "semphr.h"


void VehicleMotors_SetI2C3Mutex(SemaphoreHandle_t mutex);

/*
 * Initialise les quatre contrôleurs moteurs DRV8234.
 */
void VehicleMotors_Init(void);

/*
 * Commande les deux côtés du véhicule.
 */
void VehicleMotors_SetLeftRight(int16_t left_cmd, int16_t right_cmd);

/*
 * Met tous les moteurs en roue libre.
 */
void VehicleMotors_AllCoast(void);

/*
 * Commande un seul moteur individuellement (mode diagnostic).
 */
void VehicleMotors_SetOne(motor_target_t motor, int16_t speed);

/*
 * Lit l'état nFAULT d'un moteur.
 */
bool VehicleMotors_ReadOneFaultPin(motor_target_t motor);

/*
 * Lit la valeur Ripple Counting d'un moteur.
 */
bool VehicleMotors_ReadOneRipple(motor_target_t motor, uint16_t *ripple);

/*
 * Indique si une erreur d'initialisation moteur est survenue.
 */
bool VehicleMotors_HasError(void);


#endif /* BSP_INC_VEHICLE_MOTORS_H_ */
