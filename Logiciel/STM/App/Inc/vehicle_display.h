/*
 * vehicle_display.h
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Interface publique du module vehicle_display.
 *
 * Ce module construit les lignes LCD du mode normal
 * et applique les mises à jour sur l'afficheur.
 *
 * Le mode diagnostic est délégué à vehicle_display_diag.c.
 */

#ifndef INC_VEHICLE_DISPLAY_H_
#define INC_VEHICLE_DISPLAY_H_

#include "app_types.h"
#include <stdint.h>

/*
 * Construit les 4 lignes LCD selon l'état actuel du véhicule.
 */
void VehicleDisplay_BuildLines(char line0[17],
                               char line1[17],
                               char line2[17],
                               char line3[17]);

/*
 * Écrit une ligne LCD seulement si elle a changé.
 */
void VehicleDisplay_WriteLineIfChanged(uint8_t row,
                                       char *prev,
                                       const char *next);

#endif /* INC_VEHICLE_DISPLAY_H_ */
