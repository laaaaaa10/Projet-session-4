/*
 * vehicle_display_diag.h
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Interface publique du module vehicle_display_diag.
 *
 * Ce module construit les écrans LCD du mode diagnostic :
 * capteurs, communication, moteurs, DEL et test afficheur.
 */

#ifndef INC_VEHICLE_DISPLAY_DIAG_H_
#define INC_VEHICLE_DISPLAY_DIAG_H_

#include "app_types.h"

/*
 * Construit l'affichage du mode diagnostic.
 */
void VehicleDisplayDiag_BuildLines(char line0[17],
                                   char line1[17],
                                   char line2[17],
                                   char line3[17],
                                   const control_cmd_t *cmd);

#endif /* INC_VEHICLE_DISPLAY_DIAG_H_ */
