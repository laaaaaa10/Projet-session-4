/*
 * vehicle_display.c
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Module d'affichage général du véhicule.
 *
 * Ce fichier construit les 4 lignes de texte du LCD selon l'état courant
 * du véhicule : IDLE, MANUAL, LINE_FOLLOW, OBSTACLE_AVOID, FAILSAFE
 * ou DIAGNOSTIC.
 *
 * L'affichage détaillé du mode diagnostic est délégué à vehicle_display_diag.c.
 */

#include <stdio.h>
#include <string.h>

#include "vehicle_display.h"
#include "vehicle_display_diag.h"
#include "vehicle_control.h"
#include "vehicle_diagnostic.h"
#include "st7920.h"
#include "vehicle_display_data.h"


/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/

static void BuildFailsafeDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void BuildIdleDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void BuildManualDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const control_cmd_t *cmd);
static void BuildLineFollowDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void BuildObstacleAvoidDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);

/*
 * Évite de réécrire une ligne LCD si son contenu n'a pas changé.
 */
void VehicleDisplay_WriteLineIfChanged(uint8_t row, char *prev, const char *next)
{
    if (strncmp(prev, next, 16) != 0)
    {
        ST7920_SetCursor(row, 0);
        ST7920_WriteString(next);

        strncpy(prev, next, 16);
        prev[16] = '\0';
    }
}


/*============================================================================
 * DISPLAY BUILDERS
 *===========================================================================*/

/*
 * Construit les 4 lignes de texte à afficher selon l'état actuel du véhicule.
 * Cette fonction ne parle pas directement au LCD : elle prépare seulement les chaînes.
 */

void VehicleDisplay_BuildLines(char line0[17],
                               char line1[17],
                               char line2[17],
                               char line3[17])
{
    control_cmd_t cmd;
    vehicle_state_t state;

    VehicleControl_GetLastCommand(&cmd);
    state = VehicleControl_GetState();

    switch (state)
    {
        case VEHICLE_STATE_FAILSAFE:
            BuildFailsafeDisplay(line0, line1, line2, line3);
            break;

        case VEHICLE_STATE_MANUAL:
            BuildManualDisplay(line0, line1, line2, line3, &cmd);
            break;

        case VEHICLE_STATE_LINE_FOLLOW:
            BuildLineFollowDisplay(line0, line1, line2, line3);
            break;

        case VEHICLE_STATE_OBSTACLE_AVOID:
            BuildObstacleAvoidDisplay(line0, line1, line2, line3);
            break;

        case VEHICLE_STATE_DIAGNOSTIC:
        	VehicleDisplayDiag_BuildLines(line0, line1, line2, line3, &cmd);
            break;

        case VEHICLE_STATE_IDLE:
        default:
            BuildIdleDisplay(line0, line1, line2, line3);
            break;
    }
}


/*============================================================================
 * PRIVATE DISPLAY BUILDERS
 *===========================================================================*/

static void BuildFailsafeDisplay(char line0[17],
                                 char line1[17],
                                 char line2[17],
                                 char line3[17])
{
    snprintf(line0, 17, "%-16s", "STATE: FAILSAFE");
    snprintf(line1, 17, "%-16s", "BT LOST");
    snprintf(line2, 17, "%-16s", "");
    snprintf(line3, 17, "%-16s", "");
}

static void BuildIdleDisplay(char line0[17],
                             char line1[17],
                             char line2[17],
                             char line3[17])
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);
    snprintf(line0, 17, "%-16s", "STATE: IDLE");
    snprintf(line1, 17, "%-16s",
    		data.bt_connected ? "BT CONNECTED" : "BT WAITING");
    snprintf(line2, 17, "%-16s", "");
    snprintf(line3, 17, "%-16s", "");
}

static void BuildManualDisplay(char line0[17],
                               char line1[17],
                               char line2[17],
                               char line3[17],
                               const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    snprintf(line0, 17, "%-16s", "STATE: MANUAL");
    snprintf(line1, 17, "SPD:%4d        ", cmd->speed);
    snprintf(line2, 17, "TRN:%4d        ", cmd->turn);
    snprintf(line3, 17, "TRM:%4d        ", cmd->trim);
}

static void BuildLineFollowDisplay(char line0[17],
                                   char line1[17],
                                   char line2[17],
                                   char line3[17])
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);

    snprintf(line0, 17, "%-16s", "STATE: LINE");

    snprintf(line1, 17, "B:%c%c%c%c%c%c%c      ",
             (data.line_raw & (1u << 6)) ? '1' : '0',
             (data.line_raw & (1u << 5)) ? '1' : '0',
             (data.line_raw & (1u << 4)) ? '1' : '0',
             (data.line_raw & (1u << 3)) ? '1' : '0',
             (data.line_raw & (1u << 2)) ? '1' : '0',
             (data.line_raw & (1u << 1)) ? '1' : '0',
             (data.line_raw & (1u << 0)) ? '1' : '0');

    snprintf(line2, 17, "RUN:%-5s      ",
             VehicleControl_IsLineFollowEnabled() ? "START" : "STOP");

    snprintf(line3, 17, "ERR:%4d       ", data.line_error);
}


static void BuildObstacleAvoidDisplay(char line0[17],
                                      char line1[17],
                                      char line2[17],
                                      char line3[17])
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);

    snprintf(line0, 17, "%-16s", "STATE: AVOID");

    snprintf(line1, 17, "RUN:%-5s      ",
             VehicleControl_IsObstacleAvoidEnabled() ? "START" : "STOP");

    snprintf(line2, 17, "L%4lu C%3lu   ",
             (unsigned long)data.prox.left_mm,
             (unsigned long)data.prox.center_mm);

    snprintf(line3, 17, "R%4lu          ",
             (unsigned long)data.prox.right_mm);
}
