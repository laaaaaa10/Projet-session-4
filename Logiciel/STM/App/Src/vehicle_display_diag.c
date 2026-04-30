/*
 * vehicle_display_diag.c
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Module d'affichage du menu diagnostic.
 *
 * Ce fichier construit les écrans du mode diagnostic :
 * capteurs, communication, moteurs, DEL et test de l'afficheur.
 *
 * Il ne gère pas la navigation du menu. La navigation est gérée par
 * vehicle_diagnostic.c.
 */

#include <stdio.h>
#include <string.h>

#include "vehicle_display_diag.h"
#include "vehicle_diagnostic.h"
#include "vehicle_motors.h"
#include "vehicle_display_data.h"


#define LINE_SENSOR_ADDR_7BIT 0x22


/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/

static void BuildDiagRootDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagSensorsDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagLineSensorDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void BuildDiagProximityDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void BuildDiagCommunicationDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagMotorListDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagMotorDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag, const control_cmd_t *cmd);
static void BuildDiagLedListDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagLedDetailDisplay(char line0[17], char line1[17], char line2[17], char line3[17], const diagnostic_ctx_t *diag);
static void BuildDiagDisplayTestDisplay(char line0[17], char line1[17], char line2[17], char line3[17]);
static void ExtractFrameFieldToLine(const char *frame, uint8_t field_index, char out[17]);


/*============================================================================
 * DISPLAY BUILDERS
 *===========================================================================*/

/*
 * Construit l'écran du mode diagnostic.
 * Le menu diagnostic est découpé en sous-écrans pour garder cette fonction lisible.
 */
void VehicleDisplayDiag_BuildLines(char line0[17],
                                   char line1[17],
                                   char line2[17],
                                   char line3[17],
                                   const control_cmd_t *cmd)
{
    diagnostic_ctx_t diag;

    if (cmd == NULL)
        return;

    VehicleDiagnostic_GetContext(&diag);

    if (diag.cursor < diag.scroll)
        diag.scroll = diag.cursor;

    if (diag.cursor > diag.scroll + 2)
        diag.scroll = diag.cursor - 2;

    switch (diag.menu)
    {
        case DIAG_MENU_ROOT:
            BuildDiagRootDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_SENSORS:
            BuildDiagSensorsDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_SENSOR_LINE:
            BuildDiagLineSensorDisplay(line0, line1, line2, line3);
            break;

        case DIAG_MENU_SENSOR_PROX:
            BuildDiagProximityDisplay(line0, line1, line2, line3);
            break;

        case DIAG_MENU_COMM:
            BuildDiagCommunicationDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_MOTOR_LIST:
            BuildDiagMotorListDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_MOTOR_AVG:
        case DIAG_MENU_MOTOR_AVD:
        case DIAG_MENU_MOTOR_ARG:
        case DIAG_MENU_MOTOR_ARD:
            BuildDiagMotorDisplay(line0, line1, line2, line3, &diag, cmd);
            break;

        case DIAG_MENU_LED_LIST:
            BuildDiagLedListDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_LED_GREEN:
        case DIAG_MENU_LED_ORANGE:
        case DIAG_MENU_LED_BLUE:
        case DIAG_MENU_LED_RED:
            BuildDiagLedDetailDisplay(line0, line1, line2, line3, &diag);
            break;

        case DIAG_MENU_DISPLAY:
            BuildDiagDisplayTestDisplay(line0, line1, line2, line3);
            break;

        default:
            snprintf(line0, 17, "%-16s", "DIAGNOSTIC");
            snprintf(line1, 17, "%-16s", "");
            snprintf(line2, 17, "%-16s", "");
            snprintf(line3, 17, "%-16s", "");
            break;
    }
}


/*
 * Affiche le menu principal du diagnostic.
 * Le curseur indique le sous-menu sélectionné.
 */

static void BuildDiagRootDisplay(char line0[17],
                                 char line1[17],
                                 char line2[17],
                                 char line3[17],
                                 const diagnostic_ctx_t *diag)
{
    const char *items[] =
    {
        "Capteurs",
        "Communication",
        "Ctrl Moteur",
        "DEL",
        "Afficheur"
    };

    uint8_t i0;
    uint8_t i1;
    uint8_t i2;

    if (diag == NULL)
        return;

    i0 = diag->scroll;
    i1 = (diag->scroll + 1 < 5) ? (diag->scroll + 1) : diag->scroll;
    i2 = (diag->scroll + 2 < 5) ? (diag->scroll + 2) : diag->scroll;

    snprintf(line0, 17, "%-16s", "STATE: DIAG");

    snprintf(line1, 17, "%c %-14s",
             (i0 == diag->cursor) ? '>' : ' ',
             items[i0]);

    snprintf(line2, 17, "%c %-14s",
             (i1 == diag->cursor) ? '>' : ' ',
             items[i1]);

    snprintf(line3, 17, "%c %-14s",
             (i2 == diag->cursor) ? '>' : ' ',
             items[i2]);
}


/*
 * Affiche la liste des diagnostics liés aux capteurs.
 */
static void BuildDiagSensorsDisplay(char line0[17],
                                    char line1[17],
                                    char line2[17],
                                    char line3[17],
                                    const diagnostic_ctx_t *diag)
{
    if (diag == NULL)
        return;

    snprintf(line0, 17, "%-16s", "CAPTEURS");

    snprintf(line1, 17, "%c %-14s",
             (diag->cursor == 0) ? '>' : ' ',
             "Capteur ligne");

    snprintf(line2, 17, "%c %-14s",
             (diag->cursor == 1) ? '>' : ' ',
             "Capteurs prox");

    snprintf(line3, 17, "%-16s", "");
}


/*
 * Affiche l'état brut du capteur de ligne.
 * Les 7 bits permettent de vérifier rapidement quels capteurs détectent la ligne.
 */
static void BuildDiagLineSensorDisplay(char line0[17],
                                       char line1[17],
                                       char line2[17],
                                       char line3[17])
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);

    snprintf(line0, 17, "%-16s", "CAPTEUR LIGNE");

    snprintf(line1, 17, "I2C: 0x%02X       ", LINE_SENSOR_ADDR_7BIT);

    snprintf(line2, 17, "B:%c%c%c%c%c%c%c      ",
             (data.line_raw & (1u << 6)) ? '1' : '0',
             (data.line_raw & (1u << 5)) ? '1' : '0',
             (data.line_raw & (1u << 4)) ? '1' : '0',
             (data.line_raw & (1u << 3)) ? '1' : '0',
             (data.line_raw & (1u << 2)) ? '1' : '0',
             (data.line_raw & (1u << 1)) ? '1' : '0',
             (data.line_raw & (1u << 0)) ? '1' : '0');

    snprintf(line3, 17, "ERR:%4d       ", data.line_error);
}


/*
 * Affiche les valeurs des capteurs de proximité.
 * Les capteurs Sharp affichent mV/distance, le capteur central affiche la distance ultrason.
 */
static void BuildDiagProximityDisplay(char line0[17],
                                      char line1[17],
                                      char line2[17],
                                      char line3[17])
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);

    char tmp[17];

    snprintf(line0, 17, "%-16s", "CAPT PROX");

    if (data.prox.left_valid)
    {
        snprintf(tmp, sizeof(tmp), "L:%lu/%lu",
                 (unsigned long)data.prox_left_mv,
                 (unsigned long)data.prox.left_mm);
    }
    else
    {
        snprintf(tmp, sizeof(tmp), "L:----/----");
    }
    snprintf(line1, 17, "%-16s", tmp);

    if (data.prox.center_valid)
    {
        snprintf(tmp, sizeof(tmp), "C:%lu mm",
                 (unsigned long)data.prox.center_mm);
    }
    else
    {
        snprintf(tmp, sizeof(tmp), "C:----");
    }
    snprintf(line2, 17, "%-16s", tmp);

    if (data.prox.right_valid)
    {
        snprintf(tmp, sizeof(tmp), "R:%lu/%lu",
                 (unsigned long)data.prox_right_mv,
                 (unsigned long)data.prox.right_mm);
    }
    else
    {
        snprintf(tmp, sizeof(tmp), "R:----/----");
    }
    snprintf(line3, 17, "%-16s", tmp);
}


/*
 * Affiche la dernière trame Bluetooth reçue.
 * La trame est découpée champ par champ pour tenir sur 4 lignes de 16 caractères.
 */
static void BuildDiagCommunicationDisplay(char line0[17],
                                          char line1[17],
                                          char line2[17],
                                          char line3[17],
                                          const diagnostic_ctx_t *diag)
{
	vehicle_display_data_t data;
	VehicleDisplayData_Get(&data);

    if (diag == NULL)
        return;

    if (data.last_rx_frame[0] == '\0')
    {
        snprintf(line0,17,"%-16s","COMMUNICATION");
        snprintf(line1,17,"%-16s","AUCUNE TRAME");
        snprintf(line2,17,"%-16s","");
        snprintf(line3,17,"%-16s","");
    }
    else
    {
        ExtractFrameFieldToLine((const char *)data.last_rx_frame, diag->comm_scroll + 0, line0);
        ExtractFrameFieldToLine((const char *)data.last_rx_frame, diag->comm_scroll + 1, line1);
        ExtractFrameFieldToLine((const char *)data.last_rx_frame, diag->comm_scroll + 2, line2);
        ExtractFrameFieldToLine((const char *)data.last_rx_frame, diag->comm_scroll + 3, line3);
    }
}


/*
 * Affiche la liste des moteurs disponibles pour le test individuel.
 */
static void BuildDiagMotorListDisplay(char line0[17],
                                      char line1[17],
                                      char line2[17],
                                      char line3[17],
                                      const diagnostic_ctx_t *diag)
{
    const char *items[] =
    {
        "Moteur AVG",
        "Moteur AVD",
        "Moteur ARG",
        "Moteur ARD"
    };

    uint8_t i0;
    uint8_t i1;
    uint8_t i2;

    if (diag == NULL)
        return;

    i0 = diag->scroll;
    i1 = diag->scroll + 1;
    i2 = diag->scroll + 2;

    snprintf(line0,17,"%-16s","CTRL MOTEUR");

    snprintf(line1,17,"%c %-14s",
             (diag->cursor == i0)?'>':' ',
             items[i0]);

    if (i1 < 4)
        snprintf(line2,17,"%c %-14s",
                 (diag->cursor == i1)?'>':' ',
                 items[i1]);
    else
        snprintf(line2,17,"%-16s","");

    if (i2 < 4)
        snprintf(line3,17,"%c %-14s",
                 (diag->cursor == i2)?'>':' ',
                 items[i2]);
    else
        snprintf(line3,17,"%-16s","");
}


/*
 * Affiche et surveille un moteur individuel en mode diagnostic.
 * La commande appliquée au moteur provient de speed + turn + trim.
 */
static void BuildDiagMotorDisplay(char line0[17],
                                  char line1[17],
                                  char line2[17],
                                  char line3[17],
                                  const diagnostic_ctx_t *diag,
                                  const control_cmd_t *cmd)
{
    const char *nom = "MOTEUR";
    motor_target_t motor = MOTOR_TARGET_NONE;
    uint16_t ripple = 0;
    bool ripple_ok;
    bool nfault;

    if ((diag == NULL) || (cmd == NULL))
        return;

    switch (diag->menu)
    {
        case DIAG_MENU_MOTOR_AVG:
            nom = "Moteur AVG";
            motor = MOTOR_TARGET_AVG;
            break;

        case DIAG_MENU_MOTOR_AVD:
            nom = "Moteur AVD";
            motor = MOTOR_TARGET_AVD;
            break;

        case DIAG_MENU_MOTOR_ARG:
            nom = "Moteur ARG";
            motor = MOTOR_TARGET_ARG;
            break;

        case DIAG_MENU_MOTOR_ARD:
            nom = "Moteur ARD";
            motor = MOTOR_TARGET_ARD;
            break;

        default:
            break;
    }

    ripple_ok = VehicleMotors_ReadOneRipple(motor, &ripple);
    nfault = VehicleMotors_ReadOneFaultPin(motor);

    snprintf(line0,17,"%-16s",nom);
    snprintf(line1,17,"S%3d T%3d R%3d", cmd->speed, cmd->turn, cmd->trim);

    if (ripple_ok)
        snprintf(line2,17,"Ripple:%6u", ripple);
    else
        snprintf(line2,17,"Ripple: ----");

    snprintf(line3,17,"nFAULT:%-7s", nfault ? "FAULT":"OK");
}


/*
 * Affiche l'état des DEL contrôlables en mode diagnostic.
 */
static void BuildDiagLedListDisplay(char line0[17],
                                    char line1[17],
                                    char line2[17],
                                    char line3[17],
                                    const diagnostic_ctx_t *diag)
{
    if (diag == NULL)
        return;

    snprintf(line0,17,"%c VERTE:%-5s",
             (diag->cursor == 0)?'>':' ',
             diag->led_green_enabled ? "ON":"OFF");

    snprintf(line1,17,"%c ORANGE:%-4s",
             (diag->cursor == 1)?'>':' ',
             diag->led_orange_enabled ? "ON":"OFF");

    snprintf(line2,17,"%c BLEUE:%-5s",
             (diag->cursor == 2)?'>':' ',
             diag->led_blue_enabled ? "ON":"OFF");

    snprintf(line3,17,"%c ROUGE:%-5s",
             (diag->cursor == 3)?'>':' ',
             diag->led_red_enabled ? "ON":"OFF");
}


/*
 * Affiche l'écran de contrôle d'une DEL.
 * START active la DEL, STOP la désactive.
 */
static void BuildDiagLedDetailDisplay(char line0[17],
                                      char line1[17],
                                      char line2[17],
                                      char line3[17],
                                      const diagnostic_ctx_t *diag)
{
    bool state = false;

    if (diag == NULL)
        return;

    switch (diag->menu)
    {
        case DIAG_MENU_LED_GREEN:
            snprintf(line0,17,"%-16s","LED VERTE");
            state = diag->led_green_enabled;
            break;

        case DIAG_MENU_LED_ORANGE:
            snprintf(line0,17,"%-16s","LED ORANGE");
            state = diag->led_orange_enabled;
            break;

        case DIAG_MENU_LED_BLUE:
            snprintf(line0,17,"%-16s","LED BLEUE");
            state = diag->led_blue_enabled;
            break;

        default:
            snprintf(line0,17,"%-16s","LED ROUGE");
            state = diag->led_red_enabled;
            break;
    }

    snprintf(line1,17,"ETAT:%-8s", state ? "ACTIVE":"OFF");
    snprintf(line2,17,"%-16s","START=ON");
    snprintf(line3,17,"%-16s","STOP=OFF");
}


/*
 * Affiche les commandes du test de l'afficheur.
 * START allume tous les pixels, STOP les éteint.
 */
static void BuildDiagDisplayTestDisplay(char line0[17],
                                        char line1[17],
                                        char line2[17],
                                        char line3[17])
{
    snprintf(line0,17,"%-16s","AFFICHEUR");
    snprintf(line1,17,"%-16s","START = ON");
    snprintf(line2,17,"%-16s","STOP = OFF");
    snprintf(line3,17,"%-16s","< retour menu");
}


/*============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/*
 * Extrait un champ de la dernière trame Bluetooth reçue et le formate
 * sur une ligne de 16 caractères pour l'afficheur LCD.
 *
 * Les champs sont séparés par ';' dans la trame :
 * <mode=1;speed=50;turn=-20;trim=0;...>
 *
 * Exemple :
 * index 0 -> mode=1
 * index 1 -> speed=50
 * index 2 -> turn=-20
 *
 * Utilisé par le menu diagnostic COMM pour faire défiler la trame
 * reçue sur l'écran 4x16.
 */
static void ExtractFrameFieldToLine(const char *frame,
                              uint8_t field_index,
                              char out[17])
{
    uint8_t current = 0;
    const char *p;
    const char *start;
    const char *end;
    size_t len;

    if ((frame == NULL) || (out == NULL))
    {
        return;
    }

    snprintf(out, 17, "%-16s", "");

    p = frame;

    while (*p != '\0')
    {
        start = p;

        while ((*p != '\0') && (*p != ';') && (*p != '>'))
        {
            p++;
        }

        end = p;

        if (current == field_index)
        {
            len = (size_t)(end - start);

            if (len > 16)
            {
                len = 16;
            }

            memset(out, ' ', 16);
            memcpy(out, start, len);
            out[16] = '\0';
            return;
        }

        current++;

        if (*p == ';')
        {
            p++;
        }
        else if (*p == '>')
        {
            if (current == field_index)
            {
                snprintf(out, 17, "%-16s", ">");
                return;
            }
            break;
        }
    }
}

