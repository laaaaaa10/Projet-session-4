/*
 * vehicle_diagnostic.c
 *
 *  Created on: 23 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * vehicle_diagnostic.c - VERSION ÉTUDIANTE
 *
 * Module de logique du menu diagnostic.
 *
 * À compléter :
 * - retour arrière avec LEFT
 * - entrée dans les sous-menus avec RIGHT ou START
 * - arrêt/reset avec STOP
 * - sélection du moteur en diagnostic
 */

/*
 * Module de logique du menu diagnostic.
 *
 * Ce fichier gère l'état interne du menu diagnostic : menu courant,
 * curseur, défilement, sélection moteur, état des DEL et mode de test
 * de l'afficheur.
 *
 * Il ne construit pas les lignes LCD. L'affichage est fait par
 * vehicle_display_diag.c.
 */

#include "vehicle_diagnostic.h"
#include <string.h>

/*============================================================================
 * PRIVATE VARIABLES
 *===========================================================================*/

static diagnostic_ctx_t g_diag = {0};

/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/
static bool RisingEdge(int prev, int now);
static uint8_t GetMenuItemCount(diag_menu_t menu);

static void HandleUpButton(void);
static void HandleDownButton(void);
static void HandleLeftButton(void);
static void HandleEnterButton(const control_cmd_t *cmd);
static void HandleStopButton(void);

static void ReturnToRoot(void);
static void ReturnToSensors(void);
static void ReturnToMotorList(void);
static void ReturnToLedList(void);

/*============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static bool RisingEdge(int prev, int now)
{
    return (prev == 0) && (now != 0);
}

static uint8_t GetMenuItemCount(diag_menu_t menu)
{
    switch (menu)
    {
        case DIAG_MENU_ROOT:      return 5; /* Capteurs, Communication, Contrôleurs moteur, DEL, Afficheur */
        case DIAG_MENU_SENSORS:   return 2; /* Capteur de ligne, Capteurs de proximité */
        case DIAG_MENU_MOTOR_LIST: return 4;
        case DIAG_MENU_LED_LIST:  return 4; /* Verte, Orange, Bleue, Rouge */
        default:                  return 0;
    }
}


static void ReturnToRoot(void)
{
    g_diag.menu = DIAG_MENU_ROOT;
    g_diag.cursor = 0;
    g_diag.scroll = 0;
}

static void ReturnToSensors(void)
{
    g_diag.menu = DIAG_MENU_SENSORS;
    g_diag.cursor = 0;
    g_diag.scroll = 0;
}

static void ReturnToMotorList(void)
{
    g_diag.menu = DIAG_MENU_MOTOR_LIST;
    g_diag.cursor = 0;
    g_diag.scroll = 0;
}

static void ReturnToLedList(void)
{
    g_diag.menu = DIAG_MENU_LED_LIST;
    g_diag.cursor = 0;
    g_diag.scroll = 0;
}


static void HandleUpButton(void)
{
    if (g_diag.menu == DIAG_MENU_COMM)
    {
        if (g_diag.comm_scroll > 0)
        {
            g_diag.comm_scroll--;
        }

        return;
    }

    if (g_diag.cursor > 0)
    {
        g_diag.cursor--;

        if (g_diag.cursor < g_diag.scroll)
        {
            g_diag.scroll = g_diag.cursor;
        }
    }
}

static void HandleDownButton(void)
{
    uint8_t item_count;

    if (g_diag.menu == DIAG_MENU_COMM)
    {
        if (g_diag.comm_scroll < 10)
        {
            g_diag.comm_scroll++;
        }

        return;
    }

    item_count = GetMenuItemCount(g_diag.menu);

    if ((item_count > 0) && (g_diag.cursor + 1 < item_count))
    {
        g_diag.cursor++;

        if (g_diag.cursor > g_diag.scroll + 2)
        {
            g_diag.scroll++;
        }
    }
}

static void HandleLeftButton(void)
{
	switch(g_diag.menu)
	{
	case DIAG_MENU_SENSORS: case DIAG_MENU_COMM: case DIAG_MENU_MOTOR_LIST: case DIAG_MENU_LED_LIST:  ReturnToRoot(); break;
	case DIAG_MENU_SENSOR_LINE: case DIAG_MENU_SENSOR_PROX: ReturnToSensors(); break;
	case DIAG_MENU_MOTOR_AVG: case DIAG_MENU_MOTOR_AVD: case DIAG_MENU_MOTOR_ARG: case DIAG_MENU_MOTOR_ARD: ReturnToMotorList(); break;
	case DIAG_MENU_LED_GREEN: case DIAG_MENU_LED_ORANGE: case DIAG_MENU_LED_BLUE: case DIAG_MENU_LED_RED: ReturnToLedList(); break;
	case DIAG_MENU_DISPLAY:	switch(g_diag.display_mode){ case DIAG_DISPLAY_ALL_ON: case DIAG_DISPLAY_ALL_OFF: g_diag.display_mode = DIAG_DISPLAY_MENU ; break;case DIAG_DISPLAY_MENU: ReturnToRoot();break;}
	}

}


static void HandleEnterButton(const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    switch(g_diag.menu)
    {
    	case DIAG_MENU_ROOT:
			switch(g_diag.cursor)
			{
			case 0: g_diag.menu = DIAG_MENU_SENSORS; break;
			case 1: g_diag.menu = DIAG_MENU_COMM; break;
			case 2: g_diag.menu = DIAG_MENU_MOTOR_LIST; break;
			case 3: g_diag.menu = DIAG_MENU_LED_LIST; break;
			case 4: g_diag.menu = DIAG_MENU_DISPLAY; break;
			}
		break;
    	case DIAG_MENU_SENSORS:
			switch(g_diag.cursor)
			{
			case 0: g_diag.menu = DIAG_MENU_SENSOR_LINE; break;
			case 1: g_diag.menu = DIAG_MENU_SENSOR_PROX; break;
			}
		break;
    	case DIAG_MENU_MOTOR_LIST:
			switch(g_diag.cursor)
			{
			case 0: g_diag.menu = DIAG_MENU_MOTOR_AVG; break;
			case 1: g_diag.menu = DIAG_MENU_MOTOR_AVD; break;
			case 2: g_diag.menu = DIAG_MENU_MOTOR_ARG; break;
			case 3: g_diag.menu = DIAG_MENU_MOTOR_ARD; break;
			}
		break;
    	case DIAG_MENU_LED_LIST:
			switch(g_diag.cursor)
			{
			case 0: g_diag.menu = DIAG_MENU_LED_GREEN; break;
			case 1: g_diag.menu = DIAG_MENU_LED_ORANGE; break;
			case 2: g_diag.menu = DIAG_MENU_LED_BLUE; break;
			case 3: g_diag.menu = DIAG_MENU_LED_RED; break;
			}
		break;
		case DIAG_MENU_LED_GREEN:
			g_diag.led_green_enabled = !g_diag.led_green_enabled;
			break;
		case DIAG_MENU_LED_ORANGE:
			g_diag.led_orange_enabled = !g_diag.led_orange_enabled;
			break;
		case DIAG_MENU_LED_BLUE:
			g_diag.led_blue_enabled = !g_diag.led_blue_enabled;
			break;
		case DIAG_MENU_LED_RED:
			g_diag.led_red_enabled = !g_diag.led_red_enabled;
			break;
    	case DIAG_MENU_DISPLAY:
			switch(g_diag.display_mode)
			{
			case 0: g_diag.display_mode = DIAG_DISPLAY_ALL_ON; break;
			case 1: g_diag.display_mode = DIAG_DISPLAY_ALL_ON; break;
			case 2: g_diag.display_mode = DIAG_DISPLAY_ALL_OFF; break;
			}
		break;
    }
}


static void HandleStopButton(void)
{

	g_diag.led_green_enabled = 0 ;
	g_diag.led_orange_enabled = 0 ;
	g_diag.led_blue_enabled = 0 ;
	g_diag.led_red_enabled = 0;
	g_diag.display_mode = DIAG_DISPLAY_MENU;
	ReturnToRoot();

}


/*============================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

void VehicleDiagnostic_Init(void)
{
    memset(&g_diag, 0, sizeof(g_diag));
    g_diag.menu = DIAG_MENU_ROOT;
    g_diag.cursor = 0;
    g_diag.scroll = 0;
    g_diag.comm_scroll = 0;
    g_diag.display_mode = DIAG_DISPLAY_MENU;
}

void VehicleDiagnostic_GetContext(diagnostic_ctx_t *ctx)
{
    if (ctx == NULL)
        return;

    *ctx = g_diag;
}


void VehicleDiagnostic_ProcessCommand(const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    if (RisingEdge(g_diag.prev_cmd.up, cmd->up))
    {
        HandleUpButton();
    }

    if (RisingEdge(g_diag.prev_cmd.down, cmd->down))
    {
        HandleDownButton();
    }

    if (RisingEdge(g_diag.prev_cmd.left, cmd->left))
    {
        HandleLeftButton();
    }

    /*
     * RIGHT et START ont le même rôle en diagnostic :
     * entrer dans un menu, sélectionner ou confirmer.
     */
    if (RisingEdge(g_diag.prev_cmd.right, cmd->right) ||
        RisingEdge(g_diag.prev_cmd.start, cmd->start))
    {
        HandleEnterButton(cmd);
    }

    if (RisingEdge(g_diag.prev_cmd.stop, cmd->stop))
    {
        HandleStopButton();
    }

    g_diag.prev_cmd = *cmd;
}

/*
 * Retourne le moteur actuellement sélectionné
 * dans le menu diagnostic moteur.
 */
bool VehicleDiagnostic_GetSelectedMotor(motor_target_t *motor)
{
    if (motor == NULL)
        return false;

    	switch(g_diag.menu)
    	{
    	case DIAG_MENU_MOTOR_AVG: *motor = MOTOR_TARGET_AVG; return true; break;
    	case DIAG_MENU_MOTOR_AVD: *motor = MOTOR_TARGET_AVD; return true; break;
    	case DIAG_MENU_MOTOR_ARG: *motor = MOTOR_TARGET_ARG; return true; break;
    	case DIAG_MENU_MOTOR_ARD: *motor = MOTOR_TARGET_ARD; return true; break;
    	case !(DIAG_MENU_MOTOR_AVG)&&!(DIAG_MENU_MOTOR_AVD)&&!(DIAG_MENU_MOTOR_ARG)&&!(DIAG_MENU_MOTOR_ARD): *motor = MOTOR_TARGET_NONE; return false; break;
    	}

}
