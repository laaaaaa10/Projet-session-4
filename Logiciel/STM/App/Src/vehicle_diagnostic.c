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
    /*
     * TODO 1 :
     * Gérer le bouton LEFT.
     *
     * Objectif :
     * - revenir au menu précédent;
     * - remettre cursor et scroll à 0;
     * - si on quitte le menu afficheur, remettre display_mode à DIAG_DISPLAY_MENU.
     *
     * Menus suggérés :
     *
     * DIAG_MENU_SENSORS
     * DIAG_MENU_COMM
     * DIAG_MENU_MOTOR_LIST
     * DIAG_MENU_LED_LIST
     * DIAG_MENU_DISPLAY
     *      -> retour au menu racine
     *
     * DIAG_MENU_SENSOR_LINE
     * DIAG_MENU_SENSOR_PROX
     *      -> retour au menu capteurs
     *
     * DIAG_MENU_MOTOR_AVG
     * DIAG_MENU_MOTOR_AVD
     * DIAG_MENU_MOTOR_ARG
     * DIAG_MENU_MOTOR_ARD
     *      -> retour à la liste des moteurs
     *
     * DIAG_MENU_LED_GREEN
     * DIAG_MENU_LED_ORANGE
     * DIAG_MENU_LED_BLUE
     * DIAG_MENU_LED_RED
     *      -> retour à la liste des DEL
     */
}


static void HandleEnterButton(const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    /*
     * TODO 2 :
     * Gérer le bouton RIGHT ou START.
     *
     * Objectif :
     * Selon le menu actuel et le curseur :
     *
     * Menu racine :
     *   curseur 0 -> DIAG_MENU_SENSORS
     *   curseur 1 -> DIAG_MENU_COMM
     *   curseur 2 -> DIAG_MENU_MOTOR_LIST
     *   curseur 3 -> DIAG_MENU_LED_LIST
     *   curseur 4 -> DIAG_MENU_DISPLAY
     *
     * Menu capteurs :
     *   curseur 0 -> DIAG_MENU_SENSOR_LINE
     *   curseur 1 -> DIAG_MENU_SENSOR_PROX
     *
     * Menu moteurs :
     *   curseur 0 -> DIAG_MENU_MOTOR_AVG
     *   curseur 1 -> DIAG_MENU_MOTOR_AVD
     *   curseur 2 -> DIAG_MENU_MOTOR_ARG
     *   curseur 3 -> DIAG_MENU_MOTOR_ARD
     *
     * Menu DEL :
     *   curseur 0 -> DIAG_MENU_LED_GREEN
     *   curseur 1 -> DIAG_MENU_LED_ORANGE
     *   curseur 2 -> DIAG_MENU_LED_BLUE
     *   curseur 3 -> DIAG_MENU_LED_RED
     *
     * Menu détail DEL :
     *   activer la DEL sélectionnée.
     *
     * Menu afficheur :
     *   alterner entre DIAG_DISPLAY_ALL_ON et DIAG_DISPLAY_ALL_OFF.
     */
}


static void HandleStopButton(void)
{
    /*
     * TODO 3 :
     * Gérer le bouton STOP en diagnostic.
     *
     * Objectif minimal :
     * - éteindre les DEL de diagnostic;
     * - remettre display_mode à DIAG_DISPLAY_MENU.
     *
     * Option :
     * - revenir au menu racine.
     */
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

    /*
     * TODO 4 :
     * Retourner le moteur sélectionné selon le menu courant.
     *
     * Menus :
     * - DIAG_MENU_MOTOR_AVG -> MOTOR_TARGET_AVG
     * - DIAG_MENU_MOTOR_AVD -> MOTOR_TARGET_AVD
     * - DIAG_MENU_MOTOR_ARG -> MOTOR_TARGET_ARG
     * - DIAG_MENU_MOTOR_ARD -> MOTOR_TARGET_ARD
     *
     * Si aucun moteur n'est sélectionné :
     * - mettre MOTOR_TARGET_NONE;
     * - retourner false.
     */

    *motor = MOTOR_TARGET_NONE;
    return false;
}
