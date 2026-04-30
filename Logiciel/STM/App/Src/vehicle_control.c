/*
 * vehicle_control.c
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * vehicle_control.c - VERSION ÉTUDIANTE
 *
 * Ce module contient la logique principale du véhicule.
 *
 * À compléter principalement :
 * - mode manuel
 * - mode suiveur de ligne
 * - mode évitement d'obstacle
 *
 * Aucun accès matériel direct ne doit être fait ici.
 */

/*
 * Module principal de logique véhicule.
 *
 * Ce fichier gère :
 * - la machine à états du véhicule;
 * - le décodage des commandes reçues;
 * - la génération des commandes moteur;
 * - le suivi de ligne;
 * - l'évitement d'obstacles;
 * - le mode failsafe en cas de perte Bluetooth.
 *
 * Aucun accès matériel direct n'est fait ici.
 */


#include "vehicle_control.h"
#include <stdio.h>
#include <string.h>

/*============================================================================
 * PRIVATE TYPES
 *===========================================================================*/

typedef struct
{
    vehicle_state_t state;
    control_cmd_t last_cmd;

    line_state_t line_state;
    int line_error;
    int line_error_filt;
    proximity_sensor_data_t prox;

    bool line_follow_enabled;
    bool obstacle_avoid_enabled;

    bool line_seen_once;
    line_state_t last_seen_dir;
    uint16_t line_lost_ticks;

    int line_error_prev;
    int line_error_integral;

} vehicle_control_ctx_t;

/*============================================================================
 * PRIVATE VARIABLES
 *===========================================================================*/

static vehicle_control_ctx_t g_vc = {0};

/*============================================================================
 * PRIVATE DEFINES
 *===========================================================================*/

#define LF_LOST_TIMEOUT_TICKS   300		//300 x 10 ms = 3000 ms = 3 s

/* ===== LINE FOLLOW TUNING ===== */
#define LF_SPEED_CENTER            30
#define LF_SPEED_MIN               20

#define LF_KP                       4
#define LF_KD                       1
#define LF_KI                       1

#define LF_CORR_MAX                30
#define LF_SPEED_REDUCTION_STEP     1
#define LF_INTEGRAL_MAX            40

#define LF_SEARCH_LEFT_MOTOR      -30
#define LF_SEARCH_RIGHT_MOTOR      30


/* ===== OBSTACLE AVOID TUNING ===== */
#define OA_SIDE_PIVOT_MM           200   /* IR latéraux: si 100..200 mm, on pivote franchement */
#define OA_SIDE_WARN_MM            300   /* correction douce plus loin */

#define OA_CENTER_BACKUP_MM        220   /* si obstacle centre < 200 mm -> recule */
#define OA_CENTER_TURN_OK_MM       220   /* pour pouvoir réavancer après pivot */

#define OA_FORWARD_SPEED            18
#define OA_FORWARD_SLOW             12

#define OA_PIVOT_FAST               80   /* pivot sur place */
#define OA_TURN_SOFT                30   /* correction douce */
#define OA_TURN_BRAKE              -45

#define OA_REVERSE_SPEED           -32


/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/

static int16_t clamp100(int16_t x);
static void VehicleControl_ResetAutoState(void);
static void MotorCommand_Clear(motor_cmd_t *mcmd);

static void BuildIdleMotorCommand(motor_cmd_t *mcmd);
static void BuildManualMotorCommand(motor_cmd_t *mcmd);
static void BuildLineFollowMotorCommand(motor_cmd_t *mcmd);
static void BuildObstacleAvoidMotorCommand(motor_cmd_t *mcmd);
static void BuildDiagnosticMotorCommand(motor_cmd_t *mcmd);
static void BuildFailsafeMotorCommand(motor_cmd_t *mcmd);
static vehicle_state_t VehicleControl_ModeToState(int mode);
static void VehicleControl_UpdateAutoEnable(const control_cmd_t *cmd);
static uint8_t VehicleControl_ComputeChecksum(const char *frame);

/*============================================================================
 * GENERAL HELPERS
 *===========================================================================*/

static int16_t clamp100(int16_t x)
{
    if (x > 100) return 100;
    if (x < -100) return -100;
    return x;
}

static void VehicleControl_ResetAutoState(void)
{
    g_vc.line_follow_enabled = false;
    g_vc.obstacle_avoid_enabled = false;
    g_vc.line_seen_once = false;
    g_vc.last_seen_dir = LINE_STATE_UNKNOWN;
    g_vc.line_lost_ticks = 0;
    g_vc.line_error = 0;
    g_vc.line_error_filt = 0;
    g_vc.line_error_prev = 0;
    g_vc.line_error_integral = 0;
}

/*
 * Remet une commande moteur à l'état sécuritaire par défaut.
 * Par défaut, le véhicule est en roue libre et aucun moteur individuel n'est sélectionné.
 */
static void MotorCommand_Clear(motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    mcmd->left_cmd = 0;
    mcmd->right_cmd = 0;
    mcmd->coast = true;

    mcmd->single_motor_mode = false;
    mcmd->target_motor = MOTOR_TARGET_NONE;
    mcmd->single_motor_cmd = 0;
}


/*
 * Convertit le mode reçu dans la trame en état véhicule.
 */
static vehicle_state_t VehicleControl_ModeToState(int mode)
{
    switch (mode)
    {
        case 0: return VEHICLE_STATE_IDLE;
        case 1: return VEHICLE_STATE_MANUAL;
        case 2: return VEHICLE_STATE_LINE_FOLLOW;
        case 3: return VEHICLE_STATE_OBSTACLE_AVOID;
        case 4: return VEHICLE_STATE_DIAGNOSTIC;
        default: return VEHICLE_STATE_IDLE;
    }
}

/*
 * Gère l'armement START/STOP des modes automatiques.
 *
 * Le suivi de ligne et l'évitement d'obstacles doivent être armés avec START.
 * STOP désarme seulement le mode automatique actuellement sélectionné.
 */
static void VehicleControl_UpdateAutoEnable(const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    if (cmd->mode == 2)
    {
        g_vc.obstacle_avoid_enabled = false;

        if (cmd->stop)
        {
            g_vc.line_follow_enabled = false;
        }

        if (cmd->start)
        {
            VehicleControl_ResetAutoState();
            g_vc.line_follow_enabled = true;
        }
    }
    else if (cmd->mode == 3)
    {
        g_vc.line_follow_enabled = false;

        if (cmd->stop)
        {
            g_vc.obstacle_avoid_enabled = false;
        }

        if (cmd->start)
        {
            VehicleControl_ResetAutoState();
            g_vc.obstacle_avoid_enabled = true;
        }
    }
    else
    {
        VehicleControl_ResetAutoState();
    }
}


/*============================================================================
 * MOTOR COMMAND BUILDERS
 *===========================================================================*/

/*
 * IDLE : le véhicule est arrêté.
 */
static void BuildIdleMotorCommand(motor_cmd_t *mcmd)
{
    MotorCommand_Clear(mcmd);
}


/*
 * MANUAL : commande différentielle simple.
 * speed contrôle l'avance/recul, turn contrôle la rotation,
 * trim corrige l'écart entre le côté gauche et le côté droit.
 */
static void BuildManualMotorCommand(motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    /*
     * TODO 1 : Mode manuel
     *
     * Objectif :
     * - speed contrôle l'avance/recul
     * - turn contrôle la rotation
     * - trim corrige l'écart gauche/droite
     *
     * Indices :
     * left_cmd  = speed + turn corrigé
     * right_cmd = speed - turn corrigé
     *
     * Attention :
     * - utiliser clamp100()
     * - si speed = 0 et turn = 0, mettre coast = true
     * - si STOP est appuyé, arrêter le véhicule
     */

    MotorCommand_Clear(mcmd);
}

/*
 * LINE FOLLOW :
 * Utilise l'erreur du capteur de ligne pour corriger la vitesse
 * gauche/droite. Si la ligne est perdue, lance une recherche
 * temporaire selon le dernier côté observé.
 */
static void BuildLineFollowMotorCommand(motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    if (!g_vc.line_follow_enabled)
    {
        MotorCommand_Clear(mcmd);
        return;
    }

    /*
     * TODO 2 : Mémoriser la dernière position connue de la ligne
     *
     * Si line_state vaut :
     * - LINE_STATE_LEFT   : ligne vue à gauche
     * - LINE_STATE_RIGHT  : ligne vue à droite
     * - LINE_STATE_CENTER : ligne vue au centre
     *
     * Mettre à jour :
     * - line_seen_once
     * - last_seen_dir
     */

    /*
     * TODO 3 : Suiveur de ligne
     *
     * Cas à gérer :
     *
     * 1. Ligne centrée / gauche / droite :
     *    - utiliser g_vc.line_error_filt
     *    - calculer une correction proportionnelle
     *    - appliquer la correction gauche/droite
     *    - remettre line_lost_ticks à 0
     *
     * 2. Ligne perdue :
     *    - incrémenter line_lost_ticks
     *    - si aucune ligne n'a jamais été vue : arrêt
     *    - si timeout dépassé : arrêt
     *    - sinon tourner dans la direction de la dernière ligne vue
     */

    MotorCommand_Clear(mcmd);
}

/*
 * OBSTACLE AVOID :
 * Utilise les capteurs gauche / centre / droite pour ralentir,
 * pivoter, reculer ou avancer selon la distance détectée.
 */
static void BuildObstacleAvoidMotorCommand(motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    if (!g_vc.obstacle_avoid_enabled)
    {
        MotorCommand_Clear(mcmd);
        return;
    }

    if (g_vc.last_cmd.stop)
    {
        MotorCommand_Clear(mcmd);
        return;
    }

    /*
     * TODO 4 : Mode évitement d'obstacle
     *
     * Données disponibles :
     * - g_vc.prox.left_mm
     * - g_vc.prox.center_mm
     * - g_vc.prox.right_mm
     *
     * Validité :
     * - g_vc.prox.left_valid
     * - g_vc.prox.center_valid
     * - g_vc.prox.right_valid
     *
     * Comportement minimal attendu :
     *
     * 1. Si aucun capteur valide :
     *    arrêter.
     *
     * 2. Si obstacle proche au centre :
     *    reculer ou pivoter.
     *
     * 3. Si obstacle proche à gauche :
     *    tourner vers la droite.
     *
     * 4. Si obstacle proche à droite :
     *    tourner vers la gauche.
     *
     * 5. Si tout est libre :
     *    avancer.
     *
     * Utiliser les constantes OA_...
     */

    MotorCommand_Clear(mcmd);
}


/*
 * DIAGNOSTIC : par défaut, le contrôle véhicule n'active aucun moteur.
 * Le test moteur individuel est appliqué ensuite par vehicle_tasks.c.
 */
static void BuildDiagnosticMotorCommand(motor_cmd_t *mcmd)
{
    MotorCommand_Clear(mcmd);
}

/*
 * FAILSAFE : perte Bluetooth ou erreur critique.
 * Le véhicule doit rester arrêté.
 */
static void BuildFailsafeMotorCommand(motor_cmd_t *mcmd)
{
    MotorCommand_Clear(mcmd);
}


/*============================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

void VehicleControl_Init(void)
{
    memset(&g_vc, 0, sizeof(g_vc));
    g_vc.state = VEHICLE_STATE_IDLE;
    g_vc.line_state = LINE_STATE_UNKNOWN;
    VehicleControl_ResetAutoState();
}


/*
 * TODO BONUS - Checksum
 *
 * Calcule le checksum d'une trame Bluetooth.
 *
 * Règle :
 * - additionner les caractères ASCII situés après '<'
 * - arrêter avant le champ ";chk="
 * - retourner la somme modulo 256
 *
 * Exemple :
 * <mode=1;speed=50;turn=-20;trim=0;start=0;stop=0;up=0;down=1;left=0;right=0;chk=123>
 *
 * On additionne seulement :
 * mode=1;speed=50;turn=-20;trim=0;start=0;stop=0;up=0;down=1;left=0;right=0
 */
static uint8_t VehicleControl_ComputeChecksum(const char *frame)
{
    uint16_t sum = 0;

    if (frame == NULL)
    {
        return 0;
    }

    /*
     * TODO 5 :
     * Vérifier que la trame commence par '<'.
     * Si ce n'est pas le cas, retourner 0.
     */

    /*
     * TODO 6 :
     * Avancer le pointeur pour ignorer le caractère '<'.
     * frame++;
     */

    /*
     * TODO 7 :
     * Parcourir les caractères un par un.
     *
     * À chaque caractère :
     * - vérifier si on est rendu au début de ";chk="
     * - si oui, arrêter la boucle
     * - sinon, ajouter le caractère à sum
     */

    /*
     * TODO 8 :
     * Retourner les 8 bits de poids faible de sum.
     * return (uint8_t)(sum & 0xFF);
     */

    return 0;
}




bool VehicleControl_ParseFrame(const char *frame, control_cmd_t *cmd)
{
    int n;

    if ((frame == NULL) || (cmd == NULL))
        return false;

    n = sscanf(frame,
               "<mode=%d;speed=%d;turn=%d;trim=%d;start=%d;stop=%d;up=%d;down=%d;left=%d;right=%d;chk=%d>",
               &cmd->mode,
               &cmd->speed,
               &cmd->turn,
               &cmd->trim,
               &cmd->start,
               &cmd->stop,
               &cmd->up,
               &cmd->down,
               &cmd->left,
               &cmd->right,
               &cmd->chk);

    if (n != 11)
        return false;

    /*
     * TODO BONUS - Validation du checksum
     *
     * Étapes :
     * 1. Calculer le checksum avec VehicleControl_ComputeChecksum(frame)
     * 2. Comparer avec cmd->chk
     * 3. Si différent, retourner false
     *
     * Exemple :
     *
     * uint8_t computed_chk;
     *
     * computed_chk = VehicleControl_ComputeChecksum(frame);
     *
     * if (computed_chk != (uint8_t)cmd->chk)
     * {
     *     return false;
     * }
     */

    /* Limites mode */
    if (cmd->mode < 0) cmd->mode = 0;
    if (cmd->mode > 4) cmd->mode = 4;

    /* Limites speed */
    if (cmd->speed > 100) cmd->speed = 100;
    if (cmd->speed < -100) cmd->speed = -100;

    /* Limites turn */
    if (cmd->turn > 100) cmd->turn = 100;
    if (cmd->turn < -100) cmd->turn = -100;

    /* Limites trim */
    if (cmd->trim > 100) cmd->trim = 100;
    if (cmd->trim < -100) cmd->trim = -100;

    /* Normalisation boutons */
    cmd->start = (cmd->start != 0) ? 1 : 0;
    cmd->stop  = (cmd->stop  != 0) ? 1 : 0;
    cmd->up    = (cmd->up    != 0) ? 1 : 0;
    cmd->down  = (cmd->down  != 0) ? 1 : 0;
    cmd->left  = (cmd->left  != 0) ? 1 : 0;
    cmd->right = (cmd->right != 0) ? 1 : 0;

    return true;
}

void VehicleControl_ProcessCommand(const control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    g_vc.last_cmd = *cmd;

    VehicleControl_UpdateAutoEnable(cmd);

    g_vc.state = VehicleControl_ModeToState(cmd->mode);
}


/*
 * Perte Bluetooth :
 * le véhicule passe immédiatement en FAILSAFE.
 */
void VehicleControl_OnTimeout(void)
{
	g_vc.state = VEHICLE_STATE_FAILSAFE;
	VehicleControl_ResetAutoState();
}


void VehicleControl_GetMotorCommand(motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    MotorCommand_Clear(mcmd);

    switch (g_vc.state)
    {
        case VEHICLE_STATE_IDLE:
            BuildIdleMotorCommand(mcmd);
            break;

        case VEHICLE_STATE_MANUAL:
            BuildManualMotorCommand(mcmd);
            break;

        case VEHICLE_STATE_LINE_FOLLOW:
            BuildLineFollowMotorCommand(mcmd);
            break;

        case VEHICLE_STATE_OBSTACLE_AVOID:
            BuildObstacleAvoidMotorCommand(mcmd);
            break;

        case VEHICLE_STATE_DIAGNOSTIC:
            BuildDiagnosticMotorCommand(mcmd);
            break;

        case VEHICLE_STATE_FAILSAFE:
            BuildFailsafeMotorCommand(mcmd);
            break;

        default:
            MotorCommand_Clear(mcmd);
            break;

    }
}

vehicle_state_t VehicleControl_GetState(void)
{
    return g_vc.state;
}

void VehicleControl_GetLastCommand(control_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    *cmd = g_vc.last_cmd;
}

void VehicleControl_SetProximityData(const proximity_sensor_data_t *prox)
{
    if (prox == NULL)
        return;

    g_vc.prox = *prox;
}


void VehicleControl_SetLineState(line_state_t line_state)
{
    g_vc.line_state = line_state;
}

/*
 * Met à jour l'erreur de ligne et applique
 * un filtre numérique simple.
 */
void VehicleControl_SetLineError(int line_error)
{
    g_vc.line_error = line_error;

    /* filtre simple: 50% ancienne valeur, 50% nouvelle */
    g_vc.line_error_filt = (g_vc.line_error_filt + line_error) / 2;
}


bool VehicleControl_IsLineFollowEnabled(void)
{
    return g_vc.line_follow_enabled;
}

bool VehicleControl_IsObstacleAvoidEnabled(void)
{
    return g_vc.obstacle_avoid_enabled;
}
