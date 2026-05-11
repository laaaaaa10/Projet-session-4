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

    int16_t last_valid_left_cmd;
    int16_t last_valid_right_cmd;

    int line_error_prev;
    int line_error_integral;


    uint8_t  oa_attempt_count;   /* nb de cycles avance→recul */
    uint16_t oa_window_ticks;    /* temps écoulé depuis le 1er essai */
    bool oa_was_advancing;   /* était en train d'avancer au tick précédent */

} vehicle_control_ctx_t;

/*============================================================================
 * PRIVATE VARIABLES
 *===========================================================================*/

static vehicle_control_ctx_t g_vc = {0};

/*============================================================================
 * PRIVATE DEFINES
 *===========================================================================*/

#define LF_LOST_TIMEOUT_TICKS   300
#define LF_FINISHED_TIMEOUT_TICKS 10

/* ===== LINE FOLLOW TUNING ===== */
#define LF_SPEED_CENTER            65  //50
#define LF_SPEED_MIN               10    //10

#define LF_KP                      7   //4
#define LF_KI                      6   //1
#define LF_KD                      4   //2

#define LF_CORR_MAX                150
#define LF_SPEED_REDUCTION_STEP     1
#define LF_INTEGRAL_MAX             2   //30

#define LF_SEARCH_LEFT_MOTOR      -95   //70
#define LF_SEARCH_RIGHT_MOTOR      95

#define LF_REPLAY_TICKS             1  // temp entre lighe perdue et essye de re trouver la ligne 


/* ===== OBSTACLE AVOID TUNING ===== */
#define OA_SIDE_PIVOT_MM           200   /* IR latéraux: si 100..200 mm, on pivote franchement */
#define OA_SIDE_WARN_MM            300   /* correction douce plus loin */

#define OA_CENTER_BACKUP_MM        220   /* si obstacle centre < 200 mm -> recule */
#define OA_CENTER_TURN_OK_MM       220   /* pour pouvoir réavancer après pivot */

#define OA_FORWARD_SPEED            50
#define OA_FORWARD_SLOW             20

#define OA_PIVOT_FAST               90   /* pivot sur place */
#define OA_TURN_SOFT                30   /* correction douce */
#define OA_TURN_BRAKE              -45

#define OA_REVERSE_SPEED           -32

#define OA_ATTEMPT_MAX          3     /* essais avant de tourner */
#define OA_ATTEMPT_WINDOW_TICKS 300   /* fenêtre 3 sec (300 x 10 ms) */


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


    g_vc.oa_attempt_count  = 0;
    g_vc.oa_window_ticks   = 0;
    g_vc.oa_was_advancing  = false;
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
    if (mcmd == NULL) {
        return;
    }
    
    int16_t speed = clamp100(g_vc.last_cmd.speed);
    int16_t turn  = clamp100(g_vc.last_cmd.turn);

    mcmd->left_cmd  = clamp100((int16_t)(speed + turn));
    mcmd->right_cmd = clamp100((int16_t)(speed - turn));

    mcmd->coast = (speed == 0 && turn == 0);  

    //MotorCommand_Clear(mcmd);
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
    if (g_vc.line_state == LINE_STATE_LEFT || g_vc.line_state == LINE_STATE_RIGHT || g_vc.line_state == LINE_STATE_CENTER)
    {
        g_vc.line_seen_once = true;
        g_vc.last_seen_dir = g_vc.line_state;
        g_vc.line_lost_ticks = 0;
    

    /*
     * TODO 3 : Suiveur de ligne8
     */

        g_vc.line_error_integral += g_vc.line_error;
        if (g_vc.line_error_integral > LF_INTEGRAL_MAX)
            g_vc.line_error_integral = LF_INTEGRAL_MAX;
        if (g_vc.line_error_integral < -LF_INTEGRAL_MAX)
            g_vc.line_error_integral = -LF_INTEGRAL_MAX;

        // correction PID
        g_vc.line_error_filt = LF_KP * g_vc.line_error
                             + LF_KD * (g_vc.line_error - g_vc.line_error_prev)
                             + LF_KI * g_vc.line_error_integral; 

        if (g_vc.line_error_filt > LF_CORR_MAX)
            g_vc.line_error_filt = LF_CORR_MAX;
        if (g_vc.line_error_filt < -LF_CORR_MAX)
            g_vc.line_error_filt = -LF_CORR_MAX;

        g_vc.line_error_prev = g_vc.line_error;

        mcmd->left_cmd = clamp100(LF_SPEED_CENTER - g_vc.line_error_filt);
        mcmd->right_cmd = clamp100(LF_SPEED_CENTER + g_vc.line_error_filt);
        mcmd->coast = false;

        //sauvegarder la dernière commande valide 
        g_vc.last_valid_left_cmd  = mcmd->left_cmd;
        g_vc.last_valid_right_cmd = mcmd->right_cmd;
    }

    // Si la ligne est perdue
    else
    {
        g_vc.line_lost_ticks++;

        if (!g_vc.line_seen_once || g_vc.line_lost_ticks > LF_LOST_TIMEOUT_TICKS)
        {
            MotorCommand_Clear(mcmd);
            return;
        }

        if (g_vc.last_seen_dir == LINE_STATE_CENTER ||
            (g_vc.last_seen_dir == LINE_STATE_LEFT  && g_vc.line_error_prev <= 1) ||
            (g_vc.last_seen_dir == LINE_STATE_RIGHT && g_vc.line_error_prev >= -1))
        {
            // si sa fais plus de X sec
            if (g_vc.line_lost_ticks >  LF_FINISHED_TIMEOUT_TICKS)
            {
                MotorCommand_Clear(mcmd);
                return;
            }
            mcmd->left_cmd  = LF_SPEED_CENTER;
            mcmd->right_cmd = LF_SPEED_CENTER;
        }
        else if (g_vc.last_seen_dir == LINE_STATE_RIGHT)
        {
            mcmd->left_cmd  = LF_SEARCH_RIGHT_MOTOR;
            mcmd->right_cmd = LF_SEARCH_LEFT_MOTOR;
        }
        else
        {
            mcmd->left_cmd  = LF_SEARCH_LEFT_MOTOR;
            mcmd->right_cmd = LF_SEARCH_RIGHT_MOTOR;
        }
        mcmd->coast = false;
    }
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
    */
    if (!g_vc.prox.left_valid && !g_vc.prox.center_valid && !g_vc.prox.right_valid)
    {
        MotorCommand_Clear(mcmd);  /* aucun capteur valide : arrêt */
        return;
    } 
    
    // front obstocal check
    if (g_vc.oa_attempt_count > 0)
    {
        g_vc.oa_window_ticks++;
        if (g_vc.oa_window_ticks > OA_ATTEMPT_WINDOW_TICKS)
        {
            g_vc.oa_attempt_count  = 0;
            g_vc.oa_window_ticks   = 0;
            g_vc.oa_was_advancing  = false;
        }
    }

    /* --- Obstacle au centre --- */
    if (g_vc.prox.center_valid && g_vc.prox.center_mm < OA_CENTER_BACKUP_MM)
    {
        /* Transition avance→recul = un essai */
        if (g_vc.oa_was_advancing)
        {
            if (g_vc.oa_attempt_count == 0)
                g_vc.oa_window_ticks = 0;   /* démarre la fenêtre */

            g_vc.oa_attempt_count++;
            g_vc.oa_was_advancing = false;
        }

        if (g_vc.oa_attempt_count >= OA_ATTEMPT_MAX)
        {
            /* 3 essais échoués : tourner à gauche */
            mcmd->left_cmd  = OA_TURN_BRAKE;
            mcmd->right_cmd = OA_TURN_SOFT;
        }
        else
        {
            mcmd->left_cmd  = OA_REVERSE_SPEED;
            mcmd->right_cmd = OA_REVERSE_SPEED;
        }
    }
    // tourne à droite ou gauche
    else if (g_vc.prox.right_valid && g_vc.prox.right_mm < OA_SIDE_WARN_MM)
    {
        mcmd->left_cmd = OA_TURN_SOFT;
        mcmd->right_cmd = OA_TURN_BRAKE;
    }
    else if (g_vc.prox.left_valid  && g_vc.prox.left_mm  < OA_SIDE_WARN_MM)
    {
        mcmd->left_cmd  = OA_TURN_BRAKE;
        mcmd->right_cmd = OA_TURN_SOFT;
    }

    // avance
    else
    {
        g_vc.oa_was_advancing = true; 
        mcmd->left_cmd = OA_FORWARD_SPEED;
        mcmd->right_cmd = OA_FORWARD_SPEED;
    }
    
    mcmd->coast = false;

    //MotorCommand_Clear(mcmd);
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
