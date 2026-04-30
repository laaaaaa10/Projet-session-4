/*
 * vehicle_tasks.c
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */
/*
 * vehicle_tasks.c - VERSION ÉTUDIANTE
 *
 * À compléter :
 * - Task_LineSensor()
 * - Task_ProximitySensors()
 * - petite partie du recalcul automatique dans Task_MainController()
 *
 */

/*
 * Module d'orchestration FreeRTOS du véhicule.
 *
 * Ce fichier crée les tâches, reçoit les trames Bluetooth, lit les capteurs,
 * met à jour la machine à états et transmet les commandes aux moteurs.
 *
 * Il ne construit pas directement les lignes LCD : l'affichage est délégué
 * à vehicle_display.c et vehicle_display_diag.c.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include "line_sensor.h"
#include "main.h"

#include "vehicle_tasks.h"
#include "app_types.h"
#include "vehicle_control.h"
#include "vehicle_motors.h"
#include "uart_frame_rx.h"
#include "vehicle_status_led.h"
#include "st7920.h"
#include "sharp_2y0a21.h"
#include "rcwl1601.h"
#include "vehicle_diagnostic.h"
#include "vehicle_display.h"
#include "vehicle_display_diag.h"
#include "vehicle_display_data.h"
#include "semphr.h"



#define LINE_SENSOR_ADDR_7BIT   0x22   /* À ajuster selon tes straps A0/A1/A2 */
#define LINE_ACTIVE_LOW         0      /* Mets 0 si tes capteurs sont actifs à 1 */

#define CTRL_QUEUE_WAIT_MS 		 20
#define BT_RX_PERIOD_MS          5
#define AUTO_CTRL_PERIOD_MS      10
#define LINE_SENSOR_PERIOD_MS    10
#define PROX_SENSOR_PERIOD_MS    20
#define RCWL_WAIT_MS             20
#define DISPLAY_PERIOD_MS        250
#define BT_TIMEOUT_MS            300

/*============================================================================
 * PRIVATE TYPES
 *===========================================================================*/

static int g_line_error = 0;
static uint8_t g_line_detected = 0;

/*============================================================================
 * PRIVATE VARIABLES
 *===========================================================================*/

static QueueHandle_t qCtrlEvents 			= NULL;
static QueueHandle_t qMotorCmd   			= NULL;

static TaskHandle_t hTaskDisplay          	= NULL;
static TaskHandle_t hTaskLineSensor       	= NULL;
static TaskHandle_t hTaskProximitySensors 	= NULL;
static TaskHandle_t hTaskBluetoothRx    	= NULL;
static TaskHandle_t hTaskMainController 	= NULL;
static TaskHandle_t hTaskMotorControl   	= NULL;
static TaskHandle_t hTaskStatusLED      	= NULL;

static SemaphoreHandle_t g_i2c3_mutex = NULL;


/* Handle capteur ultrason (utilisé aussi dans callback EXTI du main.c) */
RCWL1601_Handle_t hrcwl;



/*============================================================================
 * EXTERNAL HANDLES
 *===========================================================================*/

extern I2C_HandleTypeDef hi2c3;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim5;


/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/
static void Task_BluetoothRx				(void *argument);
static void Task_MainController				(void *argument);
static void Task_MotorControl				(void *argument);
static void Task_Display					(void *argument);
static void Task_LineSensor					(void *argument);
static void Task_ProximitySensors			(void *argument);

static void PublishMotorCommand				(const motor_cmd_t *mcmd);
static void ApplyDiagnosticMotorCommand		(motor_cmd_t *mcmd);
static void UpdateDiagnosticLeds			(void);
static void HandleBluetoothFrameEvent		(const control_cmd_t *cmd);
static void HandleBluetoothTimeout			(TickType_t *last_rx_tick);
static bool IsAutoControlState				(void);


/*============================================================================
 * GENERAL HELPERS
 *===========================================================================*/

/*
 * Convertit les 7 bits du capteur de ligne en état logique.
 * Met aussi à jour g_line_error pour l'affichage et le contrôle.
 */
static line_state_t DecodeLineState(uint8_t raw)
{
    uint8_t s;

#if LINE_ACTIVE_LOW
    s = (uint8_t)(~raw) & 0x7F;
#else
    s = raw & 0x7F;
#endif

    g_line_detected = s;

    switch (s)	// On choisit une forme non-linéaire d'erreur
    {
        case 0b0000000:  return LINE_STATE_LOST;

        case 0b1000000:  g_line_error = -12; return LINE_STATE_RIGHT;
        case 0b1100000:  g_line_error = -8; return LINE_STATE_RIGHT;
        case 0b0100000:  g_line_error = -6;  return LINE_STATE_RIGHT;
        case 0b0110000:  g_line_error = -4;  return LINE_STATE_RIGHT;
        case 0b0010000:  g_line_error = -2;  return LINE_STATE_RIGHT;
        case 0b0011000:  g_line_error = -1;  return LINE_STATE_RIGHT;

        case 0b0001000:  g_line_error = 0;   return LINE_STATE_CENTER;

        case 0b0001100:  g_line_error = 1;   return LINE_STATE_LEFT;
        case 0b0000100:  g_line_error = 2;   return LINE_STATE_LEFT;
        case 0b0000110:  g_line_error = 4;   return LINE_STATE_LEFT;
        case 0b0000010:  g_line_error = 6;   return LINE_STATE_LEFT;
        case 0b0000011:  g_line_error = 8;  return LINE_STATE_LEFT;
        case 0b0000001:  g_line_error = 12;  return LINE_STATE_LEFT;

        default:
        	g_line_error = 0;
            return LINE_STATE_UNKNOWN;
    }
}



static uint32_t SharpRawToMilliVolts(uint16_t raw)
{
    return ((uint32_t)raw * 3300U) / 4095U;
}


/*
 * Lit les deux capteurs Sharp configurés en mode ADC scan.
 * Retourne false si une des conversions échoue.
 *
 * ADC1 configuré en mode scan séquentiel :
 * Rank 1 = Sharp gauche
 * Rank 2 = Sharp droite
 *
 * L'ordre des lectures dépend directement de CubeMX.
 */
static bool ReadBothSharpRaw(uint16_t *raw_left, uint16_t *raw_right)
{
    if ((raw_left == NULL) || (raw_right == NULL))
    {
        return false;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return false;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return false;
    }
    *raw_left = (uint16_t)HAL_ADC_GetValue(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return false;
    }
    *raw_right = (uint16_t)HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);
    return true;
}


/*============================================================================
 * MOTOR COMMAND AND CONTROL HELPERS
 *===========================================================================*/

/*
 * Envoie la commande moteur la plus récente à la tâche moteur.
 * La queue est remise à zéro pour ne conserver que la commande la plus récente.
 */
static void PublishMotorCommand(const motor_cmd_t *mcmd)
{
    if (mcmd == NULL)
        return;

    xQueueReset(qMotorCmd);
    xQueueSend(qMotorCmd, mcmd, 0);
}


/*
 * En mode diagnostic moteur, remplace la commande gauche/droite normale
 * par une commande sur un seul moteur sélectionné.
 */
static void ApplyDiagnosticMotorCommand(motor_cmd_t *mcmd)
{
    motor_target_t selected_motor;
    control_cmd_t last_cmd;
    int16_t diag_speed;

    if (mcmd == NULL)
        return;

    if (VehicleControl_GetState() != VEHICLE_STATE_DIAGNOSTIC)
        return;

    if (!VehicleDiagnostic_GetSelectedMotor(&selected_motor))
        return;

    VehicleControl_GetLastCommand(&last_cmd);

    diag_speed = (int16_t)(last_cmd.speed + last_cmd.turn + last_cmd.trim);

    if (diag_speed > 100)  diag_speed = 100;
    if (diag_speed < -100) diag_speed = -100;

    mcmd->single_motor_mode = true;
    mcmd->target_motor = selected_motor;
    mcmd->single_motor_cmd = diag_speed;
    mcmd->coast = false;
}


/*
 * Applique immédiatement l'état des DEL choisi dans le menu diagnostic.
 */
static void UpdateDiagnosticLeds(void)
{
    diagnostic_ctx_t diag;

    if (VehicleControl_GetState() != VEHICLE_STATE_DIAGNOSTIC)
        return;

    VehicleDiagnostic_GetContext(&diag);

    VehicleStatusLed_SetGreen(diag.led_green_enabled);
    VehicleStatusLed_SetBlue(diag.led_blue_enabled);
    VehicleStatusLed_SetRed(diag.led_red_enabled);
}


/*
 * Indique si le véhicule est dans un mode qui nécessite un recalcul moteur périodique.
 */
static bool IsAutoControlState(void)
{
    vehicle_state_t state = VehicleControl_GetState();

    return ((state == VEHICLE_STATE_LINE_FOLLOW) ||
            (state == VEHICLE_STATE_OBSTACLE_AVOID));
}


/*
 * Traite une trame Bluetooth valide :
 * - met à jour la machine à états;
 * - traite le menu diagnostic si nécessaire;
 * - produit une nouvelle commande moteur.
 */
static void HandleBluetoothFrameEvent(const control_cmd_t *cmd)
{
    motor_cmd_t mcmd;

    if (cmd == NULL)
        return;

    VehicleControl_ProcessCommand(cmd);

    if (VehicleControl_GetState() == VEHICLE_STATE_DIAGNOSTIC)
    {
        VehicleDiagnostic_ProcessCommand(cmd);
    }

    UpdateDiagnosticLeds();

    VehicleStatusLed_SetVehicleState(VehicleControl_GetState());

    VehicleControl_GetMotorCommand(&mcmd);
    ApplyDiagnosticMotorCommand(&mcmd);
    PublishMotorCommand(&mcmd);
}


/*
 * Sécurité Bluetooth.
 * Si aucune trame n'est reçue depuis trop longtemps, le véhicule passe en FAILSAFE.
 */
static void HandleBluetoothTimeout(TickType_t *last_rx_tick)
{
    motor_cmd_t mcmd;

    if (last_rx_tick == NULL)
        return;

    VehicleControl_OnTimeout();

    VehicleStatusLed_SetBtState(BT_STATE_DISCONNECTED);
    VehicleDisplayData_SetBluetoothConnected(false);
    VehicleStatusLed_SetVehicleState(VehicleControl_GetState());

    VehicleControl_GetMotorCommand(&mcmd);
    PublishMotorCommand(&mcmd);

    *last_rx_tick = xTaskGetTickCount();
}




/*============================================================================
 * TASKS
 *===========================================================================*/


static void Task_BluetoothRx(void *argument)
{
    char frame_local[128];
    control_cmd_t cmd;
    ctrl_event_t evt;

    (void)argument;

    for (;;)
    {
        if (UartFrameRx_Fetch(frame_local, sizeof(frame_local)))
        {
            if (VehicleControl_ParseFrame(frame_local, &cmd))
            {
            	VehicleDisplayData_SetLastRxFrame(frame_local);

            	VehicleStatusLed_SetBtState(BT_STATE_CONNECTED);
            	VehicleDisplayData_SetBluetoothConnected(true);

                evt.type = CTRL_EVT_FRAME_RX;
                evt.cmd  = cmd;
                xQueueSend(qCtrlEvents, &evt, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BT_RX_PERIOD_MS));
    }
}

static void Task_MainController(void *argument)
{
    ctrl_event_t evt;
    motor_cmd_t mcmd;

    TickType_t last_rx_tick;
    TickType_t last_auto_ctrl_tick;

    (void)argument;

    VehicleControl_Init();
    VehicleDiagnostic_Init();

    last_rx_tick = xTaskGetTickCount();
    last_auto_ctrl_tick = xTaskGetTickCount();

    for (;;)
    {
        if (xQueueReceive(qCtrlEvents, &evt, pdMS_TO_TICKS(CTRL_QUEUE_WAIT_MS)) == pdTRUE)
        {
            if (evt.type == CTRL_EVT_FRAME_RX)
            {
                last_rx_tick = xTaskGetTickCount();
                HandleBluetoothFrameEvent(&evt.cmd);
            }
        }

        /*
         * TODO 1 :
         * En mode automatique, il faut recalculer périodiquement la commande moteur.
         *
         * À faire :
         * - vérifier si l'état courant est un état automatique avec IsAutoControlState()
         * - vérifier si AUTO_CTRL_PERIOD_MS est écoulé
         * - appeler VehicleControl_GetMotorCommand(&mcmd)
         * - publier la commande avec PublishMotorCommand(&mcmd)
         * - mettre à jour last_auto_ctrl_tick
         */

        /*
        if (...)
        {
            ...
        }
        */

        if ((xTaskGetTickCount() - last_rx_tick) > pdMS_TO_TICKS(BT_TIMEOUT_MS))
        {
            HandleBluetoothTimeout(&last_rx_tick);
        }
    }
}



static void Task_MotorControl(void *argument)
{
    motor_cmd_t mcmd;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(qMotorCmd, &mcmd, portMAX_DELAY) == pdTRUE)
        {
        	if (mcmd.coast)
        	{
        	    VehicleMotors_AllCoast();
        	}
        	else if (mcmd.single_motor_mode)
        	{
        	    VehicleMotors_SetOne(mcmd.target_motor, mcmd.single_motor_cmd);
        	}
        	else
        	{
        	    VehicleMotors_SetLeftRight(mcmd.left_cmd, mcmd.right_cmd);
        	}
        }
    }
}



static void Task_Display(void *argument)
{
    TickType_t lastWakeTime;
    diagnostic_ctx_t diag;

    char line0[17];
    char line1[17];
    char line2[17];
    char line3[17];

    static char prev_line0[17] = "";
    static char prev_line1[17] = "";
    static char prev_line2[17] = "";
    static char prev_line3[17] = "";

    static uint8_t fb_on[ST7920_FB_SIZE];
    static uint8_t fb_off[ST7920_FB_SIZE];
    static bool fb_init_done = false;
    static bool graphic_test_was_on = false;

    (void)argument;

    ST7920_Init();
    ST7920_GraphicMode(0);
    ST7920_ClearText();

    memset(prev_line0, 0, sizeof(prev_line0));
    memset(prev_line1, 0, sizeof(prev_line1));
    memset(prev_line2, 0, sizeof(prev_line2));
    memset(prev_line3, 0, sizeof(prev_line3));

    if (!fb_init_done)
    {
        memset(fb_on,  0xFF, sizeof(fb_on));
        memset(fb_off, 0x00, sizeof(fb_off));
        fb_init_done = true;
    }

    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (VehicleControl_GetState() == VEHICLE_STATE_DIAGNOSTIC)
        {
            VehicleDiagnostic_GetContext(&diag);

            if (diag.display_mode == DIAG_DISPLAY_ALL_ON)
            {
                if (!graphic_test_was_on)
                {
                    ST7920_GraphicMode(0);
                    ST7920_ClearText();
                    graphic_test_was_on = true;
                }

                ST7920_Update(fb_on);
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
                continue;
            }
            else if (diag.display_mode == DIAG_DISPLAY_ALL_OFF)
            {
                if (!graphic_test_was_on)
                {
                    ST7920_GraphicMode(0);
                    ST7920_ClearText();
                    graphic_test_was_on = true;
                }

                ST7920_Update(fb_off);
                vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
                continue;
            }
        }

        if (graphic_test_was_on)
        {
            ST7920_GraphicMode(1);
            ST7920_ClearGraphic();

            ST7920_GraphicMode(0);
            ST7920_ClearText();

            memset(prev_line0, 0, sizeof(prev_line0));
            memset(prev_line1, 0, sizeof(prev_line1));
            memset(prev_line2, 0, sizeof(prev_line2));
            memset(prev_line3, 0, sizeof(prev_line3));

            graphic_test_was_on = false;
        }

        VehicleDisplay_BuildLines(line0, line1, line2, line3);

        VehicleDisplay_WriteLineIfChanged(0, prev_line0, line0);
        VehicleDisplay_WriteLineIfChanged(1, prev_line1, line1);
        VehicleDisplay_WriteLineIfChanged(2, prev_line2, line2);
        VehicleDisplay_WriteLineIfChanged(3, prev_line3, line3);

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    }
}

static void Task_LineSensor(void *argument)
{
    TickType_t lastWakeTime;
    uint8_t raw;
    line_state_t line_state;

    (void)argument;

    LineSensor_Init(&hi2c3, LINE_SENSOR_ADDR_7BIT);
    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /*
         * TODO 2 :
         * Lire le capteur de ligne.
         *
         * À faire :
         * - prendre le mutex I2C3 avec xSemaphoreTake()
         * - lire les 7 capteurs avec LineSensor_ReadRaw()
         * - libérer le mutex avec xSemaphoreGive()
         */

        raw = 0;

        /*
         * TODO 3 :
         * Décoder la valeur brute.
         *
         * À faire :
         * - appeler DecodeLineState(raw)
         * - mettre à jour VehicleDisplayData_SetLineData()
         * - envoyer l'état au module de contrôle avec VehicleControl_SetLineState()
         * - si la ligne est valide, envoyer l'erreur avec VehicleControl_SetLineError()
         */

        line_state = LINE_STATE_UNKNOWN;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LINE_SENSOR_PERIOD_MS));
    }
}




static void Task_ProximitySensors(void *argument)
{
    TickType_t lastWakeTime;
    proximity_sensor_data_t prox;
    uint32_t dmm;

    uint16_t raw_left = 0;
    uint16_t raw_right = 0;
    uint32_t mv_left = 0;
    uint32_t mv_right = 0;

    (void)argument;

    RCWL1601_Init(&hrcwl,
                  GPIOB, GPIO_PIN_5,
                  GPIOB, GPIO_PIN_7,
                  &htim5,
                  40);

    HAL_TIM_Base_Start(&htim5);

    memset(&prox, 0, sizeof(prox));
    lastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        /*
         * TODO 4 :
         * Lire les deux capteurs Sharp.
         *
         * À faire :
         * - appeler ReadBothSharpRaw(&raw_left, &raw_right)
         * - convertir raw_left/raw_right en mV avec SharpRawToMilliVolts()
         * - convertir les mV en distance avec SHARP_2Y0A21_MilliVoltsToDistanceMm()
         * - remplir prox.left_mm, prox.right_mm
         * - remplir prox.left_valid, prox.right_valid
         *
         * Suggestion :
         * Si la conversion échoue, considérer que l'objet est loin :
         * distance = 500 mm, valid = true.
         */

        /*
         * TODO 5 :
         * Lire le capteur ultrason central.
         *
         * À faire :
         * - appeler RCWL1601_Trigger(&hrcwl)
         * - attendre RCWL_WAIT_MS
         * - appeler RCWL1601_Process(&hrcwl)
         * - appeler RCWL1601_GetDistanceMm(&hrcwl, &dmm)
         * - remplir prox.center_mm et prox.center_valid
         *
         * Suggestion :
         * Si aucune distance n'est reçue, considérer que rien n'est devant :
         * distance = 600 mm, valid = true.
         */

        /*
         * TODO 6 :
         * Publier les données de proximité.
         *
         * À faire :
         * - VehicleDisplayData_SetProximityData(&prox, mv_left, mv_right)
         * - VehicleControl_SetProximityData(&prox)
         */

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(PROX_SENSOR_PERIOD_MS));
    }
}



/*============================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

void VehicleTasks_Create(void)
{
    qCtrlEvents = xQueueCreate(8, sizeof(ctrl_event_t));
    qMotorCmd   = xQueueCreate(1, sizeof(motor_cmd_t));

    g_i2c3_mutex = xSemaphoreCreateMutex();

    configASSERT(g_i2c3_mutex != NULL);
    configASSERT(qCtrlEvents != NULL);
	configASSERT(qMotorCmd != NULL);

    VehicleMotors_SetI2C3Mutex(g_i2c3_mutex);

    VehicleStatusLed_Init();

    xTaskCreate(Task_BluetoothRx, "BluetoothRx", 512, NULL, 3, &hTaskBluetoothRx);
    xTaskCreate(Task_MainController, "MainCtrl", 512, NULL, 2, &hTaskMainController);
    xTaskCreate(Task_MotorControl, "MotorCtrl", 512, NULL, 2, &hTaskMotorControl);
    xTaskCreate(Task_StatusLED, "StatusLED", 256, NULL, 1, &hTaskStatusLED);
    xTaskCreate(Task_Display, "Display", 512, NULL, 1, &hTaskDisplay);
    xTaskCreate(Task_LineSensor, "LineSensor", 256, NULL, 2, &hTaskLineSensor);
    xTaskCreate(Task_ProximitySensors, "ProxSense", 512, NULL, 2, &hTaskProximitySensors);


    configASSERT(hTaskBluetoothRx != NULL);
    configASSERT(hTaskMainController != NULL);
    configASSERT(hTaskMotorControl != NULL);
    configASSERT(hTaskStatusLED != NULL);
    configASSERT(hTaskDisplay != NULL);
    configASSERT(hTaskLineSensor != NULL);
    configASSERT(hTaskProximitySensors != NULL);
}
