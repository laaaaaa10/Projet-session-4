/*
 * vehicle_motors.c
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Module matériel des moteurs du véhicule.
 *
 * Ce fichier associe les quatre moteurs physiques aux quatre contrôleurs DRV8234.
 * Il convertit les commandes signées (-100 à +100) en commandes DRV8234 :
 * - duty cycle;
 * - direction;
 * - roue libre;
 * - lecture nFAULT;
 * - lecture Ripple Counting.
 *
 * La logique de déplacement reste dans vehicle_control.c.
 */

#include "vehicle_motors.h"

#include "main.h"
#include "drv8234.h"
#include "app_types.h"
#include "semphr.h"

/*============================================================================
 * EXTERNAL HANDLES
 *===========================================================================*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c3;

/*============================================================================
 * PRIVATE VARIABLES
 *===========================================================================*/

static DRV8234_Handle_t hmotor_front_left;
static DRV8234_Handle_t hmotor_rear_left;
static DRV8234_Handle_t hmotor_front_right;
static DRV8234_Handle_t hmotor_rear_right;

static bool g_vehicle_motors_error = false;

static SemaphoreHandle_t g_i2c3_mutex = NULL;

/*============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 *===========================================================================*/

static bool Motor_InitOne(DRV8234_Handle_t *hmotor,
                          I2C_HandleTypeDef *hi2c,
                          uint8_t addr,
                          GPIO_TypeDef *nsleep_port,
                          uint16_t nsleep_pin,
                          GPIO_TypeDef *nfault_port,
                          uint16_t nfault_pin);

static void Motor_SetSignedSpeed(DRV8234_Handle_t *hmotor, int16_t speed);
static void LeftSide_SetSignedSpeed(int16_t speed);
static void RightSide_SetSignedSpeed(int16_t speed);
static DRV8234_Handle_t *Motor_GetHandle(motor_target_t motor);

static void I2C3_Lock(void);
static void I2C3_Unlock(void);
/*============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/*
 * Initialise un contrôleur DRV8234 avec sa configuration matérielle :
 * bus I2C, adresse, nSLEEP et nFAULT.
 */

static bool Motor_InitOne(DRV8234_Handle_t *hmotor,
                          I2C_HandleTypeDef *hi2c,
                          uint8_t addr,
                          GPIO_TypeDef *nsleep_port,
                          uint16_t nsleep_pin,
                          GPIO_TypeDef *nfault_port,
                          uint16_t nfault_pin)
{
    DRV8234_Config_t cfg = {0};

    cfg.hi2c = hi2c;
    cfg.i2c_addr = addr;
    cfg.nsleep_port = nsleep_port;
    cfg.nsleep_pin = nsleep_pin;
    cfg.nfault_port = nfault_port;
    cfg.nfault_pin = nfault_pin;
    cfg.bridge_mode = DRV8234_MODE_PH_EN;
    cfg.ocp_mode = DRV8234_FAULT_RETRY;
    cfg.tsd_mode = DRV8234_FAULT_RETRY;
    cfg.en_ovp = true;
    cfg.cbc_rep = false;

    return (DRV8234_Init(hmotor, &cfg) == DRV8234_OK);
}

/*
 * Applique une vitesse signée à un moteur.
 * Valeur positive = avant, valeur négative = arrière, 0 = roue libre.
 */
static void Motor_SetSignedSpeed(DRV8234_Handle_t *hmotor, int16_t speed)
{
    uint8_t duty;

    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    if (speed == 0)
    {
        DRV8234_Coast(hmotor);
        return;
    }

    duty = (uint8_t)((speed >= 0) ? speed : -speed);

    if (DRV8234_SetPWMDutyCycle(hmotor, duty) != DRV8234_OK)
        return;

    if (speed > 0)
    {
        if (DRV8234_Drive(hmotor, DRV8234_DIR_FORWARD) != DRV8234_OK)
            return;
    }
    else
    {
        if (DRV8234_Drive(hmotor, DRV8234_DIR_REVERSE) != DRV8234_OK)
            return;
    }
}

/*
 * Applique la même vitesse aux deux moteurs du côté gauche.
 */
static void LeftSide_SetSignedSpeed(int16_t speed)
{
    Motor_SetSignedSpeed(&hmotor_front_left, speed);
    Motor_SetSignedSpeed(&hmotor_rear_left,  speed);
}

/*
 * Applique la même vitesse aux deux moteurs du côté droit.
 */
static void RightSide_SetSignedSpeed(int16_t speed)
{
    I2C3_Lock();

    Motor_SetSignedSpeed(&hmotor_front_right, speed);
    Motor_SetSignedSpeed(&hmotor_rear_right,  speed);

    I2C3_Unlock();
}

/*
 * Retourne le handle DRV8234 associé à un moteur physique.
 * Utilisé par les fonctions de diagnostic moteur.
 */
static DRV8234_Handle_t *Motor_GetHandle(motor_target_t motor)
{
    switch (motor)
    {
        case MOTOR_TARGET_AVG: return &hmotor_front_left;
        case MOTOR_TARGET_AVD: return &hmotor_front_right;
        case MOTOR_TARGET_ARG: return &hmotor_rear_left;
        case MOTOR_TARGET_ARD: return &hmotor_rear_right;
        default: return NULL;
    }
}

static void I2C3_Lock(void)
{
    if (g_i2c3_mutex != NULL)
    {
        xSemaphoreTake(g_i2c3_mutex, portMAX_DELAY);
    }
}

static void I2C3_Unlock(void)
{
    if (g_i2c3_mutex != NULL)
    {
        xSemaphoreGive(g_i2c3_mutex);
    }
}


/*============================================================================
 * PUBLIC FUNCTIONS
 *===========================================================================*/

void VehicleMotors_Init(void)
{
    g_vehicle_motors_error = false;

    if (!Motor_InitOne(&hmotor_front_left,
                       &hi2c1,
                       0x66,
                       GPIOE, GPIO_PIN_2,
                       GPIOE, GPIO_PIN_7))
    {
        g_vehicle_motors_error = true;
    }

    if (!Motor_InitOne(&hmotor_rear_left,
                       &hi2c1,
                       0x64,
                       GPIOE, GPIO_PIN_2,
                       GPIOE, GPIO_PIN_7))
    {
        g_vehicle_motors_error = true;
    }

    if (!Motor_InitOne(&hmotor_front_right,
                       &hi2c3,
                       0x66,
                       GPIOE, GPIO_PIN_4,
                       GPIOD, GPIO_PIN_2))
    {
        g_vehicle_motors_error = true;
    }

    if (!Motor_InitOne(&hmotor_rear_right,
                       &hi2c3,
                       0x64,
                       GPIOE, GPIO_PIN_4,
                       GPIOD, GPIO_PIN_2))
    {
        g_vehicle_motors_error = true;
    }
}

bool VehicleMotors_HasError(void)
{
    return g_vehicle_motors_error;
}

/*
 * Les moteurs droits sont inversés mécaniquement.
 * On inverse donc le signe pour que +100 signifie avancer pour les deux côtés.
 */
void VehicleMotors_SetLeftRight(int16_t left_cmd, int16_t right_cmd)
{
    LeftSide_SetSignedSpeed(left_cmd);
    RightSide_SetSignedSpeed(-right_cmd);
}

void VehicleMotors_AllCoast(void)
{
    DRV8234_Coast(&hmotor_front_left);
    DRV8234_Coast(&hmotor_rear_left);

    I2C3_Lock();
    DRV8234_Coast(&hmotor_front_right);
    DRV8234_Coast(&hmotor_rear_right);
    I2C3_Unlock();
}

void VehicleMotors_SetOne(motor_target_t motor, int16_t speed)
{
    VehicleMotors_AllCoast();

    switch (motor)
    {
        case MOTOR_TARGET_AVG:
            Motor_SetSignedSpeed(&hmotor_front_left, speed);
            break;

        case MOTOR_TARGET_AVD:
            I2C3_Lock();
            Motor_SetSignedSpeed(&hmotor_front_right, -speed);
            I2C3_Unlock();
            break;

        case MOTOR_TARGET_ARG:
            Motor_SetSignedSpeed(&hmotor_rear_left, speed);
            break;

        case MOTOR_TARGET_ARD:
            I2C3_Lock();
            Motor_SetSignedSpeed(&hmotor_rear_right, -speed);
            I2C3_Unlock();
            break;

        default:
            VehicleMotors_AllCoast();
            break;
    }
}



bool VehicleMotors_ReadOneFaultPin(motor_target_t motor)
{
    DRV8234_Handle_t *h = Motor_GetHandle(motor);

    if (h == NULL)
        return true;

    if (h->config.nfault_port == NULL)
        return true;

    return HAL_GPIO_ReadPin(h->config.nfault_port,
                            h->config.nfault_pin) == GPIO_PIN_RESET;
}

bool VehicleMotors_ReadOneRipple(motor_target_t motor, uint16_t *ripple)
{
    DRV8234_Handle_t *h = Motor_GetHandle(motor);
    uint8_t raw = 0;

    if ((h == NULL) || (ripple == NULL))
        return false;

    if (DRV8234_ReadReg(h, DRV8234_REG_RC_STATUS1, &raw) != DRV8234_OK)
        return false;

    *ripple = raw;
    return true;
}

void VehicleMotors_SetI2C3Mutex(SemaphoreHandle_t mutex)
{
    g_i2c3_mutex = mutex;
}
