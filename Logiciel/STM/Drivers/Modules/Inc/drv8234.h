
/**
 * @file    drv8234.h
 * @brief   Driver simplifié pour le DRV8234 (pont en H DC brushed)
 * @target  STM32F407 (HAL I2C)
 * @version 2.0 (version allégée - I2C uniquement)
 *
 *    Created on: 1 mars 2026
 *    Modified on: 2026
 *      Author: Jonathan Marois
 *
 * Description :
 *  Driver minimaliste du DRV8234 utilisant exclusivement l'interface I2C.
 *  Cette version est destinée à un usage pédagogique et à des applications simples
 *  où seul le contrôle de base du moteur est requis.
 *
 * Fonctionnalités couvertes :
 *  - Communication I2C (lecture / écriture de registres)
 *  - Contrôle du pont en H via I2C (mode PH/EN ou PWM)
 *  - Commandes moteur :
 *      • Forward / Reverse
 *      • Brake
 *      • Coast
 *  - Réglage du duty cycle (EXT_DUTY)
 *  - Gestion du mode veille (nSLEEP)
 *  - Lecture et effacement des défauts (FAULT register)
 *  - Support de la broche nFAULT (lecture simple)
 *
 * Limitations de cette version :
 *  - Aucun contrôle via GPIO (EN/IN1, PH/IN2 non utilisés)
 *  - Pas de mesure de courant (IPROPI / ADC retiré)
 *  - Pas de régulation avancée (courant, vitesse, tension)
 *  - Pas de ripple counting
 *  - Pas de stall detection avancé
 *
 * NOTE :
 *  Les adresses de registres proviennent de la section 7.6 du datasheet
 *  DRV8234 (Texas Instruments). Toujours valider avec la version la plus récente.
 *
 * Adresse I2C de base 0x30 (7 bits) → 0x60 en 8 bits
 * (modifiable selon A1/A0 — tri-level)
 */


#ifndef DRV8234_H
#define DRV8234_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * ADRESSES I2C
 * ========================================================================== */
#define DRV8234_I2C_BASE_ADDR    0x30U
#define DRV8234_I2C_ADDR(a1, a0) ((DRV8234_I2C_BASE_ADDR + (a1) * 3U + (a0)) << 1)

/* ============================================================================
 * REGISTRES
 * ========================================================================== */
#define DRV8234_REG_FAULT         0x00U
#define DRV8234_REG_RC_STATUS1    0x01U
#define DRV8234_REG_RC_STATUS2    0x02U
#define DRV8234_REG_RC_STATUS3    0x03U
#define DRV8234_REG_REG_STATUS1   0x04U
#define DRV8234_REG_REG_STATUS2   0x05U
#define DRV8234_REG_REG_STATUS3   0x06U

#define DRV8234_REG_CONFIG0       0x09U
#define DRV8234_REG_CONFIG1       0x0AU
#define DRV8234_REG_CONFIG2       0x0BU
#define DRV8234_REG_CONFIG3       0x0CU
#define DRV8234_REG_CONFIG4       0x0DU

#define DRV8234_REG_REG_CTRL0     0x0EU
#define DRV8234_REG_REG_CTRL1     0x0FU
#define DRV8234_REG_REG_CTRL2     0x10U

// Ripple Counting
#define DRV8234_REG_RC_CTRL0      0x11U
#define DRV8234_REG_RC_CTRL1      0x12U
#define DRV8234_REG_RC_CTRL2      0x13U
#define DRV8234_REG_RC_CTRL3      0x14U
#define DRV8234_REG_RC_CTRL4      0x15U

#define DRV8234_RC_CTRL0_EN_RC           (1U << 7)
#define DRV8234_RC_CTRL0_DIS_EC          (1U << 6)
#define DRV8234_RC_CTRL0_RC_HIZ          (1U << 5)
#define DRV8234_RC_CTRL0_FLT_GAIN_SEL_POS 	3U
#define DRV8234_RC_CTRL0_CS_GAIN_SEL_POS  	0U
/* ============================================================================
 * BITS / MASQUES
 * ========================================================================== */

/* FAULT (0x00) */
#define DRV8234_FAULT_FAULT         (1U << 7)
#define DRV8234_FAULT_STALL         (1U << 5)
#define DRV8234_FAULT_OCP           (1U << 4)
#define DRV8234_FAULT_OVP           (1U << 3)
#define DRV8234_FAULT_TSD           (1U << 2)
#define DRV8234_FAULT_NPOR          (1U << 1)
#define DRV8234_FAULT_CNT_DONE      (1U << 0)

/* CONFIG0 (0x09) */
#define DRV8234_CONFIG0_EN_OUT      (1U << 7)
#define DRV8234_CONFIG0_EN_OVP      (1U << 6)
#define DRV8234_CONFIG0_EN_STALL    (1U << 5)
#define DRV8234_CONFIG0_VSNS_SEL    (1U << 4)
#define DRV8234_CONFIG0_CLR_CNT     (1U << 2)
#define DRV8234_CONFIG0_CLR_FLT     (1U << 1)
#define DRV8234_CONFIG0_DUTY_CTRL   (1U << 0)

/* CONFIG3 (0x0C) */
#define DRV8234_CONFIG3_IMODE_POS   6U
#define DRV8234_CONFIG3_IMODE_MASK  (0x03U << DRV8234_CONFIG3_IMODE_POS)
#define DRV8234_CONFIG3_SMODE       (1U << 5)
#define DRV8234_CONFIG3_INT_VREF    (1U << 4)
#define DRV8234_CONFIG3_TBLANK      (1U << 3)
#define DRV8234_CONFIG3_TDEG        (1U << 2)
#define DRV8234_CONFIG3_OCP_MODE    (1U << 1)
#define DRV8234_CONFIG3_TSD_MODE    (1U << 0)

/* CONFIG4 (0x0D) */
#define DRV8234_CONFIG4_RC_REP_POS    6U
#define DRV8234_CONFIG4_RC_REP_MASK   (0x03U << DRV8234_CONFIG4_RC_REP_POS)
#define DRV8234_CONFIG4_STALL_REP     (1U << 5)
#define DRV8234_CONFIG4_CBC_REP       (1U << 4)
#define DRV8234_CONFIG4_PMODE         (1U << 3)
#define DRV8234_CONFIG4_I2C_BC        (1U << 2)
#define DRV8234_CONFIG4_I2C_EN_IN1    (1U << 1)
#define DRV8234_CONFIG4_I2C_PH_IN2    (1U << 0)

/* REG_CTRL0 (0x0E) */
#define DRV8234_REGCTRL0_EN_SS          (1U << 5)
#define DRV8234_REGCTRL0_REG_CTRL_POS   3U
#define DRV8234_REGCTRL0_REG_CTRL_MASK  (0x03U << DRV8234_REGCTRL0_REG_CTRL_POS)
#define DRV8234_REGCTRL0_PWM_FREQ       (1U << 2)
#define DRV8234_REGCTRL0_W_SCALE_POS    0U
#define DRV8234_REGCTRL0_W_SCALE_MASK   (0x03U << DRV8234_REGCTRL0_W_SCALE_POS)

/* REG_CTRL0 modes */
#define DRV8234_REGCTRL_MODE_FIXED_OFF   0x00U
#define DRV8234_REGCTRL_MODE_CYCLE       0x01U
#define DRV8234_REGCTRL_MODE_SPEED       0x02U
#define DRV8234_REGCTRL_MODE_VOLTAGE     0x03U

/* IMODE */
#define DRV8234_IMODE_DISABLED           0x00U
#define DRV8234_IMODE_TINRUSH_ONLY       0x01U
#define DRV8234_IMODE_ALWAYS             0x02U

/* REG_CTRL2 (0x10) */
#define DRV8234_REGCTRL2_OUT_FLT_POS    6U
#define DRV8234_REGCTRL2_OUT_FLT_MASK   (0x03U << DRV8234_REGCTRL2_OUT_FLT_POS)
#define DRV8234_REGCTRL2_EXT_DUTY_MASK  0x3FU

/* ============================================================================
 * TYPES
 * ========================================================================== */
typedef enum {
    DRV8234_OK           = 0,
    DRV8234_ERR_I2C      = 1,
    DRV8234_ERR_PARAM    = 2,
    DRV8234_ERR_FAULT    = 3,
    DRV8234_ERR_TIMEOUT  = 4,
    DRV8234_ERR_NOT_INIT = 5,
} DRV8234_Status_t;

typedef enum {
    DRV8234_MODE_PH_EN = 0,
    DRV8234_MODE_PWM   = 1,
} DRV8234_BridgeMode_t;

typedef enum {
    DRV8234_DIR_FORWARD = 0,
    DRV8234_DIR_REVERSE = 1,
} DRV8234_Direction_t;

typedef enum {
    DRV8234_STATE_COAST   = 0,
    DRV8234_STATE_BRAKE   = 1,
    DRV8234_STATE_FORWARD = 2,
    DRV8234_STATE_REVERSE = 3,
    DRV8234_STATE_SLEEP   = 4,
} DRV8234_BridgeState_t;

typedef enum {
    DRV8234_FAULT_LATCH = 0,
    DRV8234_FAULT_RETRY = 1,
} DRV8234_FaultMode_t;

/* ============================================================================
 * CONFIGURATION
 * ========================================================================== */
typedef struct {
    /* I2C */
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;   /* adresse HAL décalée (7 bits << 1) */

    /* GPIO essentiels */
    GPIO_TypeDef      *nsleep_port;
    uint16_t           nsleep_pin;

    GPIO_TypeDef      *nfault_port;
    uint16_t           nfault_pin;

    /* Mode du pont */
    DRV8234_BridgeMode_t bridge_mode;

    /* Protections */
    DRV8234_FaultMode_t ocp_mode;
    DRV8234_FaultMode_t tsd_mode;
    bool                en_ovp;
    bool                cbc_rep;
} DRV8234_Config_t;

typedef struct {
    DRV8234_Config_t      config;
    DRV8234_BridgeState_t state;
    uint8_t               fault_reg;
    bool                  fault_active;
    bool                  initialized;
} DRV8234_Handle_t;

/* ============================================================================
 * API BAS NIVEAU
 * ========================================================================== */
DRV8234_Status_t DRV8234_WriteReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t value);
DRV8234_Status_t DRV8234_ReadReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t *value);
DRV8234_Status_t DRV8234_ModifyReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t mask, uint8_t value);

/* ============================================================================
 * API PRINCIPALE
 * ========================================================================== */
DRV8234_Status_t DRV8234_Init(DRV8234_Handle_t *hdrv, const DRV8234_Config_t *config);
DRV8234_Status_t DRV8234_DeInit(DRV8234_Handle_t *hdrv);

DRV8234_Status_t DRV8234_Sleep(DRV8234_Handle_t *hdrv);
DRV8234_Status_t DRV8234_Wake(DRV8234_Handle_t *hdrv);
bool             DRV8234_IsAsleep(const DRV8234_Handle_t *hdrv);

DRV8234_Status_t DRV8234_SetBridgeMode(DRV8234_Handle_t *hdrv, DRV8234_BridgeMode_t mode);

DRV8234_Status_t DRV8234_Drive(DRV8234_Handle_t *hdrv, DRV8234_Direction_t dir);
DRV8234_Status_t DRV8234_Brake(DRV8234_Handle_t *hdrv);
DRV8234_Status_t DRV8234_Coast(DRV8234_Handle_t *hdrv);

DRV8234_Status_t DRV8234_SetPWMDutyCycle(DRV8234_Handle_t *hdrv, uint8_t duty_percent);

DRV8234_Status_t DRV8234_ClearFault(DRV8234_Handle_t *hdrv);
DRV8234_Status_t DRV8234_ReadFault(DRV8234_Handle_t *hdrv, uint8_t *fault);
bool             DRV8234_IsFaultActive(DRV8234_Handle_t *hdrv);

DRV8234_Status_t DRV8234_EnableRippleCounting(DRV8234_Handle_t *hdrv);
DRV8234_Status_t DRV8234_ReadRippleSpeedRaw(DRV8234_Handle_t *hdrv, uint8_t *speed_raw);

#endif /* DRV8234_H */














