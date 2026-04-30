/**
 * @file    drv8234.c
 * @brief   Implémentation du driver DRV8234 (version I2C simplifiée)
 * @author  Jonathan Marois
 *
 * Description :
 *  Implémentation d’un driver léger pour le DRV8234 basé uniquement sur I2C.
 *  Cette version ne supporte pas le contrôle par GPIO ni les fonctionnalités avancées
 *  du composant, afin de simplifier l'utilisation et la compréhension.
 *
 * Architecture du driver :
 *
 *  1. Couche I2C bas niveau
 *     - DRV8234_ReadReg()
 *     - DRV8234_WriteReg()
 *     - DRV8234_ModifyReg()
 *
 *  2. Initialisation du DRV8234
 *     - Configuration des registres CONFIG0, CONFIG3, CONFIG4
 *     - Activation du mode I2C_BC
 *     - Activation des sorties (EN_OUT)
 *
 *  3. Contrôle du moteur
 *     - DRV8234_Drive()
 *     - DRV8234_Brake()
 *     - DRV8234_Coast()
 *     - DRV8234_SetPWMDutyCycle()
 *
 *  4. Gestion du mode veille
 *     - DRV8234_Sleep()
 *     - DRV8234_Wake()
 *
 *  5. Gestion des défauts
 *     - Lecture du registre FAULT
 *     - Effacement via CLR_FLT
 *
 * Choix de conception :
 *  - Utilisation exclusive du contrôle I2C (CONFIG4.I2C_BC = 1)
 *  - Utilisation du duty cycle externe (EXT_DUTY)
 *  - Code volontairement simplifié pour usage pédagogique
 *
 * Non supporté dans cette version :
 *  - Mesure de courant (IPROPI)
 *  - Régulation interne (current/speed/voltage)
 *  - Ripple counting
 *  - Stall detection avancé
 *  - Interruptions sur nFAULT
 */

#include "drv8234.h"
#include <string.h>

/* ============================================================================
 * DÉLAI I2C
 * ========================================================================== */
#define DRV8234_I2C_TIMEOUT_MS   100U

/* ============================================================================
 * MACROS INTERNES
 * ========================================================================== */
#define DRV8234_CHECK_PTR(h) do {           \
    if ((h) == NULL) return DRV8234_ERR_PARAM; \
} while (0)

#define DRV8234_CHECK_INIT(h) do {          \
    if ((h) == NULL) return DRV8234_ERR_PARAM; \
    if (!(h)->initialized) return DRV8234_ERR_NOT_INIT; \
} while (0)

/* ============================================================================
 * I2C BAS NIVEAU
 * ========================================================================== */
DRV8234_Status_t DRV8234_WriteReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t value)
{
    DRV8234_CHECK_PTR(hdrv);
    if (hdrv->config.hi2c == NULL) return DRV8234_ERR_PARAM;

    uint8_t buf[2] = { reg, value };

    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(
        hdrv->config.hi2c,
        hdrv->config.i2c_addr,
        buf,
        2,
        DRV8234_I2C_TIMEOUT_MS
    );

    return (ret == HAL_OK) ? DRV8234_OK : DRV8234_ERR_I2C;
}

DRV8234_Status_t DRV8234_ReadReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t *value)
{
    DRV8234_CHECK_PTR(hdrv);
    if (value == NULL) return DRV8234_ERR_PARAM;
    if (hdrv->config.hi2c == NULL) return DRV8234_ERR_PARAM;

    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(
        hdrv->config.hi2c,
        hdrv->config.i2c_addr,
        &reg,
        1,
        DRV8234_I2C_TIMEOUT_MS
    );
    if (ret != HAL_OK) return DRV8234_ERR_I2C;

    ret = HAL_I2C_Master_Receive(
        hdrv->config.hi2c,
        hdrv->config.i2c_addr,
        value,
        1,
        DRV8234_I2C_TIMEOUT_MS
    );

    return (ret == HAL_OK) ? DRV8234_OK : DRV8234_ERR_I2C;
}

DRV8234_Status_t DRV8234_ModifyReg(DRV8234_Handle_t *hdrv, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t current = 0;
    DRV8234_Status_t st = DRV8234_ReadReg(hdrv, reg, &current);
    if (st != DRV8234_OK) return st;

    current = (uint8_t)((current & ~mask) | (value & mask));
    return DRV8234_WriteReg(hdrv, reg, current);
}

/* ============================================================================
 * INITIALISATION
 * ========================================================================== */
DRV8234_Status_t DRV8234_Init(DRV8234_Handle_t *hdrv, const DRV8234_Config_t *config)
{
    DRV8234_Status_t st;
    uint8_t reg;
    uint8_t config0_reg;

    DRV8234_CHECK_PTR(hdrv);
    if (config == NULL) return DRV8234_ERR_PARAM;
    if (config->hi2c == NULL) return DRV8234_ERR_PARAM;
    if (config->nsleep_port == NULL) return DRV8234_ERR_PARAM;

    memcpy(&hdrv->config, config, sizeof(DRV8234_Config_t));

    hdrv->state        = DRV8234_STATE_COAST;
    hdrv->fault_reg    = 0;
    hdrv->fault_active = false;
    hdrv->initialized  = false;

    /* Réveil du driver */
    HAL_GPIO_WritePin(hdrv->config.nsleep_port, hdrv->config.nsleep_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    /* CONFIG0 :
       EN_OUT = 0 pendant configuration des bits protégés
       DUTY_CTRL = 1 pour EXT_DUTY
    */
    config0_reg = DRV8234_CONFIG0_DUTY_CTRL;
    if (config->en_ovp) {
        config0_reg |= DRV8234_CONFIG0_EN_OVP;
    }

    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG0, config0_reg);
    if (st != DRV8234_OK) return st;

    /* CONFIG3 */
    reg = 0;
    reg |= (DRV8234_IMODE_DISABLED << DRV8234_CONFIG3_IMODE_POS);

    if (config->ocp_mode == DRV8234_FAULT_RETRY) {
        reg |= DRV8234_CONFIG3_OCP_MODE;
    }
    if (config->tsd_mode == DRV8234_FAULT_RETRY) {
        reg |= DRV8234_CONFIG3_TSD_MODE;
    }

    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG3, reg);
    if (st != DRV8234_OK) return st;

    /* CONFIG4 :
       - I2C_BC = 1 toujours
       - PMODE selon le mode choisi
       - CBC_REP selon config
       - bits I2C_EN_IN1 / I2C_PH_IN2 laissés à 0 au départ
    */
    reg = DRV8234_CONFIG4_I2C_BC;

    if (config->bridge_mode == DRV8234_MODE_PWM) {
        reg |= DRV8234_CONFIG4_PMODE;
    }
    if (config->cbc_rep) {
        reg |= DRV8234_CONFIG4_CBC_REP;
    }

    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG4, reg);
    if (st != DRV8234_OK) return st;

    /* REG_CTRL0 : mode simple */
    reg = (DRV8234_REGCTRL_MODE_FIXED_OFF << DRV8234_REGCTRL0_REG_CTRL_POS);
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_REG_CTRL0, reg);
    if (st != DRV8234_OK) return st;

    /* REG_CTRL1 inutile ici */
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_REG_CTRL1, 0x00);
    if (st != DRV8234_OK) return st;

    /* duty initial = 0 */
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_REG_CTRL2, 0x00);
    if (st != DRV8234_OK) return st;

    /* Active les sorties */
    config0_reg |= DRV8234_CONFIG0_EN_OUT;
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG0, config0_reg);
    if (st != DRV8234_OK) return st;

    hdrv->initialized = true;

    st = DRV8234_ClearFault(hdrv);
    if (st != DRV8234_OK) {
        hdrv->initialized = false;
        return st;
    }

    return DRV8234_OK;
}

DRV8234_Status_t DRV8234_DeInit(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);
    DRV8234_Sleep(hdrv);
    hdrv->initialized = false;
    return DRV8234_OK;
}

/* ============================================================================
 * SLEEP / WAKE
 * ========================================================================== */
DRV8234_Status_t DRV8234_Sleep(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    HAL_GPIO_WritePin(hdrv->config.nsleep_port, hdrv->config.nsleep_pin, GPIO_PIN_RESET);
    hdrv->state = DRV8234_STATE_SLEEP;
    return DRV8234_OK;
}

DRV8234_Status_t DRV8234_Wake(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    HAL_GPIO_WritePin(hdrv->config.nsleep_port, hdrv->config.nsleep_pin, GPIO_PIN_SET);
    HAL_Delay(2);

    hdrv->state = DRV8234_STATE_COAST;
    return DRV8234_ClearFault(hdrv);
}

bool DRV8234_IsAsleep(const DRV8234_Handle_t *hdrv)
{
    if (hdrv == NULL) return true;
    if (hdrv->config.nsleep_port == NULL) return true;

    return (HAL_GPIO_ReadPin(hdrv->config.nsleep_port, hdrv->config.nsleep_pin) == GPIO_PIN_RESET);
}

/* ============================================================================
 * CONTRÔLE DU PONT EN H
 * ========================================================================== */
DRV8234_Status_t DRV8234_SetBridgeMode(DRV8234_Handle_t *hdrv, DRV8234_BridgeMode_t mode)
{
    DRV8234_CHECK_INIT(hdrv);

    hdrv->config.bridge_mode = mode;

    return DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG4,
        DRV8234_CONFIG4_PMODE,
        (mode == DRV8234_MODE_PWM) ? DRV8234_CONFIG4_PMODE : 0
    );
}

DRV8234_Status_t DRV8234_Drive(DRV8234_Handle_t *hdrv, DRV8234_Direction_t dir)
{
    DRV8234_CHECK_INIT(hdrv);

    uint8_t bits = 0;

    if (hdrv->config.bridge_mode == DRV8234_MODE_PH_EN) {
        /* PH/EN :
           EN=1 toujours pour rouler
           PH=0 -> forward
           PH=1 -> reverse
        */
        bits = DRV8234_CONFIG4_I2C_EN_IN1;
        if (dir == DRV8234_DIR_REVERSE) {
            bits |= DRV8234_CONFIG4_I2C_PH_IN2;
        }
    } else {
        /* PWM :
           forward -> IN1=1 IN2=0
           reverse -> IN1=0 IN2=1
        */
        bits = (dir == DRV8234_DIR_FORWARD)
             ? DRV8234_CONFIG4_I2C_EN_IN1
             : DRV8234_CONFIG4_I2C_PH_IN2;
    }

    DRV8234_Status_t st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG4,
        DRV8234_CONFIG4_I2C_EN_IN1 | DRV8234_CONFIG4_I2C_PH_IN2,
        bits
    );

    if (st == DRV8234_OK) {
        hdrv->state = (dir == DRV8234_DIR_FORWARD)
                    ? DRV8234_STATE_FORWARD
                    : DRV8234_STATE_REVERSE;
    }

    return st;
}

DRV8234_Status_t DRV8234_Brake(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    uint8_t bits;

    if (hdrv->config.bridge_mode == DRV8234_MODE_PH_EN) {
        /* PH/EN : EN=0 */
        bits = 0;
    } else {
        /* PWM : IN1=1, IN2=1 */
        bits = DRV8234_CONFIG4_I2C_EN_IN1 | DRV8234_CONFIG4_I2C_PH_IN2;
    }

    DRV8234_Status_t st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG4,
        DRV8234_CONFIG4_I2C_EN_IN1 | DRV8234_CONFIG4_I2C_PH_IN2,
        bits
    );

    if (st == DRV8234_OK) {
        hdrv->state = DRV8234_STATE_BRAKE;
    }

    return st;
}

DRV8234_Status_t DRV8234_Coast(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    DRV8234_Status_t st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG4,
        DRV8234_CONFIG4_I2C_EN_IN1 | DRV8234_CONFIG4_I2C_PH_IN2,
        0
    );

    if (st == DRV8234_OK) {
        hdrv->state = DRV8234_STATE_COAST;
    }

    return st;
}

DRV8234_Status_t DRV8234_SetPWMDutyCycle(DRV8234_Handle_t *hdrv, uint8_t duty_percent)
{
    DRV8234_CHECK_INIT(hdrv);

    if (duty_percent > 100) {
        return DRV8234_ERR_PARAM;
    }

    DRV8234_Status_t st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG0,
        DRV8234_CONFIG0_DUTY_CTRL,
        DRV8234_CONFIG0_DUTY_CTRL
    );
    if (st != DRV8234_OK) {
        return st;
    }

    uint8_t duty_reg = (uint8_t)((duty_percent * 63U) / 100U);

    return DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_REG_CTRL2,
        DRV8234_REGCTRL2_EXT_DUTY_MASK,
        duty_reg
    );
}

/* ============================================================================
 * GESTION DES FAUTES
 * ========================================================================== */
DRV8234_Status_t DRV8234_ReadFault(DRV8234_Handle_t *hdrv, uint8_t *fault)
{
    DRV8234_CHECK_INIT(hdrv);
    if (fault == NULL) return DRV8234_ERR_PARAM;

    DRV8234_Status_t st = DRV8234_ReadReg(hdrv, DRV8234_REG_FAULT, fault);
    if (st != DRV8234_OK) {
        return st;
    }

    hdrv->fault_reg = *fault;
    hdrv->fault_active = ((*fault & (DRV8234_FAULT_FAULT |
                                     DRV8234_FAULT_STALL |
                                     DRV8234_FAULT_OCP |
                                     DRV8234_FAULT_OVP |
                                     DRV8234_FAULT_TSD)) != 0U);

    return DRV8234_OK;
}

bool DRV8234_IsFaultActive(DRV8234_Handle_t *hdrv)
{
    if (hdrv == NULL || !hdrv->initialized) {
        return false;
    }

    uint8_t fault = 0;
    if (DRV8234_ReadFault(hdrv, &fault) != DRV8234_OK) {
        return false;
    }

    return hdrv->fault_active;
}

DRV8234_Status_t DRV8234_ClearFault(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    DRV8234_Status_t st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG0,
        DRV8234_CONFIG0_CLR_FLT,
        DRV8234_CONFIG0_CLR_FLT
    );
    if (st != DRV8234_OK) {
        return st;
    }

    /* Remet le bit à 0 pour éviter de le laisser à 1 */
    st = DRV8234_ModifyReg(
        hdrv,
        DRV8234_REG_CONFIG0,
        DRV8234_CONFIG0_CLR_FLT,
        0
    );
    if (st != DRV8234_OK) {
        return st;
    }

    hdrv->fault_reg = 0;
    hdrv->fault_active = false;

    return DRV8234_OK;
}


/* ============================================================================
 * RIPPLE COUNTING
 * ========================================================================== */

DRV8234_Status_t DRV8234_EnableRippleCounting(DRV8234_Handle_t *hdrv)
{
    DRV8234_CHECK_INIT(hdrv);

    DRV8234_Status_t st;
    uint8_t config0;

    /* RC_CTRLx modifiables seulement quand EN_OUT = 0 */
    st = DRV8234_ReadReg(hdrv, DRV8234_REG_CONFIG0, &config0);
    if (st != DRV8234_OK) return st;

    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG0,
                          (uint8_t)(config0 & ~DRV8234_CONFIG0_EN_OUT));
    if (st != DRV8234_OK) return st;

    /* Réglage minimal pour essai
       EN_RC = 1
       DIS_EC = 0
       RC_HIZ = 0
       FLT_GAIN_SEL = 01b
       CS_GAIN_SEL  = 001b   (2 A)
    */
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_RC_CTRL0,
                          (uint8_t)(DRV8234_RC_CTRL0_EN_RC |
                                    (1U << DRV8234_RC_CTRL0_FLT_GAIN_SEL_POS) |
                                    (1U << DRV8234_RC_CTRL0_CS_GAIN_SEL_POS)));
    if (st != DRV8234_OK) return st;

    /* Laisse les autres paramètres à leur valeur par défaut pour un premier test */
    st = DRV8234_WriteReg(hdrv, DRV8234_REG_CONFIG0, config0);
    if (st != DRV8234_OK) return st;

    return DRV8234_OK;
}

DRV8234_Status_t DRV8234_ReadRippleSpeedRaw(DRV8234_Handle_t *hdrv, uint8_t *speed_raw)
{
    DRV8234_CHECK_INIT(hdrv);
    if (speed_raw == NULL) return DRV8234_ERR_PARAM;

    return DRV8234_ReadReg(hdrv, DRV8234_REG_RC_STATUS1, speed_raw);
}















