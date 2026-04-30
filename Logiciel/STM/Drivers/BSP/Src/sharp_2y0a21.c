/*
 * sharp_2y0a21.c
 *
 *  Created on: 16 avr. 2026
 *      Author: Jonathan Marois
 */


#include "sharp_2y0a21.h"

static HAL_StatusTypeDef sharp_adc_select_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel      = channel;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

    return HAL_ADC_ConfigChannel(hadc, &sConfig);
}

void SHARP_2Y0A21_Init(SHARP_2Y0A21_Handle_t *h,
                       ADC_HandleTypeDef *hadc,
                       uint32_t adc_channel,
                       uint32_t vref_mv,
                       uint16_t adc_max)
{
    if (h == NULL)
    {
        return;
    }

    h->hadc             = hadc;
    h->adc_channel      = adc_channel;
    h->vref_mv          = vref_mv;
    h->adc_max          = adc_max;
    h->raw_last         = 0;
    h->mv_last          = 0;
    h->distance_mm_last = 0;
}

SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadRaw(SHARP_2Y0A21_Handle_t *h, uint16_t *raw)
{
    uint32_t value;

    if ((h == NULL) || (raw == NULL) || (h->hadc == NULL))
    {
        return SHARP_2Y0A21_ERROR;
    }

    if (sharp_adc_select_channel(h->hadc, h->adc_channel) != HAL_OK)
    {
        return SHARP_2Y0A21_ERROR;
    }

    if (HAL_ADC_Start(h->hadc) != HAL_OK)
    {
        return SHARP_2Y0A21_ERROR;
    }

    if (HAL_ADC_PollForConversion(h->hadc, 10) != HAL_OK)
    {
        HAL_ADC_Stop(h->hadc);
        return SHARP_2Y0A21_ERROR;
    }

    value = HAL_ADC_GetValue(h->hadc);
    HAL_ADC_Stop(h->hadc);

    *raw = (uint16_t)value;
    h->raw_last = (uint16_t)value;

    return SHARP_2Y0A21_OK;
}

SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadRawAverage(SHARP_2Y0A21_Handle_t *h,
                                                  uint8_t samples,
                                                  uint16_t *raw)
{
    uint32_t sum = 0;
    uint16_t one_raw = 0;
    uint8_t i;

    if ((h == NULL) || (raw == NULL) || (samples == 0))
    {
        return SHARP_2Y0A21_ERROR;
    }

    for (i = 0; i < samples; i++)
    {
        if (SHARP_2Y0A21_ReadRaw(h, &one_raw) != SHARP_2Y0A21_OK)
        {
            return SHARP_2Y0A21_ERROR;
        }

        sum += one_raw;
    }

    *raw = (uint16_t)(sum / samples);
    h->raw_last = *raw;

    return SHARP_2Y0A21_OK;
}

uint32_t SHARP_2Y0A21_RawToMilliVolts(const SHARP_2Y0A21_Handle_t *h, uint16_t raw)
{
    if ((h == NULL) || (h->adc_max == 0))
    {
        return 0;
    }

    return ((uint32_t)raw * h->vref_mv) / h->adc_max;
}

/*
 * Approximation pour Sharp GP2Y0A21 / 2Y0A21 :
 * plage utile typique ~100 mm à 800 mm.
 *
 * Formule approchée souvent suffisante en embarqué :
 * distance_cm ≈ 27.86 / (Vout - 0.42)
 * avec Vout en volts
 *
 * Ici on travaille en mV :
 * distance_mm = 10 * 27.86 / ((mv/1000) - 0.42)
 *
 * Pour éviter les float :
 * distance_mm ≈ 278600 / (mv - 420)
 */
SHARP_2Y0A21_Status_t SHARP_2Y0A21_MilliVoltsToDistanceMm(uint32_t mv, uint32_t *distance_mm)
{
    uint32_t d_mm;

    if (distance_mm == NULL)
    {
        return SHARP_2Y0A21_ERROR;
    }

    if (mv <= 420U)
    {
        return SHARP_2Y0A21_OUT_OF_RANGE;
    }

    d_mm = 278600U / (mv - 420U);

    if ((d_mm < 100U) || (d_mm > 800U))
    {
        return SHARP_2Y0A21_OUT_OF_RANGE;
    }

    *distance_mm = d_mm;
    return SHARP_2Y0A21_OK;
}

SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadDistanceMm(SHARP_2Y0A21_Handle_t *h,
                                                  uint8_t samples,
                                                  uint32_t *distance_mm)
{
    uint16_t raw;
    uint32_t mv;
    SHARP_2Y0A21_Status_t status;

    if ((h == NULL) || (distance_mm == NULL))
    {
        return SHARP_2Y0A21_ERROR;
    }

    status = SHARP_2Y0A21_ReadRawAverage(h, samples, &raw);
    if (status != SHARP_2Y0A21_OK)
    {
        return status;
    }

    mv = SHARP_2Y0A21_RawToMilliVolts(h, raw);
    h->mv_last = mv;

    status = SHARP_2Y0A21_MilliVoltsToDistanceMm(mv, distance_mm);
    if (status == SHARP_2Y0A21_OK)
    {
        h->distance_mm_last = *distance_mm;
    }

    return status;
}


