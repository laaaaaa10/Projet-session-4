/*
 * sharp_2y0a21.h
 *
 *  Created on: 16 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef BSP_INC_SHARP_2Y0A21_H_
#define BSP_INC_SHARP_2Y0A21_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    SHARP_2Y0A21_OK = 0,
    SHARP_2Y0A21_ERROR,
    SHARP_2Y0A21_OUT_OF_RANGE
} SHARP_2Y0A21_Status_t;

typedef struct
{
    ADC_HandleTypeDef *hadc;
    uint32_t adc_channel;

    uint32_t vref_mv;
    uint16_t adc_max;

    uint16_t raw_last;
    uint32_t mv_last;
    uint32_t distance_mm_last;
} SHARP_2Y0A21_Handle_t;

void SHARP_2Y0A21_Init(SHARP_2Y0A21_Handle_t *h,
                       ADC_HandleTypeDef *hadc,
                       uint32_t adc_channel,
                       uint32_t vref_mv,
                       uint16_t adc_max);

SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadRaw(SHARP_2Y0A21_Handle_t *h, uint16_t *raw);
SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadRawAverage(SHARP_2Y0A21_Handle_t *h,
                                                  uint8_t samples,
                                                  uint16_t *raw);

uint32_t SHARP_2Y0A21_RawToMilliVolts(const SHARP_2Y0A21_Handle_t *h, uint16_t raw);

SHARP_2Y0A21_Status_t SHARP_2Y0A21_MilliVoltsToDistanceMm(uint32_t mv, uint32_t *distance_mm);

SHARP_2Y0A21_Status_t SHARP_2Y0A21_ReadDistanceMm(SHARP_2Y0A21_Handle_t *h,
                                                  uint8_t samples,
                                                  uint32_t *distance_mm);


#endif /* BSP_INC_SHARP_2Y0A21_H_ */
