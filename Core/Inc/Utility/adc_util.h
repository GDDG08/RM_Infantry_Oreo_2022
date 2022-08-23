/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Utility\adc_util.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-08-23 23:29:54
 */

#ifndef ADC_UTIL_H
#define ADC_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_ADC)

#include "adc.h"
#include "string.h"

extern uint32_t Adc_valueBuf[30];
extern float ADC_shooterVoltage;

void Adc_Init(void);
void Adc_GetData(void);
void Adc_Decode(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
