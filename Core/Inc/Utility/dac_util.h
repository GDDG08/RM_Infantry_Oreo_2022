/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Utility\dac_util.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:58:35
 */

#ifndef DAC_UTIL_H
#define DAC_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_DAC)

#include "dac.h"

typedef enum {
    DAC_OFF = 0,
    DAC_ON = 1
} Dac_DacStateEnum;

typedef struct {
    Dac_DacStateEnum state;
    DAC_HandleTypeDef* hdac;
    uint32_t ch;
    float value;
    uint32_t Dac_DecodeValue;
} Dac_DacHandleTypeDef;

extern Dac_DacHandleTypeDef CurrentDac;

void Dac_StopDac(void);
void Dac_SetCurrent(float value);
void Dac_StopDac(void);
void Dac_DecodeValue(void);
void Dac_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
