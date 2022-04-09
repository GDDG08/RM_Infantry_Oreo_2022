/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\dac_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:56:15
 */

#include "dac_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_DAC)

#include "const.h"

Dac_DacHandleTypeDef CurrentDac;
DAC_HandleTypeDef* Current_Dac_HANDLER = &hdac;

/**
 * @brief      DAC initialization
 * @param      NULL
 * @retval     NULL
 */
void Dac_Init() {
    // Initialization related parameters
    CurrentDac.state = DAC_OFF;
    CurrentDac.ch = DAC_CHANNEL_1;
    CurrentDac.Dac_DecodeValue = 0;
    CurrentDac.hdac = Current_Dac_HANDLER;
    CurrentDac.value = 0;

    // Close DAC output after initialization
    HAL_DAC_Stop(CurrentDac.hdac, CurrentDac.ch);
}

/**
 * @brief      Turn on DAC and DMA
 * @param      value :Set current value(unit: A)
 * @retval     NULL
 */
void Dac_SetCurrent(float value) {
    // decoding
    CurrentDac.value = value;
    Dac_DecodeValue();

    // Set dma and dac
    HAL_DAC_SetValue(CurrentDac.hdac, CurrentDac.ch, DAC_ALIGN_12B_R, CurrentDac.Dac_DecodeValue);
    HAL_DAC_Start(CurrentDac.hdac, CurrentDac.ch);
    CurrentDac.state = DAC_ON;
}

/**
 * @brief      Close DAC
 * @param      NULL
 * @retval     NULL
 */
void Dac_StopDAC() {
    HAL_DAC_Stop(CurrentDac.hdac, CurrentDac.ch);
    HAL_DAC_Stop_DMA(CurrentDac.hdac, CurrentDac.ch);
    CurrentDac.state = DAC_OFF;
}

/**
 * @brief      Calculate DAC set value
 * @param      NULL
 * @retval     NULL
 */
void Dac_DecodeValue() {
    float voltage = CurrentDac.value * Const_DAC_DetectRES * Const_DAC_GAIN * 4.3f;
    if (voltage >= 3.1f) {
        voltage = 3.1f;
    }
    float decode = voltage * 4096 / 3.3f;
    CurrentDac.Dac_DecodeValue = decode;
}

#endif
