/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\adc_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:45
 */

#include "adc_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_ADC)

#include "const.h"

uint32_t Adc_valueBuf[30];  // Adc data array
float Adc_decodeBuf[30];    // Adc decode data

/**
 * @brief      Adc peripheral initialization
 * @param      NULL
 * @retval     NULL
 */
void Adc_Init() {
    	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);						//Adc calibration
			HAL_Delay(10);
    	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&Adc_valueBuf, 1);		//start Adc DMA,Get the first group data.
}

/**
 * @brief      Get Adc data
 * @param      hadc1 : adc handle
 * @param      ADC_valueBuf : adc_value array
 * @retval     NULL
 */
void Adc_GetData() {
    //HAL_ADC_Start_DMA(&hadc2, (uint32_t*)&Adc_valueBuf, 10);
}

/**
 * @brief      Decode Adc data
 * @param      NULL
 * @retval     NULL
 */
void Adc_Decode() {
	Adc_decodeBuf[0] = (float)Adc_valueBuf[0] / 65535.0f * 11.0f * 2.9f;  // adc decode 24V AVCC
}

#endif
