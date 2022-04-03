/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Peripheral\sensor_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:25
 */

#include "sensor_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_SENSOR)

#include "const.h"

Sen_CAPBasisValueTypeDef Sen_basisValue;
Sen_PowerValueTypeDef Sen_powerValue;
Sen_FilterTypeDef Sen_Filter;

/**
 * @brief      Gets the pointer to the sensor power data object
 * @param      NULL
 * @retval     Pointer to sensor power data object
 */
Sen_PowerValueTypeDef* Sen_GetPowerDataPtr() {
    return &Sen_powerValue;
}

/**
 * @brief      Gets the pointer to the sensor basis data object
 * @param      NULL
 * @retval     Pointer to sensor basis data object
 */
Sen_CAPBasisValueTypeDef* Sen_GetBasisDataPtr() {
    return &Sen_basisValue;
}

/**
 * @brief      Sen initialization
 * @param      NULL
 * @retval     NULL
 */
void Sen_Init() {
    Adc_Init();
    Filter_LowPassInit(0.1, &Sen_Filter.CapVoltageParam);
    Filter_LowPassInit(0.1, &Sen_Filter.ChasissVoltageParam);
    Filter_LowPassInit(0.1, &Sen_Filter.VccVoltageParam);
}

/**
 * @brief      Filtering data
 * @param      Sen_Filter: Filter structure
 * @param      Sen_basisValue: Filter data
 * @retval     NULL
 */
void Sen_Filtering(Sen_FilterTypeDef* Sen_Filter, Sen_CAPBasisValueTypeDef* Sen_basisValue) {
    // ADC Filter
    Sen_basisValue->CapVoltage = Filter_LowPass(Sen_basisValue->CapVoltage, &Sen_Filter->CapVoltageParam, &Sen_Filter->CapVoltage);
    Sen_basisValue->VccVoltage = Filter_LowPass(Sen_basisValue->VccVoltage, &Sen_Filter->VccVoltageParam, &Sen_Filter->VccVoltage);
    Sen_basisValue->ChasissVoltage = Filter_LowPass(Sen_basisValue->ChasissVoltage, &Sen_Filter->ChasissVoltageParam, &Sen_Filter->ChasissVoltage);
}

/**
 * @brief      Decode Sen data
 * @param      NULL
 * @retval     NULL
 */
void Sensor_Decode() {
    Sen_CAPBasisValueTypeDef* basisValue = Sen_GetBasisDataPtr();
    Sen_PowerValueTypeDef* powerValue = Sen_GetPowerDataPtr();

    Adc_GetData();
    Adc_Decode();

    /*ADC_decodeBuf What a fool to deal with */
    basisValue->VccVoltage = Adc_decodeBuf[0] * Const_ADC_V_VGAIN;
    basisValue->ChasissVoltage = Adc_decodeBuf[2] * Const_ADC_V_VGAIN;
    basisValue->CapVoltage = Adc_decodeBuf[1] * Const_ADC_V_VGAIN;

    Sen_Filtering(&Sen_Filter, basisValue);

    powerValue->CapRestEnergy = 0.5f * Const_ADC_CapValue * basisValue->CapVoltage * basisValue->CapVoltage;
    powerValue->CapPercent = (uint8_t)(powerValue->CapRestEnergy * 100 / Const_ADC_Cap_TotalEnergy);
}

#endif
