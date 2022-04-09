/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Algorithm\ahrs_alg.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:49:07
 */

#ifndef AHRS_ALG_H
#define AHRS_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "math_alg.h"

void AHRS_Init(float quat[4]);
void AHRS_GetAngle(float q[4], float* yaw, float* pitch, float* roll);
void AHRS_MahonyUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_MahonyUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void AHRS_MadgwickUpdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRS_MadgwickUpdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

#ifdef __cplusplus
}
#endif

#endif
