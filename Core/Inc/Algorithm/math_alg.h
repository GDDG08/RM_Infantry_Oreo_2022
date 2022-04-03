/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Algorithm\math_alg.h
 * @Descripttion : 
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:49:34
 */

#ifndef MATH_ALG_H
#define MATH_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "arm_math.h"

#define mat arm_matrix_instance_f32
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32
#define mat_inv arm_mat_inverse_f32

/**
 * @brief      Limit function
 * @param      input :Limited value
 * @param      max :Max limite value
 * @retval     NULL
 */
#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

/**
 * @brief      Maximum and minimum limiting
 * @param      input :Limited value
 * @param      max :Max limite value
 * @param      min :Min limite value
 * @retval     NULL
 */
#define LimitMaxMin(input, max, min) \
    {                                \
        if (input > max) {           \
            input = max;             \
        } else if (input < min) {    \
            input = min;             \
        }                            \
    }

typedef struct {
    float acc;
    float dec;
} Math_SlopeParamTypeDef;

float Math_RadToAngle(float rad);
float Math_Fal(float e, float alpha, float zeta);
int16_t Math_Fsg(float x, float d);
int16_t Math_Sign(float Input);
float Math_InvSqrt(float x);
void Math_InitSlopeParam(Math_SlopeParamTypeDef* pparam, float acc, float dec);
float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam);
float Math_Differential(float arr[], uint8_t order);

#ifdef __cplusplus
}
#endif

#endif
