/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Algorithm\filter_alg.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-19 16:46:58
 */

#ifndef FILTER_ALG_H
#define FILTER_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "math_alg.h"

#define MAX_LENGTH 30

typedef struct {
    float cut_off_frq;
    float filt_para;
    float last_tick;
    float calc_frq;
} Filter_LowPassParamTypeDef;

typedef struct {
    float filted_val;
    float filted_last_val;
} Filter_LowPassTypeDef;

typedef struct {
    uint8_t length;
    float val[MAX_LENGTH];
    float sum;
} Filter_WindowTypeDef;

typedef struct {
    double ybuf[4];
    double xbuf[4];
    float filted_val;
} Filter_Bessel_TypeDef;

void Filter_LowPassInit(float param, Filter_LowPassParamTypeDef* pparam);
float Filter_LowPass(float val, Filter_LowPassParamTypeDef* fparam, Filter_LowPassTypeDef* filt);
void Filter_AverInit(Filter_WindowTypeDef* filt, uint8_t length);
float Filter_Aver(float val, Filter_WindowTypeDef* filt);
float Filter_Bessel(float val, Filter_Bessel_TypeDef* filt);

#ifdef __cplusplus
}
#endif

#endif
