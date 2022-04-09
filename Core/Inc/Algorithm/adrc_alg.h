/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Algorithm\adrc_alg.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:49:12
 */

#ifndef ADRC_ALG_H
#define ADRC_ALG_H

#include "math_alg.h"

typedef struct {
    float x1;
    float x2;
    float r;
    float h;
    uint16_t N0;

    float h0;
    float fh;

    float z1;
    float z2;
    float z3;
    float e;
    float y;
    float fe;
    float fe1;
    float beta_01;
    float beta_02;
    float beta_03;

    float e0;
    float e1;
    float e2;
    float u0;
    float u;

    float beta_0;
    float beta_1;
    float beta_2;  // u0=beta_1*e1+beta_2*e2+(beta_0*e0);

    float alpha1;  // u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
    float alpha2;  // 0<alpha1<1<alpha2
    float zeta;

    float h1;  // u0=-fhan(e1,e2,r,h1);
    uint16_t N1;

    float c;  // u0=-fhan(e1,c*e2*e2,r,h1);
    float b0;

} ADRC_FhanDataTypeDef;

void ADRC_Init(ADRC_FhanDataTypeDef* fhan_input1, ADRC_FhanDataTypeDef* fhan_input2, float adrc_unit[][15]);
void Fhan_ADRC(ADRC_FhanDataTypeDef* fhan_input, float expect_ADRC);
void ADRS_ESO(ADRC_FhanDataTypeDef* fhan_input);
void ADRC_NolinearConbination(ADRC_FhanDataTypeDef* fhan_input);
float ADRC_Calc(ADRC_FhanDataTypeDef* fhan_input, float expect_ADRC, float feedback_ADRC);

#endif
