/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Algorithm\adrc_alg.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:59:16
 */

#include "adrc_alg.h"

/**
 * @brief      ADRC parameter initialization
 * @param      fhan_input1 :Fhan function input 1
 * @param      fhan_input2 :Fhan function input 2
 * @param      adrc_unit :Initialize array
 * @retval     NULL
 */
void ADRC_Init(ADRC_FhanDataTypeDef* fhan_input1, ADRC_FhanDataTypeDef* fhan_input2, float adrc_unit[][15]) {
    fhan_input1->r = adrc_unit[0][0];
    fhan_input1->h = adrc_unit[0][1];
    fhan_input1->N0 = (uint16_t)(adrc_unit[0][2]);
    fhan_input1->beta_01 = adrc_unit[0][3];
    fhan_input1->beta_02 = adrc_unit[0][4];
    fhan_input1->beta_03 = adrc_unit[0][5];
    fhan_input1->b0 = adrc_unit[0][6];
    fhan_input1->beta_0 = adrc_unit[0][7];
    fhan_input1->beta_1 = adrc_unit[0][8];
    fhan_input1->beta_2 = adrc_unit[0][9];
    fhan_input1->N1 = (uint16_t)(adrc_unit[0][10]);
    fhan_input1->c = adrc_unit[0][11];

    fhan_input1->alpha1 = adrc_unit[0][12];
    fhan_input1->alpha2 = adrc_unit[0][13];
    fhan_input1->zeta = adrc_unit[0][14];

    fhan_input2->r = adrc_unit[1][0];
    fhan_input2->h = adrc_unit[1][1];
    fhan_input2->N0 = (uint16_t)(adrc_unit[1][2]);
    fhan_input2->beta_01 = adrc_unit[1][3];
    fhan_input2->beta_02 = adrc_unit[1][4];
    fhan_input2->beta_03 = adrc_unit[1][5];
    fhan_input2->b0 = adrc_unit[1][6];
    fhan_input2->beta_0 = adrc_unit[1][7];
    fhan_input2->beta_1 = adrc_unit[1][8];
    fhan_input2->beta_2 = adrc_unit[1][9];
    fhan_input2->N1 = (uint16_t)(adrc_unit[1][10]);
    fhan_input2->c = adrc_unit[1][11];

    fhan_input2->alpha1 = adrc_unit[1][12];
    fhan_input2->alpha2 = adrc_unit[1][13];
    fhan_input2->zeta = adrc_unit[1][14];
}

/**
 * @brief      ADRC Fhan function
 * @param      fhan_input :Fhan function input
 * @param      expect_ADRC :adrc expect para
 * @retval     NULL
 */
void Fhan_ADRC(ADRC_FhanDataTypeDef* fhan_input, float expect_ADRC) {
    float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
    float x1_delta = 0;

    x1_delta = fhan_input->x1 - expect_ADRC;
    fhan_input->h0 = fhan_input->N0 * fhan_input->h;
    d = fhan_input->r * fhan_input->h0 * fhan_input->h0;  // d=rh^2;
    a0 = fhan_input->h0 * fhan_input->x2;                 // a0=h*x2
    y = x1_delta + a0;                                    // y=x1+a0
    a1 = sqrt(d * (d + 8 * fabs(y)));                     // a1=sqrt(d*(d+8*ABS(y))])
    a2 = a0 + Math_Sign(y) * (a1 - d) / 2;                // a2=a0+sign(y)*(a1-d)/2;
    a = (a0 + y) * Math_Fsg(y, d) + a2 * (1 - Math_Fsg(y, d));

    fhan_input->fh = -fhan_input->r * (a / d) * Math_Fsg(a, d) - fhan_input->r * Math_Sign(a) * (1 - Math_Fsg(a, d));
    fhan_input->x1 += fhan_input->h * fhan_input->x2;
    fhan_input->x2 += fhan_input->h * fhan_input->fh;
}

/**
 * @brief      ADRC extended state observer
 * @param      fhan_input :Fhan function input
 * @retval     NULL
 */
void ADRS_ESO(ADRC_FhanDataTypeDef* fhan_input) {
    fhan_input->e = fhan_input->z1 - fhan_input->y;

    fhan_input->fe = Math_Fal(fhan_input->e, 0.5, fhan_input->h);
    fhan_input->fe1 = Math_Fal(fhan_input->e, 0.25, fhan_input->h);

    fhan_input->z1 += fhan_input->h * (fhan_input->z2 - fhan_input->beta_01 * fhan_input->e);
    fhan_input->z2 += fhan_input->h * (fhan_input->z3 - fhan_input->beta_02 * fhan_input->fe + fhan_input->b0 * fhan_input->u);

    fhan_input->z3 += fhan_input->h * (-fhan_input->beta_03 * fhan_input->fe1);
}

/**
 * @brief      Nonlinear combination calculation of ADRC
 * @param      fhan_input :Fhan function input
 * @retval     NULL
 */
void ADRC_NolinearConbination(ADRC_FhanDataTypeDef* fhan_input) {
    float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
    float Sy = 0, Sa = 0;

    fhan_input->h1 = fhan_input->N1 * fhan_input->h;

    d = fhan_input->r * fhan_input->h1 * fhan_input->h1;
    a0 = fhan_input->h1 * fhan_input->c * fhan_input->e2;
    y = fhan_input->e1 + a0;
    a1 = sqrt(d * (d + 8 * fabs(y)));
    a2 = a0 + Math_Sign(y) * (a1 - d) / 2;

    Sy = Math_Fsg(y, d);
    a = (a0 + y - a2) * Sy + a2;
    Sa = Math_Fsg(a, d);
    fhan_input->u0 = -fhan_input->r * ((a / d) - Math_Sign(a)) * Sa - fhan_input->r * Math_Sign(a);

    a = (a0 + y) * Math_Fsg(y, d) + a2 * (1 - Math_Fsg(y, d));

    fhan_input->fh = -fhan_input->r * (a / d) * Math_Fsg(a, d) - fhan_input->r * Math_Sign(a) * (1 - Math_Fsg(a, d));
}

/**
 * @brief      ADRC calculation function
 * @param      fhan_input :Fhan function input
 * @param      expect_ADRC :Expected value
 * @param      feedback_ADRC :Feedback value
 * @retval     Output with disturbance compensation
 */
float ADRC_Calc(ADRC_FhanDataTypeDef* fhan_input, float expect_ADRC, float feedback_ADRC) {
    Fhan_ADRC(fhan_input, expect_ADRC);
    fhan_input->y = feedback_ADRC;

    ADRS_ESO(fhan_input);
    fhan_input->e0 += fhan_input->e1 * fhan_input->h;
    fhan_input->e1 = fhan_input->x1 - fhan_input->z1;
    fhan_input->e2 = fhan_input->x2 - fhan_input->z2;

    ADRC_NolinearConbination(fhan_input);
    fhan_input->u = fhan_input->u0;
    LimitMaxMin(fhan_input->u, 200, -200);
    return fhan_input->u;
}
