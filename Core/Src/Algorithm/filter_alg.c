/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Src\Algorithm\filter_alg.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 21:12:23
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-19 16:53:41
 */

#include "filter_alg.h"

#include "const.h"

/**
 * @brief      low_pass_filter_init
 * @param      param :Low pass filter param
 * @param      Filter_LowPassParamTypeDef: low pass filter param stuct
 * @retval     filtering result
 */
void Filter_LowPassInit(float param, Filter_LowPassParamTypeDef* pparam) {
    pparam->filt_para = param;
    pparam->last_tick = 0;
}

/**
 * @brief      low_pass_filter
 * @param      val: inital value
 * @param      pparam : the low pass filter param sturct
 * @param      filt: low pass filter sturct
 * @retval     filtering result
 */
float Filter_LowPass(float val, Filter_LowPassParamTypeDef* pparam, Filter_LowPassTypeDef* filt) {
    // calculate cut off frequence
    uint32_t period = HAL_GetTick() - pparam->last_tick;
    pparam->last_tick = HAL_GetTick();
    if ((pparam->filt_para > 0) && (pparam->filt_para <= 1)) {
        filt->filted_val = pparam->filt_para * val + (1 - pparam->filt_para) * filt->filted_last_val;
        filt->filted_last_val = filt->filted_val;
        if (period > 0)
            pparam->cut_off_frq = pparam->filt_para / (2 * PI * (float)period * 0.001f);
        pparam->calc_frq = 1000 / (float)period;
        return filt->filted_val;
    } else
        return val;
}

/**
 * @brief      average_filter_init
 * @param      filt :average_filter sturct
 * @param      length  :buff len
 * @retval     filtering result
 */
void Filter_AverInit(Filter_WindowTypeDef* filt, uint8_t length) {
    filt->length = length;
    filt->sum = 0;
}

/**
 * @brief      average_filter
 * @param      val  :inital value
 * @param      filt :average_filter sturct
 * @retval     filtering result
 */
float Filter_Aver(float val, Filter_WindowTypeDef* filt) {
    filt->sum = 0;
    for (int i = 0; i < filt->length - 1; i++) {
        filt->val[i] = filt->val[i + 1];
    }
    filt->val[filt->length - 1] = val;
    for (int i = 0; i < filt->length; i++) {
        filt->sum += filt->val[i];
    }
    return filt->sum / filt->length;
}

/**
 * @brief      bessel_filter
 * @param      val  :inital value
 * @param      filt :bessel_filter sturct
 * @retval     filtering result
 */
float Filter_Bessel(float val, Filter_Bessel_TypeDef* filt) {
    for (int i = 3; i > 0; i--) {
        filt->xbuf[i] = filt->xbuf[i - 1];
        filt->ybuf[i] = filt->ybuf[i - 1];
    }
    filt->xbuf[0] = val;
    filt->ybuf[0] = 0.0001507 * filt->xbuf[1] + 0.0005675 * filt->xbuf[2] + 0.0001336 * filt->xbuf[3] + 2.765 * filt->ybuf[1] - 2.552 * filt->ybuf[2] + 0.7866 * filt->ybuf[3];
    filt->filted_val = filt->ybuf[0];
    return filt->filted_val;
}
