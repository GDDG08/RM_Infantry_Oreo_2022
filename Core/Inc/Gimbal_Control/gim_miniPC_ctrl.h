/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_miniPC_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-27 15:19:09
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-01 20:55:13
 */

#ifndef GIM_MINIPC_CTRL_H
#define GIM_MINIPC_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_MINIPC)

#include "minipc_periph.h"
#include "math_alg.h"
#include "filter_alg.h"
// #include "kalman_alg.h"

typedef enum {
    MiniPC_TARGET_LOST = 0u,
    MiniPC_TARGET_FOLLOWING = 1u
} MiniPC_TargetFollowModeEnum;

typedef enum {
    MiniPC_ARMOR = 0u,
    MiniPC_BIG_BUFF = 1u,
    MiniPC_SMALL_BUFF = 2u,
    MiniPC_SENTRY = 11u
} MiniPC_AutoAimModeEnum;

typedef enum {
    MiniPC_ABSOLUTE = 0u,
    MiniPC_RELATIVE = 1u
} MiniPC_AutoControlModeEnum;

typedef struct {
    float pitch;
    float yaw;
} MiniPC_OffsetTypeDef;

typedef struct {
    uint8_t enable_aim_output;

    float yaw_angle;
    float pitch_angle;
    float distance;

    float yaw_ref_filtered;
    float pitch_ref_filtered;
    float distance_filtered;

    Filter_LowPassParamTypeDef yaw_fil_param;
    Filter_LowPassTypeDef yaw_fil;

    Filter_LowPassParamTypeDef pitch_fil_param;
    Filter_LowPassTypeDef pitch_fil;

    Filter_LowPassParamTypeDef distance_fil_param;
    Filter_LowPassTypeDef distance_fil;

    uint32_t get_target_time;
    MiniPC_AutoAimModeEnum aim_mode;
    MiniPC_AutoControlModeEnum control_mode;
    MiniPC_TargetFollowModeEnum target_state;
    MiniPC_OffsetTypeDef output_offset;
} MiniPC_MiniPCControlTypeDef;

MiniPC_MiniPCControlTypeDef* MiniPC_GetMiniPCControlDataPtr(void);
MiniPC_MiniPCControlTypeDef* MiniPC_GetMiniPCDecodeDataPtr(void);
void MiniPC_ControlInit(void);
void MiniPC_InitOffsetParam(float para[4][2]);
void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode);
void MiniPC_CalcAutoAim(void);
void MiniPC_SetFollowMode(MiniPC_TargetFollowModeEnum mode);
void MiniPC_SetTargetFollowMode(void);
void MiniPC_UpdateControlData(void);
void MiniPC_SetAutoAimOutput(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
