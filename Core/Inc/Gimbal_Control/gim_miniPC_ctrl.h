/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_miniPC_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-27 15:19:09
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-04 22:41:30
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
    MiniPC_SENTRY = 3u,
    MiniPC_GIMBAL_DEBUG = 4u
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
    int8_t horizental;
    int8_t vertical;
} MiniPC_OffsetTuneCntTypeDef;

typedef struct {
    uint8_t enable_aim_output;

    float yaw_angle;
    float pitch_angle;
    float distance;

    float yaw_ref_filtered;
    float pitch_ref_filtered;
    float distance_filtered;

    float yaw_ref_calc;
    float pitch_ref_calc;

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
    MiniPC_OffsetTuneCntTypeDef vision_offset[5];
    uint8_t isChangeTarget;
} MiniPC_MiniPCControlTypeDef;

typedef enum {
    DISTANCE_ERROR = 0,
    DISTANCE_CLOSE = 1,
    DISTANCE_MIDDLE = 2,
    DISTANCE_FAR = 3,
} DistanceLevel_ModeType_e;

typedef struct {
    float x;
    float y;
    float z;
    float vx;
    float vz;
    float ax;
    float az;

    float last_vx;
    float last_vz;

    float x_predict_val;
    float z_predict_val;

    DistanceLevel_ModeType_e distance_level;
} TargetData_t;

extern TargetData_t TargetData_predict, TargetData_predict_re;

extern float vx_Chassis, vz_Chassis;  // z即为板通中的y

extern float kt_Predict_x, kk_Predict_x;
extern float k_Predict_z;
extern float k_Predict_Correction;
extern float k_Predict_Offset_y;

extern float Autoaim_OffsetPitch, Autoaim_OffsetYaw;
extern float Predict_Offest_x, Predict_Offset_y;

extern uint16_t TimeCountPredict, Target_x_SuddenStart_Count, Target_x_SuddenStop_Count, Target_z_SuddenStart_Count, Target_z_SuddenStop_Count;
extern uint8_t is_x_SuddenStart, is_x_SuddenStop, is_z_SuddenStart, is_z_SuddenStop;

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

void GimbalMove_AutoAimRef_Change(void);
void AutoAim_Para_Init(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
