/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_gimbal_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-10-29 20:20:13
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-23 21:38:57
 */

#ifndef GIM_GIMBAL_CTRL_H
#define GIM_GIMBAL_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_GIMBAL_GIM)

#include "pid_alg.h"
#include "motor_periph.h"
#include "bmi055_periph.h"
#include "bmi088_periph.h"
#include "hi22x_periph.h"
#include "remote_periph.h"

typedef enum {
    Gimbal_NOAUTO = 0u,
    Gimbal_ARMOR = 1u,
    Gimbal_IMU_DEBUG = 2u,
    Gimbal_BIG_ENERGY = 3u,
    Gimbal_SMALL_ENERGY = 4u
} Gimbal_ModeEnum;

typedef enum {
    GimbalYaw_MODE_NULL = 0u,
    GimbalYaw_MODE_NO_AUTO = 1u,
    GimbalYaw_MODE_ARMOR = 2u,
    GimbalYaw_MODE_IMU_DEBUG = 3u,
    GimbalYaw_MODE_BIG_ENERGY = 4u,
    GimbalYaw_MODE_SMALL_ENERGY = 5u
} GimbalYaw_GimbalYawModeEnum;

typedef struct {
    Gimbal_ModeEnum present_mode;
    Gimbal_ModeEnum last_mode;
    uint8_t mode_change_flag;
} Gimbal_ModeTypeDef;

typedef struct {
    float pitch_angle_ref;
    float yaw_angle_ref;
} Gimbal_AngleRefTypeDef;

typedef struct {
    uint8_t control_state;  // Whether to enable control 1 Yes 0 No
    uint8_t output_state;   // Whether to enable output 1 Yes 0 No

    float pitch_position_fdb;  // Gimbal pitch IMU angle feedback value
    float pitch_speed_fdb;     // Gimbal pitch IMU angular velocity feedback value

    Gimbal_AngleRefTypeDef angle;
    Gimbal_ModeTypeDef mode;
    GimbalYaw_GimbalYawModeEnum yaw_mode;
} Gimbal_GimbalTypeDef;

extern Motor_MotorParamTypeDef GimbalPitch_gimbalPitchMotorParamBigEnergy;
extern Motor_MotorParamTypeDef GimbalPitch_gimbalPitchMotorParamSmallEnergy;
extern Motor_MotorParamTypeDef GimbalPitch_gimbalPitchMotorParamArmor;
extern Motor_MotorParamTypeDef GimbalPitch_gimbalPitchMotorParamIMUDebug;
extern Motor_MotorParamTypeDef GimbalPitch_gimbalPitchMotorParamNoAuto;

extern Gimbal_GimbalTypeDef Gimbal_gambalControlData;
extern float AutoControl_offset_pitch;
extern float AutoControl_offset_yaw;

Gimbal_GimbalTypeDef*
Gimbal_GetGimbalControlPtr(void);

void Gimbal_Task(void const* argument);
void Gimbal_InitOffset(void);
void GimbalPitch_Output(void);
void Gimbal_CtrlYaw(void);
void Gimbal_CtrlPitch(void);
void Gimbal_ChangeMode(Gimbal_ModeEnum mode);
float Gimbal_LimitPitch(float ref);
void Gimbal_SetPitchRef(float ref);
void Gimbal_SetPitchAutoRef(float ref);
float Gimbal_LimitYaw(float ref);
void Gimbal_SetYawRef(float ref);
void Gimbal_SetYawRefDelta(float ref);
void Gimbal_SetYawAutoRef(float ref);

#endif

#ifdef __cplusplus
}
#endif

#endif
