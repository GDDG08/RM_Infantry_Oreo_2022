/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Chassis_Control\cha_gimbal_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 10:27:08
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:49:42
 */

#ifndef CHA_GIMBAL_CTRL_H
#define CHA_GIMBAL_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_GIMBAL_YAW_CHA)

#include "motor_periph.h"
#include "filter_alg.h"
#include "math_alg.h"
#include "cha_chassis_ctrl.h"

typedef enum {
    GimbalYaw_MODE_NULL = 0u,
    GimbalYaw_MODE_NO_AUTO = 1u,
    GimbalYaw_MODE_ARMOR = 2u,
    GimbalYaw_MODE_IMU_DEBUG = 3u,
    GimbalYaw_MODE_BIG_ENERGY = 4u,
    GimbalYaw_MODE_SMALL_ENERGY = 5u
} GimbalYaw_GimbalYawModeEnum;

typedef struct {
    float yaw_ref;                 // Gimbal Yaw angle target value
    float yaw_position_fdb;        // Gimbal Yaw IMU angle feedback value
    float yaw_speed_fdb;           // Gimbal Yaw IMU angular velocity feedback value
    uint8_t yaw_ref_limit_status;  // Gimbal Yaw limit status
    uint8_t yaw_count;

    uint8_t mode_changed;                         // Whether the gimbal Yaw mode is changed 1 Yes 0 No
    GimbalYaw_GimbalYawModeEnum mode, last_mode;  // Gimbal Yaw mode, the mode before Gimbal Yaw

    uint8_t control_state;  // Whether to enable control 1 Yes 0 No
    uint8_t output_state;   // Whether to enable output 1 Yes 0 No
    uint8_t pending_state;  // Gimbal Yaw Occupy Lock 1 Yes 0 No

    Filter_LowPassParamTypeDef ref_fil_param;
    Filter_LowPassTypeDef ref_fil;
} GimbalYaw_GimbalYawTypeDef;

extern Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamBigEnergy;
extern Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamSmallEnergy;
extern Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamArmor;
extern Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamIMUDebug;
extern Motor_MotorParamTypeDef GimbalYaw_gimbalYawMotorParamNoAuto;

extern GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

void Gimbal_Task(void const* argument);
void GimbalYaw_InitGimbalYaw(void);
void GimbalYaw_StartGimbalYawControlTimer(void);
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr(void);
void GimbalYaw_SetGimbalYawControlState(uint8_t state);
void GimbalYaw_SetGimbalYawOutputState(uint8_t state);
void GimbalYaw_SetMode(GimbalYaw_GimbalYawModeEnum mode);
void GimbalYaw_SetEncoderFdb(void);
void GimbalYaw_SetYawRef(float yaw_ref);
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb);
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb);
void GimbalYaw_Control(void);
void GimbalYaw_Output(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
