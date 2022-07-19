/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_remote_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-19 10:18:26
 */

#ifndef GIM_REMOTE_CTRL_H
#define GIM_REMOTE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_REMOTE)

#include "remote_periph.h"
#include "servo_periph.h"
#include "math_alg.h"

typedef enum {
    Chassis_OFF = 0,
    Chassis_ON = 1
} Chassis_ChassisStateEnum;

typedef enum {
    AutoAim_ARMOR = 0u,
    AutoAim_BIG_BUFF,
    AutoAim_SMALL_BUFF,
    AutoAim_SENTRY,
    AutoAim_ARMOR_TEMP,
    AutoAim_GIMBAL_DEBUG
} Remote_AutoAimModeEnum;

typedef struct {
    uint8_t pending;
    Chassis_ChassisStateEnum chassis_state;
    float yaw_angle_offset;
    Remote_AutoAimModeEnum aim_mode;
    uint8_t onAim;
    uint8_t onZoomCtrl;
} Remote_RemoteControlTypeDef;

extern uint8_t Remote_Mag_State;

void Remote_RemotrControlInit(void);
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr(void);
void Remote_ControlCom(void);
void Remote_MouseShooterModeSet(void);
void Remote_RemoteShooterModeSet(void);
void Remote_RemoteProcess(void);
void Remote_KeyMouseProcess(void);
void Remote_ChangeChassisState(uint8_t chassis_mode);
void Remote_SwitchMagState(void);
void Remote_SwitchGryoState(uint8_t);
void Remote_SwitchAutoAimMode(uint8_t mode);
void Remote_SetAutoAimState(uint8_t state);
void Remote_AutoAimModeControl(void);
uint8_t Remote_Gesturejudge(void);
void Remote_Gesture(void);
void Remote_GestureFunction_1(void);
void Remote_GestureFunction_2(void);
void Remote_GestureFunction_3(void);
void Remote_GestureFunction_4(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
