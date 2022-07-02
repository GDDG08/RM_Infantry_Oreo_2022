/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_remote_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-01 16:09:24
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

typedef struct {
    uint8_t pending;
    Chassis_ChassisStateEnum chassis_state;
    float yaw_angle_offset;
} Remote_RemoteControlTypeDef;

void Remote_RemotrControlInit(void);
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr(void);
void Remote_ControlCom(void);
void Remote_MouseShooterModeSet(void);
void Remote_RemoteShooterModeSet(void);
void Remote_RemoteProcess(void);
void Remote_KeyMouseProcess(void);
void Remote_ChangeChassisState(uint8_t chassis_mode);
void Remote_SwitchGryoState(void);
void Remote_SwitchAutoAimState(uint8_t mode);
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
