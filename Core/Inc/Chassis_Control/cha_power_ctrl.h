/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Chassis_Control\cha_power_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-08 16:38:29
 */

#ifndef CHA_POWER_CTRL_H
#define CHA_POWER_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_POWER)

#include "supercap_ctrl.h"
#include "const.h"
#include "motor_periph.h"
#include "referee_periph.h"
#include "math_alg.h"
#include "cha_chassis_ctrl.h"
#include "buscomm_ctrl.h"

typedef enum {
    Motor_State_Stop = 0,
    Motor_State_Starting = 1,
    Motor_State_Stable = 2,
    Motor_State_Down = 3
} Motor_MotorModeEnum;

typedef enum {
    POWER_UNLIMITED = 0X00,
    POWER_LIMIT = 0X01,
} Power_ControlModeMnum;

typedef enum {
    POWER_VER_CLASSISC = 0X00,
    POWER_VER_MODERN = 0X01,
} Power_ControlVerMnum;

typedef struct
{
    Power_ControlModeMnum PowerCtrl_State;  //功率控制状态
    Power_ControlVerMnum PowerCtrl_Ver;

    Motor_MotorGroupTypeDef* Mecanum_Chassis_Motor;

    PID_PIDTypeDef Mecanum_current_pid[4];
    PID_PIDTypeDef Power_pid;
    Motor_MotorModeEnum Mecanum_motor_mode[4];  //麦克纳姆轮系四个电机状态
    uint8_t ChassisStarting_flag;
    uint8_t ChassisDown_flag;
    float Mecanum_motor_current[4];                                //麦克纳姆轮系四个电机编码器电流值
    Filter_LowPassTypeDef Mecanum_MotorCurrent_lpf[4];             //电机编码器电流值滤波
    Filter_LowPassParamTypeDef Mecanum_MotorCurrent_lpf_param[4];  //电机编码器电流值滤波param
    Filter_LowPassTypeDef Mecanum_SpeedOutput_lpf[4];              //速度环输出低通滤波
    Filter_LowPassParamTypeDef Mecanum_SpeedOutput_lpf_param[4];   //速度环输出低通滤波param

    Filter_WindowTypeDef Mecanum_SpeedOutput_aft[4];  //速度环输出滑动均值滤波
    // float SpeedOutput_aftData[4][20];         //速度环输出滑动均值滤波数据存储

    float Power_scale;
    float Power_offset;
} PowerCtrl_Data_t;

PowerCtrl_Data_t* PowerCtrl_GetPowerDataPtr(void);
void Power_ForceChangePowerMode(Power_ControlModeMnum mode);
void PowerCtrl(void);
void Output_Control(void);
void ChassisSpeedPID_StartingAndDownCalc(PID_PIDTypeDef* pid, uint8_t motor_num);
void ChassisCurrentPID_StartingAndDownCalc(uint8_t motor_num);
void Motor_ModeControl(void);
PowerCtrl_Data_t* PowerCtrl_GetPowerDataPty(void);
void PowerCtrl_Init(uint8_t Ctrl_state, Motor_MotorGroupTypeDef* Mecanum);

// @OLD Version
typedef struct {
    float power_limit;
    float warning_power;
    float warning_power_buff;

    float no_judge_total_current_limit;
    float buffer_total_current_limit;
    float power_total_current_limit;

    float now_power;
    float now_power_buff;

    float total_current_limit;
    float total_current;
} Power_DataTypeDef;

extern Power_DataTypeDef Power_data;
// extern Power_ControlModeMnum Power_ControlMode;

void Power_InitPower(Power_ControlModeMnum mode);
Power_DataTypeDef* Power_GetPowerDataPty(void);
void Power_SetLimitPower(void);
// void Power_ForceChangePowerMode(Power_ControlModeMnum mode);
void Power_ChangePowerMode(Power_ControlModeMnum mode);
void Power_PowerControl(Motor_MotorGroupTypeDef* chassis);

#ifdef __cplusplus
}
#endif

#endif

#endif
