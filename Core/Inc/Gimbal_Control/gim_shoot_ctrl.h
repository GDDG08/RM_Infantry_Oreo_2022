/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_shoot_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-01 22:10:58
 */

#ifndef GIM_SHOOT_CTRL_H
#define GIM_SHOOT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_SHOOTER)

#include "pid_alg.h"
#include "math_alg.h"
#include "motor_periph.h"
#include "gpio_util.h"
#include "flash_util.h"

#define SHOOT_15M_SPEED_ADDRESS ADDR_FLASH_SECTOR_8
#define SHOOT_18M_SPEED_ADDRESS ADDR_FLASH_SECTOR_9
#define SHOOT_30M_SPEED_ADDRESS ADDR_FLASH_SECTOR_10

typedef enum {
    Feeder_NULL = 0u,
    Feeder_SINGLE = 1u,
    Feeder_FAST_CONTINUE = 2u,
    Feeder_LOW_CONTINUE = 3u,
    Feeder_LOCKED_ROTOR = 4u,
    Feeder_REFEREE = 5u,
    Feeder_FINISH = 6u
} Shoot_FeederModeEnum;

typedef enum {
    Shoot_NULL = 0u,
    Shoot_FAST = 1u,
    Shoot_SLOW = 2u,
    Shoot_REFEREE = 3u
} Shoot_ShooterModeEnum;

typedef struct {
    float left_shoot_speed;
    float right_shoot_speed;
    float feeder_shoot_speed;
} Shoot_ShootSpeedTypeDef;

typedef struct {
    uint32_t speed_15mm_offset;
    uint32_t speed_18mm_offset;
    uint32_t speed_30mm_offset;
} Shoot_ShootSpeedOffsetFlashTypeDef;

typedef struct {
    float speed_15mm_offset;
    float speed_18mm_offset;
    float speed_30mm_offset;
} Shoot_ShootSpeedOffsetTypeDef;

typedef struct {
    float shooter_17mm_cooling_heat;
    float shooter_17mm_cooling_rate;
    float shooter_17mm_heat_remain;

    float current_speed;
    uint8_t current_pidnum;

    uint16_t heat_tracking;
} Shooter_HeatCtrlTypeDef;

typedef struct {
    uint8_t shooter_control;
    Shoot_ShooterModeEnum shooter_mode;
    Shoot_FeederModeEnum feeder_mode;
    Shoot_FeederModeEnum last_feeder_mode;

    uint8_t single_shoot_done;
    uint8_t change_shooter_mode_complete;

    Shoot_ShootSpeedTypeDef shoot_speed;
    Shoot_ShootSpeedOffsetTypeDef shoot_speed_offset;
    Shoot_ShootSpeedOffsetFlashTypeDef speed_offset_flash;

    float shooter_speed_15mpers;
    float shooter_speed_18mpers;
    float shooter_speed_30mpers;

    float last_shoot_speed_ref;
    uint8_t slope_direction;
    float slope_output;
    float ref_output;
    float slope_step;
    float dertaRef;
    float lastfdb;

    float speed_limit;

    Shooter_HeatCtrlTypeDef heat_ctrl;
} Shoot_StatusTypeDef;

extern Motor_MotorParamTypeDef Shooter_shooterLeftMotorParam;
extern Motor_MotorParamTypeDef Shooter_shooterRightMotorParam;
extern Motor_MotorParamTypeDef Shooter_feederMotorParam;

extern Shoot_StatusTypeDef Shooter_ShooterControl;

void Shoot_Task(void const* argument);
void Shooter_InitShooter(void);
void Shooter_SpeedOffsetFlashInit(void);
Shoot_StatusTypeDef* Shooter_GetShooterControlPtr(void);
void Shooter_ModifySpeedOffset(int8_t multiple);
void Shooter_ChangeShooterMode(Shoot_ShooterModeEnum mode);
void Shooter_ChangeFeederMode(Shoot_FeederModeEnum mode);
float Shooter_GetShootSpeedOffset(void);
void Shooter_Control(void);
void Shooter_InitShooterMotor(void);
void Shooter_HeatCtrlInit(void);
float Shooter_GetRefereeSpeed(void);
void Shooter_UpdataControlData(void);
void Shooter_SetFeederSpeed(float speed);
void Shooter_SetShooterSpeed(float speed);
void Shooter_ForceChangeFeederMode(Shoot_FeederModeEnum mode);
void Shooter_FeederMotorLockedJudge(void);
void Shooter_MotorLockedHandle(void);
void Shooter_AngleCorrect(void);
void Shooter_RealAngleCorrect(void);
uint8_t Shooter_HeatCtrl(void);
void Shooter_Overspeedtest(void);
void Shooter_CalcRef(void);
void Shooter_ShootControl(void);
void Shooter_SingleShootCtrl(void);
void Shooter_SingleShootReset(void);
void Shooter_FeederControl(void);
void Shooter_ShooterMotorOutput(void);
float* Shooter_GetRefereeSpeedPtr(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
