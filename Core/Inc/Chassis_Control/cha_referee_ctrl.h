/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Chassis_Control\cha_referee_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 10:27:08
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:49:54
 */

#ifndef CHA_REFEREE_CTRL_H
#define CHA_REFEREE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_CTRL_CHASSIS)

#include "stm32g4xx_hal.h"
void Referee_Setup(void);
void Referee_SetWidthMode(uint8_t mode);
void Referee_SetAimMode(uint8_t mode);
void Referee_SetCapState(uint8_t state);
void Referee_SetPitchAngle(float angle);
void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode);

#if __FN_IF_ENABLE(__FN_CTRL_REFEREE)

typedef struct {
    uint8_t width_mode, width_mode_last;  // 1 for gyro mode, 0 for normal mode
    uint8_t aim_mode, aim_mode_last;      // 0~2 correspond to 15,18,30 m/s bullet speed
    uint8_t auto_aim_mode, auto_aim_mode_last;
    uint8_t cha_mode, cha_mode_last;
    uint8_t cap_state;  // cap percent, 0 ~ 100
    float pitch_angle;
} Referee_DrawDataTypeDef;

void Referee_Task(void const* argument);

void Referee_SetupAimLine(void);
void Referee_UpdateAimLine(void);
void Referee_SetupCrosshair(void);
void Referee_UpdateCrosshair(void);
void Referee_SetupWidthMark(void);
void Referee_UpdateWidthMark(void);
void Referee_SetupCapState(void);
void Referee_UpdateCapState(void);
void Referee_SetupPitchMeter(void);
void Referee_UpdatePitchMeter(void);
void Referee_SetupModeDisplay(void);
void Referee_UpdateModeDisplay(void);
void Referee_SetupErrorDisplay(void);
void Referee_UpdateErrorDisplay(void);
void Referee_SetupAllString(void);
void Referee_Update(void);

#endif

#ifdef __cplusplus
}
#endif

#endif

#endif
