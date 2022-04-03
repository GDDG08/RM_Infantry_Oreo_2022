/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\SuperCap_Control\supercap_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-20 12:00:45
 */

#ifndef SUPERCAP_CTRL_H
#define SUPERCAP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#include "buscomm_ctrl.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
#include "cha_power_ctrl.h"

typedef struct {
    uint8_t SuperCap_State;  //电容反馈在线状态

    uint8_t cap_mode_Remote;  // 电容开关  分为遥控器控制电容、堵转电容（暂未用到）与起步电容
    uint8_t cap_mode_Stall;
    uint8_t cap_mode_Starting;
    uint16_t starting_time;

    uint8_t cap_boost_mode;  //电容升压开关

    uint8_t cap_state;  //这个暂时还没改动 与UI绘制有关 之后会改

    float Sum_PowerReally;    //底盘总功率
    float Sum_CurrentReally;  //底盘总电流
    float Chassis_voltage;    //底盘电压

    uint32_t last_update_time;  //上次更新时间
} CAP_CtrlDataTypeDef;

void SuperCap_Task(void const* argument);
CAP_CtrlDataTypeDef* Cap_GetCapDataPtr(void);
void Cap_Init(void);
void Cap_ResetCapData(void);
void Cap_Update(void);

#elif __FN_IF_ENABLE(__FN_SUPER_CAP)

#include "sensor_periph.h"
#include "led_periph.h"
#include "dac_util.h"
#include "gpio_util.h"

typedef enum {
    Power_PATH_NULL = 0,
    Power_PATH_REFEREE = 1,
    Power_PATH_CAP = 2
} POWER_PathEnum;

typedef enum {
    CAP_MODE_OFF = 0,  // Super cap off
    CAP_MODE_ON = 1,   // Super cap is on
    CAP_MODE_ERROR = 2
} CAP_StateEnum;

typedef enum {
    CAP_CHARGE_OFF = 0,
    CAP_CHARGE_ON = 1
} CAP_ChargeStateEnum;

typedef struct {
    float power_limit;

    CAP_ChargeStateEnum charge_state;
    POWER_PathEnum power_path;
    CAP_StateEnum cap_state;
} CAP_ControlValueTypeDef;

extern CAP_ControlValueTypeDef Cap_ControlState;

void SuperCap_Task(void const* argument);
void Cap_SetChargeCurrent(float current);
void Cap_Init(void);
CAP_ControlValueTypeDef* Cap_GetCapControlPtr(void);
void Cap_JudgeCapState(void);
void Cap_CapCharge(void);
void Cap_ChangePowerPath(POWER_PathEnum path);
void Cap_Control(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
