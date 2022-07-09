/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Chassis_Control\cha_referee_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 10:27:08
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-06 22:36:01
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
void Referee_SetChassisMode(uint8_t mode);
void Referee_SetAutoAimMode(uint8_t mode);
void Referee_SetWidthMode(uint8_t mode);
void Referee_SetAimMode(uint8_t mode);
void Referee_SetCapState(uint8_t state,uint8_t cap_boost_mode_fnl,uint8_t cap_mode_fnl);
void Referee_SetPitchAngle(float angle);
void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode);
void Referee_SetShooterStateMode(uint8_t state);
void Referee_SetMagazineStateMode(uint8_t state);
void Referee_SetMinipc_Offset(uint8_t angle_x,uint8_t angle_y);         //以上函数都是在板通任务或底盘任务函数里引用，来给draw结构体赋值
		
#if __FN_IF_ENABLE(__FN_CTRL_REFEREE)

typedef struct {
    uint8_t width_mode, width_mode_last;  //  1 for gyro mode, 0 for normal mode 车宽线
    uint8_t aim_mode, aim_mode_last;      // 0~2 correspond to 15,18,30 m/s bullet speed  等级决定弹速决定瞄准线类型

    uint8_t cha_mode, cha_mode_last;   //底盘模式
    uint8_t cap_state;  // cap percent, 0 ~ 100
	uint8_t cap_state_last;
	    uint8_t cap_mode_fnl,cap_mode_fnl_last;
    uint8_t cap_boost_mode_fnl,cap_boost_mode_fnl_last;
    float pitch_angle;
	float pitch_angle_last;

    uint8_t magazine_state,magazine_state_last;   //弹舱盖
    uint8_t shooter_state,shooter_state_last;//摩擦轮
    uint8_t minipc_mode,minipc_mode_last; //自瞄模式
    uint8_t minipc_target_id;
    int8_t minipc_offset_horizental,minipc_offset_horizental_last;      
    int8_t minipc_offset_vertical,minipc_offset_vertical_last;      // XY偏置
} Referee_DrawDataTypeDef;

void Referee_Task(void const* argument);
//以下Setup函数都是固定的不会改变的字符，在Referee_Setup()里集中调用，而Referee_Setup()在全车初始化函数里调用，因为字符太多画出来要占太多时间所以放在初始化
//而Update函数则是绘画需要刷新的图形，例如瞄准线、数值数据、状态数据
void Referee_SetupAimLine(void);
void Referee_UpdateAimLine(void);  //瞄准线绘画函数
void Referee_SetupCrosshair(void);
void Referee_UpdateCrosshair(void);  //中央准星圆圈绘画函数
void Referee_SetupWidthMark(void);
void Referee_UpdateWidthMark(void);  //车宽线绘画函数
void Referee_SetupCapState(void);
void Referee_UpdateCapState(void);   //电容状态绘画函数
void Referee_SetupPitchMeter(void);
void Referee_UpdatePitchMeter(void);  //Pitch角度绘画函数
void Referee_SetupModeDisplay(void);  // 绘画自瞄模式和底盘模式的固定字符
void Referee_UpdateMinipcModeDisplay(void);//更新自瞄状态
void Referee_UpdateChassisModeDisplay(void); //更新底盘状态

void Referee_UpdateErrorDisplay(void);
void Referee_SetupAllString(void);  //以上三函数暂未用上

void Referee_SetupMagazineState(void);
void Referee_UpdateMagazineState(void);   //弹舱盖绘画函数
void Referee_SetupShooterState(void);
void Referee_UpdateShooterState(void);   //摩擦轮状态绘画函数
void Referee_SetupMinipcOffset_x(void);
void Referee_UpdateMinipcOffset_x(void);
void Referee_SetupMinipcOffset_y(void);
void Referee_UpdateMinipcOffset_y(void);  // x,y偏置绘画函数

void Referee_Update(void);    //总更新函数

#endif

#ifdef __cplusplus
}
#endif

#endif

#endif
