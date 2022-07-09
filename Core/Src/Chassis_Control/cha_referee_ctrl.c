/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Chassis_Control\cha_referee_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : HUHAO
 * @LastEditTime : 2022-07-06 22:35:49
 */

#include "cha_referee_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_CHASSIS)

#include "referee_periph.h"
#include "cmsis_os.h"

#if !__FN_IF_ENABLE(__FN_CTRL_REFEREE)
void Referee_SetWidthMode(uint8_t mode) {}

void Referee_SetAimMode(uint8_t mode) {}

void Referee_SetCapState(uint8_t state) {}

void Referee_SetPitchAngle(float angle) {}

void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode) {}

void Referee_Setup() {}

#else

#define REFEREE_TASK_PERIOD 50

uint8_t referee_setup_flag = 0;

/**
 * @brief          Referee draw task
 * @param          NULL
 * @retval         NULL
 */
void Referee_Task(void const* argument) {
    for (;;) {
        while (!GLOBAL_INIT_FLAG) {
            osDelay(1);
        }
        if (referee_setup_flag == 1)
            Referee_Update();
        osDelay(REFEREE_TASK_PERIOD);
    }
}

/********** Drawing Constants **********/

// ����ͼ�㣺ͼ��0 ~ 9����ͼ���ڸǵ�ͼ��
// ���ھ������µķ�ͼ�㹦�ܣ�����ǰ��ͼ��ʹ��3������ͼ��ʹ��2
// ���������ڲ������ڵ�������½���ʹ��ͼ��2

// �������꣺���½�Ϊ (0, 0)��ˮƽ����Ϊ X����ֱ����Ϊ Y

const uint8_t AIM_LINE_LAYER = 2;
const Draw_Color AIM_LINE_COLOR = Draw_COLOR_GREEN;
const uint8_t AIM_LINE_LINE_MODE = 3;
const uint8_t AIM_LINE_LINE_NUM = 3 + 1;
const uint16_t AIM_LINES[AIM_LINE_LINE_MODE][AIM_LINE_LINE_NUM][6] = {
    // ID, Width, X1, Y1, X2, Y2
    {
        // Mode 0: 15 m/s
        {0x101, 2, 960, 500, 960, 620},   // Vertical Line
        {0x102, 4, 910, 610, 1010, 610},  // Horizontal Line 1
        {0x103, 2, 820, 560, 1100, 560},  // Horizontal Line 2
        {0x104, 2, 910, 510, 1010, 510}   // Horizontal Line 3
    },
    {
        // Mode 1: 18 m/s
        {0x101, 2, 960, 500, 960, 620},   // Vertical Line
        {0x102, 4, 910, 610, 1010, 600},  // Horizontal Line 1
        {0x103, 2, 820, 560, 1100, 560},  // Horizontal Line 2
        {0x104, 2, 910, 520, 1010, 520}   // Horizontal Line 3
    },
    {
        // Mode 2: 30 m/s
        {0x101, 2, 960, 500, 960, 620},   // Vertical Line
        {0x102, 4, 910, 600, 1010, 600},  // Horizontal Line 1
        {0x103, 2, 820, 560, 1100, 560},  // Horizontal Line 2
        {0x104, 2, 910, 520, 1010, 520}   // Horizontal Line 3
    }};

const uint8_t CROSSHAIR_LAYER = 2;
const Draw_Color CROSSHAIR_COLOR = Draw_COLOR_GREEN;
const uint16_t CROSSHAIR[5] = {0x201, 2, 960, 560, 10};  // ID, Width, X, Y, R

const uint8_t WIDTH_MARK_LAYER = 2;
const Draw_Color WIDTH_MARK_COLOR = Draw_COLOR_YELLOW;
const uint16_t WIDTH_MARK_GYRO[2][6] = {
    // ID, Width, X0, Y0, X1, Y1
    {0x301, 2, 660, 400, 660, 200},    // Left Mark Line, Normal
    {0x302, 2, 1260, 400, 1260, 200},  // Right Mark Line, Normal
};
const uint16_t WIDTH_MARK_NORMAL[2][6] = {
    {0x301, 2, 660, 400, 960, 200},   // Left Mark Line, Gyro Mode
    {0x302, 2, 1260, 400, 960, 200},  // Right Mark Line, Gyro Mode
};

const uint8_t CAP_STATE_LAYER[2] = {3, 2};  // Foreground, Background
const Draw_Color CAP_STATE_COLOR[5] = {
    Draw_COLOR_WHITE,   // Background
    Draw_COLOR_GREEN,   // Text
    Draw_COLOR_GREEN,   // Foreground, Full (50% ~ 100%)
    Draw_COLOR_YELLOW,  // Foreground, Insufficient (10% ~ 50%)
    Draw_COLOR_ORANGE   // Foreground, Empty (0% ~ 10%)
};
const uint16_t CAP_STATE[4] = {6, 960, 240, 40};              // Width, X, Y, R
const uint16_t CAP_STATE_CIRCLE = 0x401;                      // Background Circle ID
const uint16_t CAP_STATE_ARC = 0x402;                         // Foreground Arc ID
const uint16_t CAP_STATE_TEXT[5] = {0x50D, 20, 2, 900, 200};  // ID, Font Size, Width, X, Y
const uint16_t CAP_STATE_VALUE_TEXT[6]={0x403, 20 , 2 ,1000,200};
const char* CAP_STATE_TEXT_STR = "CAP:   %";

const uint8_t PITCH_METER_LAYER = 2;
const Draw_Color PITCH_METER_COLOR = Draw_COLOR_GREEN;
const uint16_t PITCH_METER_TEXT[5] = {0x501, 20, 2, 1750, 800};  // ID, Font Size, Width, X, Y
const char* PITCH_METER_TEXT_STR = ":PITCH    \0";
const uint16_t PITCH_METER_VALUE[6] = {0x502, 20, 3, 2, 1620, 800};  // ID, Font Size, Precision, Width, X, Y

const uint8_t AIM_MODE_LAYER = 2;
const Draw_Color AIM_MODE_COLOR = Draw_COLOR_GREEN;
const uint16_t AIM_MODE_TEXT[5] = {0x503, 20, 2, 1750, 850};        // ID, Font Size, Width, X, Y
const uint16_t AIM_MODE_VALUE_TEXT[5] = {0x504, 20, 2, 1520, 850};  // ID, Font Size, Width, X, Y
const char* AIM_MODE_TEXT_STR = ":AIM";
const char* NORMAL_AIM_TEXT_STR = "NORMAL\0";
const char* ARMOR_AIM_TEXT_STR = "ARMOR\0";
const char* DEBUG_AIM_TEXT_STR = "DEBUG\0";
const char* BIG_BUFF_AIM_TEXT_STR = "BIG_BUF\0";
const char* SMALL_BUFF_AIM_TEXT_STR = "SMALL_BUF";
const char* SENTRY_AIM_TEXT_STR = "SENTRY";

const uint8_t CHASSIS_MODE_LAYER = 2;
const Draw_Color CHASSIS_MODE_COLOR = Draw_COLOR_GREEN;
const uint16_t CHASSIS_MODE_TEXT[5] = {0x505, 20, 2, 1750, 900};        // ID, Font Size, Width, X, Y
const uint16_t CHASSIS_MODE_VALUE_TEXT[5] = {0x506, 20, 2, 1550, 900};  // ID, Font Size, Width, X, Y
const char* CHASSIS_MODE_TEXT_STR = ":CHASSIS\0";
const char* NORMAL_RUN_TEXT_STR = "NORMAL\0";
const char* GYRO_RUN_TEXT_STR = "GYRO\0";
const char* SUPERGYRO_RUN_TEXT_STR = "SUPERGYRO\0";
const char* STOP_RUN_TEXT_STR = "STOP\0";
const char* DISCO_RUN_TEXT_STR = "DISCO\0";
const char* CRAB_RUN_TEXT_STR = "CRAB\0";
const char* ASS_RUN_TEXT_STR = "ASS\0";


const uint8_t MAGAZINE_STATE_LAYER=2;
const Draw_Color MAGAZINE_STATE_COLOR = Draw_COLOR_GREEN;
const uint16_t MAGAZINE_STATE_TEXT[5] = {0x507, 20, 2, 1750, 700};        // ID, Font Size, Width, X, Y
const uint16_t MAGAZINE_STATE_VALUE_TEXT[5] = {0x508, 20, 2, 1680, 700};  // ID, Font Size, Width, X, Y
const char* MAGAZINE_STATE_TEXT_STR = ":DOOR    \0";
const char* MAGAZINE_ON_TEXT_STR = "ON     \0";
const char* MAGAZINE_OFF_TEXT_STR = "OFF        \0";

const uint8_t SHOOTER_STATE_LAYER=2;
const Draw_Color SHOOTER_STATE_COLOR = Draw_COLOR_GREEN;
const uint16_t SHOOTER_STATE_TEXT[5] = {0x509, 20, 2, 1750, 750};        // ID, Font Size, Width, X, Y
const uint16_t SHOOTER_STATE_VALUE_TEXT[5] = {0x50A, 20, 2, 1680, 750};  // ID, Font Size, Width, X, Y
const char* SHOOTER_STATE_TEXT_STR = ":SHOOTER";
const char* SHOOTER_ON_TEXT_STR = "ON      \0";
const char* SHOOTER_OFF_TEXT_STR = "OFF     \0";

const uint8_t OFFSET_X_LAYER=2;
const Draw_Color OFFSET_X_COLOR = Draw_COLOR_GREEN;
const uint16_t OFFSET_X_TEXT[5] = {0x50B, 20, 2, 1800, 650};        // ID, Font Size, Width, X, Y
const uint16_t OFFSET_X_VALUE_TEXT[6] = {0x50E, 20,  2, 1700, 650};  // ID, Font Size, Precision, Width, X, Y
const char* OFFSET_X_TEXT_STR = ":X     \0";

const uint8_t OFFSET_Y_LAYER=2;
const Draw_Color OFFSET_Y_COLOR = Draw_COLOR_GREEN;
const uint16_t OFFSET_Y_TEXT[5] = {0x50C, 20, 2, 1800, 600};        // ID, Font Size, Width, X, Y
const uint16_t OFFSET_Y_VALUE_TEXT[6] = {0x50F, 20,  2, 1700, 600};  // ID, Font Size, Precision, Width, X, Y
const char* OFFSET_Y_TEXT_STR = ":Y     \0";

/********** END OF Drawing Constants **********/

Referee_DrawDataTypeDef Referee_DrawData;


/**
* @brief      draw结构体赋值函数，赋值数据用于UI绘图
 * @param      mode: ������ģʽ��1ΪС���ݣ�0Ϊ��ͨ��
 * @retval     ��
 */

void Referee_SetWidthMode(uint8_t mode) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->width_mode = mode;
}

void Referee_SetChassisMode(uint8_t mode){
	 Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	draw->cha_mode=mode;
}

void Referee_SetAutoAimMode(uint8_t mode){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	draw->minipc_mode=mode;
	
}

void	Referee_SetShooterStateMode(uint8_t state){
			Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	draw->shooter_state=state;
		
		}
void	Referee_SetMagazineStateMode(uint8_t state){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	draw->magazine_state=state;

}
void Referee_SetMinipc_Offset(uint8_t angle_x,uint8_t angle_y){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	draw->minipc_offset_horizental=angle_x;
	draw->minipc_offset_vertical=angle_y;

}

void Referee_SetAimMode(uint8_t mode) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (mode > 2)
        return;
    draw->aim_mode = mode;
}

void Referee_SetCapState(uint8_t state) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->cap_state = state;
}

void Referee_SetPitchAngle(float angle) {     
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->pitch_angle = angle;
}

void Referee_SetMagazineState(uint8_t magazine_state) {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->magazine_state = magazine_state;
}

//
void Referee_SetupAimLine() {
    // draw_cnt: 4
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t(*aim_lines)[6] = AIM_LINES[0];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
	
}

/**
 * @brief      更新瞄准线
 * @param      
 * @retval     
 */

void Referee_UpdateAimLine() {
    // draw_cnt: 4 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (draw->aim_mode_last == draw->aim_mode)
        return;
		
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t(*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}

/**
 * @brief      初始化瞄准准星
 * @param      ��
 * @retval     ��
 */
void Referee_SetupCrosshair() {
    // draw_cnt: 1
    Draw_AddCircle(CROSSHAIR[0], CROSSHAIR_LAYER, CROSSHAIR_COLOR, CROSSHAIR[1], CROSSHAIR[2], CROSSHAIR[3], CROSSHAIR[4]);
}

/**
 * @brief     无需更新准星
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateCrosshair() {
    // nothing
}

/**
 * @brief      初始化车宽线
 * @param      ��
 * @retval     ��
 */
void Referee_SetupWidthMark() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    draw->width_mode_last = draw->width_mode;
    const uint16_t(*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_AddLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}

/**
 * @brief      更新车宽线
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateWidthMark() {
    // draw_cnt: 2 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    if (draw->width_mode_last == draw->width_mode)
        return;
    draw->width_mode_last = draw->width_mode;
    const uint16_t(*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_ModifyLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}

/**
* @brief      初始化电容字符CAP:
 * @param      ��
 * @retval     ��
 */
void Referee_SetupCapState() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
//    Draw_AddCircle(CAP_STATE_CIRCLE, CAP_STATE_LAYER[1], CAP_STATE_COLOR[0], CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3]);
//    int value = draw->cap_state;
//    value = (int)(-draw->pitch_angle + 10) * 2;
//    Draw_Color color;
//    if (value > 100)
//        return;
//    else if (value >= 50)
//        color = CAP_STATE_COLOR[2];
//    else if (value >= 20)
//        color = CAP_STATE_COLOR[3];
//    else
//        color = CAP_STATE_COLOR[4];
//    uint16_t start_angle = 0;
//    uint16_t end_angle = 0;
//    if (value > 0 && value <= 100)
//        end_angle = (uint16_t)(360.0 * value / 100.0);
//    Draw_AddArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
	
	 Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
	
}

/**
 * @brief      更新电容数据
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateCapState() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;

    int value = draw->cap_state;
    Draw_Color color;
    if (value > 100)
        return;
    else if (value >= 50)
        color = CAP_STATE_COLOR[2];
    else if (value >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];
 if(draw->cap_state_last==101){
		Draw_AddInt(CAP_STATE_VALUE_TEXT[0], CAP_STATE_LAYER[1], color, CAP_STATE_VALUE_TEXT[1], CAP_STATE_VALUE_TEXT[2], CAP_STATE_VALUE_TEXT[3], CAP_STATE_VALUE_TEXT[4], value);
			draw->cap_state_last=draw->cap_state;
		}
			Draw_ModifyInt(CAP_STATE_VALUE_TEXT[0], CAP_STATE_LAYER[1], color, CAP_STATE_VALUE_TEXT[1], CAP_STATE_VALUE_TEXT[2], CAP_STATE_VALUE_TEXT[3], CAP_STATE_VALUE_TEXT[4], value);


}

/**
* @brief      初始化Pitch轴字符Pitch:
 * @param      ��
 * @retval     ��
 */
void Referee_SetupPitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);
  
}

/**
 * @brief      更新Pitch轴数据
 * @param      ��
 * @retval     ��
 */
void Referee_UpdatePitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    // Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
    if(draw->pitch_angle_last==50){
		Draw_AddFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], 0,PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
			draw->pitch_angle_last=draw->pitch_angle;
		}
				Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], 0,PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5],value);
draw->pitch_angle_last=draw->pitch_angle;
}

/**
 * @brief     弃用，分别写函数赋值
 * @param      auto_aim_mode: ����ģʽ��0 ~ 3��Ӧ �����顢װ�װ����顢С�������顢���������飩
 * @param      cha_mode: ����ģʽ ��0 ~ 1��Ӧ ���������˶� �� С����ģʽ��
 * @retval     ��
 */
void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode) {

}

/**
* @brief      初始化底盘模式和自瞄模式字符串 CHASSIS:和AIM：
 * @param      ��
 * @retval     ��
 */
void Referee_SetupModeDisplay() {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
    Draw_AddString(AIM_MODE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_TEXT[1], AIM_MODE_TEXT[2], AIM_MODE_TEXT[3], AIM_MODE_TEXT[4], AIM_MODE_TEXT_STR);

    Draw_AddString(CHASSIS_MODE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_TEXT[1], CHASSIS_MODE_TEXT[2], CHASSIS_MODE_TEXT[3], CHASSIS_MODE_TEXT[4], CHASSIS_MODE_TEXT_STR);
}

/**
 * @brief     更新底盘模式字符串
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateChassisModeDisplay(){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
		if(draw->cha_mode_last==10){
			    Draw_AddString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], NORMAL_RUN_TEXT_STR);
			draw->cha_mode_last=2;
		}
    if (draw->cha_mode_last != draw->cha_mode) {
        draw->cha_mode_last = draw->cha_mode;
        switch (draw->cha_mode) {
            case 2:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], STOP_RUN_TEXT_STR);
                break;
            case 3:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], NORMAL_RUN_TEXT_STR);
                break;
            case 5:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], GYRO_RUN_TEXT_STR);
                break;
            case 4:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], SUPERGYRO_RUN_TEXT_STR);
                break;
            case 8:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], DISCO_RUN_TEXT_STR);
                break;
            case 7:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], CRAB_RUN_TEXT_STR);
                break;	
            case 6:
                Draw_ModifyString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], ASS_RUN_TEXT_STR);
                break;					
            default:
                break;
        }
    }
}
/**
 * @brief     更新自瞄模式字符串
 * @param      ��
 * @retval     ��
 */
void Referee_UpdateMinipcModeDisplay() {
    Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	
	if(draw->minipc_mode_last ==10){
	Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], ARMOR_AIM_TEXT_STR);
	draw->minipc_mode_last=0;
	}
		if (draw->minipc_mode_last != draw->minipc_mode) {
        draw->minipc_mode_last = draw->minipc_mode;
        switch (draw->minipc_mode_last) {
            case 10: //  waiting for  no_auto 
                Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], NORMAL_AIM_TEXT_STR);
                break;
            case 0:
                Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], ARMOR_AIM_TEXT_STR);
                break;
						case 4:
								Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], DEBUG_AIM_TEXT_STR);
								break;
            case 1:
                Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], BIG_BUFF_AIM_TEXT_STR);
                break;
            case 2:
                Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], SMALL_BUFF_AIM_TEXT_STR);
                break;
            case 3  :
                Draw_ModifyString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], SENTRY_AIM_TEXT_STR);
                break;
            default:
                break;
        }
    }

	
}
//弃用
void Referee_SetupAllString() {
    // cmd_cnt: 2
    // Referee_RefereeDataTypeDef *Referee = &Referee_DrawData;

    //    Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
    //    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);

    Draw_AddString(AIM_MODE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_TEXT[1], AIM_MODE_TEXT[2], AIM_MODE_TEXT[3], AIM_MODE_TEXT[4], AIM_MODE_TEXT_STR);
    Draw_AddString(CHASSIS_MODE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_TEXT[1], CHASSIS_MODE_TEXT[2], CHASSIS_MODE_TEXT[3], CHASSIS_MODE_TEXT[4], CHASSIS_MODE_TEXT_STR);
}

/**
 * @brief     因为绘画图形需要先Add，所以需要来个第一次检验，后面有改动再Modify
 * @param      ��
 * @retval     ��
 */
void Referee_SetLastMode(){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData; // 1、make an original value 2、 help set up first value
	draw->aim_mode_last=10;
	draw->cap_state_last=101;
	draw->pitch_angle_last=50;
	draw->cha_mode_last=10;
	draw->magazine_state_last=10;
	draw->minipc_mode_last=10;
	draw->shooter_state_last=10;
	draw->minipc_mode_last=10;
	draw->minipc_offset_horizental_last=126;
	draw->minipc_offset_vertical_last=126;  
}

/**
 * @brief     弹舱盖初始化和更新
 * @param      ��
 * @retval     ��
 */
void Referee_SetupMagazineState(){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	 Draw_AddString(MAGAZINE_STATE_TEXT[0], MAGAZINE_STATE_LAYER, MAGAZINE_STATE_COLOR, MAGAZINE_STATE_TEXT[1], MAGAZINE_STATE_TEXT[2], MAGAZINE_STATE_TEXT[3], MAGAZINE_STATE_TEXT[4], MAGAZINE_STATE_TEXT_STR);
}
void Referee_UpdateMagazineState(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	if(draw->magazine_state_last==10){
		Draw_AddString(MAGAZINE_STATE_VALUE_TEXT[0], MAGAZINE_STATE_LAYER, MAGAZINE_STATE_COLOR, MAGAZINE_STATE_VALUE_TEXT[1], MAGAZINE_STATE_VALUE_TEXT[2], MAGAZINE_STATE_VALUE_TEXT[3], MAGAZINE_STATE_VALUE_TEXT[4], MAGAZINE_OFF_TEXT_STR);
		draw->magazine_state_last=0;
	}
	if(draw->magazine_state_last!=draw->magazine_state){
		if(draw->magazine_state==1){
		Draw_ModifyString(MAGAZINE_STATE_VALUE_TEXT[0], MAGAZINE_STATE_LAYER, MAGAZINE_STATE_COLOR, MAGAZINE_STATE_VALUE_TEXT[1], MAGAZINE_STATE_VALUE_TEXT[2], MAGAZINE_STATE_VALUE_TEXT[3], MAGAZINE_STATE_VALUE_TEXT[4], MAGAZINE_ON_TEXT_STR);
			}else if(draw->magazine_state==0){
				Draw_ModifyString(MAGAZINE_STATE_VALUE_TEXT[0], MAGAZINE_STATE_LAYER, MAGAZINE_STATE_COLOR, MAGAZINE_STATE_VALUE_TEXT[1], MAGAZINE_STATE_VALUE_TEXT[2], MAGAZINE_STATE_VALUE_TEXT[3], MAGAZINE_STATE_VALUE_TEXT[4], MAGAZINE_OFF_TEXT_STR);
			}
			draw->magazine_state_last=draw->magazine_state;
		}
	
}

/**
 * @brief     摩擦轮初始化和更新
 * @param      ��
 * @retval     ��
 */
void Referee_SetupShooterState(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	Draw_AddString(SHOOTER_STATE_TEXT[0], SHOOTER_STATE_LAYER, SHOOTER_STATE_COLOR, SHOOTER_STATE_TEXT[1], SHOOTER_STATE_TEXT[2], SHOOTER_STATE_TEXT[3], SHOOTER_STATE_TEXT[4], SHOOTER_STATE_TEXT_STR);
}
void Referee_UpdateShooterState(){
	Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	if(draw->shooter_state_last==10){
		Draw_AddString(SHOOTER_STATE_VALUE_TEXT[0], SHOOTER_STATE_LAYER, SHOOTER_STATE_COLOR, SHOOTER_STATE_VALUE_TEXT[1], SHOOTER_STATE_VALUE_TEXT[2], SHOOTER_STATE_VALUE_TEXT[3], SHOOTER_STATE_VALUE_TEXT[4], SHOOTER_OFF_TEXT_STR);
		draw->shooter_state_last=draw->shooter_state;
	}
	if(draw->shooter_state_last!=draw->shooter_state){
		if(draw->shooter_state==1){
		Draw_ModifyString(SHOOTER_STATE_VALUE_TEXT[0], SHOOTER_STATE_LAYER, SHOOTER_STATE_COLOR, SHOOTER_STATE_VALUE_TEXT[1], SHOOTER_STATE_VALUE_TEXT[2], SHOOTER_STATE_VALUE_TEXT[3], SHOOTER_STATE_VALUE_TEXT[4], SHOOTER_ON_TEXT_STR);
			}else if(draw->shooter_state==0){
				Draw_ModifyString(SHOOTER_STATE_VALUE_TEXT[0], SHOOTER_STATE_LAYER, SHOOTER_STATE_COLOR, SHOOTER_STATE_VALUE_TEXT[1], SHOOTER_STATE_VALUE_TEXT[2], SHOOTER_STATE_VALUE_TEXT[3], SHOOTER_STATE_VALUE_TEXT[4], SHOOTER_OFF_TEXT_STR);
			}
			draw->shooter_state_last=draw->shooter_state;
		}
}

/**
 * @brief     X和Y偏置 的 初始化和更新
 * @param      ��
 * @retval     ��
 */
void Referee_SetupMinipcOffset_x(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	Draw_AddString(OFFSET_X_TEXT[0], OFFSET_X_LAYER, OFFSET_X_COLOR, OFFSET_X_TEXT[1], OFFSET_X_TEXT[2], OFFSET_X_TEXT[3], OFFSET_X_TEXT[4], OFFSET_X_TEXT_STR);
}
void Referee_UpdateMinipcOffset_x(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	   if(draw->minipc_offset_horizental_last==126){
		Draw_AddInt(OFFSET_X_VALUE_TEXT[0], OFFSET_X_LAYER, OFFSET_X_COLOR, OFFSET_X_VALUE_TEXT[1],OFFSET_X_VALUE_TEXT[2], OFFSET_X_VALUE_TEXT[3], OFFSET_X_VALUE_TEXT[4], draw->minipc_offset_horizental);
			draw->minipc_offset_horizental_last=draw->minipc_offset_horizental;
		}
				Draw_ModifyInt(OFFSET_X_VALUE_TEXT[0],OFFSET_X_LAYER, OFFSET_X_COLOR, OFFSET_X_VALUE_TEXT[1], OFFSET_X_VALUE_TEXT[2], OFFSET_X_VALUE_TEXT[3], OFFSET_X_VALUE_TEXT[4],draw->minipc_offset_horizental);
draw->minipc_offset_horizental_last=draw->minipc_offset_horizental;
}
void Referee_SetupMinipcOffset_y(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	Draw_AddString(OFFSET_Y_TEXT[0], OFFSET_Y_LAYER, OFFSET_Y_COLOR, OFFSET_Y_TEXT[1], OFFSET_Y_TEXT[2], OFFSET_Y_TEXT[3], OFFSET_Y_TEXT[4], OFFSET_Y_TEXT_STR);
}
void Referee_UpdateMinipcOffset_y(){
Referee_DrawDataTypeDef* draw = &Referee_DrawData;
	   if(draw->minipc_offset_vertical_last==126){
		Draw_AddInt(OFFSET_Y_VALUE_TEXT[0], OFFSET_Y_LAYER, OFFSET_Y_COLOR, OFFSET_Y_VALUE_TEXT[1], OFFSET_Y_VALUE_TEXT[2], OFFSET_Y_VALUE_TEXT[3], OFFSET_Y_VALUE_TEXT[4], draw->minipc_offset_vertical);
			draw->minipc_offset_vertical_last=draw->minipc_offset_vertical;
		}
				Draw_ModifyInt(OFFSET_Y_VALUE_TEXT[0], OFFSET_Y_LAYER, OFFSET_Y_COLOR, OFFSET_Y_VALUE_TEXT[1], OFFSET_Y_VALUE_TEXT[2], OFFSET_Y_VALUE_TEXT[3], OFFSET_Y_VALUE_TEXT[4],draw->minipc_offset_vertical);
draw->minipc_offset_vertical_last=draw->minipc_offset_vertical;
	
}


/**
 * @brief      初始化函数，在全车初始化里调用
 * @param      ��
 * @retval     ��
 */
static int last_time = -1000;
int now;
void Referee_Setup() {
//    now = HAL_GetTick();
//    if (now - last_time < 5000)
//        return;
//    last_time = now;
 //   Draw_ClearAll();  // cmd_cnt: 1, total_cmd_cnt: 1 

//		Referee_SetupAimLine();       // draw_cnt: 4
//    Referee_SetupCrosshair();     // draw_cnt: 1
//    Referee_SetupWidthMark();     // draw_cnt: 2, send(7), total_cmd_cnt: 2
//    Referee_SetupErrorDisplay();  // draw_cnt: 0, send(2)send(1), total_cmd_cnt: 4
		for(int j=1;j<=20;j++){  
			Referee_SetupPitchMeter();    // draw_cnt: 1
			Referee_SetupModeDisplay();   // draw_cnt: 2
			Referee_SetupMagazineState();
			Referee_SetupShooterState();
			Referee_SetupMinipcOffset_x();
			Referee_SetupMinipcOffset_y();
			Referee_SetupCapState();
			Referee_SetupCrosshair();     // draw_cnt: 1
			Referee_SetupWidthMark();     // draw_cnt: 2, send(7), total_cmd_cnt: 2
		}
    // Referee_SetupAllString();  // cmd_cnt: 2, total_cmd_cnt: 6

    Referee_DrawingBufferFlush();  // useless since string cmd sent previously

		Referee_SetLastMode();
    referee_setup_flag = 1;
}

/**
 * @brief      更新UI函数，在Referee_Task()调用
 * @param      ��
 * @retval     ��
 */

uint8_t Update_turn=1;
void Referee_Update() {
	//为了避免超出发送图形上限，采用依次发送的方法
        switch (Update_turn) {
            case 1:
								Referee_SetupAimLine();
                Referee_UpdateAimLine();       // draw_cnt: if bullet speed changed 4, else 0
								Update_turn++;
                break;
            case 2:
								Referee_UpdatePitchMeter();	
								Update_turn++; 
                break;
						case 3:
								Referee_UpdateChassisModeDisplay();   // draw_cnt: 0
								Update_turn++; 
								break;
            case 4:
								Referee_UpdateMinipcModeDisplay();   // draw_cnt: 0
								Update_turn++; 
                break;
            case 5:
								Referee_UpdateMagazineState();
								Update_turn++; 
                break;
						case 6:
							Referee_UpdateShooterState();
						Update_turn++; 							
						break;
						case 7:
						Referee_UpdateMinipcOffset_x();
						Update_turn++;
						break;
						case 8:
						Referee_UpdateMinipcOffset_y();  // x,y偏置绘画函数
							Update_turn++;
						break;
						case 9:
				Referee_UpdateCapState();      // draw_cnt: 1
						Update_turn=1;
						break;
            default:
                break;
        }




//    Referee_UpdateAimLine();       // draw_cnt: if bullet speed changed 4, else 0
//   // Referee_UpdateWidthMark();     // draw_cnt: if gyro mode changed 2, else 0
//   // Referee_UpdateCapState();      // draw_cnt: 1
//    Referee_UpdatePitchMeter();    // draw_cnt: 1
//    Referee_UpdateModeDisplay();   // draw_cnt: 0
//	
//	Referee_UpdateMagazineState();
//    Referee_UpdateErrorDisplay();  // draw_cnt: 0

    Referee_DrawingBufferFlush();  // max draw_cnt: 8, cmd_cnt:2
                                   // min draw_cnt: 2, cmd_cnt:1
}

#endif
#endif
