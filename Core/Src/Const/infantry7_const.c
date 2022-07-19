/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Const\infantry7_const.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-19 10:27:44
 */

#include "configure.h"
#include "infantry7_const.h"
#include "stdio.h"
#include "string.h"

Const_ConstTypeDef Infantry_7_Const;

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
static const float Const_chassisMotorParam_infantry_7[4][3][4][5] = {
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{75, 0, 0.5, 0, 13000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamStop
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{75, 0, 0.5, 0, 11000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamNormal
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{20, 0, 0.5, 0, 13000}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // Chassis_chassisMotorParamGyro
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{10, 0.01, 0, 1000, 400}, {-1, -1}, {0, 0}, {-1, -1}}}  // Chassis_followPIDParam
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax}, {d_fil,delta_fil},{kf_1,kf_2}, {kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter
};

static const float Const_gimbalYawMotorParam_infantry_7[5][3][4][5] = {
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{300, 0.3, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{15, 1, 0, 50, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},      // GimbalYaw_gimbalYawMotorParamAimBigEnergy
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{300, 0.3, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{15, 1, 0, 50, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},      // GimbalYaw_gimbalYawMotorParamAimSmallEnergy
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamArmor
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}},  // GimbalYaw_gimbalYawMotorParamIMUDebug
    {{{1.1, 0.1, 0, 10000, 26000}, {-1, -1}, {0, 0}, {-1, -1}}, {{320, 0.8, 10, 10000, 30000}, {0.3, -1}, {0, 0}, {-1, -1}}, {{11, 0.02, 0, 500, 1000}, {-1, -1}, {50, 100}, {0.8, 0.7}}}   // GimbalYaw_gimbalYawMotorParamNoAuto
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax}, {d_fil,delta_fil},{kf_1,kf_2}, {kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter
};
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
static const float Const_gimbalPitchMotorParam_infantry_7[5][3][4][5] = {
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{250, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.4, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},    // GimbalPitch_gimbalPitchMotorParamAimBigEnergy
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{250, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{22, 0.4, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},    // GimbalPitch_gimbalPitchMotorParamAimSmallEnergy
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{180, 0.1, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 10, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},  // GimbalPitch_gimbalPitchMotorParamArmor
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{180, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}},   // GimbalPitch_gimbalPitchMotorParamIMUDebug
    {{{1.1, 0.1, 0, 8000, 30000}, {-1, -1}, {0, 0}, {-1, -1}}, {{180, 0.2, 20, 15000, 30000}, {0.4, -1}, {0, 0}, {-1, -1}}, {{25, 0.08, 0, 100, 500}, {-1, -1}, {100, 160}, {0.18, 0.3}}}    // GimbalPitch_gimbalPitchMotorParamNoAuto
    //  {           Current  PID                          }   {                 SPEED PID                       }  {                    POSITION PID                 }
    //  {Kp, Ki, Kd, SumMax, OutMax},{d_fil,delta_fil},{kf_1,kf_2},{kf1_fil_param,kf2_fil_param};
    //              PID Group            PID Filter    Feedforward      Feedforward Filter

    // {{1.05,  0.1,  0,  8000, 26000}, {0.1, 0},  {0, 0}, {0, 0} }, {220, 0.4, 0, 15000, 30000, 0.1, 0, 0, 0, 0}, {11, 0, 0, 1000, 1000, 0.1, 0.55, 6, 1, 0.02}}
    // Position PID param
};

static const float Const_ShooterMotorParam_infantry_7[3][2][3][4][5] = {
    {{{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{8, 0.4, 20, 20, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},    // 15 Left
     {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{8, 0.4, 20, 20, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}}},   // 15 Right
    {{{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{8, 0.09, 10, 15, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},   // 18 Left
     {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{8, 0.09, 10, 15, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}}},  // 18 Right
    {{{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{9, 0.1, 10, 15, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}},    // 30 Left
     {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{9, 0.1, 10, 15, 35}, {-1, -1}, {0, 0}, {-1, -1}}, {{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}}},   // 30 Right
};

static const float Const_FeederMotorParam_infantry_7[1][3][4][5] = {
    {{{0, 0, 0, 0, 0}, {-1, -1}, {0, 0}, {-1, -1}}, {{500, 0.01, 0, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}, {{8.35, 0, 0.11, 10000, 20000}, {-1, -1}, {0, 0}, {-1, -1}}}  // feeder motor
};

static const float Const_AutoAimOffset_infantry_7[4][2] = {
    {0.0f, 0.0f},  // Armor
    {0.0f, 0.0f},  // Buff_Small
    {0.0f, 0.0f},  // Buff_Big
    {0.0f, 0.0f}   // Sentry
    // pitch, yaw
};
#endif

void Const_Infantry_7_Init(Const_ConstTypeDef* x) {
#if __FN_IF_ENABLE(__FN_SUPER_CAP)
    /*          ADC Control related constants       */
    x->ADC_V_VGAIN = 11.0f;
    x->ADC_V_C_HolzerGAIN = 5.0f;
    x->ADC_V_C_BuckOutResGAIN = 6.25f;
    x->ADC_V_C_BuckInputResGAIN = 25.0f;
    x->ADC_Cap_TotalEnergy = 2000.0f;
    x->ADC_CapValue = 6.0f;
    x->ADC_CurrentErrorVoltage = 0.0f;

    /*          DAC Control related constants       */
    x->DAC_GAIN = 20.0f;
    x->DAC_DetectRES = 0.002f;

    /*          Super Cap control const             */
    x->Cap_MinVoltage = 15.0f;
    x->Cap_ChargeReservedPower = 5.0f;
    x->Cap_AvailableVoltage = 18.0f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    /*      infantry chasiss const                  */
    x->YAW_MOTOR_INIT_OFFSET = -58.5f;
    x->YAW_MAXANGLE = 60.0f;

    memcpy(x->chassisMotorParam, Const_chassisMotorParam_infantry_7, sizeof(Const_chassisMotorParam_infantry_7));
    memcpy(x->gimbalYawMotorParam, Const_gimbalYawMotorParam_infantry_7, sizeof(Const_gimbalYawMotorParam_infantry_7));

    /*          Chassis control filter const        */
    x->Chassis_MOVE_REF_TO_MOTOR_REF = 0.7f;
    x->Chassis_ROTATE_REF_TO_MOTOR_REFf = 0.6f;
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    /*      infantry gimbal const       */
    x->MOUSE_PITCH_ANGLE_TO_FACT = 0.005f;
    x->MOUSE_YAW_ANGLE_TO_FACT = 0.01f;
    x->MOUSE_CHASSIS_ACCELERATE = 2.0f;
    x->MOUSE_CHASSIS_SLOWDOWN = 1.5f;
    x->MOUSE_CHASSIS_MAX_SPEED = 600;
    x->MOUSE_CHASSIS_MAX_GYRO_SPEED = 300;

    x->MiniPC_Follow_Target_Time = 100;
    x->MiniPC_Lost_Target_Time = 100;
    // MiniPC_New_Target_Time;                = 200;

    memcpy(x->gimbalPitchMotorParam, Const_gimbalPitchMotorParam_infantry_7, sizeof(Const_gimbalPitchMotorParam_infantry_7));
    memcpy(x->ShooterMotorParam, Const_ShooterMotorParam_infantry_7, sizeof(Const_ShooterMotorParam_infantry_7));
    memcpy(x->FeederMotorParam, Const_FeederMotorParam_infantry_7, sizeof(Const_FeederMotorParam_infantry_7));

    memcpy(x->AutoAimOffset, Const_AutoAimOffset_infantry_7, sizeof(Const_AutoAimOffset_infantry_7));

    /*          Gimbal pitch limit                  */
    x->PITCH_UMAXANGLE = 12.5f;
    x->PITCH_UMAXANGLE_GRYO = 8.0f;
    x->PITCH_DMAXANGLE = -22.2f;
    x->YAW_MAXANGLE = 55.0f;
    x->PITCH_MOTOR_INIT_OFFSETf = -31.0f;
    x->SERVO_INIT_OFFSET = 0.0f;

    x->CVKF_NT_YAW = 80;

    /*          Gimbal pitch limit                  */
    x->YAW_MOTOR_INIT_OFFSET = -58.5f;

    x->ShooterLockedCurrent = 3000.0f;
    x->ShooterLockedSpeed = 20.0f;
    x->ShooterLockedTime = 200.0f;
    x->ShooterRelockedTime = 500.0f;
    x->ShooterLockedReverseSpeed = 0.0f;

    x->ShooterSlowSpeed = 150.0f;
    x->ShooterFastSpeed = 230.0f;

#if __FN_IF_ENABLE(__FN_SHOOTER_PID)
    x->Shooter15mpers = 15.8f;
    x->Shooter18mpers = 19.0f;
    x->Shooter30mpers = 28.5f;
#else
    x->Shooter15mpers = 15.0f;
    x->Shooter18mpers = 16.0f;
    x->Shooter30mpers = 26.0f;
#endif

    x->FeederSlowSpeed = 50.0f;
    x->FeederFastSpeed = 100.0f;
    x->FeederWaitSpeed = 10.0f;

    x->HeatCtrlFastLimit = 75;
    x->HeatCtrlSlowLimit = 40;
    x->HeatCtrlWaitLimit = 10;
    x->HeatCtrlSingleCount = 10;
    x->HeatCtrlStopLimit = 10;

    x->energy_yaw_offset = 0.9f;
    x->energy_pitch_offset = 0.3f;

#endif
}
