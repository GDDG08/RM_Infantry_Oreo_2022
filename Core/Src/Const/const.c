/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Const\const.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-06-22 20:12:49
 */

#include "const.h"

#include "motor_periph.h"
#include "cha_chassis_ctrl.h"
#include "supercap_ctrl.h"
#include "pid_alg.h"
#include "gim_gimbal_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "key_periph.h"
#include "gim_miniPC_ctrl.h"

#include "infantry3_const.h"
#include "infantry4_const.h"
#include "infantry5_const.h"
#include "infantry6_const.h"
#include "infantry6_const.h"
#include "infantry7_const.h"
#include "infantry7_const.h"
#include "infantry8_const.h"

#include "stdio.h"
#include "string.h"

Const_ConstTypeDef Const_Infantry;

uint8_t yyy;
/**
 * @brief      Initializing constant parameters based on infantry encoding
 * @param      NULL
 * @retval     NULL
 */
void Const_Init() {
    Const_Infantry_3_Init(&Infantry_3_Const);
    Const_Infantry_4_Init(&Infantry_4_Const);
    Const_Infantry_5_Init(&Infantry_5_Const);
    Const_Infantry_6_Init(&Infantry_6_Const);
    Const_Infantry_7_Init(&Infantry_7_Const);
    Const_Infantry_8_Init(&Infantry_8_Const);

    uint8_t infantry_code = Key_GetEquipCode();

const_error:

    infantry_code = Key_GetEquipCode();
    switch (infantry_code) {
        case 0x03:
            Const_Infantry = Infantry_3_Const;
            break;
        case 0x04:
            Const_Infantry = Infantry_4_Const;
            break;
        case 0x05:
            Const_Infantry = Infantry_5_Const;
            break;
        case 0x06:
            Const_Infantry = Infantry_6_Const;
            break;
        case 0x07:
            Const_Infantry = Infantry_7_Const;
            break;
        case 0x08:
            Const_Infantry = Infantry_8_Const;
            break;
        default:
            goto const_error;
    }
    Const_Copy();
}

float Const_chassisMotorParam[4][3][4][5];
float Const_gimbalYawMotorParam[5][3][4][5];
float Const_gimbalPitchMotorParam[5][3][4][5];
float Const_ShooterMotorParam[3][2][3][4][5];
float Const_FeederMotorParam[1][3][4][5];

float Const_AutoAimOffset[4][2];

void Const_SetChasisMotorParam() {
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    Motor_InitMotorParam(&Chassis_chassisMotorParamStop, Const_chassisMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Chassis_chassisMotorParamNormal, Const_chassisMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Chassis_chassisMotorParamGyro, Const_chassisMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    PID_InitPIDParam(&Chassis_followPIDParam, Const_chassisMotorParam[3][2][0][0], Const_chassisMotorParam[3][2][0][1], Const_chassisMotorParam[3][2][0][2], Const_chassisMotorParam[3][2][0][3],
                     Const_chassisMotorParam[3][2][0][4], Const_chassisMotorParam[3][2][1][0], Const_chassisMotorParam[3][2][1][1], Const_chassisMotorParam[3][2][2][0],
                     Const_chassisMotorParam[3][2][2][1], Const_chassisMotorParam[3][2][3][0], Const_chassisMotorParam[3][2][3][1], PID_POSITION);
#endif
}

void Const_SetGimbalYawMotorParam() {
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamBigEnergy, Const_gimbalYawMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamSmallEnergy, Const_gimbalYawMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamArmor, Const_gimbalYawMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamIMUDebug, Const_gimbalYawMotorParam[3], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalYaw_gimbalYawMotorParamNoAuto, Const_gimbalYawMotorParam[4], PID_POSITION, PID_POSITION, PID_POSITION);
#endif
}

void Const_SetGimbalPitchMotorParam() {
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamBigEnergy, Const_gimbalPitchMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamSmallEnergy, Const_gimbalPitchMotorParam[1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamArmor, Const_gimbalPitchMotorParam[2], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamIMUDebug, Const_gimbalPitchMotorParam[3], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&GimbalPitch_gimbalPitchMotorParamNoAuto, Const_gimbalPitchMotorParam[4], PID_POSITION, PID_POSITION, PID_POSITION);
#endif
}

void Const_SetShooterPIDParam() {
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    Motor_InitMotorParam(&Shooter_shooterLeftMotor_15_Param, Const_ShooterMotorParam[0][0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterRightMotor_15_Param, Const_ShooterMotorParam[0][1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterLeftMotor_18_Param, Const_ShooterMotorParam[1][0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterRightMotor_18_Param, Const_ShooterMotorParam[1][1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterLeftMotor_30_Param, Const_ShooterMotorParam[2][0], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_shooterRightMotor_30_Param, Const_ShooterMotorParam[2][1], PID_POSITION, PID_POSITION, PID_POSITION);
    Motor_InitMotorParam(&Shooter_feederMotorParam, Const_FeederMotorParam[0], PID_POSITION, PID_POSITION, PID_POSITION);
#endif
}

void Const_SetAutoAimOffset() {
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    MiniPC_InitOffsetParam(Const_AutoAimOffset);
#endif
}

/**
 * @brief      Copy corresponding constants
 * @param      NULL
 * @retval     NULL
 */
static void Const_Copy() {
    /*      Super Cap Const         */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

    /*          ADC Control related constants       */
    Const_ADC_V_VGAIN = Const_Infantry.ADC_V_VGAIN;
    Const_ADC_V_C_HolzerGAIN = Const_Infantry.ADC_V_C_HolzerGAIN;
    Const_ADC_V_C_BuckOutResGAIN = Const_Infantry.ADC_V_C_BuckOutResGAIN;
    Const_ADC_V_C_BuckInputResGAIN = Const_Infantry.ADC_V_C_BuckInputResGAIN;
    Const_ADC_Cap_TotalEnergy = Const_Infantry.ADC_Cap_TotalEnergy;
    Const_ADC_CapValue = Const_Infantry.ADC_CapValue;
    Const_ADC_CurrentErrorVoltage = Const_Infantry.ADC_CurrentErrorVoltage;
    /*          DAC Control related constants       */
    Const_DAC_GAIN = Const_Infantry.DAC_GAIN;
    Const_DAC_DetectRES = Const_Infantry.DAC_DetectRES;
    /*          Super Cap control const             */
    Cap_MinVoltage = Const_Infantry.Cap_MinVoltage;
    Cap_ChargeReservedPower = Const_Infantry.Cap_ChargeReservedPower;
    Cap_AvailableVoltage = Const_Infantry.Cap_AvailableVoltage;
#endif

    /*      infantry chasiss const                  */
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    /*          Motor control constant              */
    Const_YAW_MOTOR_INIT_OFFSET = Const_Infantry.YAW_MOTOR_INIT_OFFSET;
    Const_YAW_MAXANGLE = Const_Infantry.YAW_MAXANGLE;

    memcpy(Const_chassisMotorParam, Const_Infantry.chassisMotorParam, sizeof(Const_Infantry.chassisMotorParam));
    memcpy(Const_gimbalYawMotorParam, Const_Infantry.gimbalYawMotorParam, sizeof(Const_Infantry.gimbalYawMotorParam));
    /*          Chassis control filter const        */
    Const_Chassis_MOVE_REF_TO_MOTOR_REF = Const_Infantry.Chassis_MOVE_REF_TO_MOTOR_REF;
    Const_Chassis_ROTATE_REF_TO_MOTOR_REF = Const_Infantry.Chassis_ROTATE_REF_TO_MOTOR_REFf;

#endif

    /*      infantry gimbal const       */

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

    /*          Remote control const                */
    MOUSE_PITCH_ANGLE_TO_FACT = Const_Infantry.MOUSE_PITCH_ANGLE_TO_FACT;
    MOUSE_YAW_ANGLE_TO_FACT = Const_Infantry.MOUSE_YAW_ANGLE_TO_FACT;
    MOUSE_CHASSIS_ACCELERATE = Const_Infantry.MOUSE_CHASSIS_ACCELERATE;
    MOUSE_CHASSIS_SLOWDOWN = Const_Infantry.MOUSE_CHASSIS_SLOWDOWN;
    MOUSE_CHASSIS_MAX_SPEED = Const_Infantry.MOUSE_CHASSIS_MAX_SPEED;
    MOUSE_CHASSIS_MAX_GYRO_SPEED = Const_Infantry.MOUSE_CHASSIS_MAX_GYRO_SPEED;

    Const_MiniPC_Follow_Target_Time = Const_Infantry.MiniPC_Follow_Target_Time;
    Const_MiniPC_Lost_Target_Time = Const_Infantry.MiniPC_Lost_Target_Time;
    // const uint32_t Const_MiniPC_New_Target_Time;

    memcpy(Const_gimbalPitchMotorParam, Const_Infantry.gimbalPitchMotorParam, sizeof(Const_Infantry.gimbalPitchMotorParam));
    memcpy(Const_ShooterMotorParam, Const_Infantry.ShooterMotorParam, sizeof(Const_Infantry.ShooterMotorParam));
    memcpy(Const_FeederMotorParam, Const_Infantry.FeederMotorParam, sizeof(Const_Infantry.FeederMotorParam));

    memcpy(Const_AutoAimOffset, Const_Infantry.AutoAimOffset, sizeof(Const_Infantry.AutoAimOffset));

    /*          Gimbal pitch limit                  */
    Const_PITCH_UMAXANGLE = Const_Infantry.PITCH_UMAXANGLE;
    Const_PITCH_UMAXANGLE_GRYO = Const_Infantry.PITCH_UMAXANGLE_GRYO;
    Const_PITCH_DMAXANGLE = Const_Infantry.PITCH_DMAXANGLE;
    Const_PITCH_MOTOR_INIT_OFFSET = Const_Infantry.PITCH_MOTOR_INIT_OFFSETf;
    /*          Gimbal yaw limit                  */
    Const_YAW_MAXANGLE = Const_Infantry.YAW_MAXANGLE;
    Const_YAW_MOTOR_INIT_OFFSET = Const_Infantry.YAW_MOTOR_INIT_OFFSET;

    Const_SERVO_INIT_OFFSET = Const_Infantry.SERVO_INIT_OFFSET;

    CVKF_NT_YAW = Const_Infantry.CVKF_NT_YAW;

    Const_ShooterLockedCurrent = Const_Infantry.ShooterLockedCurrent;
    Const_ShooterLockedSpeed = Const_Infantry.ShooterLockedSpeed;
    Const_ShooterLockedTime = Const_Infantry.ShooterLockedTime;
    Const_ShooterRelockedTime = Const_Infantry.ShooterRelockedTime;
    Const_ShooterLockedReverseSpeed = Const_Infantry.ShooterLockedReverseSpeed;

    Const_ShooterSlowSpeed = Const_Infantry.ShooterSlowSpeed;
    Const_ShooterFastSpeed = Const_Infantry.ShooterFastSpeed;

    Const_Shooter15mpers = Const_Infantry.Shooter15mpers;
    Const_Shooter18mpers = Const_Infantry.Shooter18mpers;
    Const_Shooter30mpers = Const_Infantry.Shooter30mpers;

    Const_FeederSlowSpeed = Const_Infantry.FeederSlowSpeed;
    Const_FeederFastSpeed = Const_Infantry.FeederFastSpeed;
    Const_FeederWaitSpeed = Const_Infantry.FeederWaitSpeed;

    Const_HeatCtrlFastLimit = Const_Infantry.HeatCtrlFastLimit;
    Const_HeatCtrlSlowLimit = Const_Infantry.HeatCtrlSlowLimit;
    Const_HeatCtrlWaitLimit = Const_Infantry.HeatCtrlWaitLimit;
    Const_HeatCtrlSingleCount = Const_Infantry.HeatCtrlSingleCount;
    Const_HeatCtrlStopLimit = Const_Infantry.HeatCtrlStopLimit;

    Const_energy_yaw_offset = Const_Infantry.energy_yaw_offset;
    Const_energy_pitch_offset = Const_Infantry.energy_pitch_offset;

#endif
}

/*      Super Cap Const         */

/*          ADC Control related constants       */
float Const_ADC_V_VGAIN;               // Voltage value division ratio 10:1
float Const_ADC_V_C_HolzerGAIN;        // Gain of Hall current sensor��chassis referee��
float Const_ADC_V_C_BuckOutResGAIN;    // Buck output current sensing gain��LT3790��
float Const_ADC_V_C_BuckInputResGAIN;  // Buck input current sensing gain��LT3790��
float Const_ADC_Cap_TotalEnergy;       // Total capacitance energy
float Const_ADC_CapValue;              // Minimum voltage of capacitor
float Const_ADC_CurrentErrorVoltage;   // Current sensor error

/*          DAC Control related constants       */
float Const_DAC_GAIN;       // DAC current set gain��LT3790��
float Const_DAC_DetectRES;  // DAC current set resistor��LT3790��

/*          Super Cap control const             */
float Cap_MinVoltage;           // Cap min voltage
float Cap_ChargeReservedPower;  // Cap charge reserved power
float Cap_AvailableVoltage;     // Cap restart voltage

/*      infantry chasiss const                  */

/*          Motor control constant              */
float Const_YAW_MOTOR_INIT_OFFSET;

/*          Chassis control filter const        */
float Const_Chassis_MOVE_REF_TO_MOTOR_REF;
float Const_Chassis_ROTATE_REF_TO_MOTOR_REF;

/*      infantry gimbal const       */

/*          Remote control const                */
float MOUSE_PITCH_ANGLE_TO_FACT;
float MOUSE_YAW_ANGLE_TO_FACT;
float MOUSE_CHASSIS_ACCELERATE;
float MOUSE_CHASSIS_SLOWDOWN;
float MOUSE_CHASSIS_MAX_SPEED;

float MOUSE_CHASSIS_MAX_GYRO_SPEED;
uint32_t Const_MiniPC_Follow_Target_Time;
uint32_t Const_MiniPC_Lost_Target_Time;
// const uint32_t Const_MiniPC_New_Target_Time;

/*          Gimbal pitch limit                  */
float Const_PITCH_UMAXANGLE;
float Const_PITCH_UMAXANGLE_GRYO;
float Const_PITCH_DMAXANGLE;
float Const_YAW_MAXANGLE;
float Const_PITCH_MOTOR_INIT_OFFSET;
float Const_SERVO_INIT_OFFSET;
float Const_YAW_MOTOR_INIT_OFFSET;

int CVKF_NT_YAW;

float Const_ShooterLockedCurrent;
float Const_ShooterLockedSpeed;
float Const_ShooterLockedTime;
float Const_ShooterRelockedTime;
float Const_ShooterLockedReverseSpeed;

float Const_ShooterSlowSpeed;
float Const_ShooterFastSpeed;

float Const_Shooter15mpers;
float Const_Shooter18mpers;
float Const_Shooter30mpers;

float Const_FeederSlowSpeed;
float Const_FeederFastSpeed;
float Const_FeederWaitSpeed;

uint16_t Const_HeatCtrlFastLimit;
uint16_t Const_HeatCtrlSlowLimit;
uint16_t Const_HeatCtrlWaitLimit;
uint16_t Const_HeatCtrlSingleCount;
uint16_t Const_HeatCtrlStopLimit;

float Const_energy_yaw_offset;
float Const_energy_pitch_offset;
