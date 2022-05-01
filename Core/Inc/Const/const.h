/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Const\const.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-01 20:27:42
 */

#ifndef CONST_H
#define CONST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#include "stm32g4xx.h"

typedef struct {
    /*          ADC Control related constants       */
    float ADC_V_VGAIN;               // The voltage reading gain resistor divider ratio is 11
    float ADC_V_C_HolzerGAIN;        // Hall current sensor gain (chassis current, referee system current)
    float ADC_V_C_BuckOutResGAIN;    // Buck output current sensing gain (lt3790)
    float ADC_V_C_BuckInputResGAIN;  // Buck input current sensing gain (lt3790)
    float ADC_Cap_TotalEnergy;       // Total capacitance energy
    float ADC_CapValue;              // Capacity of capacitor bank
    float ADC_CurrentErrorVoltage;   // ACS712 error

    /*          DAC Control related constants       */
    float DAC_GAIN;       // DAC Current setting gain (lt3790)
    float DAC_DetectRES;  // DAC Current setting resistor (lt3790)

    /*          Super Cap control const             */
    float Cap_MinVoltage;
    float Cap_ChargeReservedPower;
    float Cap_AvailableVoltage;

    /*      infantry chasiss const                  */
    float YAW_MOTOR_INIT_OFFSET;

    float chassisMotorParam[4][3][4][5];
    float gimbalYawMotorParam[5][3][4][5];
    /*          Chassis control filter const        */
    float Chassis_MOVE_REF_TO_MOTOR_REF;
    float Chassis_ROTATE_REF_TO_MOTOR_REFf;

    /*      infantry gimbal const       */
    float MOUSE_PITCH_ANGLE_TO_FACT;
    float MOUSE_YAW_ANGLE_TO_FACT;
    float MOUSE_CHASSIS_ACCELERATE;
    float MOUSE_CHASSIS_SLOWDOWN;
    float MOUSE_CHASSIS_MAX_SPEED;
    float MOUSE_CHASSIS_MAX_GYRO_SPEED;

    uint32_t MiniPC_Follow_Target_Time;
    uint32_t MiniPC_Lost_Target_Time;
    // uint32_t MiniPC_New_Target_Time;

    float gimbalPitchMotorParam[5][3][4][5];
    float ShooterMotorParam[2][3][4][5];
    float FeederMotorParam[1][3][4][5];

    float AutoAimOffset[4][2];

    /*          Gimbal pitch limit                  */
    float PITCH_UMAXANGLE;
    float PITCH_UMAXANGLE_GRYO;
    float PITCH_DMAXANGLE;
    float YAW_MAXANGLE;
    float PITCH_MOTOR_INIT_OFFSETf;
    float SERVO_INIT_OFFSET;
    int CVKF_NT_YAW;
    /*          Gimbal pitch limit                  */

    float ShooterLockedCurrent;
    float ShooterLockedSpeed;
    float ShooterLockedTime;
    float ShooterRelockedTime;
    float ShooterLockedReverseSpeed;

    float ShooterSlowSpeed;
    float ShooterFastSpeed;

    float Shooter15mpers;
    float Shooter18mpers;
    float Shooter30mpers;

    float FeederSlowSpeed;
    float FeederFastSpeed;
    float FeederWaitSpeed;

    uint16_t HeatCtrlFastLimit;
    uint16_t HeatCtrlSlowLimit;
    uint16_t HeatCtrlWaitLimit;
    uint16_t HeatCtrlSingleCount;
    uint16_t HeatCtrlStopLimit;

    float energy_yaw_offset;
    float energy_pitch_offset;
} Const_ConstTypeDef;

/*      Super Cap Const         */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

/*          ADC Control related constants       */
extern float Const_ADC_V_VGAIN;               // Voltage value division ratio 10:1
extern float Const_ADC_V_C_HolzerGAIN;        // Gain of Hall current sensor��chassis referee��
extern float Const_ADC_V_C_BuckOutResGAIN;    // Buck output current sensing gain��LT3790��
extern float Const_ADC_V_C_BuckInputResGAIN;  // Buck input current sensing gain��LT3790��
extern float Const_ADC_Cap_TotalEnergy;       // Total capacitance energy
extern float Const_ADC_CapValue;              // Minimum voltage of capacitor
extern float Const_ADC_CurrentErrorVoltage;   // Current sensor error

/*          DAC Control related constants       */
extern float Const_DAC_GAIN;       // DAC current set gain��LT3790��
extern float Const_DAC_DetectRES;  // DAC current set resistor��LT3790��

/*          Super Cap control const             */
extern float Cap_MinVoltage;           // Cap min voltage
extern float Cap_ChargeReservedPower;  // Cap charge reserved power
extern float Cap_AvailableVoltage;     // Cap restart voltage
#endif

/*      infantry chasiss const                  */
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
/*          Motor control constant              */
extern float Const_YAW_MOTOR_INIT_OFFSET;

/*          Chassis control filter const        */
extern float Const_Chassis_MOVE_REF_TO_MOTOR_REF;
extern float Const_Chassis_ROTATE_REF_TO_MOTOR_REF;

extern float Const_YAW_MAXANGLE;

#endif

/*      infantry gimbal const       */

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

/*          Remote control const                */
extern float MOUSE_PITCH_ANGLE_TO_FACT;
extern float MOUSE_YAW_ANGLE_TO_FACT;
extern float MOUSE_CHASSIS_ACCELERATE;
extern float MOUSE_CHASSIS_SLOWDOWN;
extern float MOUSE_CHASSIS_MAX_SPEED;

extern float MOUSE_CHASSIS_MAX_GYRO_SPEED;
extern uint32_t Const_MiniPC_Follow_Target_Time;
extern uint32_t Const_MiniPC_Lost_Target_Time;
// const uint32_t Const_MiniPC_New_Target_Time;

/*          Gimbal pitch limit                  */
extern float Const_PITCH_UMAXANGLE;
extern float Const_PITCH_UMAXANGLE_GRYO;
extern float Const_PITCH_DMAXANGLE;
extern float Const_YAW_MAXANGLE;
extern float Const_PITCH_MOTOR_INIT_OFFSET;
extern float Const_SERVO_INIT_OFFSET;
extern float Const_YAW_MOTOR_INIT_OFFSET;

extern float Const_ShooterLockedCurrent;
extern float Const_ShooterLockedSpeed;
extern float Const_ShooterLockedTime;
extern float Const_ShooterRelockedTime;
extern float Const_ShooterLockedReverseSpeed;

extern float Const_ShooterSlowSpeed;
extern float Const_ShooterFastSpeed;

extern int CVKF_NT_YAW;

extern float Const_Shooter15mpers;
extern float Const_Shooter18mpers;
extern float Const_Shooter30mpers;

extern float Const_FeederSlowSpeed;
extern float Const_FeederFastSpeed;
extern float Const_FeederWaitSpeed;

extern uint16_t Const_HeatCtrlFastLimit;
extern uint16_t Const_HeatCtrlSlowLimit;
extern uint16_t Const_HeatCtrlWaitLimit;
extern uint16_t Const_HeatCtrlSingleCount;
extern uint16_t Const_HeatCtrlStopLimit;

extern float Const_energy_yaw_offset;
extern float Const_energy_pitch_offset;

#endif

void Const_Init(void);
static void Const_Copy(void);
void Const_SetChasisMotorParam(void);
void Const_SetGimbalYawMotorParam(void);
void Const_SetGimbalPitchMotorParam(void);
void Const_SetShooterPIDParam(void);
void Const_SetAutoAimOffset(void);

#endif

#ifdef __cplusplus
}
#endif
