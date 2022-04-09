/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\motor_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:09
 */

#ifndef MOTOR_PERIPH_H
#define MOTOR_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#if __FN_IF_ENABLE(__FN_PERIPH_MOTOR)

#include "fdcan_util.h"
#include "pwm_util.h"
#include "pid_alg.h"
#include "math_alg.h"
#include "filter_alg.h"

typedef enum {
    Motor_TYPE_NOT_CONNECTED = 0,
    Motor_TYPE_CAN_MOTOR = 1,
    Motor_TYPE_PWM_MOTOR = 2,
    Motor_TYPE_OTHER_MOTOR = 3
} Motor_MotorTypeEnum;  // Motor drive type

typedef struct {
    // Motor feedback
    int16_t angle;
    int16_t speed;
    int16_t current;
    int8_t temp;
    // Used to calculate continuous angles
    int16_t last_angle;
    int16_t round_count;
    int8_t has_init;  // Initial flag of the plucking wheel
    float init_offset;
    float limited_angle;
    float consequent_angle;
    // PWM encoder
    TIM_HandleTypeDef* htim;
    uint8_t direction;
    uint16_t counter;
    uint32_t last_update_time;
} Motor_EncoderTypeDef;

typedef struct _motor_type {
    uint8_t pid_num, cur_pid, cur_pid_div;
    PID_PIDTypeDef pid_cur, pid_pos, pid_spd;
    Motor_EncoderTypeDef encoder;
    Motor_MotorTypeEnum type;
    PWM_PWMHandleTypeDef pwm;
    float duty;
    void (*callback)(struct _motor_type*);
    Filter_LowPassParamTypeDef fdb_fil_param;
    Filter_LowPassTypeDef fdb_fil;
} Motor_MotorTypeDef;

typedef struct {
    PID_PIDParamTypeDef pid_param_cur, pid_param_pos, pid_param_spd;
} Motor_MotorParamTypeDef;

typedef struct {
    uint8_t motor_num;
    Motor_MotorTypeDef* motor_handle[4];
    Motor_MotorTypeEnum type;
    FDCAN_HandleTypeDef* can_handle;
    FDCAN_TxHeaderTypeDef can_header;
} Motor_MotorGroupTypeDef;

typedef void (*Motor_EncoderCallbackFuncTypeDef)(Motor_MotorTypeDef*);

extern const uint32_t Const_Motor_MOTOR_OFFLINE_TIME;

/********** VOLATILE USER CODE **********/

extern Motor_MotorGroupTypeDef Motor_chassisMotors;
extern Motor_MotorGroupTypeDef Motor_gimbalMotors;
extern Motor_MotorGroupTypeDef Motor_feederMotors;
extern Motor_MotorGroupTypeDef Motor_shooterMotors;

extern Motor_MotorTypeDef Motor_chassisMotor1;
extern Motor_MotorTypeDef Motor_chassisMotor2;
extern Motor_MotorTypeDef Motor_chassisMotor3;
extern Motor_MotorTypeDef Motor_chassisMotor4;

extern Motor_MotorTypeDef Motor_gimbalMotorYaw;
extern Motor_MotorTypeDef Motor_gimbalMotorPitch;

extern Motor_MotorTypeDef Motor_feederMotor;

extern Motor_MotorTypeDef Motor_shooterMotorLeft;
extern Motor_MotorTypeDef Motor_shooterMotorRight;

// extern float GimbalMotorYaw_INIT_Offset;
/********** VOLATILE USER CODE END **********/

void Motor_EncoderDecodeCallback(FDCAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
void Motor_InitAllMotors(void);
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint8_t pid_num, uint8_t cur_pid, float fdb_param, TIM_HandleTypeDef* htim, uint32_t ch, TIM_HandleTypeDef* htim_enc, Motor_EncoderCallbackFuncTypeDef callback);
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, FDCAN_HandleTypeDef* phcan, uint16_t stdid);
void Motor_InitMotorParam(Motor_MotorParamTypeDef* pparam, float pidpara[][4][5], PID_ModeEnum cur_mode, PID_ModeEnum spd_mode, PID_ModeEnum pos_mode);
void Motor_ResetMotorPID(Motor_MotorTypeDef* pmotor);
void Motor_ResetMotorGroupPID(Motor_MotorGroupTypeDef* pmotor_group);
float Motor_GetMotorRef(Motor_MotorTypeDef* pmotor);
float Motor_GetMotorFdb(Motor_MotorTypeDef* pmotor);
float Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor);
void Motor_CalcMotorOutputRingOverrided(Motor_MotorTypeDef* pmotor, uint8_t pid_no, Motor_MotorParamTypeDef* pparam);
void Motor_SetMotorRef(Motor_MotorTypeDef* pmotor, float ref);
void Motor_SetMotorFdb(Motor_MotorTypeDef* pmotor, uint8_t pid_no, float fdb);
void Motor_SetMotorSlopeRef(Motor_MotorTypeDef* pmotor, float ref, Motor_MotorParamTypeDef* pparam);
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output);
void Motor_CalcMotorOutput(Motor_MotorTypeDef* pmotor, Motor_MotorParamTypeDef* pparam);
void Motor_CalcMotorGroupOutput(Motor_MotorGroupTypeDef* pmotor_group, Motor_MotorParamTypeDef* pparam);
void Motor_SendMotorPWMOutput(Motor_MotorTypeDef* pmotor);
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef* pmotor_group);
uint8_t Motor_IsAnyMotorOffline(void);
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor);
void Motor_ReadPWMEncoder(Motor_MotorTypeDef* pmotor);
void Motor_DecodeEncoder(uint8_t rxdata[], Motor_MotorTypeDef* pmotor);

#endif

#ifdef __cplusplus
}
#endif

#endif
