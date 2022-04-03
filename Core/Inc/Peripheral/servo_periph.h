/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\servo_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:17
 */

#ifndef SERVO_PERIPH_H
#define SERVO_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_PERIPH_SERVO)

#include "pwm_util.h"

typedef enum {
    Servo_OFF = 0,
    Servo_ON = 1
} Servo_ServoStateEnum;

typedef struct {
    PWM_PWMHandleTypeDef pwm;
    float angle;
    Servo_ServoStateEnum state;
} Servo_ServoTypeDef;

extern Servo_ServoTypeDef Servo_ammoContainerCapServo;

void Servo_InitAllServos(void);
Servo_ServoStateEnum Servo_GetServoState(Servo_ServoTypeDef* servo);
void Servo_StartServo(Servo_ServoTypeDef* servo);
void Servo_StopServo(Servo_ServoTypeDef* servo);
float Servo_GetServoAngle(Servo_ServoTypeDef* servo);
void Servo_SetServoAngle(Servo_ServoTypeDef* servo, float angle);
void Servo_InitServo(Servo_ServoTypeDef* servo, TIM_HandleTypeDef* htim, uint32_t ch);

#endif

#ifdef __cplusplus
}
#endif

#endif
