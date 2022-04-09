/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Peripheral\servo_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:51
 */

#include "servo_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_SERVO)

#include "const.h"

/********** VOLATILE USER CODE **********/

Servo_ServoTypeDef Servo_ammoContainerCapServo;

/**
 * @brief      Initialize all steering gear
 * @param      NULL
 * @retval     NULL
 */
void Servo_InitAllServos() {
    Servo_InitServo(&Servo_ammoContainerCapServo, &htim1, TIM_CHANNEL_1);
}

/**
 * @brief      Return to steering gear status
 * @param      servo: The pointer points to the actuator object
 * @retval     Steering gear status
 */
Servo_ServoStateEnum Servo_GetServoState(Servo_ServoTypeDef* servo) {
    return servo->state;
}

/**
 * @brief      Start the steering gear
 * @param      servo: The pointer points to the actuator object
 * @retval     NULL
 */
void Servo_StartServo(Servo_ServoTypeDef* servo) {
    servo->state = Servo_ON;
    PWM_StartPWM(&(servo->pwm));
}

/**
 * @brief      Stop the servo
 * @param      servo: The pointer points to the actuator object
 * @retval     NULL
 */
void Servo_StopServo(Servo_ServoTypeDef* servo) {
    servo->state = Servo_OFF;
    PWM_StopPWM(&(servo->pwm));
}

/**
 * @brief      Return to steering angle
 * @param      servo: The pointer points to the actuator object
 * @retval     Steering gear angle
 */
float Servo_GetServoAngle(Servo_ServoTypeDef* servo) {
    return servo->angle;
}

/**
 * @brief      Set steering angle
 * @param      servo: The pointer points to the actuator object
 * @param      angle: Steering gear angle
 * @retval     NULL
 */
void Servo_SetServoAngle(Servo_ServoTypeDef* servo, float angle) {
    servo->angle = angle;
    PWM_SetPWMDuty(&(servo->pwm), angle / 275.0f * 0.1f + Const_SERVO_INIT_OFFSET);
}

/**
 * @brief      Initialize the steering gear
 * @param      servo: Pointer to steering gear object
 * @param      htim: Timer handle
 * @param      ch: PWM channel number
 * @retval     NULL
 */
void Servo_InitServo(Servo_ServoTypeDef* servo, TIM_HandleTypeDef* htim, uint32_t ch) {
    servo->state = Servo_OFF;

    PWM_InitPWM(&(servo->pwm), htim, ch);
    PWM_SetPWMFreq(&(servo->pwm), 50);
    Servo_StartServo(&Servo_ammoContainerCapServo);

    Servo_SetServoAngle(servo, 300.0f);
}

#endif
