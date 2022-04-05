/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \Infantry_Oreo\Core\Src\Peripheral\motor_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-05 11:02:02
 */

#include "motor_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MOTOR)

#include "const.h"
#include "stdio.h"
#include "stdlib.h"

const int Const_CURRENT_PID_FREQ_DIV = 5;
/********** ENCODER CALLBACK FUNCTION **********/

/**
 * @brief      Chassis motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void chassis_encoder_callback(Motor_MotorTypeDef* pmotor) {
    Motor_SetMotorFdb(pmotor, 1, (float)pmotor->encoder.speed / 19.0f);
}

/**
 * @brief      Gimbal motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void gimbal_encoder_callback(Motor_MotorTypeDef* pmotor) {
    Motor_SetMotorFdb(pmotor, 0, Filter_LowPass((float)pmotor->encoder.current, &pmotor->fdb_fil_param, &pmotor->fdb_fil));
    // Calculate angle difference and number of cycles
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;  // The increase of mechanical angle is positive
    if (diff < -5500)                                               // Make a positive turn
        pmotor->encoder.round_count++;
    else if (diff > 5500)  // Turn around in the opposite direction
        pmotor->encoder.round_count--;

    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    // For yaw axis processing, the small gyroscope is rotated to the same position as the PTZ according to the nearest distance after stopping
    if (pmotor == &Motor_gimbalMotorYaw) {
        if (pmotor->encoder.limited_angle < Const_YAW_MOTOR_INIT_OFFSET - 180 && Const_YAW_MOTOR_INIT_OFFSET >= 180)
            pmotor->encoder.limited_angle += 360;
        else if (pmotor->encoder.limited_angle > Const_YAW_MOTOR_INIT_OFFSET + 180 && Const_YAW_MOTOR_INIT_OFFSET < 180)
            pmotor->encoder.limited_angle -= 360;
    }
}

/**
 * @brief      Feeder motor encoder callback
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void feeder_encoder_callback(Motor_MotorTypeDef* pmotor) {
    Motor_SetMotorFdb(pmotor, 1, (float)pmotor->encoder.speed / 36.0f);
    Motor_SetMotorFdb(pmotor, 2, (float)pmotor->encoder.consequent_angle);
    // Calculate the angle difference and the number of turns
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;
    if (diff < -5500)
        pmotor->encoder.round_count++;
    else if (diff > 5500)
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f +
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    // over
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count;
        PID_AddPIDRef(&(pmotor->pid_pos), -10 * pmotor->encoder.round_count);
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count;
        PID_AddPIDRef(&(pmotor->pid_pos), 10 * pmotor->encoder.round_count);
        pmotor->encoder.round_count = 0;
    }
    // Motor initialization
    if (!pmotor->encoder.has_init) {
        pmotor->encoder.init_offset = pmotor->encoder.consequent_angle;
        pmotor->encoder.has_init = 1;
    }
}

/**
 * @brief      Shooter encoder callback (pwm motor)
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void shooter_encoder_callback(Motor_MotorTypeDef* pmotor) {
}

/********** VOLATILE USER CODE **********/

const uint32_t Const_Motor_MOTOR_OFFLINE_TIME = 1000;
const uint32_t Const_Motor_MOTOR_TX_EXTID = 0x01;
const uint32_t Const_Motor_MOTOR_TX_DLC = FDCAN_DLC_BYTES_8;
// const uint32_t Const_Motor_MOTOR_RX_DLC = 8;

Motor_MotorGroupTypeDef* Motor_groupHandle[4];

Motor_MotorGroupTypeDef Motor_chassisMotors;
Motor_MotorGroupTypeDef Motor_gimbalMotors;
Motor_MotorGroupTypeDef Motor_feederMotors;
Motor_MotorGroupTypeDef Motor_shooterMotors;

Motor_MotorTypeDef Motor_chassisMotor1;
Motor_MotorTypeDef Motor_chassisMotor2;
Motor_MotorTypeDef Motor_chassisMotor3;
Motor_MotorTypeDef Motor_chassisMotor4;

Motor_MotorTypeDef Motor_gimbalMotorYaw;
Motor_MotorTypeDef Motor_gimbalMotorPitch;

Motor_MotorTypeDef Motor_feederMotor;

Motor_MotorTypeDef Motor_shooterMotorLeft;
Motor_MotorTypeDef Motor_shooterMotorRight;

// float GimbalMotorYaw_INIT_Offset = 0.0f;
/**
 * @brief      Motor encoder decoding callback function
 * @param      canid: CAN Handle number
 * @param      stdid: CAN identifier
 * @param      rxdata: CAN rx data buff
 * @retval     NULL
 */
// uint8_t gimbalYawINIT = 0;
void Motor_EncoderDecodeCallback(FDCAN_HandleTypeDef* phfdcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    if (phfdcan == &hfdcan1) {
        switch (stdid) {
            case 0x201: {
                Motor_DecodeEncoder(rxdata, &Motor_chassisMotor1);
                break;
            }
            case 0x202: {
                Motor_DecodeEncoder(rxdata, &Motor_chassisMotor2);
                break;
            }
            case 0x203: {
                Motor_DecodeEncoder(rxdata, &Motor_chassisMotor3);
                break;
            }
            case 0x204: {
                Motor_DecodeEncoder(rxdata, &Motor_chassisMotor4);
                break;
            }
            case 0x205: {
                Motor_DecodeEncoder(rxdata, &Motor_gimbalMotorYaw);

                // if (!gimbalYawINIT) {
                //     GimbalMotorYaw_INIT_Offset = Motor_gimbalMotorYaw.encoder.limited_angle - Const_YAW_MOTOR_INIT_OFFSET;
                //     gimbalYawINIT = 1;
                // }
                break;
            }
            default: {
                break;
            }
        }
    }
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    if (phfdcan == &hfdcan1) {
        switch (stdid) {
            case 0x204: {
                Motor_DecodeEncoder(rxdata, &Motor_feederMotor);
                break;
            }
            case 0x206: {
                Motor_DecodeEncoder(rxdata, &Motor_gimbalMotorPitch);
                break;
            }
            default: {
                break;
            }
        }
    }
#endif
}

/********** VOLATILE USER CODE END **********/

/**
 * @brief      Initialize all motors
 * @param      NULL
 * @retval     NULL
 */
void Motor_InitAllMotors() {
    Motor_groupHandle[0] = &Motor_chassisMotors;
    Motor_InitMotorGroup(&Motor_chassisMotors, Motor_TYPE_CAN_MOTOR, 4, &hfdcan1, 0x200);
    Motor_InitMotor(&Motor_chassisMotor1, Motor_TYPE_CAN_MOTOR, 1, 0, -1, NULL, 0, NULL, chassis_encoder_callback);
    Motor_InitMotor(&Motor_chassisMotor2, Motor_TYPE_CAN_MOTOR, 1, 0, -1, NULL, 0, NULL, chassis_encoder_callback);
    Motor_InitMotor(&Motor_chassisMotor3, Motor_TYPE_CAN_MOTOR, 1, 0, -1, NULL, 0, NULL, chassis_encoder_callback);
    Motor_InitMotor(&Motor_chassisMotor4, Motor_TYPE_CAN_MOTOR, 1, 0, -1, NULL, 0, NULL, chassis_encoder_callback);
    Motor_chassisMotors.motor_handle[0] = &Motor_chassisMotor1;
    Motor_chassisMotors.motor_handle[1] = &Motor_chassisMotor2;
    Motor_chassisMotors.motor_handle[2] = &Motor_chassisMotor3;
    Motor_chassisMotors.motor_handle[3] = &Motor_chassisMotor4;

    Motor_groupHandle[1] = &Motor_gimbalMotors;
    Motor_InitMotorGroup(&Motor_gimbalMotors, Motor_TYPE_CAN_MOTOR, 2, &hfdcan1, 0x1FF);
    Motor_InitMotor(&Motor_gimbalMotorYaw, Motor_TYPE_CAN_MOTOR, 2, 1, -1, NULL, 0, NULL, gimbal_encoder_callback);
    Motor_InitMotor(&Motor_gimbalMotorPitch, Motor_TYPE_CAN_MOTOR, 2, 1, -1, NULL, 0, NULL, gimbal_encoder_callback);
    Motor_gimbalMotors.motor_handle[0] = &Motor_gimbalMotorYaw;
    Motor_gimbalMotors.motor_handle[1] = &Motor_gimbalMotorPitch;

    Motor_groupHandle[2] = &Motor_feederMotors;
    Motor_InitMotorGroup(&Motor_feederMotors, Motor_TYPE_CAN_MOTOR, 1, &hfdcan1, 0x200);
    Motor_InitMotor(&Motor_feederMotor, Motor_TYPE_CAN_MOTOR, 2, 0, -1, NULL, 0, NULL, feeder_encoder_callback);
    Motor_feederMotors.motor_handle[3] = &Motor_feederMotor;

    Motor_groupHandle[3] = &Motor_shooterMotors;
    Motor_InitMotorGroup(&Motor_shooterMotors, Motor_TYPE_PWM_MOTOR, 2, NULL, 0);
    Motor_InitMotor(&Motor_shooterMotorLeft, Motor_TYPE_PWM_MOTOR, 1, 0, 0.05, &htim2, TIM_CHANNEL_1, &htim3, shooter_encoder_callback);
    Motor_InitMotor(&Motor_shooterMotorRight, Motor_TYPE_PWM_MOTOR, 1, 0, 0.05, &htim16, TIM_CHANNEL_1, &htim5, shooter_encoder_callback);
    Motor_shooterMotors.motor_handle[0] = &Motor_shooterMotorLeft;
    Motor_shooterMotors.motor_handle[1] = &Motor_shooterMotorRight;
}

/**
 * @brief      Initialize motor parameters
 * @param      pparam: Pointer to the motor parameter object
 * @param      pid_num: Pid ring number
 * @param      pidpara: PID parameter array
 * @param      acc: slope function parameters of acc
 * @param      dec: slope function parameters of dec
 * @retval     NULL
 */
void Motor_InitMotorParam(Motor_MotorParamTypeDef* pparam, float pidpara[][4][5], PID_ModeEnum cur_mode, PID_ModeEnum spd_mode, PID_ModeEnum pos_mode) {
    if (pparam == NULL)
        return;
    PID_InitPIDParam(&(pparam->pid_param_cur), pidpara[0][0][0], pidpara[0][0][1], pidpara[0][0][2], pidpara[0][0][3], pidpara[0][0][4], pidpara[0][1][0], pidpara[0][1][1], pidpara[0][2][0], pidpara[0][2][1], pidpara[0][3][0], pidpara[0][3][1], cur_mode);
    PID_InitPIDParam(&(pparam->pid_param_spd), pidpara[1][0][0], pidpara[1][0][1], pidpara[1][0][2], pidpara[1][0][3], pidpara[1][0][4], pidpara[1][1][0], pidpara[1][1][1], pidpara[1][2][0], pidpara[1][2][1], pidpara[1][3][0], pidpara[1][3][1], spd_mode);
    PID_InitPIDParam(&(pparam->pid_param_pos), pidpara[2][0][0], pidpara[2][0][1], pidpara[2][0][2], pidpara[2][0][3], pidpara[2][0][4], pidpara[2][1][0], pidpara[2][1][1], pidpara[2][2][0], pidpara[2][2][1], pidpara[2][3][0], pidpara[2][3][1], pos_mode);
}

/**
 * @brief      Initialize the motor
 * @param      pmotor: Pointer to motor object
 * @param      type: Type of motor (pwm or can)
 * @param      pid_num: Pid ring number
 * @param      cur_pid: Current ring (1 for presence)
 * @param      htim: Pwm motor timer handle
 * @param      ch: Pwm motor timer channel
 * @param      htim_enc: Pwm motor encoder handle
 * @param      callback: Motor callback function
 * @retval     NULL
 */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint8_t pid_num, uint8_t cur_pid, float fdb_param, TIM_HandleTypeDef* htim, uint32_t ch, TIM_HandleTypeDef* htim_enc, Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL)
        return;
    pmotor->encoder.last_update_time = HAL_GetTick();
    pmotor->type = type;
    pmotor->pid_num = pid_num;
    pmotor->cur_pid = cur_pid;
    pmotor->callback = callback;
    Filter_LowPassInit(fdb_param, &pmotor->fdb_fil_param);
    pmotor->cur_pid_div = 0;
    PID_ClearPID(&(pmotor->pid_cur));
    PID_ClearPID(&(pmotor->pid_spd));
    PID_ClearPID(&(pmotor->pid_pos));
    if (pmotor->type == Motor_TYPE_PWM_MOTOR) {
        PWM_InitPWM(&(pmotor->pwm), htim, ch);
        PWM_SetPWMFreq(&(pmotor->pwm), 500);
        PWM_SetPWMDuty(&(pmotor->pwm), 0);
        PWM_StartPWM(&(pmotor->pwm));
        pmotor->encoder.htim = htim_enc;
        if (htim_enc != NULL) {
            HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);
        }
    }
}

/**
 * @brief      Initialization of motor group
 * @param      pgroup: Pointer to motor group
 * @param      type: Type of motor (pwm or can)
 * @param      motor_num: Number of motor group
 * @param      phfdcan: Pointer of can handle
 * @param      stdid: Motor id
 * @retval     NULL
 */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, FDCAN_HandleTypeDef* phfdcan, uint16_t stdid) {
    if (pgroup == NULL)
        return;
    pgroup->motor_num = motor_num;
    pgroup->type = type;
    if (type == Motor_TYPE_CAN_MOTOR) {
        if (phfdcan == NULL)
            return;
        pgroup->can_handle = phfdcan;
        FDCAN_InitTxHander(&(pgroup->can_header), stdid, Const_Motor_MOTOR_TX_DLC, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    }
    for (int i = 0; i < 4; ++i) {
        pgroup->motor_handle[i] = NULL;
    }
}

/**
 * @brief      Reset motor PID
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void Motor_ResetMotorPID(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;
    PID_ClearPID(&(pmotor->pid_cur));
    PID_ClearPID(&(pmotor->pid_spd));
    PID_ClearPID(&(pmotor->pid_pos));
}

/**
 * @brief      Empty motor group PID
 * @param      pmotor_group: Pointer to motor group object
 * @retval     NULL
 */
void Motor_ResetMotorGroupPID(Motor_MotorGroupTypeDef* pmotor_group) {
    if (pmotor_group == NULL)
        return;
    for (int i = 0; i < pmotor_group->motor_num; ++i) {
        Motor_ResetMotorPID(pmotor_group->motor_handle[i]);
    }
}

/**
 * @brief      Get the target value of motor PID
 * @param      pmotor: Pointer to motor object
 * @retval     tagart value
 */
float Motor_GetMotorRef(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return 0.0f;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return 0.0f;
    if (pmotor->pid_num == 1) {
        return PID_GetPIDRef(&(pmotor->pid_spd));
    } else if (pmotor->pid_num == 2) {
        return PID_GetPIDRef(&(pmotor->pid_pos));
    }
    return 0;
}

/**
 * @brief      Get motor PID feedback value
 * @param      pmotor: Pointer to motor object
 * @retval     Feedback value
 */
float Motor_GetMotorFdb(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return 0.0f;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return 0.0f;
    if (pmotor->pid_num == 1) {
        return PID_GetPIDFdb(&(pmotor->pid_spd));
    } else if (pmotor->pid_num == 2) {
        return PID_GetPIDFdb(&(pmotor->pid_pos));
    }
    return 0;
}

/**
 * @brief      Get motor PID output value
 * @param      pmotor: Pointer to motor object
 * @retval     Output value
 */
float Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return 0.0f;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return 0.0f;
    if (pmotor->cur_pid) {
        return PID_GetPIDOutput(&(pmotor->pid_cur));
    } else {
        return PID_GetPIDOutput(&(pmotor->pid_spd));
    }
}

/**
 * @brief      Set motor PID feedback value
 * @param      pmotor: Pointer to motor object
 * @param      pid_no: PID Serial number
 * @param      fdb: Feedback value
 * @retval     NULL
 */
void Motor_SetMotorFdb(Motor_MotorTypeDef* pmotor, uint8_t pid_no, float fdb) {
    if (pmotor == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;

    if (pid_no == 0) {
        PID_SetPIDFdb(&(pmotor->pid_cur), fdb);
    } else if (pid_no == 1) {
        PID_SetPIDFdb(&(pmotor->pid_spd), fdb);
    } else if (pid_no == 2) {
        PID_SetPIDFdb(&(pmotor->pid_pos), fdb);
    }
}

/**
 * @brief      Set motor PID target value (no slope function)
 * @param      pmotor: Pointer to motor object
 * @param      ref: target value
 * @retval     NULL
 */
void Motor_SetMotorRef(Motor_MotorTypeDef* pmotor, float ref) {
    if (pmotor == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;
    if (pmotor->pid_num == 1) {
        PID_SetPIDRef(&(pmotor->pid_spd), ref);
    } else if (pmotor->pid_num == 2) {
        PID_SetPIDRef(&(pmotor->pid_pos), ref);
    }
}

/**
 * @brief      Set motor PID output value
 * @param      pmotor: Pointer to motor object
 * @param      output: target value
 * @retval     NULL
 */
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    if (pmotor == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;
    if (pmotor->cur_pid) {
        pmotor->pid_cur.output = output;
    } else {
        pmotor->pid_spd.output = output;
    }
}

/**
 * @brief      Calculation of motor PID output
 * @param      pmotor: Pointer to motor object
 * @param      pparam: Pointer to motor parameter object
 * @retval     NULL
 */
void Motor_CalcMotorOutput(Motor_MotorTypeDef* pmotor, Motor_MotorParamTypeDef* pparam) {
    if (pmotor == NULL || pparam == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;

    if (!pmotor->cur_pid || pmotor->cur_pid_div >= Const_CURRENT_PID_FREQ_DIV) {
        if (pmotor->pid_num == 1) {
            PID_CalcPID(&(pmotor->pid_spd), &(pparam->pid_param_spd));
        } else if (pmotor->pid_num == 2) {
            PID_CalcPID(&(pmotor->pid_pos), &(pparam->pid_param_pos));
            PID_SetPIDRef(&(pmotor->pid_spd), PID_GetPIDOutput(&(pmotor->pid_pos)));
            PID_CalcPID(&(pmotor->pid_spd), &(pparam->pid_param_spd));
        }
        pmotor->cur_pid_div = 0;
    }
    pmotor->cur_pid_div++;

    if (pmotor->cur_pid) {
        PID_SetPIDRef(&(pmotor->pid_cur), PID_GetPIDOutput(&(pmotor->pid_spd)));
        PID_CalcPID(&(pmotor->pid_cur), &(pparam->pid_param_cur));
    }
}

/**
 * @brief      Set motor PID target value overriding pid_num set
 * @param      pmotor: Pointer to motor object
 * @param      pid_no: Force pid ring number
 * @param      pparam: Pointer to motor parameter object
 * @retval     NULL
 */
void Motor_CalcMotorOutputRingOverrided(Motor_MotorTypeDef* pmotor, uint8_t pid_no, Motor_MotorParamTypeDef* pparam) {
    if (pmotor == NULL || pparam == NULL)
        return;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return;
    if (pid_no == 1) {
        PID_CalcPID(&(pmotor->pid_spd), &(pparam->pid_param_spd));
    } else if (pid_no == 2) {
        PID_CalcPID(&(pmotor->pid_pos), &(pparam->pid_param_pos));
        PID_SetPIDRef(&(pmotor->pid_spd), PID_GetPIDOutput(&(pmotor->pid_pos)));
        PID_CalcPID(&(pmotor->pid_spd), &(pparam->pid_param_spd));
    }
    if (pmotor->cur_pid) {
        PID_SetPIDRef(&(pmotor->pid_cur), PID_GetPIDOutput(&(pmotor->pid_spd)));
        PID_CalcPID(&(pmotor->pid_cur), &(pparam->pid_param_cur));
    }
}

/**
 * @brief      Calculate PID output of motor group
 * @param      pgroup: Pointer to motor group object
 * @param      pparam: Pointer to motor parameter object
 * @retval     NULL
 */
void Motor_CalcMotorGroupOutput(Motor_MotorGroupTypeDef* pgroup, Motor_MotorParamTypeDef* pparam) {
    if (pgroup == NULL || pparam == NULL)
        return;
    for (int i = 0; i < 4; ++i) {
        Motor_CalcMotorOutput(pgroup->motor_handle[i], pparam);
    }
}

/**
 * @brief      Sending motor PWM output
 * @param      pmotor: The pointer points to the motor to be sent
 * @retval     NULL
 */
void Motor_SendMotorPWMOutput(Motor_MotorTypeDef* pmotor) {
    // if (pmotor == NULL)
    return;
    if (pmotor->type != Motor_TYPE_PWM_MOTOR)
        return;
    float output = Motor_GetMotorOutput(pmotor);
    // satori
    // float duty = output * 0.00011136f + 0.53522f;
    float duty = output * 0.00011136f + 0.47522f;
    pmotor->duty = duty;
    // if (duty < 0.58f) duty = 0.58f;
    if (duty < 0.5f)
        duty = 0.5f;
    if (duty > 0.98f)
        duty = 0.98f;
    PWM_SetPWMDuty(&(pmotor->pwm), duty);
}

/**
 * @brief      Transmitter output
 * @param      pgroup: Pointer to the motor group to send
 * @retval     NULL
 */
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef* pgroup) {
    if (pgroup == NULL)
        return;
    if (pgroup->type == Motor_TYPE_CAN_MOTOR) {
        uint8_t txdata[8];
        txdata[0] = (uint8_t)((int16_t)Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
        txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
        txdata[2] = (uint8_t)((int16_t)Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
        txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]);
        txdata[4] = (uint8_t)((int16_t)Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
        txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
        txdata[6] = (uint8_t)((int16_t)Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
        txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);
        FDCAN_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
    }
    if (pgroup->type == Motor_TYPE_PWM_MOTOR) {
        for (int i = 0; i < pgroup->motor_num; ++i) {
            Motor_SendMotorPWMOutput(pgroup->motor_handle[i]);
        }
    }
}

/**
 * @brief      Read motor PWM encoder
 * @param      pmotor: The pointer points to the motor group to be sent
 * @retval     NULL
 */
void Motor_ReadPWMEncoder(Motor_MotorTypeDef* pmotor) {
    static int fdb = 0;

    if (pmotor == NULL)
        return;
    if (pmotor->type != Motor_TYPE_PWM_MOTOR)
        return;
    pmotor->encoder.direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(pmotor->encoder.htim);
    pmotor->encoder.counter = __HAL_TIM_GET_COUNTER(pmotor->encoder.htim);

    if (pmotor->encoder.direction == 0 /* 1? */)
        fdb = pmotor->encoder.counter;
    else
        fdb = 65535 - pmotor->encoder.counter;
    if (fdb == 65535)
        fdb = 0;

    __HAL_TIM_SET_COUNTER(pmotor->encoder.htim, 0);
    Motor_SetMotorFdb(pmotor, 1, Filter_LowPass(fdb, &pmotor->fdb_fil_param, &pmotor->fdb_fil));
}

/**
 * @brief      Judge whether any motor is offline
 * @param      NULL
 * @retval     Offline or not (1 is yes, 0 is no)
 */
uint8_t Motor_IsAnyMotorOffline() {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (Motor_IsMotorOffline(Motor_groupHandle[i]->motor_handle[j])) {
                // TODO: Global Error Handler
                return 1;
            }
        }
    }
    return 0;
}

/**
 * @brief      Judge whether the motor is offline
 * @param      pmotor: Pointer to motor object
 * @retval     Offline or not (1 is yes, 0 is no)
 */
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED)
        return 0;
    uint32_t now = HAL_GetTick();
    return (now - pmotor->encoder.last_update_time) > Const_Motor_MOTOR_OFFLINE_TIME;
}

/**
 * @brief      Decoding motor encoder feedback data
 * @param      rxdata: Information received by can
 * @param      pmotor: Pointer to motor object
 * @retval     NULL
 */
void Motor_DecodeEncoder(uint8_t rxdata[], Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL)
        return;

    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle = rxdata[0] << 8 | rxdata[1];
    pmotor->encoder.speed = rxdata[2] << 8 | rxdata[3];
    pmotor->encoder.current = rxdata[4] << 8 | rxdata[5];
    pmotor->encoder.temp = rxdata[6];

    pmotor->callback(pmotor);

    pmotor->encoder.last_update_time = HAL_GetTick();
}

#endif
