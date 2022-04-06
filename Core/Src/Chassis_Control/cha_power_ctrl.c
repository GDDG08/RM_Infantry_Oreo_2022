/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \Infantry_Oreo\Core\Src\Chassis_Control\cha_power_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-04 10:17:13
 */

#include "cha_power_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_POWER)

PowerCtrl_Data_t PowerCtrl_Data;

float CWPIDParam_temp[4][5] = {
    {75, 0, 0.5, 0, 11000},
    {-1, -1},
    {0, 0},
    {-1, -1}};
float CCWPIDParam_temp[4][5] = {
    {75, 0, 0.5, 0, 11000},
    {-1, -1},
    {0, 0},
    {-1, -1}};

PID_PIDParamTypeDef Chassis_SpeedCWPIDParam;
PID_PIDParamTypeDef Chassis_SpeedCCWPIDParam;
// PID_PIDParamTypeDef Chassis_SpeedCWPIDParam = {.kp = 30, .ki = 0.05, .kd = 3500, .sum_max = 4000, .output_max = 13000};
// PID_PIDParamTypeDef Chassis_SpeedCCWPIDParam = {.kp = 35, .ki = 0, .kd = 3700, .sum_max = 3000, .output_max = 16000};
PID_PIDParamTypeDef PowerCtrl_CurrentParam = {.kp = 1.3, .ki = 0.1, .kd = 130, .sum_max = 5000, .output_max = 16000};
PID_PIDParamTypeDef PowerCtrl_PIDParam = {.kp = 0.0045, .ki = 0.00057, .kd = 0, .sum_max = 1700, .output_max = 1};

float Power_ref = 60.0f;

/**
 * @brief   获取功率控制数据
 * @param 	None
 * @retval	None
 * @note	None
 */
PowerCtrl_Data_t* PowerCtrl_GetPowerDataPtr(void) {
    return &PowerCtrl_Data;
}

/**
 * @brief      Force change power contorl mode
 * @param      NULL
 * @retval     NULL
 */
void Power_ForceChangePowerMode(Power_ControlModeMnum mode) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();
    PowCtr->PowerCtrl_State = mode;
    // PowCtr->PowerCtrl_State = POWER_UNLIMITED;
}

/**
 * @brief 	功率控制初始化
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerCtrl_Init(Power_ControlModeMnum Ctrl_state, Motor_MotorGroupTypeDef* Mecanum) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    PowCtr->PowerCtrl_State = Ctrl_state;
    PowCtr->Mecanum_Chassis_Motor = Mecanum;

    PID_InitPIDParam(&Chassis_SpeedCWPIDParam, CWPIDParam_temp[0][0], CWPIDParam_temp[0][1], CWPIDParam_temp[0][2], CWPIDParam_temp[0][3], CWPIDParam_temp[0][4], CWPIDParam_temp[1][0], CWPIDParam_temp[1][1], CWPIDParam_temp[2][0], CWPIDParam_temp[2][1], CWPIDParam_temp[3][0], CWPIDParam_temp[3][1], PID_POSITION);
    PID_InitPIDParam(&Chassis_SpeedCCWPIDParam, CCWPIDParam_temp[0][0], CCWPIDParam_temp[0][1], CCWPIDParam_temp[0][2], CCWPIDParam_temp[0][3], CCWPIDParam_temp[0][4], CCWPIDParam_temp[1][0], CCWPIDParam_temp[1][1], CCWPIDParam_temp[2][0], CCWPIDParam_temp[2][1], CCWPIDParam_temp[3][0], CCWPIDParam_temp[3][1], PID_POSITION);

    Filter_LowPassInit(2 * PI * 0.001f * 2, &PowCtr->Mecanum_MotorCurrent_lpf_param[0]);
    Filter_LowPassInit(2 * PI * 0.001f * 2, &PowCtr->Mecanum_MotorCurrent_lpf_param[1]);
    Filter_LowPassInit(2 * PI * 0.001f * 2, &PowCtr->Mecanum_MotorCurrent_lpf_param[2]);
    Filter_LowPassInit(2 * PI * 0.001f * 2, &PowCtr->Mecanum_MotorCurrent_lpf_param[3]);

    Filter_LowPassInit(2 * PI * 0.001f * 5, &PowCtr->Mecanum_SpeedOutput_lpf_param[0]);
    Filter_LowPassInit(2 * PI * 0.001f * 5, &PowCtr->Mecanum_SpeedOutput_lpf_param[1]);
    Filter_LowPassInit(2 * PI * 0.001f * 5, &PowCtr->Mecanum_SpeedOutput_lpf_param[2]);
    Filter_LowPassInit(2 * PI * 0.001f * 5, &PowCtr->Mecanum_SpeedOutput_lpf_param[3]);

    Filter_AverInit(&PowCtr->Mecanum_SpeedOutput_aft[0], 20);
    Filter_AverInit(&PowCtr->Mecanum_SpeedOutput_aft[1], 20);
    Filter_AverInit(&PowCtr->Mecanum_SpeedOutput_aft[2], 20);
    Filter_AverInit(&PowCtr->Mecanum_SpeedOutput_aft[3], 20);

    PowCtr->Mecanum_motor_mode[0] = Motor_State_Stop;
    PowCtr->Mecanum_motor_mode[1] = Motor_State_Stop;
    PowCtr->Mecanum_motor_mode[2] = Motor_State_Stop;
    PowCtr->Mecanum_motor_mode[3] = Motor_State_Stop;

    PowCtr->Power_offset = 5.0f;

    PowCtr->ChassisDown_flag = 0;
    PowCtr->ChassisStarting_flag = 0;
}

/*------------------------------------------------------------------*/
/**
 * @brief      底盘速度环PID计算 主要就是实现加减速两套pid
 * @param
 * @retval
 */
void ChassisSpeedPID_StartingAndDownCalc(PID_PIDTypeDef* pid, uint8_t motor_num) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    if (PowCtr->Mecanum_motor_mode[motor_num] != Motor_State_Down) {
        PID_CalcPID(pid, &Chassis_SpeedCWPIDParam);
    } else {
        PID_CalcPID(pid, &Chassis_SpeedCCWPIDParam);
    }
}

/**
 * @brief      麦克纳姆轮底盘电流环PID计算 主要就是实现加速计算电流环 减速不计算
 * @param
 * @retval
 */
void ChassisCurrentPID_StartingAndDownCalc(uint8_t motor_num) {
    Chassis_ChassisTypeDef* chassis = Chassis_GetChassisControlPtr();
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    if (PowCtr->Mecanum_motor_mode[motor_num] != Motor_State_Down) {
        PowCtr->Mecanum_motor_current[motor_num] = Filter_LowPass(
            PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->encoder.current,
            &PowCtr->Mecanum_MotorCurrent_lpf_param[motor_num],
            &PowCtr->Mecanum_MotorCurrent_lpf[motor_num]);
        PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output = Filter_LowPass(PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output,
                                                                                                &PowCtr->Mecanum_SpeedOutput_lpf_param[motor_num], &PowCtr->Mecanum_SpeedOutput_lpf[motor_num]);
        PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output = Filter_Aver(PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output,
                                                                                             &PowCtr->Mecanum_SpeedOutput_aft[motor_num]);
        PID_SetPIDRef(&PowCtr->Mecanum_current_pid[motor_num], PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output);
        PID_SetPIDFdb(&PowCtr->Mecanum_current_pid[motor_num], PowCtr->Mecanum_motor_current[motor_num]);
        PID_CalcPID(&PowCtr->Mecanum_current_pid[motor_num], &PowerCtrl_CurrentParam);
    } else {
        PowCtr->Mecanum_current_pid[motor_num].output = PowCtr->Mecanum_Chassis_Motor->motor_handle[motor_num]->pid_spd.output;
    }
}

/**
 * @brief 	判断单个电机状态
 * @param 	None
 * @retval	None
 * @note	None

void JudgeMotor_Mode(PID_PIDTypeDef *speed_pid, uint8_t motor_num) {
    PowerCtrl_Data_t *PowCtr =PowerCtrl_GetPowerDataPtr();

    if ((speed_pid->ref) > 0.5f) {
        if (speed_pid->ref > 1.1f * (speed_pid->fdb)) {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Starting;
        } else if (speed_pid->ref < 0.9f * (speed_pid->fdb)) {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Down;
        } else {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Stable;
        }
    } else if (speed_pid->ref < -0.5f) {
        if (fabs(speed_pid->ref) > 1.1f * fabs(speed_pid->fdb)) {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Starting;
        } else if (fabs(speed_pid->ref) < 0.9f * fabs(speed_pid->fdb)) {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Down;
        } else {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Stable;
        }
    } else {
        if (fabs(speed_pid->fdb) > 2.0f) {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Down;
        } else {
            PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Stop;
        }
    }
}
*/

/**
 * @brief 	判断单个电机状态
 * @param 	None
 * @retval	None
 * @note	None
 */
void JudgeMotor_Mode(PID_PIDTypeDef* speed_pid, uint8_t motor_num, float last_ref) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    if (fabs(speed_pid->ref) - fabs(last_ref) >= 0.3f) {
        PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Starting;
    } else if (fabs(last_ref) - fabs(speed_pid->ref) >= 0.3f) {
        PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Down;
    } else if (fabs(speed_pid->ref) <= 1.0f) {
        PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Stop;
    } else {
        PowCtr->Mecanum_motor_mode[motor_num] = Motor_State_Stable;
    }
}
/**
 * @brief 	电机整体状态控制
 * @param 	None
 * @retval	None
 * @note	None
 */
void Motor_ModeControl(void) {
    Chassis_ChassisTypeDef* chassis = Chassis_GetChassisControlPtr();
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    CAP_CtrlDataTypeDef* capctrl = Cap_GetCapDataPtr();

    JudgeMotor_Mode(&PowCtr->Mecanum_Chassis_Motor->motor_handle[0]->pid_spd, 0, chassis->lastMotor_ref[0]);
    JudgeMotor_Mode(&PowCtr->Mecanum_Chassis_Motor->motor_handle[1]->pid_spd, 1, chassis->lastMotor_ref[1]);
    JudgeMotor_Mode(&PowCtr->Mecanum_Chassis_Motor->motor_handle[2]->pid_spd, 2, chassis->lastMotor_ref[2]);
    JudgeMotor_Mode(&PowCtr->Mecanum_Chassis_Motor->motor_handle[3]->pid_spd, 3, chassis->lastMotor_ref[3]);

    //起步电容状态机
    if (capctrl->starting_time >= 300 && capctrl->cap_mode_Starting == 1) {
        capctrl->starting_time = 0;
        capctrl->cap_mode_Starting = 0;

        if (PowCtr->Mecanum_motor_mode[0] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[1] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[2] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[3] == Motor_State_Starting) {
            capctrl->cap_mode_Starting = 1;

        } else {
            capctrl->cap_mode_Starting = 0;
        }
    } else if (capctrl->cap_mode_Starting == 1) {
        capctrl->starting_time++;
        capctrl->cap_mode_Starting = 1;
    } else {
        capctrl->cap_mode_Starting = 0;

        if (PowCtr->Mecanum_motor_mode[0] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[1] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[2] == Motor_State_Starting &&
            PowCtr->Mecanum_motor_mode[3] == Motor_State_Starting) {
            capctrl->cap_mode_Starting = 1;

        } else {
            capctrl->cap_mode_Starting = 0;
        }
    }

    if (capctrl->cap_mode_Starting == 1) {
        PowCtr->ChassisStarting_flag = 1;
    } else if (PowCtr->Mecanum_motor_mode[0] == Motor_State_Down &&
               PowCtr->Mecanum_motor_mode[1] == Motor_State_Down &&
               PowCtr->Mecanum_motor_mode[2] == Motor_State_Down &&
               PowCtr->Mecanum_motor_mode[3] == Motor_State_Down) {
        PowCtr->ChassisDown_flag = 1;
    } else {
        PowCtr->ChassisStarting_flag = 0;
        PowCtr->ChassisDown_flag = 0;
    }
}

/**
 * @brief 	功率PID计算
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerPID_Cal(float Powerref, float Powerfdb) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();
    Chassis_ChassisTypeDef* chassis = Chassis_GetChassisControlPtr();

    if (Powerfdb > Powerref) {
        PID_SetPIDRef(&PowCtr->Power_pid, Powerref);
        PID_SetPIDFdb(&PowCtr->Power_pid, Powerfdb);
        PID_CalcPID(&PowCtr->Power_pid, &PowerCtrl_PIDParam);
        PowCtr->Power_scale = PowCtr->Power_pid.output;
    } else {
        PowCtr->Power_scale = 0;
    }
}

/**
 * @brief 	功率偏移量计算
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerOffset_Cal(void) {
    Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();

    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();

    if (referee->chassis_power >= referee->max_chassis_power - 0.5) {
        if (referee->chassis_power_buffer <= 20) {
            PowCtr->Power_offset += 3.0f;
        } else {
            PowCtr->Power_offset += 0.5f;
        }
    } else if (referee->chassis_power <= referee->max_chassis_power - 8.0) {
        PowCtr->Power_offset -= 0.5f;
    }
    LimitMaxMin(PowCtr->Power_offset, 20.0f, 10.0f);
}

/**
 * @brief 	麦克纳姆轮系功控方案
 * @param 	None
 * @retval	None
 * @note	None
 */
void PowerCtrl(void) {
    PowerCtrl_Data_t* PowCtr = PowerCtrl_GetPowerDataPtr();
    Chassis_ChassisTypeDef* chassis = Chassis_GetChassisControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    CAP_CtrlDataTypeDef* capctrl = Cap_GetCapDataPtr();
    Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();

    if (PowCtr->PowerCtrl_State == POWER_UNLIMITED) {
        Motor_CalcMotorGroupOutput(&Motor_chassisMotors, chassis->current_param);
        for (uint8_t i = 0; i < 4; i++) {
            PowCtr->Mecanum_Chassis_Motor->motor_handle[i]->cur_pid = 0;
        }
    } else {
        for (uint8_t i = 0; i < 4; i++) {
            PowCtr->Mecanum_Chassis_Motor->motor_handle[i]->cur_pid = 1;
        }
        ChassisSpeedPID_StartingAndDownCalc(&PowCtr->Mecanum_Chassis_Motor->motor_handle[0]->pid_spd, 0);  //速度环PID计算
        ChassisSpeedPID_StartingAndDownCalc(&PowCtr->Mecanum_Chassis_Motor->motor_handle[1]->pid_spd, 1);
        ChassisSpeedPID_StartingAndDownCalc(&PowCtr->Mecanum_Chassis_Motor->motor_handle[2]->pid_spd, 2);
        ChassisSpeedPID_StartingAndDownCalc(&PowCtr->Mecanum_Chassis_Motor->motor_handle[3]->pid_spd, 3);

        ChassisCurrentPID_StartingAndDownCalc(0);  //电流环PID计算
        ChassisCurrentPID_StartingAndDownCalc(1);
        ChassisCurrentPID_StartingAndDownCalc(2);
        ChassisCurrentPID_StartingAndDownCalc(3);

        if (capctrl->cap_boost_mode == 1 || capctrl->cap_mode_Starting == 1)  //三种模式 急速、加速、普通匀速
        {
            PowerPID_Cal((Power_ref >= 0) ? Power_ref : 100.0f, capctrl->Sum_PowerReally);  //即功率期望极大
        } else if ((capctrl->cap_mode_Remote | capctrl->cap_mode_Stall) == 1) {
            PowerPID_Cal((Power_ref >= 0) ? Power_ref : ((float)referee->max_chassis_power + 20.0f), capctrl->Sum_PowerReally);
        } else {
            PowerOffset_Cal();  //计算OFFSET 保证不超功率
            PowerPID_Cal((Power_ref >= 0) ? Power_ref : ((float)(referee->max_chassis_power) - PowCtr->Power_offset), capctrl->Sum_PowerReally);
            // PowerPID_Cal(60.0f, capctrl->Sum_PowerReally);
        }

        if (referee->chassis_power_buffer <= 20) {
            PowCtr->Power_scale = 0.99;  //紧急避险 基本不会出现
        }

        //最终输出电流环结果乘以功率PID计算分度系数
        PowCtr->Mecanum_current_pid[0].output = (PowCtr->Mecanum_current_pid[0].output) * (1 - fabs(PowCtr->Power_scale));
        PowCtr->Mecanum_current_pid[1].output = (PowCtr->Mecanum_current_pid[1].output) * (1 - fabs(PowCtr->Power_scale));
        PowCtr->Mecanum_current_pid[2].output = (PowCtr->Mecanum_current_pid[2].output) * (1 - fabs(PowCtr->Power_scale));
        PowCtr->Mecanum_current_pid[3].output = (PowCtr->Mecanum_current_pid[3].output) * (1 - fabs(PowCtr->Power_scale));

        Motor_SetMotorOutput(PowCtr->Mecanum_Chassis_Motor->motor_handle[0], (PowCtr->Mecanum_current_pid[0].output));
        Motor_SetMotorOutput(PowCtr->Mecanum_Chassis_Motor->motor_handle[1], (PowCtr->Mecanum_current_pid[1].output));
        Motor_SetMotorOutput(PowCtr->Mecanum_Chassis_Motor->motor_handle[2], (PowCtr->Mecanum_current_pid[2].output));
        Motor_SetMotorOutput(PowCtr->Mecanum_Chassis_Motor->motor_handle[3], (PowCtr->Mecanum_current_pid[3].output));
    }
}

/**
 * @brief 	Motor output control
 * @param 	None
 * @retval	None
 * @note	None
 */
void Output_Control(void) {
    Motor_ModeControl();  //电机运动状态解算
    PowerCtrl();
}

//@OLD Version
// Power_DataTypeDef Power_data;
// Power_ControlModeMnum Power_ControlMode;

// /**
//   * @brief      Power control initialization
//   * @param      NULL
//   * @retval     NULL
//   */
// void Power_InitPower() {
//     Power_data.power_limit = 40.0f;
//     Power_data.warning_power = 30.0f;
//     Power_data.warning_power_buff = 40.0f;

//     Power_data.no_judge_total_current_limit = 64000.0f;
//     Power_data.buffer_total_current_limit = 16000.0f;
//     Power_data.power_total_current_limit = 20000.0f;

//     Power_ControlMode = POWER_LIMIT;
// }

// /**
//   * @brief      Gets the pointer to the power control data object
//   * @param      NULL
//   * @retval     Pointer to power control data object
//   */
// Power_DataTypeDef* Power_GetPowerDataPty() {
//     return &Power_data;
// }

// /**
//   * @brief      Force change power contorl mode
//   * @param      NULL
//   * @retval     NULL
//   */
// void Power_ForceChangePowerMode(Power_ControlModeMnum mode) {
//     Power_ControlMode = mode;
// }

// /**
//   * @brief      Set limit power (linear model)
//   * @param      NULL
//   * @retval     NULL
//   */
// void Power_SetLimitPower() {
//     Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();

//     Power_data.power_limit = (float)referee->max_chassis_power;
//     Power_data.warning_power = Power_data.power_limit * 0.5f;
//     Power_data.warning_power_buff = 40.0f;

//     Power_data.now_power = (float)referee->chassis_power;
//     Power_data.now_power_buff = (float)referee->chassis_power_buffer;
//     Power_data.total_current = 0.0f;
// }

// /**
//   * @brief      Power control
//   * @param      chassis: Pointer to chassis motor group struct
//   * @retval     NULL
//   */
// void Power_PowerControl(Motor_MotorGroupTypeDef* chassis) {
//     if (Power_ControlMode != POWER_LIMIT) {
//         return;
//     }

//     Power_SetLimitPower();

//     if (Power_data.now_power_buff < Power_data.warning_power_buff) {
//         float power_scale;
//         if (Power_data.now_power_buff > 5.0f) {
//             //scale down WARNING_POWER_BUFF
//             power_scale = Power_data.now_power_buff / Power_data.warning_power_buff;
//         } else {
//             //only left 0% of WARNING_POWER_BUFF
//             power_scale = 0.0f / Power_data.warning_power_buff;
//         }
//         Power_data.total_current_limit = Power_data.buffer_total_current_limit * power_scale;
//     }  //scale down
//     else {
//         //power > WARNING_POWER
//         if (Power_data.now_power > Power_data.warning_power) {
//             float power_scale;
//             //power < limited
//             if (Power_data.now_power < Power_data.power_limit) {
//                 //scale down
//                 power_scale = (Power_data.power_limit - Power_data.now_power) / (Power_data.power_limit - Power_data.warning_power);
//             } else {
//                 power_scale = 0.0f;
//             }  //power > limited
//             Power_data.total_current_limit = Power_data.buffer_total_current_limit + Power_data.power_total_current_limit * power_scale;
//         } else {
//             Power_data.total_current_limit = Power_data.buffer_total_current_limit + Power_data.power_total_current_limit;
//         }  //power < WARNING_POWER
//     }
//     //calculate the original motor current set
//     for (uint8_t i = 0; i < 4; i++) {
//         Power_data.total_current += fabs(chassis->motor_handle[i]->pid_spd.output);
//     }
//     if (Power_data.total_current > Power_data.total_current_limit) {
//         float current_scale = Power_data.total_current_limit / Power_data.total_current;
//         chassis->motor_handle[0]->pid_spd.output *= current_scale;
//         chassis->motor_handle[1]->pid_spd.output *= current_scale;
//         chassis->motor_handle[2]->pid_spd.output *= current_scale;
//         chassis->motor_handle[3]->pid_spd.output *= current_scale;
//     }
// }

#endif
