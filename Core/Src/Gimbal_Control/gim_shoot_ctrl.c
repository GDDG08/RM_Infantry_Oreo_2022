/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_shoot_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-14 14:28:04
 */

#include "gim_shoot_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_SHOOTER)

#include "gim_remote_ctrl.h"
#include "buscomm_ctrl.h"
#include "gim_gimbal_ctrl.h"
#include "const.h"
#include "cmsis_os.h"

#define SHOOTER_TASK_PERIOD 1
#define SHOOTER_SPEED_INCREMENT 8  // For gain :0.1

Motor_MotorParamTypeDef Shooter_shooterLeftMotorParam;
Motor_MotorParamTypeDef Shooter_shooterRightMotorParam;
Motor_MotorParamTypeDef Shooter_feederMotorParam;

Shoot_StatusTypeDef Shooter_ShooterControl;

/**
 * @brief          Shooter task
 * @param          NULL
 * @retval         NULL
 */
void Shoot_Task(void const* argument) {
    for (;;) {
        while (!GLOBAL_INIT_FLAG) {
            osDelay(1);
        }

        Shooter_Control();
        osDelay(SHOOTER_TASK_PERIOD);
    }
}

/**
 * @brief      shooter control initialization
 * @param      NULL
 * @retval     NULL
 */
void Shooter_InitShooter() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shooter_control = 1;

    shooter->feeder_mode = Feeder_NULL;
    shooter->heat_ctrl.shooter_17mm_cooling_heat = 0;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = 0;

    shooter->shooter_mode = Shoot_NULL;
    shooter->shoot_speed.feeder_shoot_speed = 0;
    shooter->shoot_speed.left_shoot_speed = 0;
    shooter->shoot_speed.right_shoot_speed = 0;

    shooter->shooter_speed_15mpers = Const_Shooter15mpers;
    shooter->shooter_speed_18mpers = Const_Shooter18mpers;
    shooter->shooter_speed_30mpers = Const_Shooter30mpers;
    // Shooter_SpeedOffsetFlashInit();

    shooter->change_shooter_mode_complete = 1;
    shooter->slope_output = 0;
    shooter->speed_limit = 0;
    shooter->slope_step = 200.0f;
    shooter->dertaRef = 0;

    Shooter_InitShooterMotor();

    Const_SetShooterPIDParam();
    // Initialization of motor parameters (including PID parameters)
    Shooter_HeatCtrlInit();
}

// /**
//  * @brief      shooter control initialization
//  * @param      NULL
//  * @retval     NULL
//  */
// void Shooter_SpeedOffsetFlashInit() {
//     Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
//     uint32_t speed_15_off, speed_18_off, speed_30_off;
//     Flash_ReadData(SHOOT_15M_SPEED_ADDRESS, &speed_15_off, 1);
//     Flash_ReadData(SHOOT_18M_SPEED_ADDRESS, &speed_18_off, 1);
//     Flash_ReadData(SHOOT_30M_SPEED_ADDRESS, &speed_30_off, 1);

//     if (speed_15_off > 300) {
//         speed_15_off = 150;
//         Flash_EraseAddress(SHOOT_15M_SPEED_ADDRESS, 1);
//         Flash_WriteSingleAddress(SHOOT_15M_SPEED_ADDRESS, &speed_15_off, 1);
//         Flash_ReadData(SHOOT_15M_SPEED_ADDRESS, &speed_15_off, 1);
//     }
//     if (speed_18_off > 300) {
//         speed_18_off = 150;
//         Flash_EraseAddress(SHOOT_18M_SPEED_ADDRESS, 1);
//         Flash_WriteSingleAddress(SHOOT_18M_SPEED_ADDRESS, &speed_18_off, 1);
//         Flash_ReadData(SHOOT_18M_SPEED_ADDRESS, &speed_18_off, 1);
//     }
//     if (speed_30_off > 300) {
//         speed_30_off = 150;
//         Flash_EraseAddress(SHOOT_30M_SPEED_ADDRESS, 1);
//         Flash_WriteSingleAddress(SHOOT_30M_SPEED_ADDRESS, &speed_30_off, 1);
//         Flash_ReadData(SHOOT_30M_SPEED_ADDRESS, &speed_30_off, 1);
//     }
//     shooter->speed_offset_flash.speed_15mm_offset = speed_15_off;
//     shooter->speed_offset_flash.speed_18mm_offset = speed_18_off;
//     shooter->speed_offset_flash.speed_30mm_offset = speed_30_off;
//     Shooter_GetShootSpeedOffset();
// }

/**
 * @brief      Get speed offset variable    (150 means offset is 0)
 * @param      NULL
 * @retval     NULL
 */
float Shooter_GetShootSpeedOffset() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    float offset_speed;

    shooter->shoot_speed_offset.speed_15mm_offset = ((((float)(shooter->speed_offset_flash.speed_15mm_offset)) - 150.0f)) / 10.0f;
    shooter->shoot_speed_offset.speed_18mm_offset = ((((float)(shooter->speed_offset_flash.speed_18mm_offset)) - 150.0f)) / 10.0f;
    shooter->shoot_speed_offset.speed_30mm_offset = ((((float)(shooter->speed_offset_flash.speed_30mm_offset)) - 150.0f)) / 10.0f;

    switch (buscomm->speed_17mm_limit) {
        case REFEREE_SHOOTER_SPEED_15:
            offset_speed = shooter->shoot_speed_offset.speed_15mm_offset;
            break;
        case REFEREE_SHOOTER_SPEED_18:
            offset_speed = shooter->shoot_speed_offset.speed_18mm_offset;
            break;
        case REFEREE_SHOOTER_SPEED_30:
            offset_speed = shooter->shoot_speed_offset.speed_30mm_offset;
            break;
        default:
            offset_speed = 15.0f;
            break;
    }

    return offset_speed;
}

/**
 * @brief      shooter control initialization
 * @param      multiple :  -1  : reduce a SHOOTER_SPEED_INCREMENT
 *                         +1  : add a SHOOTER_SPEED_INCREMENT
 *                         else: multiple SHOOTER_SPEED_INCREMENT
 * @retval     NULL
 */
// void Shooter_ModifySpeedOffset(int8_t multiple) {
//     BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
//     Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
//     uint32_t offset_speed;
//     switch (buscomm->speed_17mm_limit) {
//         case 15:
//             Flash_ReadData(SHOOT_15M_SPEED_ADDRESS, &offset_speed, 1);
//             offset_speed += (SHOOTER_SPEED_INCREMENT * multiple);
//             Flash_EraseAddress(SHOOT_15M_SPEED_ADDRESS, 1);
//             Flash_WriteSingleAddress(SHOOT_15M_SPEED_ADDRESS, &offset_speed, 1);
//             Flash_ReadData(SHOOT_15M_SPEED_ADDRESS, &offset_speed, 1);
//             shooter->speed_offset_flash.speed_15mm_offset = offset_speed;
//             break;
//         case 18:
//             Flash_ReadData(SHOOT_18M_SPEED_ADDRESS, &offset_speed, 1);
//             offset_speed += (SHOOTER_SPEED_INCREMENT * multiple);
//             Flash_EraseAddress(SHOOT_18M_SPEED_ADDRESS, 1);
//             Flash_WriteSingleAddress(SHOOT_18M_SPEED_ADDRESS, &offset_speed, 1);
//             Flash_ReadData(SHOOT_18M_SPEED_ADDRESS, &offset_speed, 1);
//             shooter->speed_offset_flash.speed_18mm_offset = offset_speed;
//             break;
//         case 30:
//             Flash_ReadData(SHOOT_30M_SPEED_ADDRESS, &offset_speed, 1);
//             offset_speed += (SHOOTER_SPEED_INCREMENT * multiple);
//             Flash_EraseAddress(SHOOT_30M_SPEED_ADDRESS, 1);
//             Flash_WriteSingleAddress(SHOOT_30M_SPEED_ADDRESS, &offset_speed, 1);
//             Flash_ReadData(SHOOT_30M_SPEED_ADDRESS, &offset_speed, 1);
//             shooter->speed_offset_flash.speed_30mm_offset = offset_speed;
//             break;
//         default:
//             break;
//     }
//     Shooter_GetShootSpeedOffset();
// }

/**
 * @brief      Shooter control
 * @param      NULL
 * @retval     NULL
 */
void Shooter_Control() {
    Shooter_UpdataControlData();

    Shooter_Overspeedtest();

    Shooter_ShootControl();

    Shooter_FeederControl();

    Shooter_ShooterMotorOutput();
}

/**
 * @brief      Gets the pointer to the shooter control data object
 * @param      NULL
 * @retval     Pointer to shooter control data object
 */
Shoot_StatusTypeDef* Shooter_GetShooterControlPtr() {
    return &Shooter_ShooterControl;
}

/**
 * @brief      Change frequent mode
 * @param      mode: Shooter mode
 * @retval     NULL
 */
void Shooter_ChangeShooterMode(Shoot_ShooterModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    // TODO
    // if (buscomm->main_shooter_power == 1)
    shooter->shooter_mode = mode;
    // else
    // shooter->shooter_mode = Shoot_NULL;
}

/**
 * @brief      Change shooter mode
 * @param      mode: Feeder mode
 * @retval     NULL
 */
void Shooter_ChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    if (shooter->feeder_mode == Feeder_LOCKED_ROTOR)
        return;
    shooter->last_feeder_mode = shooter->feeder_mode;
    shooter->feeder_mode = mode;
    if ((shooter->feeder_mode != shooter->last_feeder_mode) &&
        ((shooter->last_feeder_mode == Feeder_LOW_CONTINUE) ||
         (shooter->last_feeder_mode == Feeder_FAST_CONTINUE) ||
         (shooter->last_feeder_mode == Feeder_REFEREE))) {
        shooter->feeder_mode = Feeder_FINISH;
        Shooter_AngleCorrect();
    }
}

/**
 * @brief      Initialize Shooter Motor
 * @param      NULL
 * @retval     NULL
 */
void Shooter_InitShooterMotor() {
    // HAL_Delay(2000);
    for (int i = 0; i < 7; i++) {
        Motor_shooterMotorLeft.pwm.duty = 0.1 * i;
        Motor_shooterMotorRight.pwm.duty = 0.1 * i;
        Motor_SendMotorGroupOutput(&Motor_shooterMotors);
        HAL_Delay(200);
    }
}

/**
 * @brief      Initialize Shooter heat control
 * @param      NULL
 * @retval     NULL
 */
void Shooter_HeatCtrlInit() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
}

/**
 * @brief      Set referee shooter speed
 * @param      NULL
 * @retval     NULL
 */
float Shooter_GetRefereeSpeed() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    float speed;
    switch (buscomm->speed_17mm_limit) {
        case REFEREE_SHOOTER_SPEED_15:
            speed = shooter->shooter_speed_15mpers;
            break;
        case REFEREE_SHOOTER_SPEED_18:
            speed = shooter->shooter_speed_18mpers;
            break;
        case REFEREE_SHOOTER_SPEED_30:
            speed = shooter->shooter_speed_30mpers;
            break;
        default:
            speed = shooter->shooter_speed_15mpers;
            break;
    }

    return speed;
}
uint8_t Shooter_GetRefereeOverSpeed() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    return buscomm->speed_17mm_fdb;
}

/**
 * @brief      Updata control data
 * @param      NULL
 * @retval     NULL
 */
void Shooter_UpdataControlData() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    shooter->heat_ctrl.shooter_17mm_cooling_heat = (float)buscomm->heat_17mm;
    shooter->heat_ctrl.shooter_17mm_cooling_rate = (float)buscomm->heat_cooling_limit;

    Motor_ReadPWMEncoder(&Motor_shooterMotorLeft);
    Motor_ReadPWMEncoder(&Motor_shooterMotorRight);

    //    Shooter_FeederMotorLockedJudge();
}

/**
 * @brief      Set feeder motor speed
 * @param      speed: Feeder motor speed ref
 * @retval     NULL
 */
void Shooter_SetFeederSpeed(float speed) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shoot_speed.feeder_shoot_speed = speed;
}

/**
 * @brief      Set shooter motor speed
 * @param      speed: shooter motor speed ref
 * @retval     NULL
 */
void Shooter_SetShooterSpeed(float speed) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->shoot_speed.left_shoot_speed = speed;
    shooter->shoot_speed.right_shoot_speed = speed;
}

/**
 * @brief      Force change shooter mode
 * @param      mode: Feeder mode
 * @retval     NULL
 */
void Shooter_ForceChangeFeederMode(Shoot_FeederModeEnum mode) {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->feeder_mode = mode;
}

/**
 * @brief      Motor locked rotor judge
 * @param      NULL
 * @retval     NULL
 */
void Shooter_FeederMotorLockedJudge() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    static int count = 0;
    if (shooter->feeder_mode != Feeder_LOCKED_ROTOR) {
        if ((abs(Motor_feederMotor.encoder.current) >= Const_ShooterLockedCurrent) &&
            (abs(Motor_feederMotor.encoder.speed) <= Const_ShooterLockedSpeed)) {
            count++;
            if (count > Const_ShooterLockedTime) {
                Shooter_ForceChangeFeederMode(Feeder_LOCKED_ROTOR);
            }
        } else
            count = 0;
    }
}

/**
 * @brief      Motor locked handle
 * @param      NULL
 * @retval     NULL
 */
void Shooter_MotorLockedHandle() {
    static int count_reverse = 0;
    Shooter_SetFeederSpeed(Const_ShooterLockedReverseSpeed);
    count_reverse++;
    if (count_reverse >= Const_ShooterRelockedTime) {
        count_reverse = 0;
        Shooter_ForceChangeFeederMode(Feeder_NULL);
    }
}

/**
 * @brief      Correct stop angle
 * @param      NULL
 * @retval     NULL
 */
void Shooter_AngleCorrect() {
    Motor_feederMotor.pid_pos.ref = Motor_feederMotor.pid_pos.fdb;
    //    Motor_feederMotor.pid_pos.ref = ((int)(Motor_feederMotor.pid_pos.fdb + 40.0f) / 45) * 45;
}

void Shooter_RealAngleCorrect() {
    Motor_feederMotor.pid_pos.ref = ((int)(Motor_feederMotor.pid_pos.fdb + 40.0f) / 45) * 45;
}

/**
 * @brief      Shooter heat control
 * @param      NULL
 * @retval     pid_num
 */
uint8_t Shooter_HeatCtrl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    // Judge if cooling rate
    if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= Const_HeatCtrlFastLimit) {  // sufficient heat remain, fast shooting
        shooter->heat_ctrl.current_speed = Const_FeederFastSpeed;
        shooter->heat_ctrl.current_pidnum = 1;
        Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
        shooter->heat_ctrl.heat_tracking = 0;
    } else if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) >= Const_HeatCtrlSlowLimit) {
        shooter->heat_ctrl.current_speed = Const_FeederSlowSpeed;
        shooter->heat_ctrl.current_pidnum = 1;
        Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
        shooter->heat_ctrl.heat_tracking = 0;
    } else if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlWaitLimit) {
        shooter->heat_ctrl.current_speed = Const_FeederWaitSpeed;
        shooter->heat_ctrl.current_pidnum = 1;
        Shooter_SetFeederSpeed(shooter->heat_ctrl.current_speed);
        shooter->heat_ctrl.heat_tracking = 0;
    } else if ((shooter->heat_ctrl.shooter_17mm_cooling_rate - shooter->heat_ctrl.shooter_17mm_cooling_heat) <= Const_HeatCtrlStopLimit) {
        // insufficient heat remain, single shooting
        //   shooter->heat_ctrl.heat_tracking += shooter->heat_ctrl.heat_tracking / 1000.0;
        //   if (shooter->heat_ctrl.heat_tracking >= Const_HeatCtrlSingleCount) {
        //       shooter->heat_ctrl.heat_tracking = 0;
        //       Shooter_SingleShootReset();
        //   }
        //   Shooter_SingleShootCtrl();
        //   shooter->heat_ctrl.current_pidnum = 2
        // no heat remain, stop shooting
        shooter->heat_ctrl.heat_tracking = 0;
        Shooter_AngleCorrect();
        shooter->heat_ctrl.current_pidnum = 2;
    }

    return shooter->heat_ctrl.current_pidnum;
}

/**
 * @brief      Shooter control
 * @param      NULL
 * @retval     NULL
 */
void Shooter_Overspeedtest() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    float referee_speed;
    if (shooter->speed_limit == 0) {
        switch (buscomm->speed_17mm_limit) {
            case REFEREE_SHOOTER_SPEED_15:
                referee_speed = 15;
                break;
            case REFEREE_SHOOTER_SPEED_18:
                referee_speed = 18;
                break;
            case REFEREE_SHOOTER_SPEED_30:
                referee_speed = 30;
                break;
            default:
                referee_speed = 15;
                break;
        }
        if (buscomm->speed_17mm_fdb > referee_speed)
            shooter->speed_limit = 1;
    }
}
void Shooter_CalcRef() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    if (shooter->change_shooter_mode_complete) {
        shooter->ref_output = shooter->shoot_speed.left_shoot_speed - shooter->speed_limit;
    } else {
        shooter->ref_output = shooter->shoot_speed.left_shoot_speed - shooter->slope_output - shooter->speed_limit;
        shooter->slope_output -= shooter->dertaRef / shooter->slope_step;
    }
}
void Shooter_ShootControl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    switch (shooter->shooter_mode) {
        case Shoot_NULL:
#if !__FN_IF_ENABLE(__FN_DEBUG_BTLOG)
//            GPIO_Reset(LASER);
#endif
            GPIO_Reset(BULLET_CHARGING);
            Shooter_SetShooterSpeed(0);
            break;
        case Shoot_FAST:
            //            GPIO_Set(LASER);
            GPIO_Set(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Const_ShooterFastSpeed);
            break;
        case Shoot_SLOW:
            //            GPIO_Set(LASER);
            GPIO_Set(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Const_ShooterSlowSpeed);
            break;
        case Shoot_REFEREE:
            //            GPIO_Set(LASER);
            GPIO_Set(BULLET_CHARGING);
            Shooter_SetShooterSpeed(Shooter_GetRefereeSpeed() /*+ Shooter_GetShootSpeedOffset()*/);
            break;
        default:
            break;
    }
    if (shooter->slope_direction) {
        if (shooter->slope_output > 0) {
            shooter->change_shooter_mode_complete = 1;
        }
    } else {
        if (shooter->slope_output < 0) {
            shooter->change_shooter_mode_complete = 1;
        }
    }

    if (shooter->last_shoot_speed_ref != shooter->shoot_speed.left_shoot_speed) {
        shooter->change_shooter_mode_complete = 0;
        shooter->slope_direction = shooter->last_shoot_speed_ref > shooter->shoot_speed.left_shoot_speed ? 1 : 0;  // 0上升1下降
        shooter->dertaRef = shooter->shoot_speed.left_shoot_speed - shooter->last_shoot_speed_ref;
        shooter->slope_output = shooter->dertaRef;
    }
    Shooter_CalcRef();
    Motor_SetMotorRef(&Motor_shooterMotorRight, shooter->ref_output);
    Motor_SetMotorRef(&Motor_shooterMotorLeft, shooter->ref_output);
    shooter->last_shoot_speed_ref = shooter->shoot_speed.left_shoot_speed;
    // Motor_SetMotorRef(&Motor_shooterMotorRight, shooter->shoot_speed.left_shoot_speed);
    // Motor_SetMotorRef(&Motor_shooterMotorLeft, shooter->shoot_speed.left_shoot_speed);

#if __FN_IF_ENABLE(__FN_SHOOTER_PID)
    Motor_CalcMotorOutput(&Motor_shooterMotorRight, &Shooter_shooterRightMotorParam);
    Motor_CalcMotorOutput(&Motor_shooterMotorLeft, &Shooter_shooterLeftMotorParam);
#endif
}

/**
 * @brief      Shooter feeder control: single shooting
 * @param      NULL
 * @retval     NULL
 */
void Shooter_SingleShootCtrl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    if (fabs(Motor_feederMotor.pid_pos.fdb - Motor_feederMotor.pid_pos.ref) > 1.0f) {  // feeder motor not ready
        // return;     // do nothing
    }
    if (!shooter->single_shoot_done) {  // not shoot yet
        Motor_feederMotor.pid_pos.ref += 45.0f;
        shooter->single_shoot_done = 1;
    }
}

/**
 * @brief      Shooter feeder control: reset single shooting
 * @param      NULL
 * @retval     NULL
 */
void Shooter_SingleShootReset() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    shooter->single_shoot_done = 0;
}

/**
 * @brief      Shooter feeder control
 * @param      NULL
 * @retval     NULL
 */
void Shooter_FeederControl() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    int current_pid_num = 0;
    switch (shooter->feeder_mode) {
        case Feeder_NULL:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(0);
            break;
        case Feeder_SINGLE:
            current_pid_num = 2;
            Shooter_SingleShootCtrl();
            break;
        case Feeder_FAST_CONTINUE:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(Const_FeederFastSpeed);
            break;
        case Feeder_LOW_CONTINUE:
            current_pid_num = 1;
            Shooter_SetFeederSpeed(Const_FeederSlowSpeed);
            break;
        case Feeder_LOCKED_ROTOR:
            current_pid_num = 1;
            Shooter_MotorLockedHandle();
            break;
        case Feeder_REFEREE:
            current_pid_num = Shooter_HeatCtrl();
            break;
        case Feeder_FINISH:
            current_pid_num = 2;
            break;
        default:
            break;
    }

    Motor_feederMotor.pid_spd.ref = shooter->shoot_speed.feeder_shoot_speed;
    Motor_CalcMotorOutputRingOverrided(&Motor_feederMotor, current_pid_num, &Shooter_feederMotorParam);
}

/**
 * @brief      Output shooter motor
 * @param      NULL
 * @retval     NULL
 */
void Shooter_ShooterMotorOutput() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    if (shooter->shooter_control == 1) {
        Motor_SendMotorGroupOutput(&Motor_shooterMotors);
        Motor_SendMotorGroupOutput(&Motor_feederMotors);
    }
}

#endif
