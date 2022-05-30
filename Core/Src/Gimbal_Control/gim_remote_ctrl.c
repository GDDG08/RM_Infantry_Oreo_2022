/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_remote_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-30 17:30:01
 */

#include "gim_remote_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_REMOTE)

#include "const.h"
#include "buscomm_ctrl.h"
#include "gim_gimbal_ctrl.h"
#include "gim_ins_ctrl.h"
#include "gim_miniPC_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_login_ctrl.h"

Remote_RemoteControlTypeDef Remote_remoteControlData;

Math_SlopeParamTypeDef Remote_ChassisFBSlope;
Math_SlopeParamTypeDef Remote_ChassisRLSlope;

/*          Remote bessel_filter                */
Filter_Bessel_TypeDef Remote_forward_backFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_right_leftFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_mouse_y_Filter = {0, 0, 0};

const float REMOTE_PITCH_ANGLE_TO_REF = 0.0005f;
const float REMOTE_YAW_ANGLE_TO_REF = 0.0015f;

/**
 * @brief      Remote Control Init
 * @param      NULL
 * @retval     NULL
 */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef* control_data = Remote_GetControlDataPtr();

    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_SLOWDOWN);
    Math_InitSlopeParam(&Remote_ChassisRLSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_SLOWDOWN);
}

/**
 * @brief      Gets the pointer to the remote control data object
 * @param      NULL
 * @retval     Pointer to remote control data object
 */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}

/**
 * @brief      Remote control command
 * @param      NULL
 * @retval     NULL
 */
void Remote_ControlCom() {
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    Remote_RemoteControlTypeDef* control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();

    control_data->pending = 1;

    // Login_LoginCmd();

    switch (data->remote.s[1]) {
            /*      right switch control mode   */
        case Remote_SWITCH_UP: {
            /* right switch up is remote normal mode */
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            Gimbal_ChangeMode(Gimbal_NOAUTO);
            Remote_RemoteProcess();
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* right switch mid is keymouse mode    */
            Remote_KeyMouseProcess();
            Remote_MouseShooterModeSet();
            //            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            Remote_Gesture();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* right switch down is auto aim mode   */
            // Gimbal_ChangeMode(Gimbal_ARMOR);
            // MiniPC_ChangeAimMode(MiniPC_ARMOR);
            Gimbal_ChangeMode(Gimbal_NOAUTO);
            MiniPC_ChangeAimMode(MiniPC_BIG_BUFF);
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            Remote_RemoteShooterModeSet();
            Remote_Gesture();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}

/**
 * @brief      Mouse shoot mode set
 * @param      NULL
 * @retval     NULL
 */
void Remote_MouseShooterModeSet() {
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();

    // Prevent launching without opening the friction wheel
    if ((shooter->shooter_mode != Shoot_REFEREE) || (Motor_shooterMotorLeft.pid_spd.fdb <= 30) || (Motor_shooterMotorRight.pid_spd.fdb <= 30)) {
        Shooter_ChangeFeederMode(Feeder_FINISH);
        return;
    }

    static int count_mouse_L = 0;
    if (data->mouse.l == 1) {
        count_mouse_L++;
        if (count_mouse_L >= 50) {
            Shooter_ChangeFeederMode(Feeder_REFEREE);
            count_mouse_L = 50;
        }
    } else {
        if (0 < count_mouse_L && count_mouse_L < 50) {
            Shooter_SingleShootReset();
            Shooter_ChangeFeederMode(Feeder_SINGLE);
        } else
            Shooter_ChangeFeederMode(Feeder_FINISH);
        count_mouse_L = 0;
    }
}

/**
 * @brief      Remote shoot mode set
 * @param      NULL
 * @retval     NULL
 */
void Remote_RemoteShooterModeSet() {
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();

    switch (data->remote.s[0]) {
            /*      left switch control mode   */
        case Remote_SWITCH_UP: {
            /* left switch up is fast shooting */
            Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* left switch mid is stop shooting    */
            Shooter_ChangeShooterMode(Shoot_REFEREE);
            Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
            Shooter_ChangeShooterMode(Shoot_REFEREE);
            // Shooter_ChangeFeederMode(Feeder_LOW_CONTINUE);
            Shooter_ChangeFeederMode(Feeder_REFEREE);

            if ((Motor_shooterMotorLeft.pid_spd.fdb >= 30) && (Motor_shooterMotorRight.pid_spd.fdb >= 30)) {
                Shooter_ChangeFeederMode(Feeder_REFEREE);
            } else
                Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief      Remote control process
 * @param      NULL
 * @retval     NULL
 */
void Remote_RemoteProcess() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();

    buscomm->chassis_fb_ref = Filter_Bessel((float)data->remote.ch[1], &Remote_forward_backFilter) * 0.5f;
    buscomm->chassis_lr_ref = Filter_Bessel((float)data->remote.ch[0], &Remote_right_leftFilter) * 0.5f;

    if (data->remote.ch[4] <= -500.0f)
        Remote_ChangeChassisState(CHASSIS_CTRL_GYRO);
    else
        Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);

    if (data->remote.ch[4] >= 500.0f)
        // Servo_SetServoAngle(&Servo_ammoContainerCapServo, 300);
        buscomm->cap_boost_mode_user = SUPERCAP_BOOST;

    if (data->remote.ch[4] <= 500.0f)
        // Servo_SetServoAngle(&Servo_ammoContainerCapServo, -30);
        buscomm->cap_boost_mode_user = SUPERCAP_UNBOOST;

    // buscomm->cap_boost_mode_user = SUPERCAP_UNBOOST;
    /*      remote chassis reference value bessel filter    */
    float yaw_ref,
        pitch_ref;

    yaw_ref = (float)data->remote.ch[2] * REMOTE_YAW_ANGLE_TO_REF;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF;

    Gimbal_SetYawRefDelta(yaw_ref);
    Gimbal_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
}

/**
 * @brief      KeyMouse control process
 * @param      NULL
 * @retval     NULL
 */
void Remote_KeyMouseProcess() {
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();
    Remote_RemoteControlTypeDef* control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    float max_chassis_speed;
    /************Control mode choise**************/

    if (data->key.x == 1) {
    }

    if (data->key.c == 1 && data->key.shift == 1) {
        buscomm->ui_cmd = 1;
    } else {
        buscomm->ui_cmd = 0;
    }
    if (data->key.c == 1) {
    }

    /*******R relode projectile******/
    static int flag_relode = 0;
    if (data->key.r == 1) {
        if ((Servo_GetServoAngle(&Servo_ammoContainerCapServo) != 300) && (flag_relode == 1)) {
            Servo_SetServoAngle(&Servo_ammoContainerCapServo, 300);
            flag_relode = 0;
        } else if ((Servo_GetServoAngle(&Servo_ammoContainerCapServo) != 0) && (flag_relode == 1)) {
            Servo_SetServoAngle(&Servo_ammoContainerCapServo, -30);
            flag_relode = 0;
        }
    } else
        flag_relode = 1;

    /******Chassis mode control*******/
    static int gyro_flag = 0, gyro_state = 0;
    if (buscomm->chassis_mode != CHASSIS_CTRL_STOP) {
        if (data->key.ctrl == 1) {  // ctrl gyro mode
            if ((gyro_state == 0) && (gyro_flag == 1)) {
                gyro_state = 1;
                gyro_flag = 0;
            } else if ((gyro_state == 1) && (gyro_flag == 1)) {
                gyro_state = 0;
                gyro_flag = 0;
            }
        } else
            gyro_flag = 1;
        if (gyro_state == 1) {
            Remote_ChangeChassisState(CHASSIS_CTRL_GYRO);
            max_chassis_speed = MOUSE_CHASSIS_MAX_GYRO_SPEED;
        } else {
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            max_chassis_speed = MOUSE_CHASSIS_MAX_SPEED;
        }
    }

    /******Gimbal mode control*******/
    static int big_energy_flag = 0, big_energy_state = 0;
    static int small_energy_flag = 0, small_energy_state = 0;
    if (data->key.b == 1) {
        if ((big_energy_state == 0) && (big_energy_flag == 1)) {
            big_energy_state = 1;
            big_energy_flag = 0;
        } else if ((big_energy_state == 1) && (big_energy_flag == 1)) {
            big_energy_state = 0;
            big_energy_flag = 0;
        }
    } else
        big_energy_flag = 1;

    if (data->key.v == 1) {
        if ((small_energy_state == 0) && (small_energy_flag == 1)) {
            small_energy_state = 1;
            small_energy_flag = 0;
        } else if ((small_energy_state == 1) && (small_energy_flag == 1)) {
            small_energy_state = 0;
            small_energy_flag = 0;
        }
    } else
        small_energy_flag = 1;

    if (data->mouse.r == 1) {
        Gimbal_ChangeMode(Gimbal_ARMOR);
        MiniPC_ChangeAimMode(MiniPC_ARMOR);
    } else if (data->mouse.r == 0) {
        if (big_energy_state == 1 && small_energy_state == 0) {
            Gimbal_ChangeMode(Gimbal_BIG_ENERGY);
            MiniPC_ChangeAimMode(MiniPC_BIG_BUFF);
            // chassis stop
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
        }

        else if (big_energy_state == 0 && small_energy_state == 1) {
            Gimbal_ChangeMode(Gimbal_SMALL_ENERGY);
            MiniPC_ChangeAimMode(MiniPC_SMALL_BUFF);
            // chassis stop
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
        } else {
            if (big_energy_state == 1 && small_energy_state == 1) {
                big_energy_state = 0;
                small_energy_state = 0;
            }
            Gimbal_ChangeMode(Gimbal_NOAUTO);
            MiniPC_ChangeAimMode(MiniPC_ARMOR);

            if (gyro_state == 1) {
                Remote_ChangeChassisState(CHASSIS_CTRL_GYRO);
                max_chassis_speed = MOUSE_CHASSIS_MAX_GYRO_SPEED;
            } else {
                Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
                max_chassis_speed = MOUSE_CHASSIS_MAX_SPEED;
            }
        }
    }
    /*******State control of friction wheel*********/
    if (data->key.q == 1)  // Q Press to open the friction wheel
        Shooter_ChangeShooterMode(Shoot_REFEREE);
    if (data->key.e == 1)  // E Press to close the friction wheel
        Shooter_ChangeShooterMode(Shoot_NULL);

    /*******Radio frequency fire rate control*******/
    if (data->key.f == 1) {
    }

    if (data->key.g == 1) {
    }

    /*if you move you will exit the auto mode*/
    if (((data->key.w == 1) || (data->key.a == 1) || (data->key.d == 1) || (data->key.s == 1)) &&
        (buscomm->chassis_mode == CHASSIS_CTRL_STOP)) {
        Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
    }

    /******** supercap control ********/
    static int cap_flag = 0;
    if (data->key.z == 1) {
        if (cap_flag == 1) {
            if (buscomm->cap_mode_user == SUPERCAP_CTRL_OFF) {
                buscomm->cap_mode_user = SUPERCAP_CTRL_ON;
            } else if (buscomm->cap_mode_user == SUPERCAP_CTRL_ON) {
                buscomm->cap_mode_user = SUPERCAP_CTRL_OFF;
            }
            cap_flag = 0;
        }
    } else
        cap_flag = 1;

    if (data->key.shift == 1) {
        buscomm->cap_boost_mode_user = SUPERCAP_BOOST;
    } else {
        buscomm->cap_boost_mode_user = SUPERCAP_UNBOOST;
    }

    /**************Front and back control*************/
    static float t_ws = 0.0f;
    static float t_ad = 0.0f;

    if (data->key.w == 1) {
        t_ws = Math_CalcSlopeRef(t_ws, max_chassis_speed, &Remote_ChassisFBSlope);
    } else if (data->key.s == 1) {
        t_ws = Math_CalcSlopeRef(t_ws, -max_chassis_speed, &Remote_ChassisFBSlope);
    } else
        t_ws = 0;
    buscomm->chassis_fb_ref = t_ws;

    /**************Left and right control*************/
    if (data->key.d == 1) {
        t_ad = Math_CalcSlopeRef(t_ad, max_chassis_speed, &Remote_ChassisRLSlope);
    } else if (data->key.a == 1) {
        t_ad = Math_CalcSlopeRef(t_ad, -max_chassis_speed, &Remote_ChassisRLSlope);
    } else
        t_ad = 0;
    buscomm->chassis_lr_ref = t_ad;

    if (minipc->target_state == MiniPC_TARGET_FOLLOWING && gimbal->mode.present_mode == Gimbal_ARMOR) {
    } else {
        // Change the control amount according to the gimbal control
        float yaw, pitch;
        yaw = (float)data->mouse.x * MOUSE_YAW_ANGLE_TO_FACT;
        pitch = Filter_Bessel((float)data->mouse.y, &Remote_mouse_y_Filter) * MOUSE_PITCH_ANGLE_TO_FACT;

        Gimbal_SetYawRefDelta(yaw);
        Gimbal_SetPitchRef(Gimbal_LimitPitch(pitch));
    }
}

/**
 * @brief      Change chassis control state
 * @param      Chassis: control mode
 * @retval     NULL
 */
void Remote_ChangeChassisState(uint8_t chassis_mode) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->chassis_mode = chassis_mode;
}

/**
 * @brief      Gesture control judge function
 * @param      NULL
 * @retval     Gesture flag
 */
uint8_t Remote_Gesturejudge() {
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();

    static uint8_t count_1 = 0, count_2 = 0, count_3 = 0, count_4 = 0;
    //  Gesture of "\/"
    if ((data->remote.ch[0] <= -650) && (data->remote.ch[1] <= -650) &&
        (data->remote.ch[2] >= 650) && (data->remote.ch[3] <= -650)) {
        count_1++;
        if (count_1 >= 200) {
            count_1 = 200;
            return 1;
        }
    } else {
        count_1 = 0;
    }

    // Gesture of "/\"
    if ((data->remote.ch[0] <= -650) && (data->remote.ch[1] >= 650) &&
        (data->remote.ch[2] >= 650) && (data->remote.ch[3] >= 650)) {
        count_2++;
        if (count_2 >= 200) {
            count_2 = 200;
            return 2;
        }
    } else {
        count_2 = 0;
    }

    // Gesture of up "//"
    if ((data->remote.ch[0] >= 650) && (data->remote.ch[1] >= 650) &&
        (data->remote.ch[2] >= 650) && (data->remote.ch[3] >= 650)) {
        count_3++;
        if (count_3 >= 200) {
            count_3 = 200;
            return 3;
        }
    } else {
        count_3 = 0;
    }

    // Gesture of down "//"
    if ((data->remote.ch[0] <= -650) && (data->remote.ch[1] <= -650) &&
        (data->remote.ch[2] <= -650) && (data->remote.ch[3] <= -650)) {
        count_4++;
        if (count_4 >= 200) {
            count_4 = 200;
            return 4;
        }
    } else {
        count_4 = 0;
    }
    return 0;
}

/**
 * @brief      Gesture control function
 * @param      NULL
 * @retval     NULL
 */
void Remote_Gesture() {
    switch (Remote_Gesturejudge()) {
        case 0:
            break;
        case 1:
            Remote_GestureFunction_1();  //  Gesture of "\/"
            break;
        case 2:
            Remote_GestureFunction_2();  // Gesture of "/\"
            break;
        case 3:
            Remote_GestureFunction_3();  // Gesture of up "//"
            break;
        case 4:
            Remote_GestureFunction_4();  // Gesture of down "\\"
        default:
            break;
    }
}

/**
 * @brief      ��\/�� control function
 * @param      NULL
 * @retval     NULL
 */
void Remote_GestureFunction_1() {
    // force open power control

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->power_limit_mode = POWER_LIMITED;
}

/**
 * @brief      ��/\�� control function
 * @param      NULL
 * @retval     NULL
 */
void Remote_GestureFunction_2() {
    // force close power control

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->power_limit_mode = POWER_UNLIMIT;
}

/**
 * @brief      ��//�� control function
 * @param      NULL
 * @retval     NULL
 */
void Remote_GestureFunction_3() {
    // super cap power control
    // Servo_SetServoAngle(&Servo_ammoContainerCapServo, 0);
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->cap_mode_user = SUPERCAP_CTRL_ON;
}

/**
 * @brief      ��\\�� control function
 * @param      NULL
 * @retval     NULL
 */
void Remote_GestureFunction_4() {
    // Servo_SetServoAngle(&Servo_ammoContainerCapServo, 90);
    // Login_LoginOn();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->cap_mode_user = SUPERCAP_CTRL_OFF;
}

#endif
