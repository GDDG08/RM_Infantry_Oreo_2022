/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_remote_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-10 03:08:21
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

#define KEY(T) (data->key.T)
#define KEY2(T1, T2) (data->key.T1 && data->key.T2)
#define KEY3(T1, T2, T3) (data->key.T1 && data->key.T2 && data->key.T3)
#define KEY_UP(T) (remoteKey_last.T && !remoteKey->T)
#define KEY_DN(T) (!remoteKey_last.T && remoteKey->T)

Remote_RemoteControlTypeDef Remote_remoteControlData;
Remote_KeyboardTypeDef remoteKey_last;
Remote_KeyboardTypeDef remoteKey_zero;

Math_SlopeParamTypeDef Remote_ChassisFBSlope;
Math_SlopeParamTypeDef Remote_ChassisRLSlope;

/*          Remote bessel_filter                */
Filter_Bessel_TypeDef Remote_forward_backFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_right_leftFilter = {0, 0, 0};
Filter_Bessel_TypeDef Remote_mouse_y_Filter = {0, 0, 0};

const float REMOTE_PITCH_ANGLE_TO_REF = 0.0005f;
const float REMOTE_YAW_ANGLE_TO_REF = 0.0015f;

float Remote_max_chassis_speed = 0;
uint8_t Remote_Chassis_SuperGyro_State = 0;
uint8_t Remote_Chassis_Gyro_State = 0;
uint8_t Remote_Chassis_Ass_State = 0;
uint8_t Remote_Chassis_Crab_State = 0;
uint8_t Remote_Chassis_Disco_State = 0;

/**
 * @brief      Remote Control Init
 * @param      NULL
 * @retval     NULL
 */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef* control_data = Remote_GetControlDataPtr();
    Remote_max_chassis_speed = MOUSE_CHASSIS_MAX_SPEED;
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
            // Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            // Gimbal_ChangeMode(Gimbal_NOAUTO);
            control_data->onAim = 0;
            control_data->aim_mode = AutoAim_ARMOR;
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
            // Gimbal_ChangeMode(Gimbal_BIG_ENERGY);
            // MiniPC_ChangeAimMode(MiniPC_BIG_BUFF);
            // Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            control_data->onAim = 0;
            control_data->aim_mode = AutoAim_BIG_BUFF;
            Remote_RemoteShooterModeSet();
            Remote_Gesture();
            break;
        }
        default:
            break;
    }
    Remote_AutoAimModeControl();
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

#if __FN_IF_ENABLE(__FN_SHOOTER_PID)
    // Prevent launching without opening the friction wheel
    if ((shooter->shooter_mode != Shoot_REFEREE) || (Motor_shooterMotorLeft.pid_spd.fdb <= 8) || (Motor_shooterMotorRight.pid_spd.fdb <= 8)) {
        Shooter_ChangeFeederMode(Feeder_FINISH);
        return;
    }
#endif
    static int count_mouse_L = 0;
    if (data->mouse.l == 1) {
        // count_mouse_L++;
        // if (count_mouse_L >= 50) {
        //     // Shooter_ChangeFeederMode(Feeder_REFEREE);
        //     Shooter_ChangeFeederMode(Feeder_SINGLE);

        //     count_mouse_L = 50;
        // }

        Shooter_ChangeFeederMode(Feeder_SINGLE);
    } else {
        // if (0 < count_mouse_L && count_mouse_L < 50) {
        //     Shooter_SingleShootReset();
        //     Shooter_ChangeFeederMode(Feeder_SINGLE);
        // } else
        Shooter_ChangeFeederMode(Feeder_FINISH);
        Shooter_SingleShootReset();
        // count_mouse_L = 0;
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

#if __FN_IF_ENABLE(__FN_SHOOTER_PID)
            if ((Motor_shooterMotorLeft.pid_spd.fdb >= 8) && (Motor_shooterMotorRight.pid_spd.fdb >= 8)) {
                Shooter_ChangeFeederMode(Feeder_REFEREE);
            } else
                Shooter_ChangeFeederMode(Feeder_FINISH);
#endif
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

    if (data->remote.ch[4] <= -500.0f) {
        Remote_Chassis_SuperGyro_State = 0;
        Remote_Chassis_Gyro_State = 1;
        Remote_Chassis_Ass_State = 0;
        Remote_Chassis_Crab_State = 0;
        Remote_Chassis_Disco_State = 0;

        // Remote_ChangeChassisState(CHASSIS_CTRL_GYRO);
    } else {
        Remote_Chassis_SuperGyro_State = 0;
        Remote_Chassis_Gyro_State = 0;
        Remote_Chassis_Ass_State = 0;
        Remote_Chassis_Crab_State = 0;
        Remote_Chassis_Disco_State = 0;

        // Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
    }

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
    Gimbal_SetPitchRefDelta(-pitch_ref);
}

/**
 * @brief      KeyMouse control process
 * @param      NULL
 * @retval     NULL
 */

void Remote_KeyMouseProcess() {
    Remote_RemoteDataTypeDef* data = Remote_GetRemoteDataPtr();
    Remote_KeyboardTypeDef* remoteKey = &(Remote_GetRemoteDataPtr()->key);
    Remote_RemoteControlTypeDef* control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    static uint8_t wait4release = 0;

    static uint8_t Ctrl_horizental = 0, Ctrl_vertical = 0;
    Ctrl_horizental = 0, Ctrl_vertical = 0;
    MiniPC_OffsetTuneCntTypeDef* vision_offset_mode = &minipc->vision_offset[minipc->aim_mode];

    if (KEY2(ctrl, shift)) {
        // wait4release = 2

        if (KEY_DN(x)) {
            Remote_Chassis_Crab_State = !Remote_Chassis_Crab_State;
            wait4release = 2;
        }

        if (KEY_DN(d)) {
            Remote_Chassis_Disco_State = !Remote_Chassis_Disco_State;
            wait4release = 2;
        }
        if (KEY_DN(v)) {
            Remote_SwitchAutoAimMode(AutoAim_GIMBAL_DEBUG);
            wait4release = 2;
        }
    } else if (KEY(ctrl)) {
        if (wait4release <= 1) {
            if (KEY_DN(q)) {  // E Press to close the friction wheel
                Shooter_ChangeShooterMode(Shoot_NULL);
                wait4release = 1;
            }
            if (KEY_DN(x)) {
                Remote_Chassis_Ass_State = !Remote_Chassis_Ass_State;
                wait4release = 1;
            }
        }
    } else if (KEY(shift)) {
        if (wait4release <= 1) {
            // UI switch
            if (KEY_DN(c)) {
                buscomm->ui_cmd = !buscomm->ui_cmd;
                wait4release = 1;
            }
            // reload
            if (KEY_DN(r)) {
                wait4release = 1;
            }
            if (KEY_DN(x)) {
                Remote_SwitchGryoState(1);
                wait4release = 1;
            }
        }
    } else {
        if (wait4release == 0) {
            // friction wheel
            if (KEY_DN(q))  // Q Press to open the friction wheel
                Shooter_ChangeShooterMode(Shoot_REFEREE);

            // gyro
            if (KEY_UP(x))
                Remote_SwitchGryoState(0);

            // supercap
            if (KEY_UP(z))
                buscomm->cap_mode_user = (buscomm->cap_mode_user == SUPERCAP_CTRL_OFF) ? SUPERCAP_CTRL_ON : SUPERCAP_CTRL_OFF;

            // autoaim mode
            if (KEY_UP(v))
                Remote_SwitchAutoAimMode(AutoAim_SMALL_BUFF);

            if (KEY_UP(b))
                Remote_SwitchAutoAimMode(AutoAim_BIG_BUFF);

            if (KEY_UP(g))
                Remote_SwitchAutoAimMode(AutoAim_SENTRY);
        }
    }

    if (wait4release == 0) {
        /******** MOVE function ******/
        if (!KEY(q) && !KEY2(ctrl, shift) && !KEY(ctrl)) {
            /**************Front and back control*************/
            static float t_ws = 0.0f;

            if (KEY(w)) {
                t_ws = Math_CalcSlopeRef(t_ws, Remote_max_chassis_speed, &Remote_ChassisFBSlope);
            } else if (KEY(s)) {
                t_ws = Math_CalcSlopeRef(t_ws, -Remote_max_chassis_speed, &Remote_ChassisFBSlope);
            } else
                t_ws = 0;
            buscomm->chassis_fb_ref = t_ws;

            /**************Left and right control*************/
            static float t_ad = 0.0f;

            if (KEY(d)) {
                t_ad = Math_CalcSlopeRef(t_ad, Remote_max_chassis_speed, &Remote_ChassisRLSlope);
            } else if (KEY(a)) {
                t_ad = Math_CalcSlopeRef(t_ad, -Remote_max_chassis_speed, &Remote_ChassisRLSlope);
            } else
                t_ad = 0;
            buscomm->chassis_lr_ref = t_ad;

            // boost
            buscomm->cap_boost_mode_user = KEY(shift) ? SUPERCAP_BOOST : SUPERCAP_UNBOOST;
        }
    } else {
        // All zero
        if (memcmp(remoteKey, &remoteKey_zero, sizeof(remoteKey_zero)) == 0)
            wait4release = 0;
    }

    // autoaim offset
    if (minipc->aim_mode == MiniPC_SMALL_BUFF || minipc->aim_mode == MiniPC_BIG_BUFF ||
        (KEY(q) && (minipc->aim_mode == MiniPC_ARMOR || minipc->aim_mode == MiniPC_SENTRY))) {
        if (KEY_DN(w))
            Ctrl_vertical = KEY(shift) ? 10 : 1;
        if (KEY_DN(s))
            Ctrl_vertical = KEY(shift) ? -10 : -1;
        if (KEY_DN(d))
            Ctrl_horizental = KEY(shift) ? 10 : 1;
        if (KEY_DN(a))
            Ctrl_horizental = KEY(shift) ? -10 : -1;

        if (KEY(f)) {
            vision_offset_mode->horizental = 0;
            vision_offset_mode->vertical = 0;
        } else {
            vision_offset_mode->horizental += Ctrl_horizental;
            vision_offset_mode->vertical += Ctrl_vertical;
        }
    }

    memcpy(&remoteKey_last, remoteKey, sizeof(remoteKey_last));

    /*********** mouse control********/
    static uint8_t mouse_r_last = 0;
    // if (minipc->aim_mode == MiniPC_ARMOR || minipc->aim_mode == MiniPC_SENTRY) {
    //     if (data->mouse.r == 1)
    //         Gimbal_ChangeMode(Gimbal_ARMOR);
    //     else
    //         Gimbal_ChangeMode(Gimbal_NOAUTO);
    // }
    // else {
    if (!mouse_r_last && data->mouse.r) {
        Remote_SetAutoAimState(1);
    } else if (mouse_r_last && !data->mouse.r) {
        Remote_SetAutoAimState(0);
    }
    // }
    mouse_r_last = data->mouse.r;

    if (gimbal->mode.present_mode == Gimbal_NOAUTO || (gimbal->mode.present_mode != Gimbal_IMU_DEBUG && minipc->target_state != MiniPC_TARGET_FOLLOWING)) {
        // Change the control amount according to the gimbal control
        float yaw, pitch;
        yaw = (float)data->mouse.x * MOUSE_YAW_ANGLE_TO_FACT;
        pitch = Filter_Bessel((float)data->mouse.y, &Remote_mouse_y_Filter) * MOUSE_PITCH_ANGLE_TO_FACT;

        Gimbal_SetYawRefDelta(yaw);
        Gimbal_SetPitchRefDelta(pitch);
    }
}

/**
 * @brief      Change chassis control state
 * @param      Chassis: control mode
 * @retval     NULL
 */
void Remote_ChangeChassisState(uint8_t chassis_mode) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    if (chassis_mode == CHASSIS_CTRL_NORMAL && Remote_Chassis_SuperGyro_State) {
        buscomm->chassis_mode = CHASSIS_CTRL_SUPERGYRO;
    } else if (chassis_mode == CHASSIS_CTRL_NORMAL && Remote_Chassis_Gyro_State) {
        buscomm->chassis_mode = CHASSIS_CTRL_GYRO;
    } else if (chassis_mode == CHASSIS_CTRL_NORMAL && Remote_Chassis_Ass_State) {
        buscomm->chassis_mode = CHASSIS_CTRL_ASS;
    } else if (chassis_mode == CHASSIS_CTRL_NORMAL && Remote_Chassis_Crab_State) {
        buscomm->chassis_mode = CHASSIS_CTRL_CRAB;
    } else if (chassis_mode == CHASSIS_CTRL_NORMAL && Remote_Chassis_Disco_State) {
        buscomm->chassis_mode = CHASSIS_CTRL_DISCO;
    } else
        buscomm->chassis_mode = chassis_mode;
}

/**
 * @brief      switch chassis gryo state
 * @param      NULL
 * @retval     NULL
 */

void Remote_SwitchGryoState(uint8_t is_super) {
    // static uint8_t gyro_state = 0;

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    if (buscomm->chassis_mode == CHASSIS_CTRL_STOP)
        return;

    if (is_super) {
        Remote_Chassis_Gyro_State = 0;
        if (Remote_Chassis_SuperGyro_State == 0) {
            Remote_ChangeChassisState(CHASSIS_CTRL_SUPERGYRO);
            Remote_max_chassis_speed = MOUSE_CHASSIS_MAX_GYRO_SPEED;
            Remote_Chassis_SuperGyro_State = 1;
        } else {
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            Remote_max_chassis_speed = MOUSE_CHASSIS_MAX_SPEED;
            Remote_Chassis_SuperGyro_State = 0;
        }
    } else {
        Remote_Chassis_SuperGyro_State = 0;
        if (Remote_Chassis_Gyro_State == 0) {
            Remote_ChangeChassisState(CHASSIS_CTRL_GYRO);
            Remote_max_chassis_speed = MOUSE_CHASSIS_MAX_GYRO_SPEED;
            Remote_Chassis_Gyro_State = 1;
        } else {
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            Remote_max_chassis_speed = MOUSE_CHASSIS_MAX_SPEED;
            Remote_Chassis_Gyro_State = 0;
        }
    }
}

/**
 * @brief      switch autoaim state
 * @param      Remote_AutoAimModeEnum mode
 * @retval     NULL
 */
void Remote_SwitchAutoAimMode(uint8_t mode) {
    Remote_RemoteControlTypeDef* remote = Remote_GetControlDataPtr();
    if (remote->aim_mode == AutoAim_ARMOR) {
        remote->aim_mode = mode;
    } else if (remote->aim_mode == mode) {
        remote->aim_mode = AutoAim_ARMOR;
    }
}

/**
 * @brief      switch autoaim state
 * @param      state :on or off
 * @retval     NULL
 */
void Remote_SetAutoAimState(uint8_t state) {
    Remote_RemoteControlTypeDef* remote = Remote_GetControlDataPtr();

    static uint8_t temp_last_mode = AutoAim_ARMOR;
    if (state == 1) {
        remote->onAim = 1;
        switch (remote->aim_mode) {
            default:
                break;
            case AutoAim_ARMOR:
            case AutoAim_ARMOR_TEMP:
            case AutoAim_SENTRY:

                break;

            case AutoAim_SMALL_BUFF:
                temp_last_mode = AutoAim_SMALL_BUFF;
                remote->aim_mode = AutoAim_ARMOR_TEMP;
                break;
            case AutoAim_BIG_BUFF:
                temp_last_mode = AutoAim_BIG_BUFF;
                remote->aim_mode = AutoAim_ARMOR_TEMP;
                break;
        }
    } else {
        remote->onAim = 0;
        switch (remote->aim_mode) {
            default:
                break;
            case AutoAim_ARMOR:
            case AutoAim_SENTRY:
                break;
            case AutoAim_ARMOR_TEMP:
                remote->aim_mode = temp_last_mode;
        }
    }
}

void Remote_AutoAimModeControl() {
    Remote_RemoteControlTypeDef* remote = Remote_GetControlDataPtr();
    switch (remote->aim_mode) {
        default:
        case AutoAim_ARMOR:
        case AutoAim_ARMOR_TEMP:
            MiniPC_ChangeAimMode(MiniPC_ARMOR);
            if (remote->onAim)
                Gimbal_ChangeMode(Gimbal_ARMOR);
            else
                Gimbal_ChangeMode(Gimbal_NOAUTO);
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            break;
        case AutoAim_SENTRY:
            MiniPC_ChangeAimMode(MiniPC_SENTRY);
            if (remote->onAim)
                Gimbal_ChangeMode(Gimbal_ARMOR);
            else
                Gimbal_ChangeMode(Gimbal_NOAUTO);
            Remote_ChangeChassisState(CHASSIS_CTRL_NORMAL);
            break;
        case AutoAim_SMALL_BUFF:
            MiniPC_ChangeAimMode(MiniPC_SMALL_BUFF);
            Gimbal_ChangeMode(Gimbal_SMALL_ENERGY);
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            break;
        case AutoAim_BIG_BUFF:
            MiniPC_ChangeAimMode(MiniPC_BIG_BUFF);
            Gimbal_ChangeMode(Gimbal_BIG_ENERGY);
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            break;
        case AutoAim_GIMBAL_DEBUG:
            MiniPC_ChangeAimMode(MiniPC_GIMBAL_DEBUG);
            Gimbal_ChangeMode(Gimbal_SMALL_ENERGY);
            Remote_ChangeChassisState(CHASSIS_CTRL_STOP);
            break;
    }
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
