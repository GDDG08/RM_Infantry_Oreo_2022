/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_miniPC_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-25 11:23:57
 */

#include "gim_miniPC_ctrl.h"

#if __FN_IF_ENABLE(__FN_CTRL_MINIPC)

#include "buscomm_ctrl.h"
#include "const.h"
#include "gim_gimbal_ctrl.h"
#include "math_alg.h"
#include "gim_shoot_ctrl.h"
#include "gim_ins_ctrl.h"

#define MINI_PC_TASK_PERIOD 1

MiniPC_MiniPCControlTypeDef MiniPC_MiniPCContrlData;

#if (__FN_INFANTRY_TYPE == 4)
int CVKF_NT_YAW = 150;
#endif
#if (__FN_INFANTRY_TYPE == 3)
int CVKF_NT_YAW = 100;
#endif
#if (__FN_INFANTRY_TYPE == 5)
int CVKF_NT_YAW = 100;
#endif

int CVKF_NT_PITCH = 7;

float before_cvkf_yaw = 0.0f;
float before_cvkf_pitch = 0.0f;
float after_predict_yaw = 0.0f;
float after_predict_pitch = 0.0f;

float autoaim_pitch_offset = -3.0f;
float autoaim_yaw_offset = 0.0f;

float autoaim_pitch_dead = 0.05f;
float autoaim_yaw_dead = 0.05f;

float autoaim_pitch_limit = 5.0f;
float autoaim_yaw_limit = 10.0f;

float energy_yaw_offset = 1.0f;    // LEFT + RIGHT -
float energy_pitch_offset = 0.2f;  // UP - DOWN +

float delta_predict = 0.0f;
float pitch_angle = 0;

/**
 * @brief      Gets the pointer to the MiniPC data object
 * @param      NULL
 * @retval     Pointer to MiniPC data object
 */
MiniPC_MiniPCControlTypeDef* MiniPC_GetMiniPCControlDataPtr() {
    return &MiniPC_MiniPCContrlData;
}

/**
 * @brief          MiniPC task
 * @param          NULL
 * @retval         NULL
 */
void MiniPC_Task(void const* argument) {
    for (;;) {
        while (!GLOBAL_INIT_FLAG) {
            osDelay(1);
        }

        // MiniPC_SendHeartPacket();
        MiniPC_SendDataPacket();
        osDelay(MINI_PC_TASK_PERIOD);
    }
}

/**
 * @brief      Init minipc data
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_ControlInit() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    minipc->enable_aim_output = 1;
    minipc->control_mode = MiniPC_ABSOLUTE;

    Filter_LowPassInit(0.4, &minipc->yaw_fil_param);
    Filter_LowPassInit(0.2, &minipc->pitch_fil_param);
    Filter_LowPassInit(0.1, &minipc->yaw_cvkf_fil_param);
    Filter_LowPassInit(0.01, &minipc->distance_fil_param);

    // CVKF Init Variables:
    minipc->cvkf_control.total = 0;
    minipc->cvkf_control.basicprocess = 1;
    minipc->cvkf_control.jumpjudge = 0;  // no function
    minipc->cvkf_control.limit = 0;
    minipc->cvkf_control.offset = 0;
    minipc->cvkf_control.output = 1;
    minipc->cvkf_control.predict = 0;
    minipc->cvkf_control.lowfilter = 1;
    minipc->cvkf_control.dead_domain_delta_ref = 1;

    // CVKF for Yaw Angle:
    Kalman_CVKalmanInitYawParam(&minipc->cvkf_data_yaw, 1 / 1000.0f, 0.0f, 0.0f);
    Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
    // CVKF for Pitch Angle:
    Kalman_CVKalmanInitPitchParam(&minipc->cvkf_data_pitch, 1 / 1000.0f, 0.0f, 0.0f);
    Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
}

/**
 * @brief      Change aiming mode
 * @param      mode: MiniPC aim mode enum
 * @retval     NULL
 */
void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode) {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

    minipc->aim_mode = mode;
    minipc_data->mode = minipc->aim_mode;
}

/**
 * @brief      MiniPC auto aim decode control
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_CalcAutoAim() {
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    MiniPC_SetTargetFollowMode();
    MiniPC_SetGimbalRef();
}

/**
 * @brief      MiniPC auto aim decode control
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_UpdateAutoAim() {
    MiniPC_UpdateControlData();
}

/**
 * @brief      Set gimbal following mode
 * @param      mode: MiniPC target follow mode enum
 * @retval     NULL
 */
void MiniPC_SetFollowMode(MiniPC_TargetFollowModeEnum mode) {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    minipc->target_state = mode;
}

/**
 * @brief      Set the state of the target being recognized
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_SetTargetFollowMode() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

    uint32_t now = HAL_GetTick();
    if (abs((now - minipc->get_target_time)) <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    } else if (abs((now - minipc->get_target_time)) >= Const_MiniPC_Lost_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_LOST);
    }
}

/**
 * @brief      Kalman prediction
 * @param      NULL
 * @retval     NULL
 */

void MiniPC_KalmanPrediction() {
    static MiniPC_TargetFollowModeEnum last_target_state = MiniPC_TARGET_LOST;
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();

    float angle_yaw = 0.0f;    // imu->angle.yaw - minipc->yaw_angle;
    float angle_pitch = 0.0f;  // imu->angle.pitch + minipc->pitch_angle;

    //********
    if (minipc->cvkf_control.lowfilter == 1) {
        angle_yaw = imu->angle.yaw - minipc->yaw_ref_filtered;
        angle_pitch = imu->angle.pitch + minipc->pitch_ref_filtered;
    } else {
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
    }
    //********

    //********
    if (minipc->target_state == MiniPC_TARGET_FOLLOWING && (last_target_state == MiniPC_TARGET_LOST)) {
        // Get New Target: Init CVKF Yaw
        angle_yaw = imu->angle.yaw - minipc->yaw_angle;
        angle_pitch = imu->angle.pitch + minipc->pitch_angle;
        // float angle_yaw = imu->angle.yaw - minipc->yaw_angle;//????
        Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, angle_yaw, Kalman_CV_CalInitSpeed(-minipc->yaw_angle));
        Kalman_CVKalmanInit(&minipc->cvkf_yaw, &minipc->cvkf_data_yaw);
        Kalman_TurnOnCVKF(&minipc->cvkf_yaw);  // Start Filtering
        // Get New Target: Init CVKF Pitch
        Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, angle_pitch, 0.0f);  // Kalman_CV_CalInitSpeed(minipc->pitch_angle));
        Kalman_CVKalmanInit(&minipc->cvkf_pitch, &minipc->cvkf_data_pitch);
        Kalman_TurnOnCVKF(&minipc->cvkf_pitch);  // Start Filtering
        // ReStart LowFilter for income Speed:

        minipc->yaw_fil.filted_last_val = minipc->yaw_angle;
        minipc->pitch_fil.filted_last_val = minipc->pitch_angle;
        minipc->yaw_fil.filted_val = minipc->yaw_angle;
        minipc->pitch_fil.filted_val = minipc->pitch_angle;
    }

    else if ((minipc->target_state == MiniPC_TARGET_FOLLOWING) && (last_target_state == MiniPC_TARGET_FOLLOWING)) {
        // Always get the new Measurement For CVKF
        if (minipc->cvkf_yaw.measure_mode == 1) {
            before_cvkf_yaw = angle_yaw;
            Kalman_MeasurementCalc(&minipc->cvkf_yaw, angle_yaw);
        } else {
            // Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_yaw);
        }

        if (minipc->cvkf_pitch.measure_mode == 1) {
            before_cvkf_pitch = angle_pitch;
            Kalman_MeasurementCalc(&minipc->cvkf_pitch, angle_pitch);
        } else {
            // Using CVKF Without Measurements For Tracking:
            Kalman_NonMeasurementCalc(&minipc->cvkf_pitch);
        }
    }

    else {
        // No Targets: Close CVKF
        Kalman_TurnOffCVKF(&minipc->cvkf_yaw);
        Kalman_CVInitSetYaw(&minipc->cvkf_data_yaw, 0.0f, 0.0f);
        Kalman_TurnOffCVKF(&minipc->cvkf_pitch);
        Kalman_CVInitSetPitch(&minipc->cvkf_data_pitch, 0.0f, 0.0f);
    }
    //*******

    //********

    last_target_state = minipc->target_state;
    //********
}

/**
 * @brief      Update minipc data
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_UpdateControlData() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    minipc->distance = minipc_data->distance / 1000.f;  // mm to m

    minipc->distance_filtered = Filter_LowPass(minipc_data->distance, &minipc->distance_fil_param, &minipc->distance_fil);

    if (minipc_data->is_get_target == 1)
        minipc->get_target_time = HAL_GetTick();

    if (minipc_data->is_get_target == 1 && gimbal->mode.present_mode == Gimbal_ARMOR) {
        // minipc->get_target_time = HAL_GetTick();

        if (minipc->cvkf_control.limit == 1) {
            if (minipc_data->yaw_angle > autoaim_yaw_limit)
                minipc->yaw_angle = autoaim_yaw_limit;
            else if (minipc_data->yaw_angle < -autoaim_yaw_limit)
                minipc->yaw_angle = -autoaim_yaw_limit;
            else
                minipc->yaw_angle = minipc_data->yaw_angle;

            if (minipc_data->pitch_angle > autoaim_pitch_limit)
                minipc->pitch_angle = autoaim_pitch_limit;
            else if (minipc_data->pitch_angle < -autoaim_pitch_limit)
                minipc->pitch_angle = -autoaim_pitch_limit;
            else
                minipc->pitch_angle = minipc_data->pitch_angle;
        } else {
            minipc->yaw_angle = minipc_data->yaw_angle;
            minipc->pitch_angle = minipc_data->pitch_angle;
        }

        minipc->yaw_ref_filtered = Filter_LowPass(minipc->yaw_angle, &minipc->yaw_fil_param, &minipc->yaw_fil);
        minipc->pitch_ref_filtered = Filter_LowPass(minipc->pitch_angle, &minipc->pitch_fil_param, &minipc->pitch_fil);

    }

    else if (gimbal->mode.present_mode == Gimbal_SMALL_ENERGY || gimbal->mode.present_mode == Gimbal_BIG_ENERGY) {
        minipc->yaw_angle = minipc_data->yaw_angle;
        minipc->pitch_angle = minipc_data->pitch_angle;
        minipc->yaw_ref_filtered = Filter_LowPass(minipc->yaw_angle, &minipc->yaw_fil_param, &minipc->yaw_fil);
        minipc->pitch_ref_filtered = Filter_LowPass(minipc->pitch_angle, &minipc->pitch_fil_param, &minipc->pitch_fil);
    }

    if (minipc->cvkf_yaw.switch_mode == 1 && minipc->cvkf_pitch.switch_mode == 1) {
        Kalman_TurnOnMeasureUpdate(&minipc->cvkf_yaw);
        Kalman_TurnOnMeasureUpdate(&minipc->cvkf_pitch);
    }
}

/**
 * @brief      Set gimbal autoaim reference
 * @param      NULL
 * @retval     NULL
 */

void MiniPC_SetAutoAimRef() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    // float cvkf_yaw_angle = 0.0f;
    // float cvkf_pitch_angle = 0.0f;

    // if ((minipc->cvkf_control.output == 1) && (minipc->cvkf_control.total == 1) && (minipc->cvkf_control.basicprocess == 1)) {
    //     MiniPC_KalmanPrediction();

    //     after_predict_yaw = Kalman_Predict_nT(&minipc->cvkf_yaw, CVKF_NT_YAW);
    //     after_predict_pitch = Kalman_Predict_nT(&minipc->cvkf_pitch, CVKF_NT_PITCH);

    //     if (minipc->cvkf_control.predict == 1) {
    //         cvkf_yaw_angle = after_predict_yaw;
    //         cvkf_pitch_angle = after_predict_pitch;
    //     } else {
    //         cvkf_yaw_angle = minipc->cvkf_yaw.angle;
    //         cvkf_pitch_angle = minipc->cvkf_pitch.angle;
    //     }

    //     static float ref_cvkf_yaw_angle = 0.0f;
    //     static float ref_cvkf_pitch_angle = 0.0f;

    //     if (minipc->cvkf_control.dead_domain_delta_ref == 1) {
    //         if (fabs(ref_cvkf_yaw_angle - cvkf_yaw_angle) > autoaim_yaw_dead) {
    //             ref_cvkf_yaw_angle = cvkf_yaw_angle;
    //         }
    //         if (fabs(ref_cvkf_pitch_angle - cvkf_pitch_angle) > autoaim_pitch_dead) {
    //             ref_cvkf_pitch_angle = cvkf_pitch_angle;
    //         }
    //     } else {
    //         ref_cvkf_yaw_angle = cvkf_yaw_angle;
    //         ref_cvkf_pitch_angle = cvkf_pitch_angle;
    //     }

    //     if (minipc->cvkf_control.offset == 1) {
    //         pitch_angle = gimbal->pitch_position_fdb + minipc->pitch_angle;

    //         if (pitch_angle >= 0.7f)
    //             autoaim_pitch_offset = -5.0f;
    //         else if (pitch_angle < -0.7f && pitch_angle >= -1.3f)
    //             autoaim_pitch_offset = -6.7f;
    //         else if (pitch_angle < -1.3f && pitch_angle >= -5.0f)
    //             autoaim_pitch_offset = -8.0f;
    //         else if (pitch_angle < -5.0f && pitch_angle >= -15.0f)
    //             autoaim_pitch_offset = -5.0f;  // shoot for sentry in Round High
    //         else if (pitch_angle < -15.0f)
    //             autoaim_pitch_offset = -3.0f;  // shoot for sentry in ~~ROAD

    //         delta_predict = after_predict_yaw - minipc->cvkf_yaw.angle;
    //         if (delta_predict >= 2.0f)
    //             autoaim_yaw_offset = 2.0f;
    //         else if (delta_predict <= -2.0f)
    //             autoaim_yaw_offset = -2.0f;
    //     }

    //     Gimbal_SetYawAutoRef(ref_cvkf_yaw_angle + autoaim_yaw_offset);
    //     Gimbal_SetPitchAutoRef(ref_cvkf_pitch_angle + autoaim_pitch_offset);
    // } else {
    if (minipc->control_mode == MiniPC_ABSOLUTE) {
        Gimbal_SetYawAutoRef(/*imu->angle.yaw + */ minipc->yaw_ref_filtered);
        Gimbal_SetPitchAutoRef(/*imu->angle.pitch + */ minipc->pitch_ref_filtered);
    } else {
        Gimbal_SetYawAutoRef(imu->angle.yaw + minipc->yaw_ref_filtered);
        Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered);
    }
    // }
}

/**
 * @brief      Set gimbal reference
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_SetGimbalRef() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_ARMOR)) {
        MiniPC_SetAutoAimRef();
    }

    else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_BIG_ENERGY)) {
        Gimbal_SetYawAutoRef(minipc->yaw_ref_filtered + energy_yaw_offset);
        Gimbal_SetPitchAutoRef(minipc->pitch_ref_filtered + energy_pitch_offset);
    } else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_SMALL_ENERGY)) {
        Gimbal_SetYawAutoRef(minipc->yaw_ref_filtered + energy_yaw_offset);
        Gimbal_SetPitchAutoRef(minipc->pitch_ref_filtered + energy_pitch_offset);
    } else
        return;
}

#endif

// /**
// * @brief      Reset Kf param
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_ResetKFParam() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
//     float angle, speed;

//     minipc->enable_aim_output = 1;
//     minipc->enable_prediction = 1;

//     static float X_YAW_Hat[2];
//     angle = imu->angle.yaw - minipc->yaw_angle;
//     speed = MiniPC_CV_CalInitSpeed(-minipc->yaw_angle);
//     X_YAW_Hat[0] = angle;
//     X_YAW_Hat[1] = speed;

//     static float X_PITCH_Hat[2];
//     angle = imu->angle.pitch + minipc->pitch_angle;
//     speed = 0;
//     X_PITCH_Hat[0] = angle;
//     X_PITCH_Hat[1] = speed;

//     static float Q_YAW_Init[4] = {
//         0.02f, 0,
//         0.0f, 40.0f};

//     static float Q_PITCH_Init[4] = {
//         0.01f, 0,
//         0.0f, 40.0f};
//     static float R_YAW_Init[1] = {
//         0.5f};

//     static float R_PITCH_Init[1] = {
//         0.5f};
//     static float P_Init[4] = {
//         1, 0,
//         0, 1};
//     static float F_Init[4] = {
//         1, dt,
//         0, 1};
//     static float H_Init[2] = {
//         1,
//         0};
//     minipc->kf_pitch.UseAutoAdjustment = 0;
//     minipc->kf_yaw.UseAutoAdjustment = 0;

//     static float state_min_variance[2] = {0.003, 0.005};

//     Kalman_FilterInit(&minipc->kf_pitch, 2, 0, 1);
//     Kalman_FilterInit(&minipc->kf_yaw, 2, 0, 1);

//     //CVKF for Yaw Angle:
//     memcpy(minipc->kf_yaw.xhat_data, X_YAW_Hat, sizeof(X_YAW_Hat));
//     memcpy(minipc->kf_yaw.H_data, H_Init, sizeof(H_Init));
//     memcpy(minipc->kf_yaw.F_data, F_Init, sizeof(F_Init));
//     memcpy(minipc->kf_yaw.P_data, P_Init, sizeof(P_Init));
//     memcpy(minipc->kf_yaw.R_data, R_YAW_Init, sizeof(R_YAW_Init));
//     memcpy(minipc->kf_yaw.Q_data, Q_YAW_Init, sizeof(Q_YAW_Init));
//     memcpy(minipc->kf_yaw.StateMinVariance, state_min_variance, sizeof(state_min_variance));

//     //CVKF for Pitch Angle:
//     memcpy(minipc->kf_pitch.xhat_data, X_PITCH_Hat, sizeof(X_PITCH_Hat));
//     memcpy(minipc->kf_pitch.H_data, H_Init, sizeof(H_Init));
//     memcpy(minipc->kf_pitch.F_data, F_Init, sizeof(F_Init));
//     memcpy(minipc->kf_pitch.P_data, P_Init, sizeof(P_Init));
//     memcpy(minipc->kf_pitch.R_data, R_PITCH_Init, sizeof(R_PITCH_Init));
//     memcpy(minipc->kf_pitch.Q_data, Q_PITCH_Init, sizeof(Q_PITCH_Init));
//     memcpy(minipc->kf_pitch.StateMinVariance, state_min_variance, sizeof(state_min_variance));
// }

// /**
// * @brief      Set gimbal following mode
// * @param      mode: MiniPC target follow mode enum
// * @retval     NULL
// */
// void MiniPC_SetFollowMode(MiniPC_TargetFollowModeEnum mode) {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

//     minipc->target_state = mode;
// }

// /**
// * @brief      Change aiming mode
// * @param      mode: MiniPC aim mode enum
// * @retval     NULL
// */
// void MiniPC_ChangeAimMode(MiniPC_AutoAimModeEnum mode) {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

//     minipc->aim_mode = mode;
//     minipc_data->mode = minipc->aim_mode;
// }

// /**
// * @brief      MiniPC auto aim decode control
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_CalcAutoAim() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

//     // Set following mode
//     MiniPC_SetTargetFollowMode();
//     MiniPC_SetGimbalRef();
// }

// /**
// * @brief      Set the state of the target being recognized
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_SetTargetFollowMode() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();

//     uint32_t now = HAL_GetTick();
//     if (now - minipc->get_target_time <= Const_MiniPC_Follow_Target_Time) {
//         MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
//     } else if (now - minipc->get_target_time >= Const_MiniPC_Lost_Target_Time) {
//         MiniPC_SetFollowMode(MiniPC_TARGET_LOST);
//     }
// }

// /**
// * @brief      Add pitch axis trajectory compensation
// * @param      angle_pitch: Set pitch axis angle
// * @retval     float: Pitch axis angle after compensation
// */
// float MiniPC_AddPitchOffset(float angle_pitch) {
//     float offset = 0.0f;
//     if (angle_pitch > 0) {
//         offset = 10.0f;
//     } else if (angle_pitch > -10.0f) {
//         offset = 5.0f;
//     } else {
//         offset = 0.1f;
//     }
//     return offset;
// }

// /**
// * @brief      Update minipc data
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_UpdateControlData() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
//     INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
//     Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

//     if (minipc_data->new_data_flag != 1)
//         return;

//     if (minipc_data->is_get_target == 1) {
//         minipc->get_target_time = HAL_GetTick();
//     }

//     if (minipc_data->is_get_target == 1 && gimbal->mode.present_mode == Gimbal_ARMOR) {
//         minipc->yaw_angle = minipc_data->yaw_angle;
//         minipc->pitch_angle = minipc_data->pitch_angle;
//         minipc->distance = minipc_data->distance;
//     }

//     else {
//         minipc->yaw_angle = Filter_LowPass(minipc_data->yaw_angle, &minipc->yaw_lowfil_param, &minipc->yaw_lowfil);
//         minipc->pitch_angle = Filter_LowPass(minipc_data->pitch_angle, &minipc->pitch_lowfil_param, &minipc->pitch_lowfil);
//         minipc->distance = Filter_LowPass(minipc_data->distance, &minipc->dis_lowfil_param, &minipc->dis_lowfil);
//     }

//     LimitMax(minipc->yaw_angle, autoaim_yaw_limit);
//     LimitMax(minipc->pitch_angle, autoaim_pitch_limit);

//     minipc->kf_yaw.MeasuredVector[0] = imu->angle.yaw - minipc->yaw_angle;
//     minipc->kf_pitch.MeasuredVector[0] = imu->angle.pitch + minipc->pitch_angle;

//     minipc_data->new_data_flag = 0;
// }

// /**
//   * @brief      Forecast N cycles without variance iteration
//   * @param      kf: Kalman filter structure
//   * @param      nT: Forecast period
//   * @retval     NULL
//   */
// float MiniPC_PredictNT(Kalman_KalmanTypeDef* kf, uint32_t nT) {
//     float pre_time = kf->F_data[1] * nT;
//     float pre_angle = kf->FilteredValue[0];
//     float pre_speed = kf->FilteredValue[1];

//     if (fabs(pre_speed) >= 3.5f) {
//         pre_angle += pre_speed * pre_time;
//     }
//     return pre_angle;
// }

// /**
// * @brief      Set CV mode speed
// * @param      NULL
// * @retval     NULL
// */
// float MiniPC_CV_CalInitSpeed(float delta_err_angle) {
//     static float delta_angle = 0.0f;
//     if (fabs(delta_err_angle) <= delta_angle) {
//         return 0.0f;
//     } else if (delta_err_angle > delta_angle) {
//         return 10.0f;
//     } else if (delta_err_angle < -delta_angle) {
//         return -10.0f;
//     }
//     return 0.0f;
// }

// /**
// * @brief      Kalman prediction
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_KalmanPrediction() {
//     static MiniPC_TargetFollowModeEnum last_target_state = MiniPC_TARGET_LOST;
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();

//     float angle_yaw = imu->angle.yaw - minipc->yaw_angle;
//     float angle_pitch = imu->angle.pitch + minipc->pitch_angle;

//     if (minipc->target_state == MiniPC_TARGET_FOLLOWING && (last_target_state == MiniPC_TARGET_LOST)) {
//         MiniPC_ResetKFParam();

//         minipc->yaw_ref_filtered = angle_yaw;
//         minipc->pitch_ref_filtered = angle_pitch;
//     }

//     else if ((minipc->target_state == MiniPC_TARGET_FOLLOWING) && (last_target_state == MiniPC_TARGET_FOLLOWING)) {
//         // Always get the new Measurement For CVKF
//         if (minipc->enable_prediction == 1) {
//             Kalman_FilterUpdate(&minipc->kf_pitch);
//             Kalman_FilterUpdate(&minipc->kf_yaw);
//             minipc->yaw_ref_filtered = minipc->kf_yaw.FilteredValue[0];
//             minipc->pitch_ref_filtered = minipc->kf_pitch.FilteredValue[0];
//         }
//     }

//     else {
//         // No Targets: Close CVKF
//         MiniPC_ResetKFParam();
//         minipc->yaw_ref_filtered = angle_yaw;
//         minipc->pitch_ref_filtered = angle_pitch;
//     }

//     last_target_state = minipc->target_state;
// }

// /**
// * @brief      Set gimbal autoaim reference
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_SetAutoAimRef() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
//     INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
//     Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

//     float after_predict_yaw;
//     float after_predict_pitch;

//     MiniPC_KalmanPrediction();

//     if (minipc->enable_prediction == 1) {
//         after_predict_yaw = MiniPC_PredictNT(&minipc->kf_yaw, CVKF_NT_YAW);        //150
//         after_predict_pitch = MiniPC_PredictNT(&minipc->kf_pitch, CVKF_NT_PITCH);  //7
//     }

//     static float ref_cvkf_yaw_angle = 0.0f;
//     static float ref_cvkf_pitch_angle = 0.0f;

//     if (fabs(ref_cvkf_yaw_angle - after_predict_yaw) > autoaim_yaw_dead) {
//         ref_cvkf_yaw_angle = after_predict_yaw;
//     }
//     if (fabs(ref_cvkf_pitch_angle - after_predict_pitch) > autoaim_pitch_dead) {
//         ref_cvkf_pitch_angle = after_predict_pitch;
//     }

//     float delta_predict = after_predict_yaw - minipc->yaw_ref_filtered;
//     float pitch_angle = gimbal->pitch_position_fdb + minipc->pitch_angle;
//     float autoaim_yaw_offset, autoaim_pitch_offset;

//     if (delta_predict >= 2.0f)
//         autoaim_yaw_offset = 2.0f;
//     else if (delta_predict <= -2.0f)
//         autoaim_yaw_offset = -2.0f;

//     if (pitch_angle >= 0.7f)
//         autoaim_pitch_offset = -5.0f;
//     else if (pitch_angle <= -0.7f && pitch_angle >= -1.5f)
//         autoaim_pitch_offset = -6.7f;
//     else if (pitch_angle <= -1.5f && pitch_angle >= -5.0f)
//         autoaim_pitch_offset = -8.0f;
//     else if (pitch_angle < -5.0f && pitch_angle >= -15.0f)
//         autoaim_pitch_offset = -5.0f;  // shoot for sentry in Round High
//     else if (pitch_angle < -15.0f)
//         autoaim_pitch_offset = -3.0f;  // shoot for sentry in ~~ROAD

//     Gimbal_SetYawAutoRef(ref_cvkf_yaw_angle + autoaim_yaw_offset);
//     Gimbal_SetPitchAutoRef(ref_cvkf_pitch_angle + autoaim_pitch_offset);
// }

// /**
// * @brief      Set gimbal reference
// * @param      NULL
// * @retval     NULL
// */
// void MiniPC_SetGimbalRef() {
//     MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
//     MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
//     INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
//     Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

//     if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_ARMOR)) {
//         MiniPC_SetAutoAimRef();
//     }

//     else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_BIG_ENERGY)) {
//         Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_angle + Const_energy_yaw_offset);
//         Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_angle + Const_energy_pitch_offset);
//     }

//     else if ((minipc->enable_aim_output) && (minipc->target_state == MiniPC_TARGET_FOLLOWING) && (gimbal->mode.present_mode == Gimbal_SMALL_ENERGY)) {
//         Gimbal_SetYawAutoRef(imu->angle.yaw - minipc->yaw_angle + Const_energy_yaw_offset);
//         Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_angle + Const_energy_pitch_offset);
//     }

//     else
//         return;
// }

// #endif
