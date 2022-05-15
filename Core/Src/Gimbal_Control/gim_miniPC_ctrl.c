/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_miniPC_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-15 21:29:30
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

MiniPC_OffsetTypeDef MiniPC_Offset_Armor;
MiniPC_OffsetTypeDef MiniPC_Offset_Buff_Small;
MiniPC_OffsetTypeDef MiniPC_Offset_Buff_Big;
MiniPC_OffsetTypeDef MiniPC_Offset_Sentry;

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

    Const_SetAutoAimOffset();
    minipc->enable_aim_output = 1;
    minipc->control_mode = MiniPC_ABSOLUTE;

    Filter_LowPassInit(0.4, &minipc->yaw_fil_param);
    Filter_LowPassInit(0.1, &minipc->pitch_fil_param);
    Filter_LowPassInit(0.01, &minipc->distance_fil_param);

    AutoAim_Para_Init();
}

/**
 * @brief      Init offset param
 * @param      float param
 * @retval     NULL
 */
void MiniPC_InitOffsetParam(float para[4][2]) {
    if (para == NULL)
        return;
    memcpy(&MiniPC_Offset_Armor, para[0], sizeof(para[0]));
    memcpy(&MiniPC_Offset_Buff_Small, para[1], sizeof(para[1]));
    memcpy(&MiniPC_Offset_Buff_Big, para[2], sizeof(para[2]));
    memcpy(&MiniPC_Offset_Sentry, para[3], sizeof(para[3]));
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
    MiniPC_SetTargetFollowMode();
    MiniPC_SetAutoAimOutput();
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
    if ((now - minipc->get_target_time) <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    } else if ((now - minipc->get_target_time) >= Const_MiniPC_Lost_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_LOST);
    }
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

    if (minipc_data->is_get_target == 0) {
        return;
    } else {
        minipc->get_target_time = HAL_GetTick();

        minipc->yaw_angle = minipc_data->yaw_angle;
        minipc->pitch_angle = minipc_data->pitch_angle;
        minipc->distance = minipc_data->distance / 1000.f;  // mm to m

        minipc->yaw_ref_filtered = Filter_LowPass(minipc->yaw_angle, &minipc->yaw_fil_param, &minipc->yaw_fil);
        minipc->pitch_ref_filtered = Filter_LowPass(minipc->pitch_angle, &minipc->pitch_fil_param, &minipc->pitch_fil);
        minipc->distance_filtered = Filter_LowPass(minipc_data->distance, &minipc->distance_fil_param, &minipc->distance_fil);

        if (minipc->aim_mode == MiniPC_ARMOR) {
            minipc->output_offset = MiniPC_Offset_Armor;
            minipc->control_mode = MiniPC_ABSOLUTE;
        } else if (minipc->aim_mode == MiniPC_SMALL_BUFF) {
            minipc->output_offset = MiniPC_Offset_Buff_Small;
            minipc->control_mode = MiniPC_RELATIVE;
        } else if (minipc->aim_mode == MiniPC_BIG_BUFF) {
            minipc->output_offset = MiniPC_Offset_Buff_Big;
            minipc->control_mode = MiniPC_RELATIVE;
        } else if (minipc->aim_mode == MiniPC_SENTRY) {
            minipc->output_offset = MiniPC_Offset_Sentry;
            minipc->control_mode = MiniPC_ABSOLUTE;
        }
    }
}

/**
 * @brief      Set gimbal autoaim reference
 * @param      NULL
 * @retval     NULL
 */

void MiniPC_SetAutoAimOutput() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();

    GimbalMove_AutoAimRef_Change();
    if ((minipc->enable_aim_output) && (gimbal->mode.present_mode != Gimbal_NOAUTO)) {
        if (minipc->control_mode == MiniPC_ABSOLUTE) {
            if (minipc_data->is_get_target) {
                Gimbal_SetYawAutoRef(minipc->yaw_ref_calc + minipc->output_offset.yaw);
                Gimbal_SetPitchAutoRef(minipc->pitch_ref_calc + minipc->output_offset.pitch);
            }
        } else {
            if (minipc->target_state == MiniPC_TARGET_FOLLOWING) {
                Gimbal_SetYawAutoRef(imu->angle.yaw + minipc->yaw_ref_filtered + minipc->output_offset.yaw);
                Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered + minipc->output_offset.pitch);
            }
        }
    }
}

TargetData_t TargetData_predict, TargetData_predict_re;

float vx_Chassis, vz_Chassis;  // z即为板通中的y

float kt_Predict_x, kk_Predict_x;
float k_Predict_z;
float k_Predict_Correction;
float k_Predict_Offset_y;

float Autoaim_OffsetPitch, Autoaim_OffsetYaw;
float Predict_Offest_x, Predict_Offset_y;

uint16_t TimeCountPredict, Target_x_SuddenStart_Count, Target_x_SuddenStop_Count, Target_z_SuddenStart_Count, Target_z_SuddenStop_Count;
uint8_t is_x_SuddenStart, is_x_SuddenStop, is_z_SuddenStart, is_z_SuddenStop;

/**
 * @brief 	自瞄相关初始化
 * @param 	None
 * @retval	None
 * @note	None
 */
void AutoAim_Para_Init() {
    vx_Chassis = vz_Chassis = 0;
    TimeCountPredict = Target_x_SuddenStart_Count = Target_x_SuddenStop_Count = Target_z_SuddenStart_Count = Target_z_SuddenStop_Count = 0;

    Autoaim_OffsetPitch = 0;
    Autoaim_OffsetYaw = 0;
    Predict_Offest_x = 100.0f;
    Predict_Offset_y = -150.0f;

    kk_Predict_x = 0;
    kt_Predict_x = 0.3f;
    k_Predict_z = 0.2f;
    k_Predict_Offset_y = 0.02f;
    k_Predict_Correction = 1.0f;

    is_x_SuddenStart = is_x_SuddenStop = is_z_SuddenStart = is_z_SuddenStop = 0;
}

/**
 * @brief   Yaw轴打弹角度解析函数
 * @param
 * @retval
 * @note
 */
float YawAngle_Shoot_Cal(float target_x, float target_z) {
    if (target_z == 0)
        return 0;
    return (atan(target_x / target_z) / 3.1415926 * 180);
}

/**
 * @brief   Pitch轴打弹角度解析函数
 * @param
 * @retval
 * @note
 */
float PitchAngle_Shoot_Cal(float target_x, float target_y, float target_z) {
    float d = sqrt(target_x * target_x + target_z * target_z);
    float g = 9.8;
    float g_2 = 9.8 * 9.8;
    float v0 = Shooter_GetRefereeSpeedFdb();
    float v0_2 = v0 * v0;
    float v0_4 = v0_2 * v0_2;
    float d_2 = d * d;
    if (1 - g_2 * d_2 / v0_4 + 2 * g * target_y / v0_2 < 0)
        return 0;
    return ((atan((v0_2) * (-1 + sqrt(1 - g_2 * d_2 / v0_4 + 2 * g * target_y / v0_2)) / (g * d))) / 3.1415926 * 180);
}

/**
 * @brief 	根据自瞄数据和预测量计算电机期望
 * @param 	None
 * @retval	None
 * @note	None
 */
void GimbalMove_AutoAimRef_Change() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

    /*利用外插法预测插帧 & 预测系数修正*/
    if (minipc_data->new_data_flag) {  //接收中断
        TargetData_predict.x = (float)minipc_data->x + Predict_Offest_x;
        TargetData_predict.y = (float)minipc_data->y + Predict_Offset_y - k_Predict_Offset_y * TargetData_predict.z;
        TargetData_predict.z = (float)minipc_data->z + k_Predict_z * (float)minipc_data->vz;
        TargetData_predict.vx = (float)minipc_data->vx;
        TargetData_predict.vz = (float)minipc_data->vz;
        TargetData_predict.ax = fabs(TargetData_predict.vx) - fabs(TargetData_predict.last_vx);
        TargetData_predict.az = fabs(TargetData_predict.vz) - fabs(TargetData_predict.last_vz);

        if (is_x_SuddenStart) {
            Target_x_SuddenStart_Count++;
            k_Predict_Correction = 1.3f;
            if (Target_x_SuddenStart_Count > 70) {
                Target_x_SuddenStart_Count = 0;
                is_x_SuddenStart = 0;
                k_Predict_Correction = 1.0f;
            }
        } else if (is_x_SuddenStop) {
            Target_x_SuddenStop_Count++;
            k_Predict_Correction = 0;
            if (Target_x_SuddenStop_Count > 80) {
                Target_x_SuddenStop_Count = 0;
                is_x_SuddenStop = 0;
                k_Predict_Correction = 1.0f;
            }
        } else if (TargetData_predict.ax >= 0) {
            if (Target_x_SuddenStart_Count == 3) {
                is_x_SuddenStart = 1;
                k_Predict_Correction = 1.5f;
            } else if (TargetData_predict.ax >= 5.0f) {
                Target_x_SuddenStart_Count++;
                k_Predict_Correction = 1.05f;
            } else {
                Target_x_SuddenStart_Count = 0;
                k_Predict_Correction = 1.0f;
            }
        } else {
            if (Target_x_SuddenStop_Count == 3) {
                is_x_SuddenStop = 1;
                k_Predict_Correction = 0.3f;
            } else if (TargetData_predict.ax <= -4.0f) {
                Target_x_SuddenStop_Count++;
                k_Predict_Correction = 0.9f;
            } else {
                Target_x_SuddenStop_Count = 0;
                k_Predict_Correction = 1.0f;
            }
        }

        if (is_z_SuddenStart) {
            Target_z_SuddenStart_Count++;
            k_Predict_z = 0.2f;
            if (Target_z_SuddenStart_Count > 45) {
                Target_z_SuddenStart_Count = 0;
                is_z_SuddenStart = 0;
                k_Predict_z = 0.12f;
            }
        } else if (is_z_SuddenStop) {
            Target_z_SuddenStop_Count++;
            k_Predict_z = 0.03f;
            if (Target_z_SuddenStop_Count > 45) {
                Target_z_SuddenStop_Count = 0;
                is_z_SuddenStop = 0;
                k_Predict_z = 0.12f;
            }
        } else if (TargetData_predict.az >= 0) {
            if (Target_z_SuddenStart_Count == 3) {
                is_z_SuddenStart = 1;
                k_Predict_z = 0.2f;
            } else if (TargetData_predict.az >= 10.0f) {
                Target_z_SuddenStart_Count++;
                k_Predict_z = 0.16f;
            } else {
                Target_z_SuddenStart_Count = 0;
                k_Predict_z = 0.12f;
            }
        } else {
            if (Target_z_SuddenStop_Count == 3) {
                is_z_SuddenStop = 1;
                k_Predict_z = 0.03f;
            } else if (TargetData_predict.az <= -10.0f) {
                Target_z_SuddenStop_Count++;
                k_Predict_z = 0.08f;
            } else {
                Target_z_SuddenStop_Count = 0;
                k_Predict_z = 0.12f;
            }
        }

        minipc_data->new_data_flag = 0;
        TimeCountPredict = 0;
        TargetData_predict.last_vx = TargetData_predict.vx;
        TargetData_predict.last_vz = TargetData_predict.vz;
    } else if (TimeCountPredict < 15) {
        TargetData_predict.x -= 0.001f * minipc_data->vx;
        TargetData_predict.z += 0.001f * minipc_data->vz;
        TimeCountPredict++;
    }

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    vx_Chassis = buscomm->chassis_lr_ref;
    vz_Chassis = buscomm->chassis_fb_ref;

    /*云台直角坐标系换算*/
    TargetData_predict_re.vx = TargetData_predict.vx - vx_Chassis;
    TargetData_predict_re.vz = TargetData_predict.vz - vz_Chassis;
    TargetData_predict_re.x = TargetData_predict.x;
    TargetData_predict_re.y = TargetData_predict.y;
    TargetData_predict_re.z = TargetData_predict.z;

    /*云台预测*/
    TargetData_predict_re.x_predict_val = TargetData_predict_re.vx * kt_Predict_x + 0.001f * kk_Predict_x * TargetData_predict_re.vx * sqrt(TargetData_predict_re.x * TargetData_predict_re.x + TargetData_predict_re.y * TargetData_predict_re.y + TargetData_predict_re.z * TargetData_predict_re.z) / Shooter_GetRefereeSpeedFdb();
    TargetData_predict_re.z_predict_val = k_Predict_z * TargetData_predict_re.vz;
    TargetData_predict_re.x += k_Predict_Correction * TargetData_predict_re.x_predict_val;
    TargetData_predict_re.z += TargetData_predict_re.z_predict_val;

    /*云台球坐标系换算*/
    minipc->yaw_ref_calc = -YawAngle_Shoot_Cal(TargetData_predict_re.x * 0.001f, TargetData_predict_re.z * 0.001f) + Autoaim_OffsetYaw;
    minipc->pitch_ref_calc = PitchAngle_Shoot_Cal(TargetData_predict_re.x * 0.001f, TargetData_predict_re.y * 0.001f, TargetData_predict_re.z * 0.001f) + Autoaim_OffsetPitch;

    // /*远中近距离判断*/
    // if (minipc_data->ID == 1) {  //英雄
    //     if (TargetData_predict_re.z >= 8500 && TargetData_predict_re.z <= 11000) {
    //         TargetData_predict_re.distance_level = DISTANCE_FAR;
    //     } else if (TargetData_predict_re.z >= 7000) {
    //         TargetData_predict_re.distance_level = DISTANCE_MIDDLE;
    //     } else if (TargetData_predict_re.z >= 0) {
    //         TargetData_predict_re.distance_level = DISTANCE_CLOSE;
    //     } else {
    //         TargetData_predict_re.distance_level = DISTANCE_ERROR;
    //     }
    // } else {
    //     if (TargetData_predict_re.z >= 8500 && TargetData_predict_re.z <= 11000) {
    //         TargetData_predict_re.distance_level = DISTANCE_FAR;
    //     } else if (TargetData_predict_re.z >= 7000) {
    //         TargetData_predict_re.distance_level = DISTANCE_MIDDLE;
    //     } else if (TargetData_predict_re.z >= 0) {
    //         TargetData_predict_re.distance_level = DISTANCE_CLOSE;
    //     } else {
    //         TargetData_predict_re.distance_level = DISTANCE_ERROR;
    //     }
    // }
}

#endif
