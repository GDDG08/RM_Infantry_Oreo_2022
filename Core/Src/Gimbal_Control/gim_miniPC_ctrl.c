/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Gimbal_Control\gim_miniPC_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-03 14:34:41
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
    if (abs((now - minipc->get_target_time)) <= Const_MiniPC_Follow_Target_Time) {
        MiniPC_SetFollowMode(MiniPC_TARGET_FOLLOWING);
    } else if (abs((now - minipc->get_target_time)) >= Const_MiniPC_Lost_Target_Time) {
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

    if ((minipc->enable_aim_output) && (gimbal->mode.present_mode != Gimbal_NOAUTO) && (minipc->target_state == MiniPC_TARGET_FOLLOWING)) {
        if (minipc->control_mode == MiniPC_ABSOLUTE) {
            Gimbal_SetYawAutoRef(minipc->yaw_ref_filtered + minipc->output_offset.yaw);
            Gimbal_SetPitchAutoRef(minipc->pitch_ref_filtered + minipc->output_offset.pitch);
        } else {
            Gimbal_SetYawAutoRef(imu->angle.yaw + minipc->yaw_ref_filtered + minipc->output_offset.yaw);
            Gimbal_SetPitchAutoRef(imu->angle.pitch + minipc->pitch_ref_filtered + minipc->output_offset.pitch);
        }
    }
}

#endif
