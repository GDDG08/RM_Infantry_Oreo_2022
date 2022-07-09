/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Common_Contrrol\buscomm_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-06 23:29:28
 *
 */

#include "buscomm_ctrl.h"
#include "buscomm_cmd.h"
#include "cmsis_os.h"

#include "const.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
#include "cha_chassis_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "cha_referee_ctrl.h"
#include "cha_power_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#include "gim_gimbal_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_minipc_ctrl.h"
#include "gim_ins_ctrl.h"
#include "key_periph.h"
#include "minipc_periph.h"

#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
#include "supercap_ctrl.h"
#endif

#define BUSCOMM_TASK_PERIOD 1
// #if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
// FDCAN_HandleTypeDef* Const_BusComm_CAN_HANDLER = &hfdcan1;
// #elif __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
FDCAN_HandleTypeDef* Const_BusComm_CAN_HANDLER = &hfdcan2;
// #endif
FDCAN_HandleTypeDef* Const_CapComm_CAN_HANDLER = &hfdcan3;

/*      infantry communication functions      */
const uint16_t Const_BusComm_TX_BUFF_LEN = 64;
const uint16_t Const_BusComm_RX_BUFF_LEN = 64;
const uint16_t Const_BusComm_OFFLINE_TIME = 200;

const uint32_t Const_BusComm_SIZE = FDCAN_DLC_BYTES_8;
const uint8_t Const_BusComm_GIMBAL_BUFF_SIZE = 4;
const uint8_t Const_BusComm_CHASSIS_BUFF_SIZE = 3;
// const uint8_t Const_BusComm_SUPERCAP_BUFF_SIZE = 1;
const uint8_t Const_BusComm_RECEIVE_SIZE = 7;
const uint8_t Const_CapComm_RECEIVE_SIZE = 3;

// Dual bus communication protocol

uint8_t BusComm_TxData[Const_BusComm_TX_BUFF_LEN];
uint8_t BusComm_RxData[Const_BusComm_RX_BUFF_LEN];
BusComm_BusCommDataTypeDef BusComm_BusCommData;

const uint32_t Const_BusComm_CAN_TX_EXTID = 0x01;

FDCAN_TxHeaderTypeDef BusComm_GimControl;
FDCAN_TxHeaderTypeDef BusComm_GimImuYaw;
FDCAN_TxHeaderTypeDef BusComm_GimChassisRef;
FDCAN_TxHeaderTypeDef BusComm_GimUIState;

FDCAN_TxHeaderTypeDef BusComm_ChaRefereeData_1;
FDCAN_TxHeaderTypeDef BusComm_ChaRefereeData_2;
FDCAN_TxHeaderTypeDef BusComm_CapMode;

// FDCAN_TxHeaderTypeDef BusComm_CapState;
FDCAN_TxHeaderTypeDef BusComm_CapState_1;
FDCAN_TxHeaderTypeDef BusComm_CapState_2;

/**
 * @brief          BusComm task
 * @param          NULL
 * @retval         NULL
 */
void BusComm_Task(void const* argument) {
    for (;;) {
        while (!GLOBAL_INIT_FLAG) {
            osDelay(1);
        }
        BusComm_SendBusCommData();
        osDelay(BUSCOMM_TASK_PERIOD);
    }
}

/**
 * @brief      Inter bus communication initialization
 * @param      NULL
 * @retval     NULL
 */
void BusComm_InitBusComm() {
    BusComm_ResetBusCommData();
    FDCAN_InitTxHander(&BusComm_GimControl, CMD_SET_CONTROL, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_GimImuYaw, CMD_SET_IMU_YAW, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_GimChassisRef, CMD_SET_CHA_REF, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_GimUIState, CMD_SET_UI_STATE, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);

    FDCAN_InitTxHander(&BusComm_ChaRefereeData_1, CMD_SET_REFEREE_DATA_1, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_ChaRefereeData_2, CMD_SET_REFEREE_DATA_2, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_CapMode, CMD_SET_CAP_MODE, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);

    // FDCAN_InitTxHander(&BusComm_CapState, CMD_SEND_CAP_STATE, Const_BusComm_SIZE,FDCAN_BRS_OFF,FDCAN_CLASSIC_CAN);

    FDCAN_InitTxHander(&BusComm_CapState_1, CMD_SET_CAP_STATE_1, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
    FDCAN_InitTxHander(&BusComm_CapState_2, CMD_SET_CAP_STATE_2, Const_BusComm_SIZE, FDCAN_BRS_OFF, FDCAN_CLASSIC_CAN);
}

/**
 * @brief      Gets the pointer to the bus communication data object
 * @param      NULL
 * @retval     Pointer to bus communication data object
 */
BusComm_BusCommDataTypeDef* BusComm_GetBusDataPtr() {
    return &BusComm_BusCommData;
}

/**
 * @brief      Check whether the dual bus communication is offline
 * @param      NULL
 * @retval     NULL
 */
uint8_t BusComm_IsBusCommOffline(uint8_t index) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    if (HAL_GetTick() - buscomm->last_update_time[index] > Const_BusComm_OFFLINE_TIME) {
        // buscomm->state = BusComm_STATE_LOST;
        return 1;
    }

    return 0;
}

/**
 * @brief      Data sending function of serial port in inter bus communication
 * @param      NULL
 * @retval     NULL
 */
void BusComm_SendBusCommData() {
    /* up data struct data    */
    BusComm_Update();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->state = BusComm_STATE_PENDING;

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

    uint32_t out_time = HAL_GetTick();
    for (int i = 0; i < Const_BusComm_CHASSIS_BUFF_SIZE; i++) {
        while (HAL_FDCAN_GetTxFifoFreeLevel(Const_BusComm_CAN_HANDLER) == 0) {
            if (HAL_GetTick() - out_time >= 2) {
                BusComm_SendBlockError();
                return;
            }
        }
        if (Buscmd_ChaSend[i].bus_func != NULL)
            Buscmd_ChaSend[i].bus_func(BusComm_TxData);
    }

#endif

// Gimbal steram
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    uint32_t out_time = HAL_GetTick();
    for (int i = 0; i < Const_BusComm_GIMBAL_BUFF_SIZE; i++) {
        while (HAL_FDCAN_GetTxFifoFreeLevel(Const_BusComm_CAN_HANDLER) == 0) {
            if (HAL_GetTick() - out_time >= 2) {
                BusComm_SendBlockError();
                return;
            }
        }
        if (Buscmd_GimSend[i].bus_func != NULL)
            Buscmd_GimSend[i].bus_func(BusComm_TxData);
    }

#endif

    // #if __FN_IF_ENABLE(__FN_SUPER_CAP)
    //     uint32_t out_time = HAL_GetTick();

    //     for (int i = 0; i < Const_BusComm_SUPERCAP_BUFF_SIZE; i++) {
    //         while (HAL_FDCAN_GetTxFifoFreeLevel(Const_BusComm_CAN_HANDLER) == 0) {
    //             if (HAL_GetTick() - out_time >= 2)
    //                 return;
    //         }
    //         if (Buscmd_CapSend[i].bus_func != NULL)
    //             Buscmd_CapSend[i].bus_func(BusComm_TxData);
    //     }
    // #endif
    buscomm->state = BusComm_STATE_CONNECTED;
}

uint32_t decode_cont = 0;
float decode_rate;
/**
 * @brief      Data decoding function of serial port in inter bus communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
void BusComm_DecodeBusCommData(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    decode_cont++;
    decode_rate = decode_cont / HAL_GetTick();

    memcpy(BusComm_RxData, buff, rxdatalen);

    for (int i = 1; i < (Const_BusComm_RECEIVE_SIZE + 1); i++) {
        if ((stdid == Buscmd_Receive[i].cmd_id) && (Buscmd_Receive[i].bus_func != NULL)) {
            Buscmd_Receive[i].bus_func(BusComm_RxData);
            return;
        }
    }
}

uint32_t decode_cont2 = 0;
float decode_rate2;
/**
 * @brief      Data decoding function of serial port in cap communication
 * @param      buff: Data buffer
 * @param      rxdatalen: data length
 * @retval     NULL
 */
void BusComm_DecodeCapCommData(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen) {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    decode_cont2++;
    decode_rate2 = decode_cont2 / HAL_GetTick();

    memcpy(BusComm_RxData, buff, rxdatalen);

    for (int i = 1; i < (Const_CapComm_RECEIVE_SIZE + 1); i++) {
        if ((stdid == Capcmd_Receive[i].cmd_id) && (Capcmd_Receive[i].bus_func != NULL)) {
            Capcmd_Receive[i].bus_func(BusComm_RxData);
            return;
        }
    }
}

uint32_t block_num = 0;
/**
 * @brief      Buscomm send block error handler
 * @param      NULL
 * @retval     NULL
 */
void BusComm_SendBlockError() {
    block_num++;
}

/**
 * @brief      Reset inter bus communication data object
 * @param      NULL
 * @retval     NULL
 */
void BusComm_ResetBusCommData() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    // buscomm->last_update_time[0] = HAL_GetTick();

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    buscomm->last_update_time[1] = HAL_GetTick();
    buscomm->yaw_relative_angle = 0;
    buscomm->yaw_encoder_angle = 0;
    buscomm->robot_id = 0;
    buscomm->heat_17mm = 0;
    buscomm->power_limit = 0;
    // buscomm->speed_17mm = 0;
    buscomm->heat_cooling_limit = 0;
    buscomm->speed_17mm_limit = 0;
    // buscomm->main_shooter_power = 0;
    buscomm->cap_rest_energy_display = 0;
    buscomm->game_outpost_alive = 1;
#endif

// Gimbal stream
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    // buscomm->last_update_time[1] = 0;
    buscomm->gimbal_yaw_mode = 0;
    buscomm->gimbal_yaw_ref = 0;
    buscomm->gimbal_imu_pos = 0.0f;
    buscomm->gimbal_imu_spd = 0.0f;
    buscomm->chassis_mode = 0;
    buscomm->chassis_fb_ref = 0.0f;
    buscomm->chassis_lr_ref = 0.0f;
    buscomm->cap_mode_user = SUPERCAP_CTRL_OFF;
    buscomm->power_limit_mode = POWER_LIMITED;
    buscomm->cap_boost_mode_user = SUPERCAP_UNBOOST;
    buscomm->ui_cmd = 0;
    buscomm->cap_rest_energy_display = 0;
    buscomm->pitch_angle = 0.0f;
    buscomm->magazine_state = 0;
    buscomm->shooter_state = 0;
    buscomm->minipc_mode = 0;
    buscomm->minipc_target_id = 0;
    buscomm->minipc_offset_horizental = 0;
    buscomm->minipc_offset_vertical = 0;
#endif

    // // SuperCap stream
    // #if __FN_IF_ENABLE(__FN_SUPER_CAP)
    //     buscomm->last_update_time[1] = 0;
    //     buscomm->cap_state = SUPERCAP_MODE_OFF;
    //     buscomm->cap_rest_energy = 0;
    //     buscomm->power_path_change_flag = 0;
    // #endif
}

/**
 * @brief      Assignment of inter bus communication structure
 * @param      NULL
 * @retval     NULL
 */
	uint8_t auto_aim_mode = 0, cha_mode = 0;  
	uint8_t mode = 0;
  uint8_t spd_id = 0;
void BusComm_Update() {
    BusComm_BusCommDataTypeDef* data = BusComm_GetBusDataPtr();

// Chassis stream
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    GimbalYaw_GimbalYawTypeDef* gimbal = GimbalYaw_GetGimbalYawPtr();
    Referee_RefereeDataTypeDef* referee = Referee_GetRefereeDataPtr();


    switch (referee->shooter_heat0_speed_limit) {
        default:
        case 15:
            mode = 0;
            spd_id = REFEREE_SHOOTER_SPEED_15;
            break;
        case 18:
            mode = 1;
            spd_id = REFEREE_SHOOTER_SPEED_18;
            break;
        case 22:
            spd_id = REFEREE_SHOOTER_SPEED_22;
            break;
        case 30:
            mode = 2;
            spd_id = REFEREE_SHOOTER_SPEED_30;
            break;
    }
    Referee_SetAimMode(mode);

    Referee_SetCapState(data->cap_rest_energy);
    Referee_SetPitchAngle(data->pitch_angle);

    if (data->chassis_mode == CHASSIS_CTRL_NORMAL)
        cha_mode = 0;
    if (data->chassis_mode == CHASSIS_CTRL_GYRO || data->chassis_mode == CHASSIS_CTRL_SUPERGYRO)
        cha_mode = 1;
    if (data->gimbal_yaw_mode == GIMBAL_YAW_CTRL_NO_AUTO)
        auto_aim_mode = 0;
    if (data->gimbal_yaw_mode == GIMBAL_YAW_CTRL_ARMOR)
        auto_aim_mode = 1;
    if (data->gimbal_yaw_mode == GIMBAL_YAW_CTRL_SMALL_ENERGY)
        auto_aim_mode = 2;
    if (data->gimbal_yaw_mode == GIMBAL_YAW_CTRL_BIG_ENERGY)
        auto_aim_mode = 3;

    Referee_SetAimMode(mode);                        //UI  shooter_speed
		Referee_SetAutoAimMode(data->minipc_mode);   //UI  aim_mode 
    Referee_SetCapState(data->cap_rest_energy);
    Referee_SetPitchAngle(data->pitch_angle);
		Referee_SetShooterStateMode(data->shooter_state);
		Referee_SetMagazineStateMode(data->magazine_state);
	  Referee_SetMinipc_Offset(data->minipc_offset_horizental,data->minipc_offset_vertical);

    data->yaw_encoder_angle = Motor_gimbalMotorYaw.encoder.consequent_angle - Const_YAW_MOTOR_INIT_OFFSET - GimbalYaw_Angle_compensate;
    // while (data->yaw_encoder_angle > 180)
    //     data->yaw_encoder_angle -= 360;
    // while (data->yaw_encoder_angle < -180)
    //     data->yaw_encoder_angle += 360;

    data->yaw_relative_angle = Motor_gimbalMotorYaw.encoder.limited_angle - Const_YAW_MOTOR_INIT_OFFSET - GimbalYaw_Angle_compensate;
    data->robot_id = referee->robot_id;
    data->power_limit = referee->max_chassis_power;
    data->heat_17mm = referee->shooter_heat0;

    data->speed_17mm_fdb = referee->bullet_speed;
    data->heat_cooling_limit = referee->shooter_heat0_cooling_limit;
    data->speed_17mm_limit = spd_id;
    // data->main_shooter_power = referee->mains_power_shooter_output;
    switch (referee->robot_id) {
        case 3:
        case 4:
        case 5:
            data->game_outpost_alive = referee->blue_outpost_HP > 0;
            break;
        case 103:
        case 104:
        case 105:
            data->game_outpost_alive = referee->red_outpost_HP > 0;
            break;
        default:
            break;
    }

#endif

        // Gimbal stream
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    Gimbal_GimbalTypeDef* gimbal = Gimbal_GetGimbalControlPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();

    data->gimbal_yaw_mode = gimbal->yaw_mode + 0x02;
    data->gimbal_yaw_ref = gimbal->angle.yaw_angle_ref;
    data->gimbal_imu_pos = imu->angle.yaw;
    data->gimbal_imu_spd = imu->speed.yaw;
    data->infantry_code = Key_GetEquipCode();

    data->pitch_angle = imu->angle.pitch;
    data->magazine_state = 0;
    data->shooter_state = shooter->shooter_mode > 0;
    data->minipc_mode = minipc->aim_mode;
    data->minipc_target_id = minipc_data->ID;
    data->minipc_offset_horizental = minipc->vision_offset[minipc->aim_mode].horizental;
    data->minipc_offset_vertical = minipc->vision_offset[minipc->aim_mode].vertical;

#endif

    //     // Super Cap stream
    // #if __FN_IF_ENABLE(__FN_SUPER_CAP)
    //     Sen_PowerValueTypeDef* powerValue = Sen_GetPowerDataPtr();

    //     if (powerValue->CapPercent >= 100)
    //         data->cap_rest_energy = 100;
    //     else
    //         data->cap_rest_energy = powerValue->CapPercent;
    // #endif
}
uint8_t flag=0;
    static uint8_t ui_cmd_last = 0;

void _cmd_mode_control() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
	


#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
    switch (buscomm->gimbal_yaw_mode) {
        case GIMBAL_YAW_CTRL_BIG_ENERGY: {
            GimbalYaw_SetMode(GimbalYaw_MODE_BIG_ENERGY);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            // GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetEncoderFdb();
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_SMALL_ENERGY: {
            GimbalYaw_SetMode(GimbalYaw_MODE_SMALL_ENERGY);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            // GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetEncoderFdb();
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_ARMOR: {
            GimbalYaw_SetMode(GimbalYaw_MODE_ARMOR);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_IMU_DEBUG: {
            GimbalYaw_SetMode(GimbalYaw_MODE_IMU_DEBUG);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetEncoderFdb();
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        case GIMBAL_YAW_CTRL_NO_AUTO: {
            GimbalYaw_SetMode(GimbalYaw_MODE_NO_AUTO);
            GimbalYaw_SetYawRef(buscomm->gimbal_yaw_ref);
            GimbalYaw_SetIMUYawPositionFdb(buscomm->gimbal_imu_pos);
            GimbalYaw_SetIMUYawSpeedFdb(buscomm->gimbal_imu_spd);
            GimbalYaw_SetGimbalYawControlState(1);
            GimbalYaw_SetGimbalYawOutputState(1);
            break;
        }
        default:
            return;  // error, stop decoding
    }

    switch (buscomm->chassis_mode) {
        case CHASSIS_CTRL_STOP: {
            Chassis_SetMode(Chassis_MODE_STOP);
            Chassis_SetForwardBackRef(0);
            Chassis_SetLeftRightRef(0);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);
            break;
        }
        case CHASSIS_CTRL_NORMAL: {
            Chassis_SetMode(Chassis_MODE_NORMAL);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(0);

            break;
        }
        case CHASSIS_CTRL_SUPERGYRO: {
            Chassis_SetMode(Chassis_MODE_SUPERGYRO);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(1);

            break;
        }
        case CHASSIS_CTRL_GYRO: {
            Chassis_SetMode(Chassis_MODE_GYRO);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(1);

            break;
        }
        case CHASSIS_CTRL_ASS: {
            Chassis_SetMode(Chassis_MODE_ASS);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(0);
            break;
        }
        case CHASSIS_CTRL_CRAB: {
            Chassis_SetMode(Chassis_MODE_CRAB);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(0);
            break;
        }
        case CHASSIS_CTRL_DISCO: {
            Chassis_SetMode(Chassis_MODE_DISCO);
            Chassis_SetForwardBackRef(buscomm->chassis_fb_ref);
            Chassis_SetLeftRightRef(buscomm->chassis_lr_ref);
            Chassis_SetChassisControlState(1);
            Chassis_SetChassisOutputState(1);

            Referee_SetWidthMode(0);
            break;
        }
        default:
            return;  // error, stop decoding
    }

    switch (buscomm->power_limit_mode) {
        case POWER_LIMITED:
            Power_ForceChangePowerMode(POWER_LIMIT);
            break;
        case POWER_UNLIMIT:
            Power_ForceChangePowerMode(POWER_UNLIMITED);
            break;
        default:
            break;
    }
		
		
				Referee_SetChassisMode(chassis->mode);			//UI   chassis_mode
#endif
}

/**
 * @brief      Interrupt callback function of can in inter Bus communication
 * @param
 * @retval     NULL
 */
void BusComm_CANRxCallback(FDCAN_HandleTypeDef* pfdhcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    if (pfdhcan == Const_BusComm_CAN_HANDLER) {
        BusComm_DecodeBusCommData(rxdata, stdid, len);
    } else if (pfdhcan == Const_CapComm_CAN_HANDLER) {
        BusComm_DecodeCapCommData(rxdata, stdid, len);
    }
}
