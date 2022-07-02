/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Common_Contrrol\buscomm_cmd.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-02 15:23:31
 */

#include "buscomm_cmd.h"
#include "buscomm_ctrl.h"
#include "const.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
#include "cha_chassis_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "referee_periph.h"
#include "cha_power_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#include "gim_gimbal_ctrl.h"
#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
#include "supercap_ctrl.h"
#endif

const uint32_t CMD_SET_CONTROL = 0x201;
const uint32_t CMD_SET_IMU_YAW = 0x203;
const uint32_t CMD_SET_CHA_REF = 0x204;

const uint32_t CMD_SET_REFEREE_DATA_1 = 0x206;
const uint32_t CMD_SET_REFEREE_DATA_2 = 0x207;

const uint32_t CMD_SET_CAP_MODE = 0x98;
// const uint32_t CMD_SEND_CAP_STATE = 0x207;
const uint32_t CMD_SET_CAP_STATE_1 = 0x299;
const uint32_t CMD_SET_CAP_STATE_2 = 0x298;

const uint32_t CMD_CHASSIS_SEND_PACK_1 = 0xA1;
const uint32_t CMD_CHASSIS_SEND_PACK_2 = 0xA2;
const uint32_t CMD_CHASSIS_SEND_PACK_3 = 0xA3;

const uint32_t CMD_GIMBAL_SEND_PACK_1 = 0xB1;
const uint32_t CMD_GIMBAL_SEND_PACK_2 = 0xB2;
const uint32_t CMD_GIMBAL_SEND_PACK_3 = 0xB3;

// const uint32_t CMD_SUPERCAP_SEND_PACK_1 = 0xC1;

/*const*/ uint32_t Const_Send_Period_Control = 0;
/*const*/ uint32_t Const_Send_Period_IMU_Yaw = 0;
/*const*/ uint32_t Const_Send_Period_Cha_ref = 5;
/*const*/ uint32_t Const_Send_Period_Referee_1 = 5;
/*const*/ uint32_t Const_Send_Period_Referee_2 = 100;
/*const*/ uint32_t Const_Send_Period_Cap_Mode = 5;

static void _send_referee_data_1(uint8_t buff[]);
static void _send_referee_data_2(uint8_t buff[]);
static void _send_control(uint8_t buff[]);
static void _send_imu_yaw(uint8_t buff[]);
static void _send_chassis_ref(uint8_t buff[]);
// static void _send_cap_state(uint8_t buff[]);
static void _send_cap_mode(uint8_t buff[]);
static void _set_referee_data_1(uint8_t buff[]);
static void _set_referee_data_2(uint8_t buff[]);
static void _set_control(uint8_t buff[]);
static void _set_imu_yaw(uint8_t buff[]);
static void _set_cha_ref(uint8_t buff[]);
// static void _set_cap_state(uint8_t buff[]);
static void _set_cap_state_1(uint8_t buff[]);
static void _set_cap_state_2(uint8_t buff[]);

BusCmd_TableEntry Buscmd_Receive[6] = {
    {0xff, NULL},
    {CMD_SET_REFEREE_DATA_1, &_set_referee_data_1},
    {CMD_SET_REFEREE_DATA_2, &_set_referee_data_2},
    {CMD_SET_CONTROL, &_set_control},
    {CMD_SET_IMU_YAW, &_set_imu_yaw},
    {CMD_SET_CHA_REF, &_set_cha_ref}};

BusCmd_TableEntry Capcmd_Receive[3] = {
    {0xff, NULL},
    {CMD_SET_CAP_STATE_1, &_set_cap_state_1},
    {CMD_SET_CAP_STATE_2, &_set_cap_state_2}};

BusCmd_TableEntry Buscmd_GimSend[3] = {
    {CMD_GIMBAL_SEND_PACK_1, &_send_control},
    {CMD_GIMBAL_SEND_PACK_2, &_send_imu_yaw},
    {CMD_GIMBAL_SEND_PACK_3, &_send_chassis_ref}};

BusCmd_TableEntry Buscmd_ChaSend[3] = {
    {CMD_CHASSIS_SEND_PACK_1, &_send_referee_data_1},
    {CMD_CHASSIS_SEND_PACK_2, &_send_referee_data_2},
    {CMD_CHASSIS_SEND_PACK_3, &_send_cap_mode}};

// BusCmd_TableEntry Buscmd_CapSend[1] = {
//     {CMD_SUPERCAP_SEND_PACK_1, &_send_cap_state}};

int counta[6];
float ratea[6];

/*      send functions driver       */
/*************** CHASSIS SEND *****************/

static void _send_referee_data_1(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_Referee_1)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_ChaRefereeData_1;
    counta[0]++;
    ratea[0] = 1000 * counta[0] / HAL_GetTick();
    memset(buff, 0, 8);

    float2buff(buscomm->yaw_encoder_angle, buff);
    i162buff((int16_t)(buscomm->speed_17mm_fdb * 100), buff + 4);
    ui162buff(buscomm->heat_17mm, buff + 6);

    FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _send_referee_data_2(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_Referee_2)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_ChaRefereeData_2;
    counta[1]++;
    ratea[1] = 1000 * counta[1] / HAL_GetTick();
    memset(buff, 0, 8);
    buff[0] = (buscomm->game_outpost_alive << 7) + buscomm->robot_id;
    buff[1] = buscomm->speed_17mm_limit << 6 ;
    i162buff(buscomm->heat_cooling_limit, buff + 2);

    FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _send_cap_mode(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_Cap_Mode)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_CapMode;
    counta[2]++;
    ratea[2] = 1000 * counta[2] / HAL_GetTick();
    memset(buff, 0, 8);
    buff[0] = buscomm->cap_mode_fnl | (0x77 << 1);
    buff[1] = buscomm->cap_boost_mode_fnl | 0x02;  // Todo bit1 error chassis
    buff[2] = buscomm->chassis_power_limit;
    buff[3] = buscomm->chassis_power_buffer;
    float2buff(buscomm->chassis_power, buff + 4);
    FDCAN_SendMessage(Const_CapComm_CAN_HANDLER, pheader, buff);
}

/*************** GIMBAL SEND *****************/
static void _send_control(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_Control)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_GimControl;
    counta[3]++;
    ratea[3] = 1000 * counta[3] / HAL_GetTick();
    memset(buff, 0, 8);
    buff[0] = (buscomm->gimbal_yaw_mode << 4) + (buscomm->chassis_mode << 2) + buscomm->power_limit_mode;
    buff[1] = (buscomm->infantry_code << 4) + (buscomm->ui_cmd << 2) + (buscomm->cap_mode_user << 1) + buscomm->cap_boost_mode_user;
    float2buff(buscomm->gimbal_yaw_ref, buff + 2);
    FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _send_imu_yaw(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_IMU_Yaw)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_GimImuYaw;
    counta[4]++;
    ratea[4] = 1000 * counta[4] / HAL_GetTick();
    memset(buff, 0, 8);
    float2buff(buscomm->gimbal_imu_spd, buff);
    float2buff(buscomm->gimbal_imu_pos, buff + 4);
    FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

static void _send_chassis_ref(uint8_t buff[]) {
    static uint32_t last_send_time = 0;
    if ((HAL_GetTick() - last_send_time) <= Const_Send_Period_Cha_ref)
        return;
    last_send_time = HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    FDCAN_TxHeaderTypeDef* pheader = &BusComm_GimChassisRef;
    counta[5]++;
    ratea[5] = 1000 * counta[5] / HAL_GetTick();
    memset(buff, 0, 8);
    float2buff(buscomm->chassis_fb_ref, buff);
    float2buff(buscomm->chassis_lr_ref, buff + 4);
    FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
}

// /*************** SUPERCAP SEND *****************/
// static void _send_cap_state(uint8_t buff[]) {
//     BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
//     FDCAN_TxHeaderTypeDef* pheader = &BusComm_CapState;
//     count7a++;
//     rate7a = 1000 * count7a / HAL_GetTick();
//     memset(buff, 0, 8);
//     buff[0] = buscomm->cap_state;
//     buff[1] = buscomm->cap_rest_energy;
//     FDCAN_SendMessage(Const_BusComm_CAN_HANDLER, pheader, buff);
// }

/*************** RECEIVE *****************/
int countb[7];
float rateb[7];
static void _set_referee_data_1(uint8_t buff[]) {
    countb[BusComm_PKG_REFEREE_1]++;
    rateb[BusComm_PKG_REFEREE_1] = 1000 * countb[BusComm_PKG_REFEREE_1] / HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    static float yaw_angle;

    yaw_angle = buff2float(buff);

    buscomm->yaw_encoder_angle = yaw_angle;
    while (yaw_angle > 180)
        yaw_angle -= 360;
    while (yaw_angle < -180)
        yaw_angle += 360;
    buscomm->yaw_relative_angle = yaw_angle;

    buscomm->speed_17mm_fdb = ((float)buff2i16(buff + 4)) / 100;
    buscomm->heat_17mm = buff2ui16(buff + 6);

    buscomm->last_update_time[BusComm_PKG_REFEREE_1] = HAL_GetTick();
}

static void _set_referee_data_2(uint8_t buff[]) {
    countb[BusComm_PKG_REFEREE_2]++;
    rateb[BusComm_PKG_REFEREE_2] = 1000 * countb[BusComm_PKG_REFEREE_2] / HAL_GetTick();

    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->robot_id = buff[0] & 0x7F;
    buscomm->game_outpost_alive = (buff[0] & 0x80) >> 7;
    buscomm->speed_17mm_limit = (buff[1] & 0xC0) >> 6;
    buscomm->heat_cooling_limit = buff2i16(buff + 2);

    buscomm->last_update_time[BusComm_PKG_REFEREE_2] = HAL_GetTick();
}

static void _set_control(uint8_t buff[]) {
    countb[BusComm_PKG_CTRL]++;
    rateb[BusComm_PKG_CTRL] = 1000 * countb[BusComm_PKG_CTRL] / HAL_GetTick();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    // Todo
    if ((buscomm->cap_mode_user == SUPERCAP_CTRL_ON) && (buff[3] == SUPERCAP_CTRL_OFF)) {
        buscomm->power_path_change_flag = HAL_GetTick();
    }

    buscomm->gimbal_yaw_mode = buff[0] >> 4;
    buscomm->chassis_mode = (buff[0] & 0x0C) >> 2;
    buscomm->power_limit_mode = buff[0] & 0x03;
    buscomm->infantry_code = buff[1] >> 4;
    buscomm->ui_cmd = buff[1] & 0x04 >> 2;
    buscomm->cap_mode_user = (buff[1] & 0x02) >> 1;
    buscomm->cap_boost_mode_user = buff[1] & 0x01;
    buscomm->gimbal_yaw_ref = buff2float(buff + 2);
    _cmd_mode_control();

    buscomm->last_update_time[BusComm_PKG_CTRL] = HAL_GetTick();
}

static void _set_imu_yaw(uint8_t buff[]) {
    countb[BusComm_PKG_IMU]++;
    rateb[BusComm_PKG_IMU] = 1000 * countb[BusComm_PKG_IMU] / HAL_GetTick();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->gimbal_imu_spd = buff2float(buff);
    buscomm->gimbal_imu_pos = buff2float(buff + 4);
    buscomm->last_update_time[BusComm_PKG_IMU] = HAL_GetTick();
}

static void _set_cha_ref(uint8_t buff[]) {
    countb[BusComm_PKG_CHA_REF]++;
    rateb[BusComm_PKG_CHA_REF] = 1000 * countb[BusComm_PKG_CHA_REF] / HAL_GetTick();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    buscomm->chassis_fb_ref = buff2float(buff);
    buscomm->chassis_lr_ref = buff2float(buff + 4);

    buscomm->last_update_time[BusComm_PKG_CHA_REF] = HAL_GetTick();
}
// int count7;
// float rate7;
// static void _set_cap_state(uint8_t buff[]) {
//     count7++;
//     rate7 = 1000 * count7 / HAL_GetTick();
//     BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

//     buscomm->cap_state = buff[0];
//     buscomm->cap_rest_energy = buff[1];
//     buscomm->cap_rest_energy_display = (float)buscomm->cap_rest_energy;
// }

static void _set_cap_state_1(uint8_t buff[]) {
    countb[BusComm_PKG_CAP_1]++;
    rateb[BusComm_PKG_CAP_1] = 1000 * countb[BusComm_PKG_CAP_1] / HAL_GetTick();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->Cap_power = buff2float(buff);
    buscomm->cap_rest_energy = buff[4];

    buscomm->last_update_time[BusComm_PKG_CAP_1] = HAL_GetTick();
}

static void _set_cap_state_2(uint8_t buff[]) {
    countb[BusComm_PKG_CAP_2]++;
    rateb[BusComm_PKG_CAP_2] = 1000 * countb[BusComm_PKG_CAP_2] / HAL_GetTick();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    buscomm->Cap_voltage = buff2float(buff);
    buscomm->Cap_current = buff2float(buff + 4);

    buscomm->last_update_time[BusComm_PKG_CAP_2] = HAL_GetTick();
}
