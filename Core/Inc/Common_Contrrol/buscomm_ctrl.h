/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Common_Contrrol\buscomm_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-04 21:42:24
 */

#ifndef BUSCOMM_CTRL_H
#define BUSCOMM_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "cmsis_os.h"
#include "FreeRTOS.h"

#include "fdcan_util.h"
#include "buff_lib.h"
#include "string.h"
#include "filter_alg.h"

extern FDCAN_HandleTypeDef* Const_BusComm_CAN_HANDLER;
extern FDCAN_HandleTypeDef* Const_CapComm_CAN_HANDLER;

extern const uint8_t Const_BusComm_FRAME_HEADER_SOF;

//      power limit mode
#define POWER_LIMITED 0x00
#define POWER_UNLIMIT 0x01
//      gimbal yaw mode
#define GIMBAL_YAW_CTRL_NO_AUTO 0x03
#define GIMBAL_YAW_CTRL_ARMOR 0x04
#define GIMBAL_YAW_CTRL_IMU_DEBUG 0x05
#define GIMBAL_YAW_CTRL_BIG_ENERGY 0x06
#define GIMBAL_YAW_CTRL_SMALL_ENERGY 0x07
//      chassis mode
#define CHASSIS_CTRL_STOP 0x00
#define CHASSIS_CTRL_NORMAL 0x01
#define CHASSIS_CTRL_GYRO 0x02
#define CHASSIS_CTRL_ASS 0x03
#define CHASSIS_CTRL_CRAB 0x04
#define CHASSIS_CTRL_DISCO 0x05
#define CHASSIS_CTRL_SUPERGYRO 0x06
//      cap mode
#define SUPERCAP_CTRL_OFF 0x00
#define SUPERCAP_CTRL_ON 0x01
//      cap boost mode
#define SUPERCAP_UNBOOST 0x00
#define SUPERCAP_BOOST 0x01
//      cap state
#define SUPERCAP_MODE_OFF 0x51
#define SUPERCAP_MODE_ON 0x52
#define SUPERCAP_MODE_ERROR 0x53
//      referee state
#define REFEREE_SHOOTER_SPEED_15 0x0
#define REFEREE_SHOOTER_SPEED_18 0x1
#define REFEREE_SHOOTER_SPEED_22 0x2
#define REFEREE_SHOOTER_SPEED_30 0x3

#define REFEREE_SHOOTER_SPEED_NORMAL = 0x0
#define REFEREE_SHOOTER_SPEED_OVER = 0x1

#define REFEREE_GAME_OUTPOST_ALIVE = 0x0
#define REFEREE_GAME_OUTPOST_DEAD = 0x1

extern FDCAN_TxHeaderTypeDef BusComm_GimControl;
extern FDCAN_TxHeaderTypeDef BusComm_GimGimbalData;
extern FDCAN_TxHeaderTypeDef BusComm_GimImuYaw;
extern FDCAN_TxHeaderTypeDef BusComm_GimChassisRef;

extern FDCAN_TxHeaderTypeDef BusComm_ChaRefereeData_1;
extern FDCAN_TxHeaderTypeDef BusComm_ChaRefereeData_2;
extern FDCAN_TxHeaderTypeDef BusComm_CapMode;

// extern FDCAN_TxHeaderTypeDef BusComm_CapState;
extern FDCAN_TxHeaderTypeDef BusComm_CapState_1;
extern FDCAN_TxHeaderTypeDef BusComm_CapState_2;

typedef enum {
    BusComm_STATE_NULL = 0,
    BusComm_STATE_CONNECTED = 1,
    BusComm_STATE_LOST = 2,
    BusComm_STATE_ERROR = 3,
    BusComm_STATE_PENDING = 4
} BusComm_BusCommStateEnum;

typedef enum {
    BusComm_PKG_REFEREE_1 = 0,
    BusComm_PKG_REFEREE_2,
    BusComm_PKG_CTRL,
    BusComm_PKG_IMU,
    BusComm_PKG_CHA_REF,
    BusComm_PKG_CAP_1,
    BusComm_PKG_CAP_2
} BusComm_BusCommPkgEnum;

typedef struct {
    BusComm_BusCommStateEnum state;
    uint32_t last_update_time[7];

    // Chassis up stream
    float yaw_relative_angle;     // Angle of chassis relative to pan tilt
    float yaw_encoder_angle;      // Angle of chassis relative to pan tilt (consequent)
    uint8_t robot_id;             // Robot ID
    uint8_t power_limit;          // Super capacitor state
    uint16_t heat_17mm;           // Heat transfer of 17mm launching mechanism
    uint16_t heat_cooling_limit;  //
    float speed_17mm_fdb;         // int16_t
    uint8_t speed_17mm_limit;     // speed enum 2bit
    // uint8_t main_shooter_power;
    uint8_t cap_mode_fnl;
    uint8_t cap_boost_mode_fnl;
    uint8_t chassis_power_limit;
    uint8_t chassis_power_buffer;
    uint8_t game_outpost_alive;
    float chassis_power;

    // Gimbal up stream
    uint8_t gimbal_yaw_mode;      // Yaw mode of gimbal    3bit
    float gimbal_yaw_ref;         // gimbal yaw target value
    float gimbal_imu_pos;         // gimbal yaw IMU angle feedback value
    float gimbal_imu_spd;         // Speed feedback value of gimbal yaw IMU
    uint8_t chassis_mode;         // Chassis mode    2bit
    float chassis_fb_ref;         // Target value of forward and back speed of chassis
    float chassis_lr_ref;         // Target value of chassis left and right speed
    uint8_t cap_mode_user;        // Capacitance mode   1bit
    uint8_t cap_boost_mode_user;  // cap boost mode    1bit
    uint8_t power_limit_mode;     // Force to change power limit mode 1bit
    // float pitch_angle;
    uint8_t ui_cmd;         // 1bit
    uint8_t infantry_code;  // 4bit

    // Super Cap up stream
    uint32_t power_path_change_flag;
    uint8_t cap_state;
    uint8_t cap_rest_energy;
    float cap_rest_energy_display;

    float Cap_power;    // Chassis Power
    float Cap_voltage;  // Chassis Voltage
    float Cap_current;  // Chassis Current
} BusComm_BusCommDataTypeDef;

extern BusComm_BusCommDataTypeDef BusComm_BusCommData;
extern osMessageQId BusCommSend_QueueHandle;

void BusComm_Task(void const* argument);
void BusComm_InitBusComm(void);
BusComm_BusCommDataTypeDef* BusComm_GetBusDataPtr(void);
uint8_t BusComm_IsBusCommOffline(uint8_t);
void BusComm_SendBusCommData(void);
void BusComm_CANRxCallback(FDCAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);
void BusComm_DecodeBusCommData(uint8_t buff[], uint32_t stdid, uint16_t rxdatalen);
void BusComm_ResetBusCommData(void);
void BusComm_Update(void);
void _cmd_mode_control(void);
void BusComm_SendBlockError(void);

#endif

#ifdef __cplusplus
}
#endif
