/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Common_Contrrol\buscomm_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-16 10:30:40
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
extern const uint8_t POWER_LIMITED;
extern const uint8_t POWER_UNLIMIT;
//      gimbal yaw mode
extern const uint8_t GIMBAL_YAW_CTRL_BIG_ENERGY;
extern const uint8_t GIMBAL_YAW_CTRL_SMALL_ENERGY;
extern const uint8_t GIMBAL_YAW_CTRL_ARMOR;
extern const uint8_t GIMBAL_YAW_CTRL_IMU_DEBUG;
extern const uint8_t GIMBAL_YAW_CTRL_NO_AUTO;
//      chassis mode
extern const uint8_t CHASSIS_CTRL_STOP;
extern const uint8_t CHASSIS_CTRL_NORMAL;
extern const uint8_t CHASSIS_CTRL_GYRO;
//      cap mode
extern const uint8_t SUPERCAP_CTRL_OFF;
extern const uint8_t SUPERCAP_CTRL_ON;
//      cap boost mode
extern const uint8_t SUPERCAP_BOOST;
extern const uint8_t SUPERCAP_UNBOOST;
//      cap state
extern const uint8_t SUPERCAP_MODE_OFF;
extern const uint8_t SUPERCAP_MODE_ON;
extern const uint8_t SUPERCAP_MODE_ERROR;

extern FDCAN_TxHeaderTypeDef BusComm_GimControl;
extern FDCAN_TxHeaderTypeDef BusComm_GimGimbalData;
extern FDCAN_TxHeaderTypeDef BusComm_GimImuYaw;
extern FDCAN_TxHeaderTypeDef BusComm_GimChassisRef;

extern FDCAN_TxHeaderTypeDef BusComm_ChaRefereeData;
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
    BusComm_PKG_REFEREE = 0,
    BusComm_PKG_CTRL = 1,
    BusComm_PKG_IMU = 2,
    BusComm_PKG_CHA_REF = 3,
    BusComm_PKG_CAP_1 = 4,
    BusComm_PKG_CAP_2 = 5
} BusComm_BusCommPkgEnum;

typedef struct {
    BusComm_BusCommStateEnum state;
    uint32_t last_update_time[6];

    // Chassis up stream
    float yaw_relative_angle;  // Angle of chassis relative to pan tilt
    uint8_t robot_id;          // Robot ID
    uint8_t power_limit;       // Super capacitor state
    uint16_t heat_17mm;        // Heat transfer of 17mm launching mechanism
    uint16_t heat_cooling_limit;
    // uint16_t speed_17mm;
    uint16_t speed_17mm_limit;
    uint8_t main_shooter_power;
    uint8_t cap_mode_fnl;
    uint8_t cap_boost_mode_fnl;
    uint8_t chassis_power_limit;
    uint8_t chassis_power_buffer;
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
    uint8_t power_limit_mode;     // Force to change power limit mode 2bit
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
