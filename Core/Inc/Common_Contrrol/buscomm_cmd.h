/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Common_Contrrol\buscomm_cmd.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-05-02 10:13:12
 */

#ifndef BUSCOMM_CMD_H
#define BUSCOMM_CMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "stm32g4xx_hal.h"

extern const uint32_t CMD_SET_CONTROL;
extern const uint32_t CMD_SET_IMU_YAW;
extern const uint32_t CMD_SET_CHA_REF;

extern const uint32_t CMD_SET_REFEREE_DATA_1;
extern const uint32_t CMD_SET_REFEREE_DATA_2;

extern const uint32_t CMD_SET_CAP_MODE;
// extern const uint32_t CMD_SEND_CAP_STATE;
extern const uint32_t CMD_SET_CAP_STATE_1;
extern const uint32_t CMD_SET_CAP_STATE_2;

extern const uint32_t CMD_CHASSIS_SEND_PACK_1;
extern const uint32_t CMD_CHASSIS_SEND_PACK_2;

extern const uint32_t CMD_GIMBAL_SEND_PACK_1;
extern const uint32_t CMD_GIMBAL_SEND_PACK_2;
extern const uint32_t CMD_GIMBAL_SEND_PACK_3;
extern const uint32_t CMD_GIMBAL_SEND_PACK_4;

extern const uint32_t CMD_SUPERCAP_SEND_PACK_1;

typedef struct {
    uint32_t cmd_id;
    void (*bus_func)(uint8_t buff[]);
} BusCmd_TableEntry;

extern BusCmd_TableEntry Buscmd_Receive[6];
extern BusCmd_TableEntry Capcmd_Receive[3];
extern BusCmd_TableEntry Buscmd_GimSend[3];
extern BusCmd_TableEntry Buscmd_ChaSend[3];
// extern BusCmd_TableEntry Buscmd_CapSend[1];

#endif

#ifdef __cplusplus
}
#endif
