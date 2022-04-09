/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\miniPC_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-05 14:25:49
 */

#ifndef MINIPC_PERIPH_H
#define MINIPC_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)

//#include "uart_util.h"
#include "usbd_cdc_if.h"
#include "buff_lib.h"
#include "math.h"

typedef enum {
    MiniPC_NULL = 0,
    MiniPC_CONNECTED = 1,
    MiniPC_LOST = 2,
    MiniPC_ERROR = 3,
    MiniPC_PENDING = 4
} MiniPC_MiniPCStateEnum;

typedef struct {
    uint8_t heart_flag;

    // up stream
    uint8_t team_color;
    uint8_t mode;

    // down stream
    uint8_t is_get_target;  // 1 to get armor plate, 0 to not get armor plate
    float yaw_angle;
    float pitch_angle;
    float distance;

    uint8_t addressee;
    MiniPC_MiniPCStateEnum state;
    uint32_t last_update_time;

    uint8_t new_data_flag;
} MiniPC_MiniPCDataTypeDef;

extern const uint8_t Const_MiniPC_ARMOR;
extern const uint8_t Const_MiniPC_BIG_BUFF;
extern const uint8_t Const_MiniPC_LITTLE_BUFF;

extern MiniPC_MiniPCDataTypeDef MiniPC_MiniPCData;
// extern UART_HandleTypeDef* Const_MiniPC_UART_HANDLER;

void MiniPC_InitMiniPC(void);
MiniPC_MiniPCDataTypeDef* MiniPC_GetMiniPCDataPtr(void);
void MiniPC_SendHeartPacket(void);
void MiniPC_SendDataPacket(void);
uint8_t MiniPC_IsMiniPCOffline(void);
// void MiniPC_RXCallback(UART_HandleTypeDef* huart);
void MiniPC_DecodeMiniPCPacket(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_HeartPacketDecode(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ArmorPacketDecode(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ResetMiniPCData(void);
void MiniPC_Update(void);
uint8_t MiniPC_VerifyMiniPCData(uint8_t* buff, uint16_t rxdatalen);

#endif

#ifdef __cplusplus
}
#endif

#endif
