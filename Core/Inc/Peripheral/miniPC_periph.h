/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\miniPC_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-07-04 22:47:39
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
    // uint8_t heart_flag;

    // up stream
    uint8_t team_color;
    uint8_t mode;
    uint8_t is_get_target;  // 1 to get armor plate, 0 to not get armor plate

    // down stream
    float yaw_angle;
    float pitch_angle;
    float distance;

    // down stream new
    uint16_t timestamp;
    int16_t x, y, z;
    int16_t vx, vz;
    uint8_t ID;

    uint8_t addressee;
    MiniPC_MiniPCStateEnum state;
    uint32_t last_update_time;
    uint8_t new_data_flag;

} MiniPC_MiniPCDataTypeDef;

typedef struct __attribute__((packed)) {
    uint16_t timestamp;
    uint8_t is_get;
    int16_t x, y, z;
    int16_t vx, vz;
    uint8_t ID;
} MiniPC_ArmorPacket_t;

// typedef struct __attribute__((packed)) {
//     uint16_t timestamp;
//     uint8_t is_get;
//     int16_t x, y, z;
//     int16_t vx, vz;
//     uint8_t ID;
// } MiniPC_BuffPacket_t;

// extern const uint8_t Const_MiniPC_ARMOR;
// extern const uint8_t Const_MiniPC_BIG_BUFF;
// extern const uint8_t Const_MiniPC_LITTLE_BUFF;

extern MiniPC_MiniPCDataTypeDef MiniPC_MiniPCData;
extern uint32_t MiniPC_Data_FrameTime;
// extern UART_HandleTypeDef* Const_MiniPC_UART_HANDLER;
extern float MiniPC_angles[5];
extern float sin_gen;

void MiniPC_InitMiniPC(void);
MiniPC_MiniPCDataTypeDef* MiniPC_GetMiniPCDataPtr(void);
void MiniPC_SendHeartPacket(void);
void MiniPC_SendDataPacket(void);
uint8_t MiniPC_IsMiniPCOffline(void);
// void MiniPC_RXCallback(UART_HandleTypeDef* huart);
void MiniPC_DecodeMiniPCPacket(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ArmorPacketDecode(void* buff, uint16_t rxdatalen);
void MiniPC_BuffPacketDecode(uint8_t* buff, uint16_t rxdatalen);
void MiniPC_ResetMiniPCData(void);
void MiniPC_Update(void);
uint8_t MiniPC_VerifyMiniPCData(uint8_t* buff, uint16_t rxdatalen);

#endif

#ifdef __cplusplus
}
#endif

#endif
