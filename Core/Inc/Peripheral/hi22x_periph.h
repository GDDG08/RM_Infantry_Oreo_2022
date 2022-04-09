/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\hi22x_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:58:07
 */

#ifndef HI22X_PERIPH_H
#define HI22X_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#include "uart_util.h"
#if __FN_IF_ENABLE(__IMU_HI22X)

#include "main.h"

typedef enum {
    HI22X_STATE_NULL = 0,
    HI22X_STATE_CONNECTED = 1,
    HI22X_STATE_LOST = 2,
    HI22X_STATE_ERROR = 3,
    HI22X_STATE_PENDING = 4
} HI22X_HI22XStateEnum;

typedef struct {
    float yaw;
    float pitch;
    float row;
} HI22X_HI22XSpeedTypeDef;

typedef struct {
    float yaw;
    float pitch;
    float row;
} HI22X_HI22XAngleTypeDef;

typedef struct {
    float yaw;
    float pitch;
    float row;
} HI22X_HI22XCountTypeDef;

typedef struct {
    HI22X_HI22XCountTypeDef count;

    HI22X_HI22XSpeedTypeDef speed;
    HI22X_HI22XAngleTypeDef angle;

    float sensor_time;
    float temperature;

    HI22X_HI22XAngleTypeDef last_angle;
    HI22X_HI22XAngleTypeDef now_angle;

    float yaw_angle_offset;
    float pitch_angle_offset;

    HI22X_HI22XStateEnum state;
    uint32_t last_update_time;
} HI22X_HI22XDataTypeDef;

HI22X_HI22XDataTypeDef* HI22X_GetHI22XDataPtr(void);
uint8_t HI22X_Init(void);
uint8_t HI22X_IsHI22XOffline(void);
void HI22X_InitAngelOffset(void);
void HI22X_ResetHI22XData(void);
void HI22X_HI22xDecodeHI22XData(uint8_t* buff, int rxdatalen);

#endif

extern UART_HandleTypeDef* Const_HI22X_UART_HANDLER;
void HI22X_RXCallback(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}
#endif
