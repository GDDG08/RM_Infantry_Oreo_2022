/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Peripheral\hi22x_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:09
 */

#include "hi22x_periph.h"

UART_HandleTypeDef* Const_HI22X_UART_HANDLER = &huart1;

#if __FN_IF_ENABLE(__IMU_HI22X)

#include "crc_alg.h"
#include "const.h"

#include "gim_ins_ctrl.h"

const uint16_t Const_HI22X_RX_BUFF_LEN = 20;
const uint16_t Const_HI22X_OFFLINE_TIME = 1000;
const uint16_t Const_HI22X_HI22X_OFFLINE_TIME = 100;
uint8_t HI22X_RxData[Const_HI22X_RX_BUFF_LEN];

HI22X_HI22XDataTypeDef HI22X_HI22XData;
CRC_MatchEnum CRC_HI22XEnum;

float speed[3];
float angle[3];

/**
 * @brief      Get pinter to the hi22x data object
 * @param      NULL
 * @retval     Pointer to hi22x data object
 */
HI22X_HI22XDataTypeDef* HI22X_GetHI22XDataPtr() {
    return &HI22X_HI22XData;
}

/**
 * @brief      Initialization hi22x
 * @param      NULL
 * @retval     NULL
 */
uint8_t HI22X_Init() {
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();

    // For Hi22x series hi22x
    HI22X_ResetHI22XData();
    HI22X_HI22XData.last_update_time = HAL_GetTick();
    Uart_InitUartDMA(Const_HI22X_UART_HANDLER);
    Uart_ReceiveDMA(Const_HI22X_UART_HANDLER, HI22X_RxData, Const_HI22X_RX_BUFF_LEN);
    HAL_Delay(INS_TASK_INIT_TIME);
    HI22X_InitAngelOffset();
    return 1;
}

/**
 * @brief      Initialization offset and set mode
 * @param      NULL
 * @retval     NULL
 */
void HI22X_InitAngelOffset() {
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();

    // float y = 0;
    // for (int i = 0; i < 100; i++)
    //     y += hi22x->angle.pitch;
    hi22x->yaw_angle_offset = hi22x->angle.yaw;
    // hi22x->pitch_angle_offset = y / 100;
}

/**
 * @brief      Judge hi22x offline
 * @param      NULL
 * @retval     Offline or not��1 is offline��0 is not��
 */
uint8_t HI22X_IsHI22XOffline() {
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();

    uint32_t now = HAL_GetTick();
    if ((now - hi22x->last_update_time) > Const_HI22X_HI22X_OFFLINE_TIME)
        hi22x->state = HI22X_STATE_LOST;
    return hi22x->state == HI22X_STATE_LOST;
}

/**
 * @brief      Reset hi22x data object
 * @param      NUULL
 * @retval     NUL
 */
void HI22X_ResetHI22XData() {
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();

    hi22x->count.pitch = 0;
    hi22x->count.yaw = 0;
    hi22x->angle.pitch = 0;
    hi22x->angle.yaw = 0;
    hi22x->now_angle.pitch = 0;
    hi22x->now_angle.yaw = 0;
    hi22x->yaw_angle_offset = 0;
    hi22x->pitch_angle_offset = 0;
}

/**
 * @brief      hi22x decode data function    ��For HI229)
 * @param      referee: Pinter to the hi22x data object
 * @param      buff: Data buffer
 * @param      rxdatalen: Data length
 * @retval     NULL
 */
void HI22X_HI22xDecodeHI22XData(uint8_t* buff, int rxdatalen) {
    HI22X_HI22XDataTypeDef* hi22x = HI22X_GetHI22XDataPtr();

    hi22x->state = HI22X_STATE_PENDING;
    hi22x->last_update_time = HAL_GetTick();

    int16_t temp[3];
    // CRC verify
    //    CRC_HI22XEnum = CRC_VerifyIMU_HI229(buff);
    //     if (CRC_HI22XEnum == NOT_MATCH) return;

    // decode hi22x speed
    int HI22X_bias = 6;
    temp[0] = (int16_t)((buff[HI22X_bias + 2] << 8) | buff[HI22X_bias + 1]);  // pitch
    temp[1] = (int16_t)((buff[HI22X_bias + 4] << 8) | buff[HI22X_bias + 3]);  // roll
    temp[2] = (int16_t)((buff[HI22X_bias + 6] << 8) | buff[HI22X_bias + 5]);  // yaw

    for (int i = 0; i < 3; i++) {
        speed[i] = (float)temp[i] * 0.1f;
    }
    hi22x->speed.pitch = speed[1];
    hi22x->speed.row = speed[0];
    hi22x->speed.yaw = speed[2];

    // decode hi22x angle
    hi22x->last_angle.pitch = hi22x->now_angle.pitch;
    hi22x->last_angle.yaw = hi22x->now_angle.yaw;
    int angle_bias = HI22X_bias + 7;
    temp[0] = (int16_t)((buff[angle_bias + 2] << 8) | buff[angle_bias + 1]);  // pitch
    temp[1] = (int16_t)((buff[angle_bias + 4] << 8) | buff[angle_bias + 3]);  // row
    temp[2] = (int16_t)((buff[angle_bias + 6] << 8) | buff[angle_bias + 5]);  // yaw

    angle[0] = (float)(temp[0] / 100.0f);
    angle[1] = (float)(temp[1] / 100.0f);
    angle[2] = (float)(temp[2] / 10.0f);

    hi22x->now_angle.pitch = angle[0];
    hi22x->now_angle.row = angle[1];
    hi22x->now_angle.yaw = angle[2];

    if (hi22x->now_angle.pitch - hi22x->last_angle.pitch < -181)
        hi22x->count.pitch++;
    if (hi22x->now_angle.pitch - hi22x->last_angle.pitch > 181)
        hi22x->count.pitch--;
    hi22x->angle.pitch = (float)hi22x->count.pitch * 360.0f + hi22x->now_angle.pitch;

    if (hi22x->now_angle.yaw - hi22x->last_angle.yaw < -181)
        hi22x->count.yaw++;
    if (hi22x->now_angle.yaw - hi22x->last_angle.yaw > 181)
        hi22x->count.yaw--;
    hi22x->angle.yaw = (float)hi22x->count.yaw * 360.0f + hi22x->now_angle.yaw;

    if (hi22x->now_angle.row - hi22x->last_angle.row < -181)
        hi22x->count.row++;
    if (hi22x->now_angle.row - hi22x->last_angle.row > 181)
        hi22x->count.row--;
    hi22x->angle.row = (float)hi22x->count.row * 360.0f + hi22x->now_angle.row;

    hi22x->angle.pitch = hi22x->angle.pitch + hi22x->pitch_angle_offset;
    hi22x->angle.yaw = hi22x->angle.yaw - hi22x->yaw_angle_offset;
    hi22x->angle.row = hi22x->angle.row;

    hi22x->last_update_time = HAL_GetTick();
    hi22x->state = HI22X_STATE_CONNECTED;
}

#endif

/**
 * @brief      hi22x UART callback function
 * @param      huart: Point to serial port handle
 * @retval     NULL
 */
void HI22X_RXCallback(UART_HandleTypeDef* huart) {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);
#if __FN_IF_ENABLE(__IMU_HI22X)
    /* handle uart data from DMA */
    int rxdatalen = Const_HI22X_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx);
    HI22X_HI22xDecodeHI22XData(HI22X_RxData, rxdatalen);
    Ins_DecodeIMUData();
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_HI22X_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
#endif
}
