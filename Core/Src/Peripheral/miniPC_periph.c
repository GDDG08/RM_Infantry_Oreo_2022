/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Peripheral\miniPC_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-17 22:42:08
 */

#include "minipc_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)

#include "const.h"
#include "buscomm_ctrl.h"
#include "gim_minipc_ctrl.h"
#include "gim_ins_ctrl.h"
#include "buff_lib.h"

// UART_HandleTypeDef* Const_MiniPC_UART_HANDLER = &huart5;

/*              Mini_PC control constant            */
/*const */ uint32_t Const_MiniPC_HEART_SENT_PERIOD = 100;  // (ms)
/*const*/ uint32_t Const_MiniPC_DATA_SENT_PERIOD = 10;     // (ms)

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
/*const*/ uint32_t Const_MiniPC_CAPT_PRE = 10;  // (ms)
/*const*/ uint32_t Const_MiniPC_CAPT_DUR = 2;   // (ms)
/*const*/ uint32_t Const_MiniPC_CAPT_AFT = 7;   // (ms)
#endif

const uint16_t Const_MiniPC_RX_BUFF_LEN = 200;           // miniPC Receive buffer length
const uint16_t Const_MiniPC_TX_BUFF_LEN = 200;           // miniPC Transmit buffer length
const uint16_t Const_MiniPC_MINIPC_OFFLINE_TIME = 1000;  // miniPC offline time
const uint16_t Const_MiniPC_TX_HEART_FRAME_LEN = 10;     // miniPC heart transmit frame length
const uint16_t Const_MiniPC_TX_DATA_FRAME_LEN = 24;      // miniPC data transmit frame length

const uint8_t Const_MiniPC_SLAVE_COMPUTER = 0x00;
const uint8_t Const_MiniPC_INFANTRY_3 = 0x03;
const uint8_t Const_MiniPC_INFANTRY_4 = 0x04;
const uint8_t Const_MiniPC_INFANTRY_5 = 0x05;

const uint8_t Const_MiniPC_COLOR_RED = 0x00;
const uint8_t Const_MiniPC_COLOR_BLUE = 0x01;

const uint8_t Const_MiniPC_ARMOR = 0x00;
const uint8_t Const_MiniPC_BIG_BUFF = 0x01;
const uint8_t Const_MiniPC_LITTLE_BUFF = 0x02;

const uint8_t Const_MiniPC_PACKET_HEADR_0 = 0xa5;
const uint8_t Const_MiniPC_PACKET_HEADR_1 = 0X5a;

const uint8_t Const_MiniPC_Heart_PACKET = 0x00;
const uint8_t Const_MiniPC_ARMOR_PACKET = 0x02;
const uint8_t Const_MiniPC_DATA_PACKET = 0x08;

const uint8_t Const_MiniPC_Heart_PACKET_DATA_LEN = 2;
const uint8_t Const_MiniPC_Data_PACKET_DATA_LEN = 16;

MiniPC_MiniPCDataTypeDef MiniPC_MiniPCData;  // miniPC data

uint8_t MiniPC_RxData[Const_MiniPC_RX_BUFF_LEN];        // miniPC receive buff
uint8_t MiniPC_TxData[Const_MiniPC_TX_BUFF_LEN];        // miniPC transmit buff
uint8_t MiniPC_TxData_state[Const_MiniPC_TX_BUFF_LEN];  // miniPC transmit buff

/**
 * @brief      Initialize minipc
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_InitMiniPC() {
    MiniPC_ResetMiniPCData();
    // Uart_InitUartDMA(Const_MiniPC_UART_HANDLER);
    // Uart_ReceiveDMA(Const_MiniPC_UART_HANDLER, MiniPC_RxData, Const_MiniPC_RX_BUFF_LEN);
}

/**
 * @brief      Get pointer to minipc data object
 * @param      NULL
 * @retval     MiniPC Pointer to data object
 */
MiniPC_MiniPCDataTypeDef* MiniPC_GetMiniPCDataPtr() {
    return &MiniPC_MiniPCData;
}

/**
 * @brief      Sent MiniPC heart pack
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_SendHeartPacket() {
    static uint32_t heart_count = 0;
    if ((HAL_GetTick() - heart_count) <= Const_MiniPC_HEART_SENT_PERIOD)
        return;

    MiniPC_Update();

    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    minipc->state = MiniPC_PENDING;

    uint8_t* buff = MiniPC_TxData;
    int size = Const_MiniPC_TX_HEART_FRAME_LEN;
    buff[0] = Const_MiniPC_PACKET_HEADR_0;
    buff[1] = Const_MiniPC_PACKET_HEADR_1;
    buff[2] = Const_MiniPC_SLAVE_COMPUTER;
    buff[3] = minipc->addressee;
    buff[4] = Const_MiniPC_Heart_PACKET;
    buff[5] = Const_MiniPC_Heart_PACKET_DATA_LEN;
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = minipc->team_color;
    buff[9] = minipc->mode;

    uint16_t checksum = 0;
    if (size % 2) {
        buff[size] = 0x00;
        size++;
    }
    for (int i = 0; i < size; i += 2) {
        checksum += buff2ui16(buff + i);
    }
    ui162buff(checksum, buff + 6);

    heart_count = HAL_GetTick();

    //    if (HAL_UART_GetState(Const_MiniPC_UART_HANDLER) & 0x01)
    //        return;  // tx busy
    //    Uart_SendMessage_IT(Const_MiniPC_UART_HANDLER, buff, size);
}

/**
 * @brief      Sent MiniPC data request pack
 * @param      NULL
 * @retval     NULL
 */
float sin_gen;
void MiniPC_SendDataPacket() {
    static uint32_t data_count = 0;

#if !__FN_IF_ENABLE(__FN_MINIPC_CAPT)
    if ((HAL_GetTick() - data_count) <= Const_MiniPC_DATA_SENT_PERIOD)
        return;
#endif

    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
    osDelay(Const_MiniPC_CAPT_PRE);
    GPIO_Set(PC_CAM);

#endif

    //		COMM DEBUG
    uint32_t time = HAL_GetTick();
    int16_t pitch = 50 * sin(time / 50.0f);// + 50 * sin(time / 100.0f);
    sin_gen = ((float)pitch) / 100.0f;
    int16_t yaw = imu->angle.yaw * 100;
    // int16_t pitch = imu->angle.pitch * 100;
    int16_t row = imu->angle.row * 100;

    int16_t yaw_speed = imu->speed.yaw * 100;
    int16_t pitch_speed = imu->speed.pitch * 100;
    uint16_t shooter_speed = buscomm->speed_17mm_limit * 100;

    minipc->state = MiniPC_PENDING;

    uint8_t* buff = MiniPC_TxData_state;
    int size = Const_MiniPC_TX_DATA_FRAME_LEN;
    buff[0] = Const_MiniPC_PACKET_HEADR_0;
    buff[1] = Const_MiniPC_PACKET_HEADR_1;
    buff[2] = Const_MiniPC_SLAVE_COMPUTER;
    buff[3] = minipc->addressee;
    buff[4] = Const_MiniPC_DATA_PACKET;
    buff[5] = Const_MiniPC_Data_PACKET_DATA_LEN;
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = 0x01;  // game_status_tower
    i162buff(yaw, buff + 9);
    i162buff(pitch, buff + 11);
    i162buff(row, buff + 13);
    i162buff(yaw_speed, buff + 15);
    i162buff(shooter_speed, buff + 17);
    i162buff(pitch_speed, buff + 19);
    buff[21] = minipc->team_color;
    buff[22] = minipc->mode;

    // Must be even
    buff[23] = 0x00;

    uint16_t checksum = 0;
    if (size % 2) {
        buff[size] = 0x00;
        size++;
    }
    for (int i = 0; i < size; i += 2) {
        checksum += buff2ui16(buff + i);
    }
    ui162buff(checksum, buff + 6);

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
    osDelay(Const_MiniPC_CAPT_DUR);
    data_count = HAL_GetTick();

#endif
    /*
    if (HAL_UART_GetState(Const_MiniPC_UART_HANDLER) & 0x01)
        return;  // tx busy
    Uart_SendMessage_IT(Const_MiniPC_UART_HANDLER, buff, Const_MiniPC_TX_DATA_FRAME_LEN);
    */
    CDC_Transmit_FS(buff, Const_MiniPC_TX_DATA_FRAME_LEN);

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
    GPIO_Reset(PC_CAM);
    osDelay(Const_MiniPC_CAPT_AFT);
#endif
}

/**
 * @brief      Determine whether minipc is offline
 * @param      NULL
 * @retval     Offline or not (1 is yes, 0 is no)
 */
uint8_t MiniPC_IsMiniPCOffline() {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    uint32_t now = HAL_GetTick();
    return (now - minipc->last_update_time) > Const_MiniPC_MINIPC_OFFLINE_TIME;
}

// /**
//  * @brief      Minipc serial port callback function
//  * @param      huart: Pointer to serial port handle
//  * @retval     NULL
//  */
// void MiniPC_RXCallback(UART_HandleTypeDef* huart) {
//     /* clear DMA transfer complete flag */
//     __HAL_DMA_DISABLE(huart->hdmarx);

//     /* handle dbus data dbus_buf from DMA */
//     uint16_t rxdatalen = Const_MiniPC_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx);

//     MiniPC_DecodeMiniPCPacket(MiniPC_RxData, rxdatalen);
//     MiniPC_UpdateControlData();

//     /* restart dma transmission */
//     __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_MiniPC_RX_BUFF_LEN);
//     __HAL_DMA_ENABLE(huart->hdmarx);
// }

/**
 * @brief      Minipc data packet decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     NULL
 */
void MiniPC_DecodeMiniPCPacket(uint8_t* buff, uint16_t rxdatalen) {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();
    minipc->last_update_time = HAL_GetTick();

    if (!MiniPC_VerifyMiniPCData(buff, rxdatalen)) {
        minipc->state = MiniPC_ERROR;
        return;
    }

    switch (buff[4]) {
        case Const_MiniPC_Heart_PACKET:
            MiniPC_HeartPacketDecode(buff, rxdatalen);
            break;
        case Const_MiniPC_ARMOR_PACKET:
            MiniPC_ArmorPacketDecode(buff, rxdatalen);
            break;
        default:
            break;
    }
}

/**
 * @brief      Minipc data heart packet decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     NULL
 */
void MiniPC_HeartPacketDecode(uint8_t* buff, uint16_t rxdatalen) {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    if (minipc->heart_flag <= 3) {
        minipc->heart_flag = 10;
    }

    minipc->state = MiniPC_CONNECTED;
}

/**
 * @brief      Minipc data armor packet decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     NULL
 */
void MiniPC_ArmorPacketDecode(uint8_t* buff, uint16_t rxdatalen) {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    minipc->new_data_flag = 1;

    minipc->is_get_target = buff[8];

    minipc->yaw_angle = (float)buff2i16(buff + 9) / 100.0f;
    minipc->pitch_angle = (float)buff2i16(buff + 11) / 100.0f;
    minipc->distance = (float)buff2i16(buff + 13);

    minipc->state = MiniPC_CONNECTED;
}

/**
 * @brief      Initialize minipc data object
 * @param      minipc: Pointer to minipc data object
 * @retval     NULL
 */
void MiniPC_ResetMiniPCData() {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    minipc->heart_flag = 0;
    minipc->distance = 0;
    minipc->is_get_target = 0;
    minipc->mode = Const_MiniPC_ARMOR;
    minipc->pitch_angle = 0;
    minipc->yaw_angle = 0;
    minipc->state = MiniPC_NULL;
    minipc->team_color = 0;
    minipc->new_data_flag = 0;
    minipc->last_update_time = HAL_GetTick();
}

/**
 * @brief      MiniPC update sent data
 * @param      NULL
 * @retval     NULL
 */
void MiniPC_Update() {
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    switch (buscomm->robot_id) {
        case 3:
            minipc->team_color = Const_MiniPC_COLOR_RED;
            minipc->addressee = Const_MiniPC_INFANTRY_3;
            break;
        case 4:
            minipc->team_color = Const_MiniPC_COLOR_RED;
            minipc->addressee = Const_MiniPC_INFANTRY_4;
            break;
        case 5:
            minipc->team_color = Const_MiniPC_COLOR_RED;
            minipc->addressee = Const_MiniPC_INFANTRY_5;
            break;
        case 103:
            minipc->team_color = Const_MiniPC_COLOR_BLUE;
            minipc->addressee = Const_MiniPC_INFANTRY_3;
            break;
        case 104:
            minipc->team_color = Const_MiniPC_COLOR_BLUE;
            minipc->addressee = Const_MiniPC_INFANTRY_4;
            break;
        case 105:
            minipc->team_color = Const_MiniPC_COLOR_BLUE;
            minipc->addressee = Const_MiniPC_INFANTRY_5;
            break;
        default:
            break;
    }
}

/**
 * @brief      Minipc data decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     Match is 1  not match is 0
 */
uint16_t checksum__ = 0, sum__ = 0;

uint8_t MiniPC_VerifyMiniPCData(uint8_t* buff, uint16_t rxdatalen) {
    const uint8_t FAILED = 0, SUCCEEDED = 1;

    if (rxdatalen <= 8)
        return FAILED;
    if (buff[0] != Const_MiniPC_PACKET_HEADR_0 || buff[1] != Const_MiniPC_PACKET_HEADR_1)
        return FAILED;

    uint16_t checksum = 0, sum = 0;
    int size = rxdatalen;

    //    return SUCCEEDED;

    sum = buff2ui16(buff + 6);
    // buff[6] = 0;
    // buff[7] = 0;

    if (size % 2) {
        buff[size] = 0x00;
        size++;
    }
    for (int i = 0; i < size; i += 2) {
        if (i == 6)
            continue;
        checksum += buff2ui16(buff + i);
    }

    checksum__ = checksum;
    sum__ = sum;

    if (checksum == sum) {
        return SUCCEEDED;
    } else
        return FAILED;
}

#endif
