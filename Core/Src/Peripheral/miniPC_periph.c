/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Peripheral\miniPC_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-06-29 19:26:44
 */

#include "minipc_periph.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)

#include "const.h"
#include "buscomm_ctrl.h"
#include "motor_periph.h"
#include "gim_minipc_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_ins_ctrl.h"
#include "buff_lib.h"

// UART_HandleTypeDef* Const_MiniPC_UART_HANDLER = &huart5;

/*              Mini_PC control constant            */
// /*const */ uint32_t Const_MiniPC_HEART_SENT_PERIOD = 100;  // (ms)
/*const*/ uint32_t Const_MiniPC_DATA_SENT_PERIOD = 10;  // (ms)

const uint16_t Const_MiniPC_MINIPC_OFFLINE_TIME = 100;  // miniPC offline time

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
/*const*/ uint32_t Const_MiniPC_CAPT_PRE = 3;  // (ms)
/*const*/ uint32_t Const_MiniPC_CAPT_DUR = 3;  // (ms)
/*const*/ uint32_t Const_MiniPC_CAPT_AFT = 0;  // (ms)
#endif

const uint16_t Const_MiniPC_RX_BUFF_LEN = 200;       // miniPC Receive buffer length
const uint16_t Const_MiniPC_TX_BUFF_LEN = 200;       // miniPC Transmit buffer length
const uint16_t Const_MiniPC_TX_DATA_FRAME_LEN = 32;  // miniPC data transmit frame length

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

const uint8_t Const_MiniPC_ARMOR_PACKET = 0x02;
const uint8_t Const_MiniPC_DATA_PACKET = 0x08;
const uint8_t Const_MiniPC_BUFF_PACKET = 0x09;

const uint8_t Const_MiniPC_Heart_PACKET_DATA_LEN = 2;
const uint8_t Const_MiniPC_Data_PACKET_DATA_LEN = 24;

MiniPC_MiniPCDataTypeDef MiniPC_MiniPCData;  // miniPC data

uint8_t MiniPC_RxData[Const_MiniPC_RX_BUFF_LEN];  // miniPC receive buff
// uint8_t MiniPC_TxData[Const_MiniPC_TX_BUFF_LEN];        // miniPC transmit buff
uint8_t MiniPC_TxData_state[Const_MiniPC_TX_BUFF_LEN];  // miniPC transmit buff

uint32_t MiniPC_Data_FrameTime;

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

float MiniPC_angles[5], MiniPC_angles_offset_imu[2], MiniPC_angles_offset_motor[2];  // yaw,pitch,roll,yawspd,pitchspd
float MiniPC_last_mode;

void MiniPC_GimbalAngleCalibrate() {
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    float angles_now_imu[5] = {imu->angle.yaw,
                               imu->angle.pitch,
                               imu->angle.row,
                               imu->speed.yaw,
                               imu->speed.pitch};

    if (minipc->aim_mode == MiniPC_SMALL_BUFF || minipc->aim_mode == MiniPC_BIG_BUFF) {
        float angles_now_motor[2] = {buscomm->yaw_relative_angle,
                                     Motor_gimbalMotorPitch.encoder.angle};

        MiniPC_angles[0] = buscomm->yaw_relative_angle;
        // MiniPC_angles[0] = angles_now_motor[0] - MiniPC_angles_offset_motor[0] + MiniPC_angles_offset_imu[0];
        // MiniPC_angles[1] = angles_now_motor[1] - MiniPC_angles_offset_motor[1] + MiniPC_angles_offset_imu[1];

        // memcpy(MiniPC_angles + 2, angles_now_imu + 2, sizeof(float) * 3);
        memcpy(MiniPC_angles + 1, angles_now_imu + 1, sizeof(float) * 4);

        if (MiniPC_last_mode != MiniPC_SMALL_BUFF && MiniPC_last_mode != MiniPC_BIG_BUFF) {
            // memcpy(MiniPC_angles_offset_imu, angles_now_imu, sizeof(float) * 2);
            // memcpy(MiniPC_angles_offset_motor, angles_now_motor, sizeof(float) * 2);
            memcpy(MiniPC_angles, angles_now_imu, sizeof(MiniPC_angles));
        }
    } else {
        memcpy(MiniPC_angles, angles_now_imu, sizeof(MiniPC_angles));
    }

    MiniPC_last_mode = minipc->aim_mode;
}

/**
 * @brief      Sent MiniPC data request pack
 * @param      NULL
 * @retval     NULL
 */
float sin_gen2;
float sin_gen;
void MiniPC_SendDataPacket() {
    static uint32_t data_count = 0;

#if !__FN_IF_ENABLE(__FN_MINIPC_CAPT)
    if ((HAL_GetTick() - data_count) <= Const_MiniPC_DATA_SENT_PERIOD)
        return;
#endif

    MiniPC_MiniPCDataTypeDef* minipc_data = MiniPC_GetMiniPCDataPtr();
    MiniPC_MiniPCControlTypeDef* minipc = MiniPC_GetMiniPCControlDataPtr();
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
    osDelay(Const_MiniPC_CAPT_PRE);
    GPIO_Set(PC_CAM);
    osDelay(Const_MiniPC_CAPT_DUR);
    GPIO_Reset(PC_CAM);
#endif

    //		COMM DEBUG
    // uint32_t time = HAL_GetTick();
    // int16_t pitch = 50 * sin(time / 50.0f);  // + 50 * sin(time / 100.0f);
    // sin_gen = ((float)pitch) / 100.0f;
    // float yaw = 50.0f * sin(time / 50.0f);  // + 50 * sin(time / 100.0f);
    // sin_gen = yaw;

    MiniPC_GimbalAngleCalibrate();
    float yaw = MiniPC_angles[0];
    int16_t pitch = MiniPC_angles[1] * 100;
    float row = MiniPC_angles[2];
    while (row > 180)
        row -= 360;
    while (row < -180)
        row += 360;
    int16_t roll = row * 100;
    int16_t yaw_speed = MiniPC_angles[3] * 100;
    int16_t pitch_speed = MiniPC_angles[4] * 100;

    sin_gen2 = Shooter_GetRefereeSpeedFdb();
    uint16_t bullet_speed = Shooter_GetRefereeSpeedFdb() * 100;
    MiniPC_Data_FrameTime = HAL_GetTick() % 60000;

    minipc_data->state = MiniPC_PENDING;

    uint8_t* buff = MiniPC_TxData_state;
    int size = Const_MiniPC_TX_DATA_FRAME_LEN;
    buff[0] = Const_MiniPC_PACKET_HEADR_0;
    buff[1] = Const_MiniPC_PACKET_HEADR_1;
    buff[2] = Const_MiniPC_SLAVE_COMPUTER;
    buff[3] = minipc_data->addressee;
    buff[4] = Const_MiniPC_DATA_PACKET;
    buff[5] = Const_MiniPC_Data_PACKET_DATA_LEN;
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = buscomm->game_outpost_alive;  // game_status_tower
    float2buff(yaw, buff + 9);
    i162buff(pitch, buff + 13);
    i162buff(roll, buff + 15);
    i162buff(yaw_speed, buff + 17);
    ui162buff(bullet_speed, buff + 19);
    i162buff(pitch_speed, buff + 21);
    buff[23] = minipc_data->team_color;
    buff[24] = minipc_data->mode;
    ui322buff(MiniPC_Data_FrameTime, buff + 25);
    buff[29] = 0;  // is change target
    buff[30] = minipc->vision_offset[minipc->aim_mode].horizental;
    buff[31] = minipc->vision_offset[minipc->aim_mode].vertical;
    // Must be even
    // buff[32] = 0x00;

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

    osDelay(Const_MiniPC_CAPT_AFT);
#endif
    data_count = HAL_GetTick();
    /*
    if (HAL_UART_GetState(Const_MiniPC_UART_HANDLER) & 0x01)
        return;  // tx busy
    Uart_SendMessage_IT(Const_MiniPC_UART_HANDLER, buff, Const_MiniPC_TX_DATA_FRAME_LEN);
    */
    CDC_Transmit_FS(buff, Const_MiniPC_TX_DATA_FRAME_LEN);
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
        case Const_MiniPC_ARMOR_PACKET:
            MiniPC_ArmorPacketDecode(buff, rxdatalen);
            break;
        case Const_MiniPC_BUFF_PACKET:
            MiniPC_BuffPacketDecode(buff, rxdatalen);
            break;
        default:
            break;
    }
    // BTlog_Send();
}

/**
 * @brief      Minipc data armor packet decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     NULL
 */
void MiniPC_ArmorPacketDecode(void* buff, uint16_t rxdatalen) {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    MiniPC_ArmorPacket_t* data_ptr = buff + 8;

    minipc->new_data_flag = 1;

    minipc->is_get_target = data_ptr->is_get;
    minipc->timestamp = data_ptr->timestamp;
    minipc->x = data_ptr->x;
    minipc->y = data_ptr->y;
    minipc->z = data_ptr->z;
    minipc->vx = data_ptr->vx;
    minipc->vz = data_ptr->vz;
    minipc->ID = data_ptr->ID;

    minipc->state = MiniPC_CONNECTED;
}

/**
 * @brief      Minipc data armor packet decoding function
 * @param      buff: Data buffer
 * @param      rxdatalen: recevie data length
 * @retval     NULL
 */
void MiniPC_BuffPacketDecode(uint8_t* buff, uint16_t rxdatalen) {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    minipc->new_data_flag = 1;

    minipc->is_get_target = buff[8];

    minipc->yaw_angle = buff2float(buff + 9);
    minipc->pitch_angle = (float)buff2i16(buff + 13) / 100.0f;
    minipc->distance = (float)buff2i16(buff + 15);

    minipc->state = MiniPC_CONNECTED;
}

/**
 * @brief      Initialize minipc data object
 * @param      minipc: Pointer to minipc data object
 * @retval     NULL
 */
void MiniPC_ResetMiniPCData() {
    MiniPC_MiniPCDataTypeDef* minipc = MiniPC_GetMiniPCDataPtr();

    // minipc->heart_flag = 0;
    minipc->team_color = 0;
    // minipc->mode = Const_MiniPC_ARMOR;
    minipc->is_get_target = 0;
    minipc->yaw_angle = 0;
    minipc->pitch_angle = 0;
    minipc->distance = 0;

    minipc->timestamp = 0;
    minipc->x = 0;
    minipc->y = 0;
    minipc->z = 0;
    minipc->vx = 0;
    minipc->vz = 0;
    minipc->ID = 0;
    minipc->addressee = 0;
    minipc->state = MiniPC_NULL;
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
