
#include "fdcan_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_CAN)

#include "motor_periph.h"
#include "buscomm_ctrl.h"

FDCAN_RxHeaderTypeDef FDCAN_RxHeader;
const uint16_t Const_FDCan_RX_BUFF_LEN = 200;
uint8_t FDCAN_RxData[Const_FDCan_RX_BUFF_LEN];

/**
 * @brief      FDCAN Error handle handling
 * @retval     NULL
 */
void FDCAN_ErrorHandler(void) {
    while (1) {
        return;
    }
}

/**
 * @brief      Initialize fdcan transmitter
 * @param      pheader: Pointer to the initialized header
 * @param      id: FDCAN Equipment number
 * @param      dlc: FDCAN Datalength
 * @param      baudrateswitch: Choose if use bandrateswitch function
 * @param      can_type: Choose use classis can or fdcan
 * @retval     NULL
 */
void FDCAN_InitTxHander(FDCAN_TxHeaderTypeDef* pheader, uint32_t id, uint32_t dlc, uint32_t baudrateswitch, uint32_t can_type) {
    pheader->Identifier = id;
    if (id >= 0x800) {
        pheader->IdType = FDCAN_EXTENDED_ID;
    } else {
        pheader->IdType = FDCAN_STANDARD_ID;
    }
    pheader->TxFrameType = FDCAN_DATA_FRAME;
    pheader->DataLength = dlc;
    pheader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pheader->BitRateSwitch = baudrateswitch;
    pheader->FDFormat = can_type;
    pheader->TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pheader->MessageMarker = 0;
}

/**
 * @brief      Initialize fdcan filter and enable FDCAN Bus Transceiver
 * @param      phfdcan: Pointer to the CAN header
 * @retval     NULL
 */
void FDCAN_IntFilterAndStart(FDCAN_HandleTypeDef* phfdcan) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0400;
    // sFilterConfig.FilterID1 = 0x0000;
    sFilterConfig.FilterID2 = 0x0000;

    if (HAL_FDCAN_ConfigFilter(phfdcan, &sFilterConfig) != HAL_OK) {
        FDCAN_ErrorHandler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(phfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        // if (HAL_FDCAN_ConfigGlobalFilter(phfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
        FDCAN_ErrorHandler();
    }
    if (HAL_FDCAN_Start(phfdcan) != HAL_OK) {
        FDCAN_ErrorHandler();
    }

    if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        FDCAN_ErrorHandler();
    }
    if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
        FDCAN_ErrorHandler();
    }
}

/**
 * @brief      Sending information to can bus
 * @param      phfdcan: Pointer to the CAN header
 * @param      pheader: Pointer to the CAN tx header
 * @param      txdata: Message to send
 * @retval     NULL
 */
FDCAN_TxHeaderTypeDef ptxhead;

void FDCAN_SendMessage(FDCAN_HandleTypeDef* phfdcan, FDCAN_TxHeaderTypeDef* ptxhead, uint8_t* pdata) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(phfdcan, ptxhead, pdata) != HAL_OK) {
        FDCAN_ErrorHandler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* phfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        HAL_FDCAN_GetRxMessage(phfdcan, FDCAN_RX_FIFO0, &FDCAN_RxHeader, FDCAN_RxData);
        FDCan_RxMessageCallback(phfdcan, &FDCAN_RxHeader, FDCAN_RxData);

        if (HAL_FDCAN_ActivateNotification(phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
            FDCAN_ErrorHandler();
        }
    }
}

/**
 * @brief      Can bus data receiving callback function that updates the motor status according to the received information
 * @param      phfdcan: Pointer to the CAN header
 * @param      rxheader: Pointer to the CAN receive header
 * @param      rxdata: The message CAN receive
 * @retval     NULL
 */
void FDCan_RxMessageCallback(FDCAN_HandleTypeDef* phfdcan, FDCAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]) {
    BusComm_CANRxCallback(phfdcan, rxheader->Identifier, rxdata, (rxheader->DataLength) >> 16);

#if __FN_IF_ENABLE(__FN_PERIPH_MOTOR)
    Motor_EncoderDecodeCallback(phfdcan, rxheader->Identifier, rxdata, (rxheader->DataLength) >> 16);  //
#endif
}

#endif
