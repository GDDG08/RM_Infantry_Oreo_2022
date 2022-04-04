/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\uart_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:56
 */

#include "uart_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_UART)

#include "remote_periph.h"
#include "minipc_periph.h"
#include "hi22x_periph.h"
#include "referee_periph.h"
#include "supercap_ctrl.h"
#include "buscomm_ctrl.h"
#include "debug_BTlog.h"

/********** VOLATILE USER CODE **********/

/**
 * @brief      UART RX Callback allocation function
 * @param      huart: uart IRQHandler id
 * @retval     NULL
 */
void Uart_RxIdleCallback(UART_HandleTypeDef* huart) {
#if __FN_IF_ENABLE(__FN_PERIPH_REMOTE)
    if (huart == Const_Remote_UART_HANDLER) {
        Remote_RXCallback(huart);
    }
#endif

#if __FN_IF_ENABLE(__FN_PERIPH_REFEREE)
    if (huart == Const_Referee_UART_HANDLER) {
        Referee_RXCallback(huart);
    }
#endif

#if __FN_IF_ENABLE(__FN_PERIPH_IMU)
    if (huart == Const_HI22X_UART_HANDLER) {
        HI22X_RXCallback(huart);
    }
#endif

#if __FN_IF_ENABLE(__FN_PERIPH_MINIPC)
//    if (huart == Const_MiniPC_UART_HANDLER) {
//        MiniPC_RXCallback(huart);
//    }
#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP_COMM)
    if (huart == Const_SuperCap_UART_HANDLER) {
        CapComm_RXCallback(huart);
    }
#endif

#if __FN_IF_ENABLE(__FN_DEBUG_BTLOG)
    if (huart == Const_BTlog_UART_HANDLER) {
        BTlog_RXCallback(huart);
    }
#endif
}
/********** VOLATILE USER CODE END **********/

/**
 * @brief      Sending information to UART (blocking mode)
 * @param      huart: UART handle
 * @param      txdata: The message to send
 * @param      size: The message length
 * @param      timeout: Timeout duration
 * @retval     NULL
 */
void Uart_SendMessage(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout) {
    /* Start the Transmission process */
    uint32_t ret = HAL_UART_Transmit(huart, txdata, size, timeout);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}

/**
 * @brief      Sending information to UART (Non blocking mode)
 * @param      huart: UART handle
 * @param      txdata: The message to send
 * @param      size: The message length
 * @retval     NULL
 */
void Uart_SendMessage_IT(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size) {
    /* Start the Transmission process */
    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}

/**
 * @brief      Sending information to UART (Non blocking mode)��force waiting��may cause delay
 * @param      huart: UART handle
 * @param      txdata: The message to send
 * @param      size: The message length
 * @param      timeout: Timeout duration
 * @retval     NULL
 */
void Uart_SendMessage_IT_Force(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout) {
    //    /* Start the Transmission process */
    //    uint32_t now = HAL_GetTick();
    //    uint32_t ret;
    //    do {
    //        ret = HAL_UART_Transmit_IT(huart, txdata, size);
    //    } while (ret != HAL_OK && HAL_GetTick() - now <= timeout);
    //    if (ret != HAL_OK) {
    //        /* Transmission request Error */
    //        Uart_ErrorHandler(ret);
    //    }
    /* Start the Transmission process */
    __HAL_UNLOCK(huart);
    uint32_t ret = HAL_UART_Transmit_IT(huart, txdata, size);
    if (ret != HAL_OK) {
        /* Transmission request Error */
        Uart_ErrorHandler(ret);
    }
}

/**
 * @brief      UART error handler
 * @param      ret: error data
 * @retval     NULL
 */
void Uart_ErrorHandler(uint32_t ret) {
    // Log_DebugPrintf("Error: UART Error!\n");
    while (1) {
        return;
    }
}

/**
 * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
 * @param      dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
 *             to 7 to select the DMA Stream.
 * @retval     The number of remaining data units in the current DMAy Streamx transfer.
 */
uint16_t Uart_DMACurrentDataCounter(DMA_HandleTypeDef* dma_handle) {
    /* Return the number of remaining data units for DMAy Streamx */
    return __HAL_DMA_GET_COUNTER(dma_handle);
}

/**
 * @brief      initialization UART DMA
 * @param      huart: UART handle
 * @retval     NULL
 */
void Uart_InitUartDMA(UART_HandleTypeDef* huart) {
    /* open uart idle it */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
}

/**
 * @brief Receive an amount of data in DMA mode.
 * @note   When the UART parity is enabled (PCE = 1), the received data contain
 *         the parity bit (MSB position).
 * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
 *         the received data is handled as a set of u16. In this case, Size must indicate the number
 *         of u16 available through pData.
 * @param huart UART handle.
 * @param pData Pointer to data buffer (u8 or u16 data elements).
 * @param Size  Amount of data elements (u8 or u16) to be received.
 * @retval HAL status
 */
HAL_StatusTypeDef Uart_ReceiveDMA(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size) {
    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_STATE_READY) {
        if ((pData == NULL) || (Size == 0U)) {
            return HAL_ERROR;
        }

        __HAL_LOCK(huart);

        /* Set Reception type to Standard reception */
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;

        if (!(IS_LPUART_INSTANCE(huart->Instance))) {
            /* Check that USART RTOEN bit is set */
            if (READ_BIT(huart->Instance->CR2, USART_CR2_RTOEN) != 0U) {
                /* Enable the UART Receiver Timeout Interrupt */
                ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_RTOIE);
            }
        }

        return (UART_Start_Receive_DMA(huart, pData, Size));
    } else {
        return HAL_BUSY;
    }
}

/**
 * @brief      UART RX callback receiver function
 * @param      huart: Point to uart handle
 * @retval     NULL
 */
void Uart_ReceiveHandler(UART_HandleTypeDef* huart) {
    // clear idle it flag after uart receive a frame data
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
        /* clear idle it flag avoid idle interrupt all the time */
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        /* handle received data in idle interrupt */
        Uart_RxIdleCallback(huart);
    }
}

#endif
