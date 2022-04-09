/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Utility\uart_util.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:22
 */

#ifndef UART_UTIL_H
#define UART_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_UART)

#include "usart.h"

#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))

void Uart_RxIdleCallback(UART_HandleTypeDef* huart);
void Uart_SendMessage(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout);
void Uart_SendMessage_IT(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size);
void Uart_SendMessage_IT_Force(UART_HandleTypeDef* huart, uint8_t txdata[], uint16_t size, uint32_t timeout);
void Uart_ErrorHandler(uint32_t ret);
uint16_t Uart_DMACurrentDataCounter(DMA_HandleTypeDef* dma_handle);
void Uart_InitUartDMA(UART_HandleTypeDef* huart);
HAL_StatusTypeDef Uart_ReceiveDMA(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);
void Uart_ReceiveHandler(UART_HandleTypeDef* huart);

#endif

#ifdef __cplusplus
}
#endif

#endif
