/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Utility\spi_util.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:53:41
 */


#ifndef SPI_UTIL_H
#define SPI_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_SPI)

#include "spi.h"
#include "dma.h"

void Spi_Init(SPI_HandleTypeDef* hspi);
void Spi_DMAInit(SPI_HandleTypeDef* hspi, uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
void Spi_DMAEnable(SPI_HandleTypeDef* hspi, uint32_t tx_buff, uint32_t rx_buff, uint16_t ndtr);
void Spi_ReceiveData(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t len);
void Spi_TransmitData(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t len);
void Spi_ReceiveDataDMA(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t len);
void Spi_TransmitDataDMA(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t len);
uint8_t Spi_SwapAbyteData(SPI_HandleTypeDef* hspi, uint8_t txdata);
void Spi_ReadMuliReg(SPI_HandleTypeDef* hspi, uint8_t* rx_data, uint8_t len);
void Spi_SwapData(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t len);
void Spi_SwapDataDMA(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t len);
void Spi_ErrorHandler(uint32_t ret);

#endif

#ifdef __cplusplus
}
#endif

#endif
