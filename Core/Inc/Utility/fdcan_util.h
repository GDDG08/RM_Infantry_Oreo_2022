
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_UTIL_H
#define __FDCAN_UTIL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_UTIL_CAN)

#include"fdcan.h"

void FDCAN_ErrorHandler(void);
void FDCAN_InitTxHander(FDCAN_TxHeaderTypeDef* pheader, uint32_t id, uint32_t dlc,uint32_t baudrateswitch, uint32_t can_type);
void FDCAN_IntFilterAndStart(FDCAN_HandleTypeDef *phfdcan) ;
void FDCAN_SendMessage(FDCAN_HandleTypeDef *phfdcan,FDCAN_TxHeaderTypeDef* ptxhead,uint8_t* pdata);
void FDCan_RxMessageCallback(FDCAN_HandleTypeDef* phfdcan, FDCAN_RxHeaderTypeDef* rxheader, uint8_t rxdata[]);


#endif
#endif
