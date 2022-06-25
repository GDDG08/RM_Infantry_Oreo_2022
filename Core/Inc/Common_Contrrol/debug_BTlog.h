/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Common_Contrrol\debug_BTlog.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-10-31 09:17:07
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-06-24 15:38:38
 */

#ifndef DEBUG_BTLOG_H
#define DEBUG_BTLOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_DEBUG_BTLOG)

#include "uart_util.h"
#include "buff_lib.h"
#include "const.h"
#include "gpio_util.h"
#include "buscomm_ctrl.h"

// const uint16_t
//     LEN_ui8 = 1,
//     LEN_ui16 = 2,
//     LEN_ui8
typedef enum {
    BYTE = 0u,
    uInt8 = 1u,
    uInt16 = 2u,
    uInt32 = 3u,
    Float = 4u,
    Char = 5u,
    Int8 = 6u,
    Int16 = 7u,
    Int32 = 8u
} BTlog_TypeEnum;

#define BTlog_tagSize 19

typedef struct {
    void* ptr;
    uint8_t size;
    uint8_t type;
    char tag[BTlog_tagSize];
    // void (*log_func)(uint8_t buff[]);
} BTlog_TableEntry;

extern UART_HandleTypeDef* Const_BTlog_UART_HANDLER;

void BTlog_Init(void);
void BTlog_Send(void);
void BTlog_CTRL(void);
void BTlog_RXCallback(UART_HandleTypeDef*);
uint8_t BTLog_VerifyData(uint8_t*, uint16_t);
#endif

#ifdef __cplusplus
}
#endif

#endif
