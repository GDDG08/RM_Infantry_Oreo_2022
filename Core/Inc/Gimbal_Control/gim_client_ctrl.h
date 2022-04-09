/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Gimbal_Control\gim_client_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-06 21:20:22
 */

#ifndef GIM_CLIENT_CTRL_H
#define GIM_CLIENT_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

#include "list_lib.h"
#include "str_lib.h"
#include "math_alg.h"

typedef struct {
    char name[3][15];
    float* value[3];

    char page_name[15];
    void (*short_press_handler)(void);
    void (*lang_press_handler)(void);
} Client_PageTypeDef;

typedef enum {
    CLIENT_NULL = 0,
    CLIENT_START = 1,
    CLIENT_ON = 2,
    CLIENT_ERROR_1 = 3,
    CLIENT_ERROR_2 = 4,
    CLIENT_ERROR_3 = 5,
    CLIENT_PAGE = 6,
    CLIENT_CODE = 7
} Client_InterfaceEnum;

typedef void (*Client_KeyEventHandlerTypeDef)(void);

void Client_Task(void const* argument);
void Client_Init(void);
void Client_PageInit(void);
void Client_CreateNewPage(Client_PageTypeDef* page, const char* val1_name, void* val_1, const char* val2_name, void* val_2, const char* val3_name, void* val_3, const char* page_name, Client_KeyEventHandlerTypeDef key_short, Client_KeyEventHandlerTypeDef key_long);
static void Client_DisplayCurrentPage(void);
void Client_KeytHandler(uint32_t message);
void Client_BackKeyShortHandler(void);
void Client_BackKeyLongHandler(void);
static void Client_DisplayInterface(void);
void Client_ChangeInterface(Client_InterfaceEnum intface);

static void shoot_state_func_short_press(void);
static void shoot_state_func_long_press(void);
static void periph_state_func_short_press(void);
static void periph_state_func_long_press(void);
static void cap_state_func_short_press(void);
static void cap_state_func_long_press(void);

#ifdef __cplusplus
}
#endif

#endif

#endif
