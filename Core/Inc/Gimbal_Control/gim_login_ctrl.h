/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Gimbal_Control\gim_login_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:50:57
 */

#ifndef GIM_LOGIN_CTRL_H
#define GIM_LOGIN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

#include "crc_alg.h"
#include "main.h"
#include "stdlib.h"

#define CODE_KEY_W 0X00
#define CODE_KEY_A 0X01
#define CODE_KEY_S 0X02
#define CODE_KEY_D 0X03
#define CODE_KEY_Q 0X04
#define CODE_KEY_E 0X05
#define CODE_KEY_R 0X06
#define CODE_KEY_F 0X07
#define CODE_KEY_G 0X08
#define CODE_KEY_Z 0X09
#define CODE_KEY_X 0X0A
#define CODE_KEY_C 0X0B
#define CODE_KEY_V 0X0C
#define CODE_KEY_B 0X0D
#define CODE_SHIFT 0XFF

#define CODE_MATCH 1
#define CODE_NO_MATCH 0

#define LOGIN_ON 1
#define LOGIN_OFF 0

extern uint8_t LOGIN_ON_FLAG;

void Login_Init(void);
uint32_t Login_CreateCode(void);
uint16_t Login_GetPassword(uint32_t code);
uint32_t Login_CheckCode(uint16_t password);
void Login_LoginOn(void);
void Login_LoginCmd(void);
uint32_t Login_GetCode(void);

#ifdef __cplusplus
}
#endif

#endif

#endif
