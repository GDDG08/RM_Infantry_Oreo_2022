/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Library\buff_lib.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:51:17
 */

#ifndef BUFF_LIB_H
#define BUFF_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx.h"
#include "string.h"

float buff2float(uint8_t* buff);
void float2buff(float f, uint8_t* buff);
int16_t buff2i16(uint8_t* buff);
uint16_t buff2ui16(uint8_t* buff);
void ui162buff(uint16_t u, uint8_t* buff);
void i162buff(int16_t u, uint8_t* buff);
uint32_t buff2ui32(uint8_t* buff);
void ui322buff(uint32_t u, uint8_t* buff);

#ifdef __cplusplus
}
#endif

#endif
