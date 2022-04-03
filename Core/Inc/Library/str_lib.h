/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Library\str_lib.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:51:37
 */


#ifndef STR_LIB_H
#define STR_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"

char* Str_Itoa(int value, char* string);
int Str_Atoi(const char* str);
void Str_HexToAscii(uint8_t* src, char* dest, int len);

#ifdef __cplusplus
}
#endif

#endif
