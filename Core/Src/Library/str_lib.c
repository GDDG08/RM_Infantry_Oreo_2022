/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Library\str_lib.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:55:15
 */

#include "str_lib.h"

/**
 * @brief      Converts a integer to an string (Decimal only)
 * @param      value ��To be coverted value
 * @param      string: Pointer to converted String
 * @retval     NULL
 */
char* Str_Itoa(int value, char* string) {
    int i, d;
    int flag = 0;
    char* ptr = string;

    if (!value) {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    if (value < 0) {
        *ptr++ = '-';
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10) {
        d = value / i;

        if (d || flag) {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }
    *ptr = 0;
    return string;
}

/**
 * @brief      Converts a string to an integer
 * @param      str: Pointer to converted String
 * @retval     NULL
 */
int Str_Atoi(const char* str) {
    int s = 0;
    uint8_t falg = 0;

    while (*str == ' ') {
        str++;
    }

    if ((*str == '-') || (*str == '+')) {
        if (*str == '-')
            falg = 1;
        str++;
    }

    while ((*str >= '0') && (*str <= '9')) {
        s = s * 10 + *str - '0';
        str++;
        if (s < 0) {
            s = 2147483647;
            break;
        }
    }
    return s * (falg ? -1 : 1);
}

/**
 * @brief      Hex transform to ascii string
 * @param      str: Pointer to converted String
 * @retval     NULL
 */
void Str_HexToAscii(uint8_t* src, char* dest, int len) {
    char dh, dl;
    int i;
    if (src == NULL || dest == NULL)
        return;
    if (len < 1)
        return;
    for (i = 0; i < len; i++) {
        dh = '0' + src[i] / 16;
        dl = '0' + src[i] % 16;
        if (dh > '9') {
            dh = dh - '9' - 1 + 'A';
        }
        if (dl > '9') {
            dl = dl - '9' - 1 + 'A';
        }
        dest[2 * i] = dh;
        dest[2 * i + 1] = dl;
    }
    dest[2 * i] = '\0';
}
