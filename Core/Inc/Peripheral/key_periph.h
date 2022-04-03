/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \infantry_-neptune\Core\Inc\Peripheral\key_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:51:54
 */

#ifndef KEY_PERIPH_H
#define KEY_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#include "gpio_util.h"

#define LONG_PRESS_EVENT 0x02
#define SHORT_PRESS_EVENT 0x01

#define INFANTRY_3 3
#define INFANTRY_4 4
#define INFANTRY_5 5
#define INFANTRY_6 6
#define INFANTRY_7 7
#define INFANTRY_8 8

uint8_t Key_GetEquipCode(void);
void Key_KeyEventHandler(GPIO_GPIOTypeDef* gpio);

#ifdef __cplusplus
}
#endif

#endif
