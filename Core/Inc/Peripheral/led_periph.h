/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\led_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:58:11
 */

#ifndef LED_PERIPH_H
#define LED_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_PERIPH_LED)

#include "stm32g4xx_hal.h"
#include "gpio.h"

typedef enum {
    LED_OFF = 0,
    LED_ON = 1
} LED_LEDStateEnum;

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    LED_LEDStateEnum state;
    GPIO_PinState off_pin_state, on_pin_state;
} LED_LEDTypeDef;

void LED_InitAllLEDs(void);
void LED_InitLED(LED_LEDTypeDef* led, GPIO_TypeDef* port, uint16_t pin, GPIO_PinState off_pin_state, GPIO_PinState on_pin_state, LED_LEDStateEnum init_state);
LED_LEDStateEnum LED_GetLEDState(LED_LEDTypeDef* led);
void LED_SetLEDState(LED_LEDTypeDef* led, LED_LEDStateEnum state);
void LED_AllOff(void);

#endif

#endif

#ifdef __cplusplus
}
#endif
