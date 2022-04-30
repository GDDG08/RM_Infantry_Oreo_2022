/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\gpio_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-29 10:08:11
 */

#include "gpio_util.h"

#include "gim_ins_ctrl.h"
#include "key_periph.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

GPIO_GPIOTypeDef CAM_START = {GPIOB, GPIO_PIN_2, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BULLET_CHARGING_START = {GPIOB, GPIO_PIN_0, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CS_ACCEL_START = {GPIOC, GPIO_PIN_4, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CS_GYRO_START = {GPIOC, GPIO_PIN_5, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef IST8310_RST_START = {GPIOC, GPIO_PIN_14, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CODE1_START = {GPIOC, GPIO_PIN_8, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CODE2_START = {GPIOC, GPIO_PIN_9, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CODE3_START = {GPIOC, GPIO_PIN_15, 0xff, 0, GPIO_PIN_RESET};

GPIO_GPIOTypeDef IST8310_DRDY_START = {GPIOC, GPIO_PIN_13, 0xC1, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT1_START = {GPIOB, GPIO_PIN_2, 0xB1, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BMI_INT3_START = {GPIOB, GPIO_PIN_14, 0xB2, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef KEY_FUNC_START = {GPIOC, GPIO_PIN_3, KEY_FUNC_EVENT_ID, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef KEY_BACK_START = {GPIOA, GPIO_PIN_15, KEY_BACK_EVENR_ID, 0, GPIO_PIN_RESET};

GPIO_GPIOTypeDef* PC_CAM = &CAM_START;
GPIO_GPIOTypeDef* BULLET_CHARGING = &BULLET_CHARGING_START;
GPIO_GPIOTypeDef* CS_ACCEL = &CS_ACCEL_START;
GPIO_GPIOTypeDef* CS_GYRO = &CS_GYRO_START;
GPIO_GPIOTypeDef* IST8310_RST = &IST8310_RST_START;
GPIO_GPIOTypeDef* IST8310_DRDY = &IST8310_DRDY_START;
GPIO_GPIOTypeDef* BMI_INT1 = &BMI_INT1_START;
GPIO_GPIOTypeDef* BMI_INT3 = &BMI_INT3_START;
GPIO_GPIOTypeDef* KEY_FUNC = &KEY_FUNC_START;
GPIO_GPIOTypeDef* KEY_BACK = &KEY_BACK_START;
GPIO_GPIOTypeDef* CODE1 = &CODE1_START;
GPIO_GPIOTypeDef* CODE2 = &CODE2_START;
GPIO_GPIOTypeDef* CODE3 = &CODE3_START;

#endif

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

GPIO_GPIOTypeDef BOOST_START = {GPIOC, GPIO_PIN_4, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef BUCK_START = {GPIOB, GPIO_PIN_3, 0xff, 0, GPIO_PIN_RESET};
GPIO_GPIOTypeDef CAP_START = {GPIOC, GPIO_PIN_5, 0xff, 0, GPIO_PIN_RESET};

GPIO_GPIOTypeDef* BOOST = &BOOST_START;
GPIO_GPIOTypeDef* BUCK = &BUCK_START;
GPIO_GPIOTypeDef* CAP = &CAP_START;

#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

#endif

/**
 * @brief      Get the GPIO pin trigger tick
 * @param      gpio :Mark of peripheral
 * @retval     trigger tick
 */
uint32_t GPIO_GetTriggerTick(GPIO_GPIOTypeDef* gpio) {
    return gpio->tick;
}

/**
 * @brief      Set GPIO
 * @param      gpio :Mark of peripheral
 * @retval     NULL
 */
void GPIO_Set(GPIO_GPIOTypeDef* gpio) {
    gpio->tick = HAL_GetTick();
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_SET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}

/**
 * @brief      Reset GPIO
 * @param      gpio :Mark of peripheral
 * @retval     NULL
 */
void GPIO_Reset(GPIO_GPIOTypeDef* gpio) {
    gpio->tick = HAL_GetTick();
    HAL_GPIO_WritePin(gpio->gpio_handle, gpio->gpio_pin, GPIO_PIN_RESET);
    gpio->pin_state = GPIO_ReadPin(gpio);
}

/**
 * @brief      Reset GPIO
 * @param      gpio :Mark of peripheral
 * @retval     The state of goio
 */
GPIO_PinState GPIO_ReadPin(GPIO_GPIOTypeDef* gpio) {
    gpio->pin_state = HAL_GPIO_ReadPin(gpio->gpio_handle, gpio->gpio_pin);
    return gpio->pin_state;
}

/**
 * @brief      HAL GPIO interrupt call back
 * @param      GPIO_Pin :Specifies the pins connected EXTI line
 * @retval     NULL
 */
void GPIO_IRQCallback(uint16_t GPIO_Pin) {
    uint32_t trigger_time = HAL_GetTick();

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
    GPIO_PinState pin_state = GPIO_PIN_RESET;
    switch (GPIO_Pin) {
        // case IST8310_DRDY_Pin:
        //     pin_state = GPIO_ReadPin(IST8310_DRDY);
        //     IST8310_DRDY->tick = trigger_time;
        //     Ins_GPIOExitCallback(IST8310_DRDY);
        //     break;
        case Buttom1_Pin:
            pin_state = GPIO_ReadPin(KEY_FUNC);

#if __FN_IF_ENABLE(__FN_MINIPC_CAPT)
            if (pin_state == GPIO_PIN_RESET) {
                GPIO_Set(PC_CAM);
            } else {
                GPIO_Reset(PC_CAM);
            }
#else
            KEY_FUNC->tick = trigger_time;
            Key_KeyEventHandler(KEY_FUNC);
#endif
            break;
        case Buttom2_Pin:
            pin_state = GPIO_ReadPin(KEY_BACK);
            KEY_BACK->tick = trigger_time;
            Key_KeyEventHandler(KEY_BACK);
            break;
        // case BMI088_INT1_Pin:
        //     pin_state = GPIO_ReadPin(BMI_INT1);
        //     BMI_INT1->tick = trigger_time;
        //     if (pin_state == GPIO_PIN_SET)
        //         Ins_GPIOExitCallback(BMI_INT1);
        //     break;
        // case BMI088_INT2_Pin:
        //     pin_state = GPIO_ReadPin(BMI_INT3);
        //     BMI_INT3->tick = trigger_time;
        //     if (pin_state == GPIO_PIN_SET)
        //         Ins_GPIOExitCallback(BMI_INT3);
        //     break;
        default:
            break;
    }
#endif
}
