/*
 * @Project      : RM_Infantry_Neptune_frame
 * @FilePath     : \Infantry_Oreo\Core\Inc\configure.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2022-01-14 22:16:51
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-05 14:19:35
 */

// Note:
// In order to avoid bus phenomenon
// if you want to change or add code function
// please operate according to the following code specification
#include "Code_specification.h"
//        ������������������������������������

#ifndef CONFIGURE_H
#define CONFIGURE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Main Control program       */

// #define __FN_BOARD_TYPE __FN_BOARD_TYPE_CHASSIS
#define __FN_BOARD_TYPE __FN_BOARD_TYPE_GIMBAL

#define __FN_BOARD_TYPE_CHASSIS 1
#define __FN_BOARD_TYPE_GIMBAL 2
#define __FN_BOARD_TYPE_SUPERCAP 3

#define __IMU_TYPE __IMU_TYPE_HI22X

#define __IMU_TYPE_BMI055 1
#define __IMU_TYPE_BMI088 2
#define __IMU_TYPE_HI22X 3

#define __IMU_AXIS_TYPE __IMU_SIX_AXIS

#define __IMU_SIX_AXIS_TYPE 6
#define __IMU_NINE_AXIS_TYPE 9

/*      ******************DEBUG********************      */
#define __FN_DEBUG_NOREFEREEHW __FN_DISABLE
#define __FN_DEBUG_BTLOG __FN_ENABLE
#define __FN_MINIPC_CAPT __FN_DISABLE

/*      **********************************************      */

extern int GLOBAL_INIT_FLAG;

#define __FN_ENABLE 1
#define __FN_DISABLE 0

#define __FN_IF_ENABLE(x) (x == __FN_ENABLE)

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_CHASSIS
#define __FN_INFANTRY __FN_ENABLE
#define __FN_INFANTRY_CHASSIS __FN_ENABLE
#else
#define __FN_INFANTRY_CHASSIS __FN_DISABLE
#endif

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_GIMBAL
#define __FN_INFANTRY __FN_ENABLE
#define __FN_INFANTRY_GIMBAL __FN_ENABLE
#else
#define __FN_INFANTRY_GIMBAL __FN_DISABLE
#endif

#if __FN_BOARD_TYPE == __FN_BOARD_TYPE_SUPERCAP
#define __FN_INFANTRY __FN_DISABLE
#define __FN_SUPER_CAP __FN_ENABLE
#else
#define __FN_SUPER_CAP __FN_DISABLE
#endif

/* Detail function define     */
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#if __IMU_TYPE == __IMU_TYPE_BMI055
#define __IMU_BMI055 __FN_ENABLE
#endif
#if __IMU_TYPE == __IMU_TYPE_BMI088
#define __IMU_BMI088 __FN_ENABLE
#endif
#if __IMU_TYPE == __IMU_TYPE_HI22X
#define __IMU_HI22X __FN_ENABLE
#endif
#endif

#if __IMU_AXIS_TYPE == __IMU_SIX_AXIS_TYPE
#define __IMU_SIX_AXIS __FN_ENABLE
#else
#define __IMU_NINE_AXIS __FN_ENABLE
#endif

/*  Control function          */

#if __FN_IF_ENABLE(__FN_SUPER_CAP)
#define __FN_CTRL_CAP_COMM __FN_ENABLE
#define __FN_CTRL_CAP __FN_ENABLE
#define __FN_WATCHDOG_CAP __FN_ENABLE
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_CTRL_REMOTE __FN_ENABLE
#define __FN_CTRL_MINIPC __FN_ENABLE
#define __FN_CTRL_GIM_COMM __FN_ENABLE
#define __FN_CTRL_GIMBAL_GIM __FN_ENABLE
#define __FN_WATCHDOG_GIM __FN_ENABLE
#define __FN_CTRL_SHOOTER __FN_ENABLE
#define __FN_CTRL_CLIENT __FN_ENABLE
#define __FN_CTRL_LOGIN __FN_ENABLE
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)
#define __FN_CTRL_CHASSIS __FN_ENABLE
#define __FN_CTRL_CHA_COMM __FN_ENABLE
#define __FN_CTRL_POWER __FN_ENABLE
#define __FN_CTRL_GIMBAL_YAW_CHA __FN_ENABLE
#define __FN_WATCHDOG_CHA __FN_ENABLE

#if !__FN_IF_ENABLE(__FN_DEBUG_NOREFEREEHW)
#define __FN_CTRL_REFEREE __FN_ENABLE
#endif
#endif

/*      Infantry function enable    */
#if __FN_IF_ENABLE(__FN_INFANTRY)

/* Base Utility Configuration */
#define __FN_UTIL_CAN __FN_ENABLE
// Enable CAN BUS
#define __FN_UTIL_UART __FN_ENABLE
// Enable serial port
#define __FN_UTIL_PWM __FN_ENABLE
// Enable PWM
#define __FN_UTIL_USB __FN_DISABLE
// Enable USB
#define __FN_UTIL_FLASH __FN_DISABLE
// Enable Flash
#define __FN_GPIO_IT __FN_ENABLE
// Enable GPIO interrupt
#define __FN_UTIL_I2C __FN_ENABLE
// Enable I2C
#define __FN_UTIL_SPI __FN_DISABLE
// Enable SPI

/* Peripheral Configuration */
#define __FN_PERIPH_MOTOR __FN_ENABLE
// Enable Motor

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_KEY __FN_ENABLE
// Enable KEY
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_OLED __FN_ENABLE
// Enable OLED
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_IMU __FN_ENABLE
// Enable IMU
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_MAG __FN_DISABLE
// Enable magnetic
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_MINIPC __FN_ENABLE
// Enable MiniPC
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_REMOTE __FN_ENABLE
// Enable remote
#endif
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

#if __FN_IF_ENABLE(__FN_DEBUG_NOREFEREEHW)
#define __FN_PERIPH_REFEREE_NOHW __FN_ENABLE
#else
#define __FN_PERIPH_REFEREE __FN_ENABLE
#endif
// Enable referee
#endif
#define __FN_PERIPH_BEEPER __FN_ENABLE
// Enable beeper
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#define __FN_PERIPH_SERVO __FN_ENABLE
// Enable servo
#endif
#define __FN_PERIPH_LED __FN_ENABLE
// Enable LED

#endif

/* Super Cap functions enable   */
#if __FN_IF_ENABLE(__FN_SUPER_CAP)

#define __FN_UTIL_CAN __FN_ENABLE
// Enable CAN BUS
#define __FN_UTIL_ADC __FN_ENABLE
// Enable ADC
#define __FN_UTIL_UART __FN_ENABLE
// Enable UART
#define __FN_UTIL_PWM __FN_ENABLE
// Enable PWM
#define __FN_UTIL_DAC __FN_ENABLE
// Enable DAC
#define __FN_PERIPH_SENSOR __FN_ENABLE
// Enable sensor
#define __FN_PERIPH_LED __FN_ENABLE
// Enable LED

#endif

#ifdef __cplusplus
}
#endif

#endif
