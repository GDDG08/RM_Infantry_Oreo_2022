/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Common_Contrrol\watchdog_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-07-24 10:27:08
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-26 20:03:42
 */

#include "watchdog_ctrl.h"

#include "buscomm_ctrl.h"
#include "supercap_ctrl.h"
#include "remote_periph.h"
#include "referee_periph.h"
#include "motor_periph.h"
#include "miniPC_periph.h"
#include "cha_gimbal_ctrl.h"
#include "cha_chassis_ctrl.h"
#include "gim_login_ctrl.h"

#include "debug_BTlog.h"

#define WATCHDOG_TASK_PERIOD 1

/**
 * @brief          WatchDog task
 * @param          NULL
 * @retval         NULL
 */
void WatchDog_Task(void const* argument) {
    for (;;) {
        WatchDog_FeedDog();
        BTlog_Send();
        osDelay(WATCHDOG_TASK_PERIOD);
    }
}

void WatchDog_FeedDog() {
    static uint8_t firstout[5] = 1;
#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

    if (BusComm_IsBusCommOffline(BusComm_PKG_CTRL)) {
    }
    if (BusComm_IsBusCommOffline(BusComm_PKG_IMU)) {
        firstout[BusComm_PKG_IMU] = 1;
        GimbalYaw_SetGimbalYawControlState(0);
        GimbalYaw_SetGimbalYawOutputState(0);
    } else {
        if (firstout[BusComm_PKG_IMU]) {
            firstout[BusComm_PKG_IMU] = 0;
            GimbalYaw_ReSetYawRef();
        }
        GimbalYaw_SetGimbalYawControlState(1);
        GimbalYaw_SetGimbalYawOutputState(1);
    }
    if (BusComm_IsBusCommOffline(BusComm_PKG_CHA_REF)) {
        Chassis_SetMode(Chassis_MODE_STOP);

    } else {
        Chassis_SetChassisControlState(1);
        Chassis_SetChassisOutputState(1);
    }
    if (BusComm_IsBusCommOffline(BusComm_PKG_CAP_1)) {
    }
    if (BusComm_IsBusCommOffline(BusComm_PKG_CAP_2)) {
    }

#elif __FN_IF_ENABLE(__FN_INFANRTY_GIMBAL)
    if (BusComm_IsBusCommOffline(BusComm_PKG_REFEREE)) {
        // if (LOGIN_ON_FLAG == 0) {
        // }
    }
#endif
}
