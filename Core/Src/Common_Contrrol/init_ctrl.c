/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Common_Contrrol\init_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:32
 */

#include "init_ctrl.h"

#include "configure.h"

#include "led_periph.h"
#include "minipc_periph.h"
#include "motor_periph.h"
#include "referee_periph.h"
#include "remote_periph.h"
#include "sensor_periph.h"
#include "servo_periph.h"
#include "magnetic_periph.h"
#include "key_periph.h"

#include "adc_util.h"

#include "cha_chassis_ctrl.h"
#include "cha_power_ctrl.h"
#include "cha_gimbal_ctrl.h"
#include "cha_referee_ctrl.h"

#include "gim_gimbal_ctrl.h"
#include "gim_remote_ctrl.h"
#include "gim_miniPC_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_client_ctrl.h"
#include "gim_login_ctrl.h"
#include "gim_ins_ctrl.h"

#include "supercap_ctrl.h"

#include "buscomm_ctrl.h"
#include "watchdog_ctrl.h"
#include "debug_BTlog.h"

#include "const.h"
#include "cmsis_os.h"

int GLOBAL_INIT_FLAG = 0;

/**
 * @brief      Initialize all peripherals
 * @param      NULL
 * @retval     NULL
 */
void Init_Task(void const* argument) {
    LED_InitAllLEDs();

#if __FN_IF_ENABLE(__FN_SUPER_CAP)

    /* basis periph init    */
    BusComm_InitBusComm();
    Can_InitFilterAndStart(&hcan1);
    Can_InitFilterAndStart(&hcan2);

    Sen_Init();
    Dac_Init();

    Const_Init();
    Cap_Init();

#if __FN_IF_ENABLE(__FN_DEBUG_BTLOG)
    BTlog_Init();
#endif
    /* control function init    */
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

    Const_Init();

    Client_Init();

    Servo_InitAllServos();

    Ins_InsInit();
    MiniPC_InitMiniPC();
		Adc_Init();
    Motor_InitAllMotors();
    BusComm_InitBusComm();
    FDCAN_IntFilterAndStart(&hfdcan1);
    FDCAN_IntFilterAndStart(&hfdcan2);
    FDCAN_IntFilterAndStart(&hfdcan3);

    Gimbal_InitOffset();
    Shooter_InitShooter();
    MiniPC_ControlInit();

    Remote_InitRemote();
    Remote_RemotrControlInit();

    Login_Init();
//   Login_LoginOn();
#if __FN_IF_ENABLE(__FN_DEBUG_BTLOG)
    BTlog_Init();
#endif
#endif

#if __FN_IF_ENABLE(__FN_INFANTRY_CHASSIS)

    Referee_InitReferee();

    Motor_InitAllMotors();
    BusComm_InitBusComm();
    FDCAN_IntFilterAndStart(&hfdcan1);
    FDCAN_IntFilterAndStart(&hfdcan2);
    FDCAN_IntFilterAndStart(&hfdcan3);

    Const_Init();
    Cap_Init();
    Chassis_InitChassis();
    GimbalYaw_InitGimbalYaw();

#if __FN_IF_ENABLE(__FN_DEBUG_BTLOG)
    BTlog_Init();
#endif
#endif

    TaskHandle_t InitTask_Handler = xTaskGetHandle(pcTaskGetName(NULL));

    for (;;) {
        GLOBAL_INIT_FLAG = 1;
        vTaskDelete(InitTask_Handler);
        osDelay(1);
    }
}

/**
 * @brief      Initialization delay
 * @param      NULL
 * @retval     NULL
 */
void Init_MainLoop() {
}
