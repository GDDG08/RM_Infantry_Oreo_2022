/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Gimbal_Control\gim_client_ctrl.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-29 19:11:51
 */

#include "gim_client_ctrl.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

#include "str_lib.h"
#include "oled_periph.h"
#include "key_periph.h"
#include "key_periph.h"
#include "cmsis_os.h"

#include "gim_ins_ctrl.h"
#include "gim_shoot_ctrl.h"
#include "gim_login_ctrl.h"
#include "buscomm_ctrl.h"

#define CLIENT_TASK_PERIOD 10
#define CLIENT_PAGE_REFRESH_PERIOD 60
#define CLIENT_PAGE_STAY_PERIOD 6000

List_ListTypeDef Client_ScreenDisplay;
Client_PageTypeDef Client_ShootState;
Client_PageTypeDef Client_PeriphState;
Client_PageTypeDef Client_CapState;

uint8_t next_page_flag = 0;
uint8_t previous_page_flag = 0;
Client_InterfaceEnum interface_flag = CLIENT_NULL;

osEvent func_event;

/**
 * @brief          Client task
 * @param          NULL
 * @retval         NULL
 */
void Client_Task(void const* argument) {
    static uint32_t page_stay_period = 0;
    // while (1) {
    //     osDelay(1);
    // }
    for (;;) {
        Client_DisplayInterface();
        if (GLOBAL_INIT_FLAG == 1) {
            func_event = osMessageGet(Key_QueueHandle, 1);
            if (func_event.status == osEventMessage) {
                page_stay_period = HAL_GetTick();
                Client_KeytHandler(func_event.value.v);
            }

            if ((HAL_GetTick() - page_stay_period) >= CLIENT_PAGE_STAY_PERIOD) {
                if ((interface_flag != CLIENT_ERROR_1) &&
                    (interface_flag != CLIENT_ERROR_2) &&
                    (interface_flag != CLIENT_ERROR_3)) {
                    Client_ChangeInterface(CLIENT_ON);
                }
            }
        }
        osDelay(CLIENT_TASK_PERIOD);
    }
}

/**
 * @brief          Client initialization
 * @param          NULL
 * @retval         NULL
 */
void Client_Init() {
    OLED_init();
    List_InitList(&Client_ScreenDisplay, &Client_ShootState);
    OLED_DisplayOn();
    Client_PageInit();
    OLED_OperateGram(PEN_CLEAR);
    OLED_RefreshGram();
    Client_ChangeInterface(CLIENT_START);
}

/**
 * @brief          Initialize all displayed pages
 * @param          NULL
 * @retval         NULL
 */
void Client_PageInit() {
    INS_IMUDataTypeDef* imu = Ins_GetIMUDataPtr();
    Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
    BusComm_BusCommDataTypeDef* buscomm = BusComm_GetBusDataPtr();

    Client_CreateNewPage(&Client_ShootState, "15mm_offset\0", &shooter->shoot_speed_offset.speed_15mm_offset, "18mm_offset\0", &shooter->shoot_speed_offset.speed_18mm_offset, "30mm_offset\0", &shooter->shoot_speed_offset.speed_30mm_offset, "shoot_Speed\0", shoot_state_func_short_press, shoot_state_func_long_press);
    Client_CreateNewPage(&Client_PeriphState, "pitch_ang____\0", &imu->angle.pitch, "yaw_ang____\0", &imu->angle.yaw, "row_ang____\0", &imu->angle.row, "imu_data___\0", periph_state_func_short_press, periph_state_func_long_press);
    // Client_CreateNewPage(&Client_CapState, "cap_precent\0", &buscomm->cap_rest_energy_display/*, "yaw_rel_ang\0", &buscomm->yaw_relative_angle*/ ,"heat_17mm\0", &buscomm->heat_17mm /*&buscomm->gimbal_yaw_ref*/, "Cap_Sta_Pag\0", cap_state_func_short_press, cap_state_func_long_press);
}

/**
 * @brief          Cartoon interface display
 * @param          NULL
 * @retval         NULL
 */
static void Client_DisplayInterface() {
    if (interface_flag == CLIENT_NULL)
        return;
    OLED_OperateGram(PEN_CLEAR);

    if (interface_flag == CLIENT_START) {
        OLED_DisplayGIF(GIF_ROCKET);
    } else if (interface_flag == CLIENT_ON) {
        OLED_DisplayGIF(GIF_FIRE);
        uint8_t code = Key_GetEquipCode();
        OLED_Printf(4, 5, "Infantry_%d\0", code);
    } else if (interface_flag == CLIENT_ERROR_1) {
        OLED_DisplayBMG(OLED_BEAR);
    } else if (interface_flag == CLIENT_ERROR_2) {
        OLED_DisplayBMG(OLED_SUR);
    } else if (interface_flag == CLIENT_ERROR_3) {
        OLED_DisplayBMG(OLED_SUB);
    } else if (interface_flag == CLIENT_PAGE) {
        Client_DisplayCurrentPage();
    } else if (interface_flag == CLIENT_CODE) {
        uint32_t login_code = Login_GetCode();
        OLED_Printf(3, 1, "The Code :%X", login_code);
    }
    OLED_RefreshGram();
}

/**
 * @brief          Cartoon interface display
 * @param          NULL
 * @retval         NULL
 */
void Client_ChangeInterface(Client_InterfaceEnum intface) {
    interface_flag = intface;
}

/**
 * @brief          Create a new display page
 * @param          NULL
 * @retval         NULL
 */
void Client_CreateNewPage(Client_PageTypeDef* page, const char* val1_name, void* val_1, const char* val2_name, void* val_2, const char* val3_name, void* val_3, const char* page_name, Client_KeyEventHandlerTypeDef key_short, Client_KeyEventHandlerTypeDef key_long) {
    page->lang_press_handler = key_long;
    page->short_press_handler = key_short;
    char* buf_1 =
        memcpy(page->name[0], val1_name, 12);
    memcpy(page->name[1], val2_name, 12);
    memcpy(page->name[2], val3_name, 12);
    memcpy(page->page_name, page_name, 12);

    page->value[0] = (float*)val_1;
    page->value[1] = (float*)val_2;
    page->value[2] = (float*)val_3;

    List_InsertEnd(&Client_ScreenDisplay, page);
}

/**
 * @brief          Show a page
 * @param          NULL
 * @retval         NULL
 */
static void Client_DisplayCurrentPage() {
    static uint32_t page_period = 0;

    if (HAL_GetTick() - page_period >= CLIENT_PAGE_REFRESH_PERIOD) {
        page_period = HAL_GetTick();
        Client_PageTypeDef* cur_page;
        if (next_page_flag == 1) {
            cur_page = List_GetNextListDataPtr(&Client_ScreenDisplay);
            next_page_flag = 0;
        } else if (previous_page_flag == 1) {
            cur_page = List_GetPreviousListDataPtr(&Client_ScreenDisplay);
            previous_page_flag = 0;
        } else {
            cur_page = List_GetCurrentListDataPtr(&Client_ScreenDisplay);
        }

        if (cur_page == NULL)
            return;

        for (int i = 0; i < 3; i++) {
            OLED_ShowString(i + 1, 1, cur_page->name[i]);
            OLED_Printf(i + 1, 13, ":%0.2f\0", (*cur_page->value[i]));
        }
        OLED_ShowString(4, 1, cur_page->page_name);
    }
}

uint8_t gpio;
uint8_t press_event;
/**
 * @brief          Back key Short press handling function
 * @param          NULL
 * @retval         NULL
 */
void Client_KeytHandler(uint32_t message) {
    gpio = (uint8_t)(0xff & message);
    press_event = (uint8_t)(0xff & (message >> 8));
    Client_PageTypeDef* cur_page;

    switch (gpio) {
        case KEY_BACK_EVENR_ID:
            if (press_event == SHORT_PRESS_EVENT) {
                Client_BackKeyShortHandler();
            } else if (press_event == LONG_PRESS_EVENT) {
                Client_BackKeyLongHandler();
            }
            break;
        case KEY_FUNC_EVENT_ID:
            cur_page = List_GetCurrentListDataPtr(&Client_ScreenDisplay);
            if (cur_page == NULL)
                return;

            if (press_event == SHORT_PRESS_EVENT) {
                if (cur_page->short_press_handler != NULL)
                    cur_page->short_press_handler();
            } else if (press_event == LONG_PRESS_EVENT) {
                if (cur_page->lang_press_handler != NULL)
                    cur_page->lang_press_handler();
            }
            break;
        default:
            break;
    }
}

/**
 * @brief          Back key Short press handling function
 * @param          NULL
 * @retval         NULL
 */
void Client_BackKeyShortHandler() {
    Client_ChangeInterface(CLIENT_PAGE);
    next_page_flag = 1;
}

/**
 * @brief          Back key Short press handling function
 * @param          NULL
 * @retval         NULL
 */
void Client_BackKeyLongHandler() {
    Client_ChangeInterface(CLIENT_CODE);
}

/*      key callback functions      */

// Client_ShootState
// short press
static void shoot_state_func_short_press() {
    Shooter_ModifySpeedOffset(1);
}

// long press
static void shoot_state_func_long_press() {
    Shooter_ModifySpeedOffset(-1);
}

// Client_PeriphState
// short press
static void periph_state_func_short_press() {
}

// long press
static void periph_state_func_long_press() {
}

// Client_CapState
// short press
static void cap_state_func_short_press() {
}

// long press
static void cap_state_func_long_press() {
}

#endif
