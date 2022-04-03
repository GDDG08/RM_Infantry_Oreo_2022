/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Gimbal_Control\gim_ins_ctrl.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-22 22:06:02
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:50:51
 */

#ifndef GIM_INS_CTRL_H
#define GIM_INS_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)

#include "ahrs_alg.h"
#include "pid_alg.h"
#include "magnetic_periph.h"
#include "pwm_util.h"
#include "uart_util.h"
#include "spi_util.h"
#include "gpio_util.h"
#include "bmi088_periph.h"
#include "bmi055_periph.h"
#include "hi22x_periph.h"

#define SPI_DMA_GYRO_LENGHT 7
#define SPI_DMA_ACCEL_LENGHT 8
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define IMU_DR_SHFITS 0
#define IMU_SPI_SHFITS 1
#define IMU_UPDATE_SHFITS 2
#define IMU_NOTIFY_SHFITS 3

#define INS_INIT_SHIFT 0
#define INS_ACCEL_UPDATE_SHIFT 1
#define INS_GYRO_UPDATE_SHIFT 2
#define INS_MAG_UPDATE_SHIFT 3

#define INS_ACCEL_FINISH_SHIFT 4
#define INS_GYRO_FINISH_SHIFT 5
#define INS_MAG_FINISH_SHIFT 6
#define INS_CLEAR_FINISH_FLAG 0X0F

#define IMU_REF_TEMP 40.0f

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define IST8310_RX_BUF_DATA_OFFSET 16

#define INS_TASK_INIT_TIME 3000

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

typedef enum {
    INS_STATE_NULL = 0,
    INS_STATE_CONNECTED = 1,
    INS_STATE_LOST = 2,
    INS_STATE_ERROR = 3,
    INS_STATE_PENDING = 4
} INS_IMUStateEnum;

typedef struct {
    float yaw;
    float pitch;
    float row;
} INS_IMUValueTypeDef;

typedef struct {
    INS_IMUValueTypeDef count;

    INS_IMUValueTypeDef mag;
    INS_IMUValueTypeDef accel;
    INS_IMUValueTypeDef speed;
    INS_IMUValueTypeDef angle;

    float sensor_time;
    float temperature;

    INS_IMUValueTypeDef last_angle;
    INS_IMUValueTypeDef now_angle;

    float yaw_angle_offset;
    float pitch_angle_offset;
    float row_angle_offset;

    INS_IMUStateEnum state;
    uint32_t last_update_time;
} INS_IMUDataTypeDef;

extern volatile uint8_t ins_flag;
extern float Ins_quat[4];
extern TaskHandle_t INS_task_local_handler;
void Ins_Task(void const* argument);
void Ins_InsInit(void);
void Ins_InitIMU(void);
INS_IMUDataTypeDef* Ins_GetIMUDataPtr(void);
uint8_t Ins_IsIMUOffline(void);
void Ins_DecodeIMUData(void);
void Ins_TempControl(void);
void Ins_GPIOExitCallback(GPIO_GPIOTypeDef* gpio);
static void Ins_ImuDMACmd(void);
void Ins_DMAIRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif

#endif
