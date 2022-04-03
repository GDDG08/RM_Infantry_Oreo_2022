/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Inc\Peripheral\bmi055_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:58:03
 */

#ifndef BMI055_PERIPH_H
#define BMI055_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#if __FN_IF_ENABLE(__IMU_BMI055)

#include "spi_util.h"
#include "gpio_util.h"
#include "bma2x2_periph.h"
#include "bmg160_periph.h"

#define SPI_BUFFER_LEN 5
#define MASK_DATA1 0xFF
#define MASK_DATA2 0x80
#define MASK_DATA3 0x7F
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX 1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE 0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE 0x80
#define BMI055_DMA_READ_SHIFT 1

extern struct bmg160_t bmg160;
extern struct bma2x2_t bma2x2;

typedef enum {
    BMI055_STATE_NULL = 0,
    BMI055_STATE_CONNECTED = 1,
    BMI055_STATE_LOST = 2,
    BMI055_STATE_ERROR = 3,
    BMI055_STATE_PENDING = 4
} BMI055_BMI055StateEnum;

typedef struct {
    float x;
    float y;
    float z;
} BMI055_BMI055AccelTypeDef;

typedef struct {
    float x;
    float y;
    float z;
} BMI055_BMI055SGyroTypeDef;

typedef struct {
    BMI055_BMI055StateEnum state;

    uint8_t init_flag;

    BMI055_BMI055AccelTypeDef accel;
    BMI055_BMI055SGyroTypeDef gyro;

    bma2x2_accel_data_typedef acc;
    bmg160_gyro_data_typedef gyr;

    float sensor_time;
    float temperature;

    uint32_t last_update_time;
} BMI055_BMI055DataTypeDef;

extern SPI_HandleTypeDef* Const_BMI055_SPI_HANDLER;

uint8_t BMI055_Init(void);

int8_t BMI055_BMGInit(void);
int8_t BMI055_BMGSpiBusWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
int8_t BMI055_BMGSpiBusRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
void BMI055_GyroReadOver(uint8_t* rx_buf);
void BMI055_GyroRead(void);

int8_t BMI055_BMAInit(void);
int8_t BMI055_BMASpiBusWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
int8_t BMI055_BMASpiBusRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt);
void BMI055_AccelReadOver(uint8_t* rx_buf);
void BMI055_AccelRead(void);
void BMI055_usDelay(uint32_t msek);

BMI055_BMI055DataTypeDef* BMI055_GetBMI055DataPtr(void);
uint8_t BMI055_IsBMI055Offline(void);

#endif

#ifdef __cplusplus
}
#endif

#endif

#endif
