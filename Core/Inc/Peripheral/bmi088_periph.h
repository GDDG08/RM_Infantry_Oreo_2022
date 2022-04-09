/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\bmi088_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-04-03 22:47:07
 */

#ifndef BMI088_PERIPH_H
#define BMI088_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"
#if __FN_IF_ENABLE(__IMU_BMI088)

#include "main.h"
#include "uart_util.h"
#include "spi_util.h"
#include "gpio_util.h"
#include "bmi088_reg.h"

#define BMI088_ACCEL_WRITE_SINGLE_REG(reg, data)           \
    {                                                      \
        GPIO_Reset(CS_ACCEL);                              \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, reg);  \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, data); \
        GPIO_Set(CS_ACCEL);                                \
    }
#define BMI088_ACCEL_READ_SINGLE_REG(reg, data)                      \
    {                                                                \
        GPIO_Reset(CS_ACCEL);                                        \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80)); \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);           \
        (data) = Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);  \
        GPIO_Set(CS_ACCEL);                                          \
    }
#define BMI088_ACCEL_READ_MULI_REG(reg, data, len)                   \
    {                                                                \
        GPIO_Reset(CS_ACCEL);                                        \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80)); \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80)); \
        Spi_ReadMuliReg(Const_BMI088_SPI_HANDLER, data, len);        \
        GPIO_Set(CS_ACCEL);                                          \
    }
#define BMI088_GYRO_WRITE_SINGLE_REG(reg, data)            \
    {                                                      \
        GPIO_Reset(CS_GYRO);                               \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, reg);  \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, data); \
        GPIO_Set(CS_GYRO);                                 \
    }
#define BMI088_GYRO_READ_SINGLE_REG(reg, data)                       \
    {                                                                \
        GPIO_Reset(CS_GYRO);                                         \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80)); \
        (data) = Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);  \
        GPIO_Set(CS_GYRO);                                           \
    }
#define BMI088_GYRO_READ_MULI_REG(reg, data, len)                    \
    {                                                                \
        GPIO_Reset(CS_GYRO);                                         \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80)); \
        Spi_ReadMuliReg(Const_BMI088_SPI_HANDLER, data, len);        \
        GPIO_Set(CS_GYRO);                                           \
    }

typedef enum {
    BMI088_STATE_NULL = 0,
    BMI088_STATE_CONNECTED = 1,
    BMI088_STATE_LOST = 2,
    BMI088_STATE_ERROR = 3,
    BMI088_STATE_PENDING = 4
} BMI088_BMI088StateEnum;

typedef struct {
    float yaw;
    float pitch;
    float row;
} BMI088_BMI088AccelTypeDef;

typedef struct {
    float yaw;
    float pitch;
    float row;
} BMI088_BMI088SpeedTypeDef;

typedef struct {
    BMI088_BMI088StateEnum state;

    BMI088_BMI088AccelTypeDef accel;
    BMI088_BMI088SpeedTypeDef speed;

    float sensor_time;
    float temperature;

    uint32_t last_update_time;
} BMI088_BMI088DataTypeDef;

extern const uint16_t Const_BMI088_OFFLINE_TIME;

extern SPI_HandleTypeDef* Const_BMI088_SPI_HANDLER;

BMI088_BMI088DataTypeDef* BMI088_GetBMI088DataPtr(void);
uint8_t BMI088_Init(void);
static uint8_t BMI088_BMI088AccelInit(void);
static uint8_t BMI088_BMI088GyroInit(void);
static uint8_t BMI088_BMI088AccelTest(void);
static uint8_t BMI088_BMI088GyroTest(void);
uint8_t BMI088_IsBMI088Offline(void);
void BMI088_InitAngelOffset(void);
void BMI088_RXCallback(UART_HandleTypeDef* huart);
void BMI088_ResetBMI088ata(void);
void BMI088_BMI088DecodeData(void);
void BMI088_BMI088TempReadOver(uint8_t* rx_buff);
void BMI088_BMI088AccelReadOver(uint8_t* rx_buff);
void BMI088_BMI088GyroReadOver(uint8_t* rx_buff);

#endif

#ifdef __cplusplus
}
#endif

#endif
