/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Peripheral\bmi088_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:06
 */

#include "bmi088_periph.h"

#if __FN_IF_ENABLE(__IMU_BMI088)

#include "const.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

SPI_HandleTypeDef* Const_BMI088_SPI_HANDLER = &hspi1;
const uint16_t Const_BMI088_OFFLINE_TIME = 200;

BMI088_BMI088DataTypeDef BMI088_BMI088Data;

/**
 * @brief      Get pinter to the bmi088 data object
 * @param      NULL
 * @retval     Pointer to bmi088 data object
 */
BMI088_BMI088DataTypeDef* BMI088_GetBMI088DataPtr() {
    return &BMI088_BMI088Data;
}

/**
 * @brief      Initialization bmi088
 * @param      NULL
 * @retval     NULL
 */
uint8_t BMI088_Init() {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();

    uint8_t error = BMI088_NO_ERROR;
    if (BMI088_BMI0xxAccelTest() != BMI088_NO_ERROR) {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    } else {
        error |= BMI088_BMI0xxAccelInit();
    }

    if (BMI088_BMI0xxGyroTest() != BMI088_NO_ERROR) {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    } else {
        error |= BMI088_BMI0xxGyroInit();
    }

    if (error == BMI088_NO_ERROR) {
        bmi088->state = BMI088_STATE_LOST;
        return 0;
    } else {
        bmi088->state = BMI088_STATE_CONNECTED;
        return 0;
    }
}

/**
 * @brief      BMI0XX bmi088 Accel Initial
 * @param      NULL
 * @retval     NULL
 */
static uint8_t BMI088_BMI0xxAccelInit() {
    volatile uint8_t res = 0;
    uint8_t write_reg_num = 0;

    // check commiunication
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // accel software reset
    BMI088_ACCEL_WRITE_SINGLE_REG(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++) {
        BMI088_ACCEL_WRITE_SINGLE_REG(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_READ_SINGLE_REG(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1]) {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

/**
 * @brief      BMI0XX bmi088 Gyro Initial
 * @param      NULL
 * @retval     NULL
 */
static uint8_t BMI088_BMI0xxGyroInit() {
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    // check commiunication
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset the gyro sensor
    BMI088_GYRO_WRITE_SINGLE_REG(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);
    // check commiunication is normal after reset
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++) {
        BMI088_GYRO_WRITE_SINGLE_REG(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_GYRO_READ_SINGLE_REG(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1]) {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/**
 * @brief      BMI0XX bmi088 Accel function test
 * @param      NULL
 * @retval     NULL
 */
static uint8_t BMI088_BMI0xxAccelTest() {
    volatile uint8_t res = 0;
    int16_t self_test_accel[2][3];
    uint8_t buff[6] = {0, 0, 0, 0, 0, 0};
    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    // check commiunication is normal
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_ACCEL_WRITE_SINGLE_REG(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++) {
        BMI088_ACCEL_WRITE_SINGLE_REG(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_READ_SINGLE_REG(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]) {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        HAL_Delay(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++) {
        BMI088_ACCEL_WRITE_SINGLE_REG(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_ACCEL_READ_SINGLE_REG(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]) {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        HAL_Delay(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_ACCEL_READ_MULI_REG(BMI088_ACCEL_XOUT_L, buff, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buff[1]) << 8) | buff[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buff[3]) << 8) | buff[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buff[5]) << 8) | buff[4];
    }

    // set self test off
    BMI088_ACCEL_WRITE_SINGLE_REG(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_SELF_TEST, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF)) {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    // reset the accel sensor
    BMI088_ACCEL_WRITE_SINGLE_REG(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680)) {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_ACCEL_READ_SINGLE_REG(BMI088_ACC_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}

/**
 * @brief      BMI0XX bmi088 Gyro function test
 * @param      NULL
 * @retval     NULL
 */
static uint8_t BMI088_BMI0xxGyroTest() {
    uint8_t res = 0;
    uint8_t retry = 0;

    // check commiunication is normal
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset the gyro sensor
    BMI088_GYRO_WRITE_SINGLE_REG(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    // check commiunication is normal after reset
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_CHIP_ID, res);
    delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_GYRO_WRITE_SINGLE_REG(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    HAL_Delay(BMI088_LONG_DELAY_TIME);

    do {
        BMI088_GYRO_READ_SINGLE_REG(BMI088_GYRO_SELF_TEST, res);
        delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL) {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

/**
 * @brief      Initialization offset and set mode
 * @param      NULL
 * @retval     NULL
 */
void BMI088_InitAngelOffset() {
}

/**
 * @brief      Judge bmi088 offline
 * @param      NULL
 * @retval     Offline or not��1 is offline��0 is not��
 */
uint8_t BMI088_IsBMI088Offline() {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();

    uint32_t now = HAL_GetTick();
    if ((now - bmi088->last_update_time) > Const_BMI088_OFFLINE_TIME)
        bmi088->state = BMI088_STATE_LOST;
    return bmi088->state == BMI088_STATE_LOST;
}

/**
 * @brief      Reset bmi088 data object
 * @param      NUULL
 * @retval     NUL
 */
void BMI088_ResetBMI088Data() {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();

    bmi088->accel.pitch = 0;
    bmi088->accel.row = 0;
    bmi088->accel.yaw = 0;
    bmi088->speed.pitch = 0;
    bmi088->speed.row = 0;
    bmi088->speed.yaw = 0;
    bmi088->temperature = 0;
    bmi088->last_update_time = HAL_GetTick();
}

/**
 * @brief      BMI088 decode data function    ��For BMI0xx)
 * @param      NULL
 * @retval     NULL
 */
void BMI088_BMI0xxDecodeData() {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();
    uint8_t buff[8] = {0, 0, 0, 0, 0, 0};
    int16_t raw_temp;

    bmi088->state = BMI088_STATE_PENDING;
    bmi088->last_update_time = HAL_GetTick();

    BMI088_ACCEL_READ_MULI_REG(BMI088_ACCEL_XOUT_L, buff, 6);

    bmi088->accel.pitch = ((int16_t)((buff[1]) << 8) | buff[0]) * BMI088_ACCEL_SEN;
    bmi088->accel.row = ((int16_t)((buff[3]) << 8) | buff[2]) * BMI088_ACCEL_SEN;
    bmi088->accel.yaw = ((int16_t)((buff[5]) << 8) | buff[4]) * BMI088_ACCEL_SEN;

    BMI088_GYRO_READ_MULI_REG(BMI088_GYRO_CHIP_ID, buff, 8);
    if (buff[0] == BMI088_GYRO_CHIP_ID_VALUE) {
        bmi088->speed.pitch = ((int16_t)((buff[3]) << 8) | buff[2]) * BMI088_GYRO_SEN;
        bmi088->speed.row = ((int16_t)((buff[5]) << 8) | buff[4]) * BMI088_GYRO_SEN;
        bmi088->speed.yaw = ((int16_t)((buff[7]) << 8) | buff[6]) * BMI088_GYRO_SEN;
    }
    BMI088_ACCEL_READ_MULI_REG(BMI088_TEMP_M, buff, 2);

    raw_temp = (int16_t)((buff[0] << 3) | (buff[1] >> 5));
    if (raw_temp > 1023) {
        raw_temp -= 2048;
    }

    bmi088->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    bmi088->state = BMI088_STATE_CONNECTED;
}

/**
 * @brief      BMI088 decode temperature data function    ��For BMI0xx)
 * @param      rx_buff :BMI088 SPI temperature buff
 * @retval     NULL
 */
void BMI088_BMI0xxTempReadOver(uint8_t* rx_buff) {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();

    bmi088->state = BMI088_STATE_PENDING;
    bmi088->last_update_time = HAL_GetTick();

    int16_t raw_temp;
    raw_temp = (int16_t)((rx_buff[0] << 3) | (rx_buff[1] >> 5));

    if (raw_temp > 1023) {
        raw_temp -= 2048;
    }
    bmi088->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    bmi088->state = BMI088_STATE_CONNECTED;
}

/**
 * @brief      BMI088 decode temperature data function    ��For BMI0xx)
 * @param      rx_buff :BMI088 SPI accel buff
 * @retval     NULL
 */
void BMI088_BMI0xxAccelReadOver(uint8_t* rx_buff) {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_GetBMI088DataPtr();

    bmi088->state = BMI088_STATE_PENDING;
    bmi088->last_update_time = HAL_GetTick();

    bmi088->accel.pitch = ((int16_t)((rx_buff[1]) << 8) | rx_buff[0]) * BMI088_ACCEL_SEN;
    bmi088->accel.row = ((int16_t)((rx_buff[3]) << 8) | rx_buff[2]) * BMI088_ACCEL_SEN;
    bmi088->accel.yaw = ((int16_t)((rx_buff[5]) << 8) | rx_buff[4]) * BMI088_ACCEL_SEN;

    bmi088->sensor_time = ((uint32_t)((rx_buff[8] << 16) | (rx_buff[7] << 8) | rx_buff[6])) * 39.0625f;

    bmi088->state = BMI088_STATE_CONNECTED;
}

/**
 * @brief      BMI088 decode temperature data function
 * @param      rx_buff :bmi088 SPI gyro buff
 * @retval     NULL
 */
void BMI088_BMI088GyroReadOver(uint8_t* rx_buff) {
    BMI088_BMI088DataTypeDef* bmi088 = BMI088_Getbmi088DataPtr();

    bmi088->state = BMI088_STATE_PENDING;
    bmi088->last_update_time = HAL_GetTick();

    bmi088->speed.pitch = ((int16_t)((rx_buff[1]) << 8) | rx_buff[0]) * BMI088_GYRO_SEN;
    bmi088->speed.row = ((int16_t)((rx_buff[3]) << 8) | rx_buff[2]) * BMI088_GYRO_SEN;
    bmi088->speed.yaw = ((int16_t)((rx_buff[5]) << 8) | rx_buff[4]) * BMI088_GYRO_SEN;

    bmi088->state = BMI088_STATE_CONNECTED;
}

#endif
