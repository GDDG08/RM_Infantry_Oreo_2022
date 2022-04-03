/*
 * @Project      : RM_Infantry_Neptune
 * @FilePath     : \infantry_-neptune\Core\Src\Peripheral\bmi055_periph.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:03
 */

#include "bmi055_periph.h"

#if __FN_IF_ENABLE(__FN_INFANTRY_GIMBAL)
#if __FN_IF_ENABLE(__IMU_BMI055)

#include "const.h"

#define BMI055_ACCEL_2G_SEN 0.002392578125f
#define BMI055_ACCEL_4G_SEN 0.00478515625f
#define BMI055_ACCEL_6G_SEN 0.007177734375f
#define BMI055_TEMP_SEN 0.5f
#define BMI055_TEMP_CONST 23.0f

#define BMI055_GYRO_2000_SEN 0.00106526f
#define BMI055_GYRO_1000_SEN 0.00053263f
#define BMI055_GYRO_500_SEN 0.00026631f
#define BMI055_GYRO_250_SEN 0.00013315f

struct bmg160_t bmg160;
struct bma2x2_t bma2x2;

SPI_HandleTypeDef* Const_BMI055_SPI_HANDLER = &hspi1;
const uint16_t Const_BMI055_OFFLINE_TIME = 200;
BMI055_BMI055DataTypeDef BMI055_BMI055Data;

/**
 * @brief      Reset bmi055 init
 * @param      NUULL
 * @retval     NUL
 */
uint8_t BMI055_Init() {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();

    GPIO_Set(CS_GYRO);
    GPIO_Set(CS_ACCEL);

    bmg160_set_soft_rst();
    bma2x2_soft_rst();
    HAL_Delay(50);

    BMI055_BMGInit();
    BMI055_BMAInit();

    Spi_Init(Const_BMI055_SPI_HANDLER);
    bmi055->init_flag = 1;

    if ((bmg160.chip_id == 0x0F) && (bma2x2.chip_id == 0xFA)) {
        bmi055->state = BMI055_STATE_CONNECTED;
        return 1;
    } else {
        bmi055->state = BMI055_STATE_LOST;
        return 0;
    }
}

/**
 * @brief      Get pinter to the BMI055 data object
 * @param      NULL
 * @retval     Pointer to BMI055 data object
 */
BMI055_BMI055DataTypeDef* BMI055_GetBMI055DataPtr() {
    return &BMI055_BMI055Data;
}

/**
 * @brief      Judge bmi055 offline
 * @param      NULL
 * @retval     Offline or not��1 is offline��0 is not��
 */
uint8_t BMI055_IsBMI055Offline() {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();

    uint32_t now = HAL_GetTick();
    if ((now - bmi055->last_update_time) > Const_BMI055_OFFLINE_TIME)
        bmi055->state = BMI055_STATE_LOST;
    return bmi055->state == BMI055_STATE_LOST;
}

/**
 * @brief      Gyro bmg160 data object
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMGInit() {
    int32_t com_rslt;

    uint8_t v_bw_uint8_t = BMG160_BW_47_HZ;
    uint8_t v_range_uint8_t = BMG160_RANGE_2000;

    bmg160.bus_write = BMI055_BMGSpiBusWrite;
    bmg160.bus_read = BMI055_BMGSpiBusRead;
    bmg160.delay_msec = BMI055_usDelay;

    com_rslt = bmg160_init(&bmg160);
    HAL_Delay(2);

    bmg160_set_bw(v_bw_uint8_t);
    bmg160_set_range_reg(v_range_uint8_t);
    HAL_Delay(5);

    bmg160_set_intr_data(BMG160_INTR1_DATA, BMG160_ENABLE);
    bmg160_set_intr_output_type(BMG160_INTR1, 0x00);
    bmg160_set_intr_level(BMG160_INTR1, 0x01);
    HAL_Delay(50);

    bmg160_set_data_enable(BMG160_ENABLE);
    HAL_Delay(10);

    return com_rslt;
}

/**
 * @brief      Gyro bmg160 data bus write
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMGSpiBusWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    int32_t iError = BMG160_INIT_VALUE;
    uint8_t array[SPI_BUFFER_LEN * C_BMG160_TWO_U8X];
    uint8_t stringpos = BMG160_INIT_VALUE;

    for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos * C_BMG160_TWO_U8X] = (reg_addr++) & MASK_DATA3;
        array[stringpos * C_BMG160_TWO_U8X + BMG160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data + stringpos);
    }

    GPIO_Reset(CS_GYRO);
    for (uint8_t i = 0; i < cnt; i++) {
        Spi_TransmitData(Const_BMI055_SPI_HANDLER, &array[i * 2], 1);
        Spi_TransmitData(Const_BMI055_SPI_HANDLER, &array[i * 2 + 1], 1);
    }
    GPIO_Set(CS_GYRO);

    return (int8_t)iError;
}

/**
 * @brief      Gyro bmg160 data bus read
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMGSpiBusRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    int32_t iError = BMG160_INIT_VALUE;
    uint8_t array[SPI_BUFFER_LEN] = {MASK_DATA1};
    uint8_t stringpos;

    array[BMG160_INIT_VALUE] = reg_addr | MASK_DATA2;

    GPIO_Reset(CS_GYRO);
    Spi_SwapData(Const_BMI055_SPI_HANDLER, array, array, cnt + 1);
    GPIO_Set(CS_GYRO);

    for (stringpos = BMG160_INIT_VALUE; stringpos < cnt; stringpos++) {
        *(reg_data + stringpos) = array[stringpos + BMG160_GEN_READ_WRITE_DATA_LENGTH];
    }

    return (int8_t)iError;
}

/**
 * @brief      Gyro bmg160 read over and decode data
 * @param      NUULL
 * @retval     NUL
 */
uint32_t gyro_cont = 0;
float gyro_rate;
void BMI055_GyroReadOver(uint8_t* rx_buf) {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();
    if (gyro_cont == 0)
        gyro_cont = HAL_GetTick();

    gyro_cont++;
    gyro_rate = gyro_cont * 1000 / HAL_GetTick();
    if (bmi055->init_flag != 1)
        return;
    bmg160_get_data_xyz_over(&bmi055->gyr, (rx_buf + 1));
    bmi055->gyro.x = bmi055->gyr.datax * BMI055_GYRO_2000_SEN;
    bmi055->gyro.y = bmi055->gyr.datay * BMI055_GYRO_2000_SEN;
    bmi055->gyro.z = bmi055->gyr.dataz * BMI055_GYRO_2000_SEN;
}

/**
 * @brief      Gyro bmg160 read and decode data
 * @param      NUULL
 * @retval     NUL
 */
void BMI055_GyroRead() {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();

    if (bmi055->init_flag != 1)
        return;
    bmg160_get_data_xyz(&bmi055->gyr);

    bmi055->gyro.x = bmi055->gyr.datax * BMI055_GYRO_2000_SEN;
    bmi055->gyro.y = bmi055->gyr.datay * BMI055_GYRO_2000_SEN;
    bmi055->gyro.z = bmi055->gyr.dataz * BMI055_GYRO_2000_SEN;
}

/**
 * @brief      Accel bma2x2 init
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMAInit() {
    int32_t com_rslt = ERROR;
    uint8_t bw_value_uint8_t = BMA2x2_BW_250HZ; /* set bandwidth of 1000 Hz*/

    bma2x2.bus_write = BMI055_BMASpiBusWrite;
    bma2x2.bus_read = BMI055_BMASpiBusRead;
    bma2x2.delay_msec = BMI055_usDelay;

    com_rslt = bma2x2_init(&bma2x2);
    HAL_Delay(10);

    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
    HAL_Delay(50);

    bma2x2_set_bw(bw_value_uint8_t);

    bma2x2_set_new_data(BMA2x2_INTR1_NEWDATA, INTR_ENABLE);
    HAL_Delay(10);
    bma2x2_set_intr_enable(BMA2x2_DATA_ENABLE, INTR_ENABLE);
    HAL_Delay(10);

    return com_rslt;
}

/**
 * @brief      Accel bma2x2 bus write
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMASpiBusWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    int32_t iError = BMA2x2_INIT_VALUE;
    uint8_t array[SPI_BUFFER_LEN * 2];
    uint8_t stringpos = BMA2x2_INIT_VALUE;

    for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
        array[stringpos * 2] = (reg_addr++) &
                               BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE;
        array[stringpos * 2 + BMA2x2_BUS_READ_WRITE_ARRAY_INDEX] =
            *(reg_data + stringpos);
    }

    GPIO_Reset(CS_ACCEL);
    Spi_TransmitData(Const_BMI055_SPI_HANDLER, array, cnt * 2);
    GPIO_Set(CS_ACCEL);

    return (int8_t)iError;
}

/**
 * @brief      Accel bma2x2 bus read
 * @param      NUULL
 * @retval     NUL
 */
int8_t BMI055_BMASpiBusRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint8_t cnt) {
    int32_t iError = BMA2x2_INIT_VALUE;
    uint8_t array_read[SPI_BUFFER_LEN];
    uint8_t stringpos;
    array_read[BMA2x2_INIT_VALUE] = reg_addr | BMA2x2_SPI_BUS_READ_CONTROL_BYTE;

    GPIO_Reset(CS_ACCEL);
    Spi_SwapData(Const_BMI055_SPI_HANDLER, array_read, array_read, cnt + 1);
    GPIO_Set(CS_ACCEL);

    for (stringpos = BMA2x2_INIT_VALUE; stringpos < cnt; stringpos++) {
        *(reg_data + stringpos) = array_read[stringpos +
                                             BMA2x2_BUS_READ_WRITE_ARRAY_INDEX];
    }
    return (int8_t)iError;
}

/**
 * @brief      Accel bma2x2 read over and decode data
 * @param      NUULL
 * @retval     NUL
 */
uint32_t acc_cont = 0;
float acc_rate;
void BMI055_AccelReadOver(uint8_t* rx_buf) {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();
    if (acc_cont == 0)
        acc_cont = HAL_GetTick();

    acc_cont++;
    acc_rate = acc_cont * 1000 / HAL_GetTick();
    if (bmi055->init_flag != 1)
        return;
    bma2x2_read_accel_xyzt_over(&bmi055->acc, (rx_buf + 1));
    bmi055->accel.x = bmi055->acc.x * BMI055_ACCEL_2G_SEN;
    bmi055->accel.y = bmi055->acc.y * BMI055_ACCEL_2G_SEN;
    bmi055->accel.z = bmi055->acc.z * BMI055_ACCEL_2G_SEN;
    bmi055->temperature = bmi055->acc.temp * BMI055_TEMP_SEN + BMI055_TEMP_CONST;
}

/**
 * @brief      Accel bma2x2 read and decode data
 * @param      NUULL
 * @retval     NUL
 */
void BMI055_AccelRead() {
    BMI055_BMI055DataTypeDef* bmi055 = BMI055_GetBMI055DataPtr();

    if (bmi055->init_flag != 1)
        return;
    bma2x2_read_accel_xyzt(&bmi055->acc);
    bmi055->accel.x = bmi055->acc.x * BMI055_ACCEL_2G_SEN;
    bmi055->accel.y = bmi055->acc.y * BMI055_ACCEL_2G_SEN;
    bmi055->accel.z = bmi055->acc.z * BMI055_ACCEL_2G_SEN;
    bmi055->temperature = bmi055->acc.temp * BMI055_TEMP_SEN + BMI055_TEMP_CONST;
}

/**
 * @brief      BMI055 for us delay
 * @param      NUULL
 * @retval     NUL
 */
void BMI055_usDelay(uint32_t msek) {
    uint32_t Delay = msek * 168 / 4;
    do {
        __NOP();
    } while (Delay--);
}

#endif

#endif
