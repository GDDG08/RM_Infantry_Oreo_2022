/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Inc\Peripheral\magnetic_periph.h
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:52:01
 */

#ifndef MAG_PERIPH_H
#define MAG_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "configure.h"

#if __FN_IF_ENABLE(__FN_PERIPH_MAG)

#include "i2c_util.h"
#include "gpio_util.h"

#define MAG_SEN 0.3f                // change to uT
#define IST8310_CHIP_ID 0x00        // IST8310 who am I �Ĵ���
#define IST8310_CHIP_ID_VALUE 0x10  // ID
#define IST8310_WRITE_REG_NUM 4
#define IST8310_IIC_ADDRESS (0x0E << 1)
#define IST8310_IIC_READ_MSB (0x80)
#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef enum {
    MAG_STATE_NULL = 0,
    MAG_STATE_CONNECTED = 1,
    MAG_STATE_LOST = 2,
    MAG_STATE_ERROR = 3,
    MAG_STATE_PENDING = 4
} MAG_MAGStateEnum;

typedef struct {
    MAG_MAGStateEnum state;
    uint32_t last_update_time;
    int error;
    float mag_x,
        mag_y,
        mag_z;
} MAG_MAGDataTypeDef;

void MAG_Init(void);
void MAG_ResetMAGData(void);
MAG_MAGDataTypeDef* MAG_GetMAGDataPtr(void);
uint8_t MAG_IsMAGOffline(void);
void MAG_MAGUpdate(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
