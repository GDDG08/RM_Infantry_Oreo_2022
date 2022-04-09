/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\i2c_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 19:56:26
 */

#include "i2c_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_I2C)

/**
 * @brief          Write data or command to i2c address
 * @param          hi2c: The i2c handle
 * @param          address: Address of sent device
 * @param          pData: To be sent data
 * @param          len: The data length
 * @retval         NULL
 */
void I2c_MasterSendMessage(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t* pData, uint16_t len) {
    if ((hi2c == NULL) || (pData == NULL)) {
        I2c_ErrorHandler(HAL_ERROR);
    }
    uint32_t ret = HAL_I2C_Master_Transmit(hi2c, address, pData, len, 1);
    if (ret != HAL_OK) {
        I2c_ErrorHandler(ret);
    }
}

/**
 * @brief          Write a byte to register by i2c
 * @param          hi2c: The i2c handle
 * @param          address: Address of sent device
 * @param          reg: Register address
 * @param          data: To be sent data
 * @retval         NULL
 */
void I2c_WriteSingleReg(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t reg, uint8_t data) {
    if (hi2c == NULL) {
        I2c_ErrorHandler(HAL_ERROR);
    }
    uint32_t ret = HAL_I2C_Mem_Write(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 1);
    if (ret != HAL_OK) {
        I2c_ErrorHandler(ret);
    }
}

/**
 * @brief          Write a data or command to i2c by register address
 * @param          hi2c: The i2c handle
 * @param          address: Address of sent device
 * @param          reg: Register address
 * @param          res: The returned register data
 * @retval         NULL
 */
void I2c_ReadSingleReg(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t reg, uint8_t* res) {
    if ((hi2c == NULL) || (res == NULL)) {
        I2c_ErrorHandler(HAL_ERROR);
    }
    uint32_t ret = HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, res, 1, 1);
    if (ret != HAL_OK) {
        I2c_ErrorHandler(ret);
    }
}

/**
 * @brief          Write multiple data or command to i2c by register address
 * @param          hi2c: The i2c handle
 * @param          address: Address of sent device
 * @param          reg: Register address
 * @param          len: The data length
 * @param          res: The returned register data
 * @retval         NULL
 */
void I2c_ReadMuliReg(I2C_HandleTypeDef* hi2c, uint16_t address, uint16_t reg, uint8_t len, uint8_t* res) {
    if ((hi2c == NULL) || (res == NULL)) {
        I2c_ErrorHandler(HAL_ERROR);
    }
    uint32_t ret = HAL_I2C_Mem_Read(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, res, len, 1);
    if (ret != HAL_OK) {
        I2c_ErrorHandler(ret);
    }
}

/**
 * @brief          write multiple byte of ist8310 by i2c
 * @param          hi2c: The i2c handle
 * @param          address: Address of sent device
 * @param          reg: Register address
 * @param          pData: To be sent data
 * @param          len: The data length
 * @retval         NULL
 */
void I2c_WriteMuliReg(I2C_HandleTypeDef* hi2c, uint16_t address, uint8_t reg, uint8_t* pData, uint8_t len) {
    if ((hi2c == NULL) || (pData == NULL)) {
        I2c_ErrorHandler(HAL_ERROR);
    }
    uint32_t ret = HAL_I2C_Mem_Write(hi2c, address, reg, I2C_MEMADD_SIZE_8BIT, pData, len, 1);
    if (ret != HAL_OK) {
        I2c_ErrorHandler(ret);
    }
}

/**
 * @brief      I2C error handler
 * @param      ret: error data
 * @retval     NULL
 */
void I2c_ErrorHandler(uint32_t ret) {
    while (1) {
        return;
    }
}

#endif
