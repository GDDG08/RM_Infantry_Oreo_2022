/*
 * @Project      : RM_Infantry_Oreo
 * @FilePath     : \Infantry_Oreo\Core\Src\Utility\flash_util.c
 * @Descripttion :
 * @Author       : GDDG08
 * @Date         : 2021-12-31 17:37:14
 * @LastEditors  : GDDG08
 * @LastEditTime : 2022-03-24 20:02:52
 */

#include "flash_util.h"

#if __FN_IF_ENABLE(__FN_UTIL_FLASH)

/**
 * @brief          Erase flash
 * @param          address: Flash address
 * @param          len: Page num
 * @retval         NULL
 */
void Flash_EraseAddress(uint32_t address, uint16_t page) {
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = Flash_GetSector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = page;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
 * @brief          Write data to one page of flash
 * @param          start_address: flash address
 * @param          buff: data point
 * @param          len: data num
 * @retval         success 1, fail 0
 */
uint8_t Flash_WriteSingleAddress(uint32_t start_address, uint32_t* buff, uint32_t len) {
    if (buff == NULL)
        return 0;

    Flash_EraseAddress(start_address, 1);

    static uint32_t uw_address;
    static uint32_t end_address;
    static uint32_t* data_buf;
    static uint32_t data_len;

    len = (len + 3) / 4;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    end_address = Flash_GetNextSector(start_address);
    data_buf = buff;
    data_len = 0;

    while (uw_address <= end_address) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uw_address, *data_buf) == HAL_OK) {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len) {
                break;
            }
        } else {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

/**
 * @brief          Write data to some pages of flash
 * @param          start_address: flash start address
 * @param          end_address: flash end address
 * @param          buff: data point
 * @param          len: data num
 * @retval         success 1, fail 0
 */
uint8_t Flash_WriteMuliAddress(uint32_t start_address, uint32_t end_address, uint32_t* buff, uint32_t len) {
    if (buff == NULL)
        return 0;
    uint32_t uw_address = 0;
    uint32_t* data_buf;
    uint32_t data_len;

    len = (len + 3) / 4;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    data_buf = buff;
    data_len = 0;
    while (uw_address <= end_address) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uw_address, *data_buf) == HAL_OK) {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len) {
                break;
            }
        } else {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}

/**
 * @brief          Read data for flash
 * @param[in]      address: flash address
 * @param[out]     buff: data point
 * @param[in]      len: data num
 * @retval         none
 */
void Flash_ReadData(uint32_t address, uint32_t* buff, uint32_t len) {
    memcpy(buff, (void*)address, len * 4);
}

/**
 * @brief          Get the sector number of flash
 * @param          address: Flash address
 * @retval         Sector number
 */
uint32_t Flash_GetSector(uint32_t address) {
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
        sector = FLASH_SECTOR_0;
    } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
        sector = FLASH_SECTOR_1;
    } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
        sector = FLASH_SECTOR_2;
    } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
        sector = FLASH_SECTOR_3;
    } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
        sector = FLASH_SECTOR_4;
    } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
        sector = FLASH_SECTOR_5;
    } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
        sector = FLASH_SECTOR_6;
    } else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7)) {
        sector = FLASH_SECTOR_7;
    } else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8)) {
        sector = FLASH_SECTOR_8;
    } else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9)) {
        sector = FLASH_SECTOR_9;
    } else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10)) {
        sector = FLASH_SECTOR_10;
    } else if ((address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_11)) {
        sector = FLASH_SECTOR_11;
    } else {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
 * @brief          Get the next page flash address
 * @param          address: Flash address
 * @retval         Next page flash address
 */
uint32_t Flash_GetNextSector(uint32_t address) {
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0)) {
        sector = ADDR_FLASH_SECTOR_1;
    } else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1)) {
        sector = ADDR_FLASH_SECTOR_2;
    } else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2)) {
        sector = ADDR_FLASH_SECTOR_3;
    } else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3)) {
        sector = ADDR_FLASH_SECTOR_4;
    } else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4)) {
        sector = ADDR_FLASH_SECTOR_5;
    } else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5)) {
        sector = ADDR_FLASH_SECTOR_6;
    } else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6)) {
        sector = ADDR_FLASH_SECTOR_7;
    } else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7)) {
        sector = ADDR_FLASH_SECTOR_8;
    } else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8)) {
        sector = ADDR_FLASH_SECTOR_9;
    } else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9)) {
        sector = ADDR_FLASH_SECTOR_10;
    } else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10)) {
        sector = ADDR_FLASH_SECTOR_11;
    } else if ((address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_11)) {
        sector = FLASH_END_ADDR;
    } else {
        sector = FLASH_END_ADDR;
    }

    return sector;
}

#endif
