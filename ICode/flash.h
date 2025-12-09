// flash.h
//#pragma once
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include <stdint.h>
#include <stdbool.h>


typedef struct __attribute__((packed,aligned(4))) {
    uint32_t magic;        // 0xA5A55A5A
    uint8_t  version;      // 
    uint8_t  addr;         // 设备地址
    uint16_t samp_freq_hz; // 
    uint16_t points;       // 
    uint8_t  rsv[20];      // 填满到 32B
    uint16_t crc;          // crc16(modbus) 覆盖前 30B
} flash_dev_cfg_t;

#define FLASH_CFG_MAGIC         0xA5A55A5Au
#define FLASH_CFG_DEFAULT_ADDR  0x00
#define FLASH_CFG_DEFAULT_FREQ    51200u       // 默认采样率
#define FLASH_CFG_DEFAULT_POINTS  4096u        // 默认点数


#define FLASH_CFG_BANK          FLASH_BANK_1        // 建议用空闲的 Bank
#define FLASH_CFG_SECTOR        FLASH_SECTOR_7      // 该 Bank 的最后一个扇区示例
#define FLASH_CFG_BASE_ADDR     ((uint32_t)0x080FFFE0u) // 扇区起始地址示例

#ifdef __cplusplus
extern "C" {
#endif

/* 读取全部配置。任意参数可传 NULL 表示不关心 */
void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points);

/* 写入全部配置（一次擦写）。任意字段填写其当前值避免覆盖 */
HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points);

/* 兼容旧接口（地址读写） */
uint8_t Flash_ReadDeviceAddr(void);
HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr);

/* 便捷更新（只改一个字段，内部会读-改-写） */
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq);
HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points);

#ifdef __cplusplus
}
#endif
