#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include <stdint.h>
#include <stdbool.h>

/* ---- 扇区配置 (基于 H723) ---- */
#define FLASH_CFG_BANK          FLASH_BANK_1        
#define FLASH_CFG_SECTOR        FLASH_SECTOR_7      // 最后一个扇区
#define FLASH_CFG_BASE_ADDR     ((uint32_t)0x080E0000u) // Sector 7 起始地址

/* ---- 默认参数定义 ---- */
#define FLASH_CFG_MAGIC         0xA5A55A5Au
#define FLASH_CFG_DEFAULT_ADDR  0x00
#define FLASH_CFG_DEFAULT_FREQ  25600u
#define FLASH_CFG_DEFAULT_POINTS 4096u

/* ---- OTA 标志位定义 ---- */
#define OTA_FLAG_UPDATE_NEEDED  0x5A5A5A5A // 标志：需要升级

/* ---- 核心配置结构体 (32 Bytes) ---- */
typedef struct __attribute__((packed,aligned(4))) {
    uint32_t magic;        // 0-3: 0xA5A55A5A
    uint8_t  version;      // 4
    uint8_t  addr;         // 5: 设备地址
    uint16_t samp_freq_hz; // 6-7: 采样率
    uint16_t points;       // 8-9: 点数
    
    /* --- 新增 OTA 相关字段 (占用原保留区) --- */
    uint32_t ota_flag;     // 10-13: 升级标志
    uint32_t fw_len;       // 14-17: 固件长度
    uint32_t fw_crc;       // 18-21: 固件CRC (预留)
    uint8_t  rsv[8];       // 22-29: 剩余保留字节
    /* ------------------------------------ */

    uint16_t crc;          // 30-31: 覆盖前30字节的CRC16
} flash_dev_cfg_t;


#ifdef __cplusplus
extern "C" {
#endif

/* ==== 基础配置接口 (App用) ==== */

/* 读取配置 (任意参数传 NULL 表示不读取) */
void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points);

/* 写入配置 (会自动保留 OTA 标志位) */
HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points);

/* ==== OTA 专用接口 (Bootloader & App用) ==== */

/* 读取 OTA 信息 (Bootloader判断是否需要升级) */
void Flash_ReadOTAInfo(uint32_t* out_flag, uint32_t* out_len);

/* 设置 OTA 信息 (App下载完请求升级，或 Bootloader升级完清除标志) */
/* 这里的 flag 传入 OTA_FLAG_UPDATE_NEEDED 表示请求升级，传入 0 表示清除 */
HAL_StatusTypeDef Flash_SetOTAInfo(uint32_t flag, uint32_t len);


/* ==== 兼容旧代码的辅助接口 ==== */
uint8_t Flash_ReadDeviceAddr(void);
HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr);
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq);
HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_H__ */