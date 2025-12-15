#include "flash.h"
#include <string.h>

// 计算 CRC
static uint16_t crc16_modbus_local(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

// 内部函数：读取完整配置到结构体
static void Flash_ReadWholeConfig(flash_dev_cfg_t* cfg)
{
    const flash_dev_cfg_t *p = (const flash_dev_cfg_t*)FLASH_CFG_BASE_ADDR;
    
    // 检查 Magic 和 CRC
    if (p->magic == FLASH_CFG_MAGIC) {
        uint16_t crc = crc16_modbus_local((const uint8_t*)p, sizeof(flash_dev_cfg_t)-2);
        if (crc == p->crc) {
            // 数据有效，直接拷贝
            memcpy(cfg, p, sizeof(flash_dev_cfg_t));
            return;
        }
    }

    // 如果数据无效（比如第一次上电），填充默认值
    memset(cfg, 0xFF, sizeof(flash_dev_cfg_t));
    cfg->magic   = FLASH_CFG_MAGIC;
    cfg->version = 2;
    cfg->addr    = FLASH_CFG_DEFAULT_ADDR;
    cfg->samp_freq_hz = FLASH_CFG_DEFAULT_FREQ;
    cfg->points  = FLASH_CFG_DEFAULT_POINTS;
    cfg->ota_flag = 0; // 默认无升级
    cfg->fw_len   = 0;
}

// 内部函数：擦除并写入完整结构体
static HAL_StatusTypeDef Flash_ProgramWholeConfig(flash_dev_cfg_t* cfg)
{
    HAL_StatusTypeDef st;
    uint32_t err = 0;

    // 1. 计算新的 CRC
    cfg->crc = crc16_modbus_local((uint8_t*)cfg, sizeof(flash_dev_cfg_t)-2);

    // 2. 解锁
    HAL_FLASH_Unlock();

    // 3. 擦除 Sector 7 (128KB)
    FLASH_EraseInitTypeDef ei = {0};
    ei.TypeErase = FLASH_TYPEERASE_SECTORS;
    ei.Banks     = FLASH_CFG_BANK;
    ei.Sector    = FLASH_CFG_SECTOR;
    ei.NbSectors = 1;
    ei.VoltageRange = FLASH_VOLTAGE_RANGE_3; // H7 必须指定电压范围

    st = HAL_FLASHEx_Erase(&ei, &err);
    if (st != HAL_OK) { HAL_FLASH_Lock(); return st; }

    // 4. 写入 (按 256bit / 32Byte 写入)
    // H723 要求 32 字节对齐写入，我们的结构体正好是 32 字节
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_CFG_BASE_ADDR, (uint32_t)((uint32_t)cfg));
    
    HAL_FLASH_Lock();
    return st;
}

/* ---- 对外接口 1：只读取配置参数 ---- */
void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg);

    if (out_addr)   *out_addr   = cfg.addr;
    if (out_freq)   *out_freq   = cfg.samp_freq_hz;
    if (out_points) *out_points = cfg.points;
}

/* ---- 对外接口 2：只读取 OTA 信息 (Bootloader 用) ---- */
void Flash_ReadOTAInfo(uint32_t* out_flag, uint32_t* out_len)
{
    flash_dev_cfg_t cfg;
    Flash_ReadWholeConfig(&cfg);
    
    if (out_flag) *out_flag = cfg.ota_flag;
    if (out_len)  *out_len  = cfg.fw_len;
}

/* ---- 对外接口 3：更新配置 (自动保留 OTA 标志) ---- */
HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points)
{
    flash_dev_cfg_t cfg;
    
    // 1. 先读出旧数据 (这一步最关键！保留了 OTA 标志)
    Flash_ReadWholeConfig(&cfg);

    // 2. 修改想要改的参数
    cfg.addr   = addr;
    cfg.samp_freq_hz = freq;
    cfg.points = points;

    // 3. 擦除并写回
    return Flash_ProgramWholeConfig(&cfg);
}

/* ---- 对外接口 4：更新 OTA 标志 (自动保留配置) ---- */
HAL_StatusTypeDef Flash_SetOTAInfo(uint32_t flag, uint32_t len)
{
    flash_dev_cfg_t cfg;
    
    // 1. 先读出旧数据 (保留了设备地址等)
    Flash_ReadWholeConfig(&cfg);

    // 2. 修改 OTA 参数
    cfg.ota_flag = flag;
    cfg.fw_len   = len;

    // 3. 擦除并写回
    return Flash_ProgramWholeConfig(&cfg);
}

// 兼容旧接口
uint8_t Flash_ReadDeviceAddr(void) {
    uint8_t a; Flash_ReadConfig(&a, NULL, NULL); return a;
}
HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(new_addr, f, p); // 内部会自动处理 Read-Modify-Write
}
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(a, new_freq, p);
}
HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points) {
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    return Flash_WriteConfig(a, f, new_points);
}