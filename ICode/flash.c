// flash.c
#include "flash.h"
#include <string.h>
#include "bytes.h"

/* 独立的 CRC16(Modbus)，避免与上层耦合 */
static uint16_t crc16_modbus_local(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void cfg_default(flash_dev_cfg_t* o){
    memset(o, 0xFF, sizeof(*o));
    o->magic   = FLASH_CFG_MAGIC;
    o->version = 2;
    o->addr    = FLASH_CFG_DEFAULT_ADDR;
    o->samp_freq_hz = FLASH_CFG_DEFAULT_FREQ;
    o->points  = FLASH_CFG_DEFAULT_POINTS;
}

void Flash_ReadConfig(uint8_t* out_addr, uint16_t* out_freq, uint16_t* out_points)
{
    const flash_dev_cfg_t *p = (const flash_dev_cfg_t*)FLASH_CFG_BASE_ADDR;

    uint8_t  addr  = FLASH_CFG_DEFAULT_ADDR;
    uint16_t freq  = FLASH_CFG_DEFAULT_FREQ;
    uint16_t points= FLASH_CFG_DEFAULT_POINTS;

    if (p->magic == FLASH_CFG_MAGIC) {
        uint16_t crc = crc16_modbus_local((const uint8_t*)p, sizeof(flash_dev_cfg_t)-2);
        if (crc == p->crc) {
            addr = (p->addr == 0xFF) ? FLASH_CFG_DEFAULT_ADDR : p->addr;
            if (p->samp_freq_hz != 0xFFFFu) freq   = p->samp_freq_hz;
            if (p->points       != 0xFFFFu)     points = p->points;
        }
    }
    if (out_addr)   *out_addr   = addr;
    if (out_freq)   *out_freq   = freq;
    if (out_points) *out_points = points;
}

static HAL_StatusTypeDef cfg_program(const flash_dev_cfg_t* in)
{
    HAL_StatusTypeDef st;
    uint32_t err = 0;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef ei = {0};
    ei.TypeErase = FLASH_TYPEERASE_SECTORS;
    ei.Banks     = FLASH_CFG_BANK;
    ei.Sector    = FLASH_CFG_SECTOR;
    ei.NbSectors = 1;
    st = HAL_FLASHEx_Erase(&ei, &err);
    if (st != HAL_OK) { HAL_FLASH_Lock(); return st; }

    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, FLASH_CFG_BASE_ADDR, (uint32_t)in);
    HAL_FLASH_Lock();
    return st;
}

HAL_StatusTypeDef Flash_WriteConfig(uint8_t addr, uint16_t freq, uint16_t points)
{
    flash_dev_cfg_t w;
    cfg_default(&w);         // 先用默认填充，再覆盖传入字段
    w.addr = addr;
    w.samp_freq_hz = freq;
    w.points = points;
    w.crc  = crc16_modbus_local((uint8_t*)&w, sizeof(w)-2);
    return cfg_program(&w);
}

uint8_t Flash_ReadDeviceAddr(void){
    uint8_t a; Flash_ReadConfig(&a, NULL, NULL); return a;
}

HAL_StatusTypeDef Flash_WriteDeviceAddr(uint8_t new_addr){
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    a = new_addr;
    return Flash_WriteConfig(a, f, p);
}

/* 便捷更新 */
HAL_StatusTypeDef Flash_UpdateFreq(uint16_t new_freq)
{
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    f = new_freq;
    return Flash_WriteConfig(a, f, p);
}

HAL_StatusTypeDef Flash_UpdatePoints(uint16_t new_points)
{
    uint8_t a; uint16_t f; uint16_t p;
    Flash_ReadConfig(&a, &f, &p);
    p = new_points;
    return Flash_WriteConfig(a, f, p);
}