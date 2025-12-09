#ifndef __DS18B20_H__
#define __DS18B20_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "delay.h"

/* ==== 根据硬件修改：数据脚所在端口与引脚 ==== */
#ifndef DS18B20_PORT
#define DS18B20_PORT   GPIOB
#endif

#ifndef DS18B20_PIN
#define DS18B20_PIN    GPIO_PIN_8
#endif

/* ==== 公共 API ==== */

/* 初始化：开启端口时钟、配置默认空闲为输入上拉、初始化DWT延时。
   返回0表示OK，<0表示失败（一般不至于）。*/
int  DS18B20_Init(void);

/* 读取温度（℃）。timeout_ms 建议 1000（1s）。
   成功返回0，*out_celsius 写入结果；失败返回负数错误码（见下）。 */
int  DS18B20_ReadTemperature(float *out_celsius, uint32_t timeout_ms);

/* （可选）只启动转换，不读取结果；通常无需单独调用。 */
int  DS18B20_StartConversion(uint32_t timeout_ms);

/* 错误码 */
enum {
    DS18B20_OK                 = 0,
    DS18B20_ERR_NO_PRESENCE    = -1,  // 复位后无设备应答
    DS18B20_ERR_CONVERT_TO     = -2,  // 转换超时
    DS18B20_ERR_CRC            = -3,  // Scratchpad CRC 校验失败
    DS18B20_ERR_PARAM          = -4,  // 传参错误（NULL 等）
};

int DS18B20_ReadROM(uint8_t rom[8]);
short Get_Tempetature(void);
void Ds18b20_Init(void);
float Tempetature_Dis(void);
#endif /* __DS18B20_H__ */
