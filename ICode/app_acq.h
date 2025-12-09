#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifndef ACQ_TIM_HANDLE
#define ACQ_TIM_HANDLE   htim2       
#endif

#ifndef ACQ_TIM_CLK_HZ
#define ACQ_TIM_CLK_HZ   (64000000u) // 你的TIM2时钟：64 MHz
#endif

/* 默认把PSC固定为1（实际分频=PSC+1=2），与Cube默认一致 */
#ifndef ACQ_TIM_PSC_DEFAULT
#define ACQ_TIM_PSC_DEFAULT  (1u)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 上电：应用频率与点数（会重配置TIM2并把点数写到全局） */
HAL_StatusTypeDef ACQ_Init(uint32_t freq_hz, uint16_t points);

/* 运行期：设置采样频率（Hz），立即生效（重载PSC/ARR） */
HAL_StatusTypeDef ACQ_SetFreqHz(uint32_t freq_hz);
uint32_t          ACQ_GetFreqHz(void);

/* 运行期：设置/获取采样点数（影响你的波形发送） */
void              ACQ_SetPoints(uint16_t points);
uint16_t          ACQ_GetPoints(void);

/* 如果需要单独启停定时器（一般不必调用；Init里已使能） */
HAL_StatusTypeDef ACQ_TimerStart(void);
HAL_StatusTypeDef ACQ_TimerStop(void);

#ifdef __cplusplus
}
#endif
