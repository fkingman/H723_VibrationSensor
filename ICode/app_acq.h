#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>

/* * 你的 STM32H7 主频 400MHz/2 = 200MHz (sysclk)
 * APB1 分频 = 2 -> APB1 = 100MHz
 * TIM2/3 在 APB1，时钟 x2 -> 200MHz
 */
#ifndef ACQ_TIM_CLK_HZ
#define ACQ_TIM_CLK_HZ   (200000000u) 
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 上电：应用频率与点数（会同时配置 TIM2 和 TIM3） */
HAL_StatusTypeDef ACQ_Init(uint32_t freq_hz, uint16_t points);

/* 运行期：设置采样频率（Hz），立即生效 */
HAL_StatusTypeDef ACQ_SetFreqHz(uint32_t freq_hz);
uint32_t          ACQ_GetFreqHz(void);

/* 运行期：设置/获取采样点数 */
void              ACQ_SetPoints(uint16_t points);
uint16_t          ACQ_GetPoints(void);

/* 启动/停止定时器（TIM2 和 TIM3 同步操作） */
HAL_StatusTypeDef ACQ_TimerStart(void);
HAL_StatusTypeDef ACQ_TimerStop(void);

#ifdef __cplusplus
}
#endif