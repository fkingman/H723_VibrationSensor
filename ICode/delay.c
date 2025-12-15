#include "delay.h"

void Delay_Init(void)
{
    /* 1. 开启 CoreDebug 中的 DWT 追踪功能 (TRCENA) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
    
    /* 2. 清空计数器 CYCCNT */
    DWT->CYCCNT = 0;
    
    /* 3. 开启 CYCCNT 计数功能 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start_tick = DWT->CYCCNT;
    
    /* 400MHz 主频下，1us = 400 个 Tick */
    /* 使用 SystemCoreClock 变量可以自适应频率变化，或者直接写 400 */
    uint32_t wait_ticks = us * (SystemCoreClock / 1000000U); 

    /* 循环等待直到计数差值达到目标 ticks (自动处理溢出) */
    while ((DWT->CYCCNT - start_tick) < wait_ticks);
}