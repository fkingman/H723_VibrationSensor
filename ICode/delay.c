#include "main.h"
/*
优点：全系列通用，只需要将宏定义CPU_FREQUENCY_MHZ根据时钟主频修改即可。
缺点：系统滴答定时器是HAL库初始化的，且必须有HAL库初始化。
*/
#define CPU_FREQUENCY_MHZ    64		// STM32时钟主频
void _delay_us(__IO uint32_t delay)
{
    int last, curr, val;
    int temp;

    while (delay != 0)
    {
        temp = delay > 900 ? 900 : delay;
        last = SysTick->VAL;
        curr = last - CPU_FREQUENCY_MHZ * temp;
        if (curr >= 0)
        {
            do
            {
                val = SysTick->VAL;
            }
            while ((val < last) && (val >= curr));
        }
        else
        {
            curr += CPU_FREQUENCY_MHZ * 1000;
            do
            {
                val = SysTick->VAL;
            }
            while ((val <= last) || (val > curr));
        }
        delay -= temp;
    }
}
/*
优点： 实现简单，如果是F1系列，HAL_RCC_GetHCLKFreq()获取的值是72000000，此方式经过测试还是比较准的，如果不考虑通用性，F1系列建议使用此种方式。

缺点： 只适用F1系列72M主频。
*/
void delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
    {
        ;
    }
}

