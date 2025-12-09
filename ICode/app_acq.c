#include "app_acq.h"
#include "tim.h"      
#include <string.h>

extern uint16_t wave_points;

static uint32_t s_freq_hz  = 0;
static uint16_t s_points   = 0;

/* ---- 计算并写入TIM2寄存器（PSC固定=ACQ_TIM_PSC_DEFAULT） ---- */
static HAL_StatusTypeDef ACQ_ReprogramTimer(uint32_t freq_hz)
{
    if (freq_hz == 0) return HAL_ERROR;

    const uint32_t timclk = (uint32_t)ACQ_TIM_CLK_HZ;
    const uint32_t psc    = (uint32_t)ACQ_TIM_PSC_DEFAULT;   // 1 → 实际分频=2

    /* 目标总分频 D ≈ timclk / freq；四舍五入后求 ARR */
    uint64_t D = ((uint64_t)timclk + (uint64_t)freq_hz/2) / (uint64_t)freq_hz;
    if (D < 2) D = 2;  // 防止 ARR 变成 0xFFFFFFFF下溢

    uint64_t arr_plus_1 = (D + psc) / (psc + 1);
    if (arr_plus_1 < 1) arr_plus_1 = 1;
    uint32_t arr = (uint32_t)arr_plus_1 - 1;

    __HAL_TIM_DISABLE(&ACQ_TIM_HANDLE);
    __HAL_TIM_SET_PRESCALER(&ACQ_TIM_HANDLE, psc);
    __HAL_TIM_SET_AUTORELOAD(&ACQ_TIM_HANDLE, arr);
    __HAL_TIM_SET_COUNTER(&ACQ_TIM_HANDLE, 0);
    ACQ_TIM_HANDLE.Instance->EGR = TIM_EGR_UG; // 刷新影子寄存器
    __HAL_TIM_ENABLE(&ACQ_TIM_HANDLE);

    return HAL_OK;
}

/* ---- 对外API ---- */
HAL_StatusTypeDef ACQ_Init(uint32_t freq_hz, uint16_t points)
{
    s_points = points ? points : 1;
    wave_points = s_points;             // 同步给你的波形模块

    s_freq_hz = freq_hz ? freq_hz : 51200u;  // 兜底（与默认相近）
    return ACQ_ReprogramTimer(s_freq_hz);
}

HAL_StatusTypeDef ACQ_SetFreqHz(uint32_t freq_hz)
{
    if (freq_hz == 0) return HAL_ERROR;
    HAL_StatusTypeDef st = ACQ_ReprogramTimer(freq_hz);
    if (st == HAL_OK) s_freq_hz = freq_hz;
    return st;
}

uint32_t ACQ_GetFreqHz(void)
{
    return s_freq_hz;
}

void ACQ_SetPoints(uint16_t points)
{
    s_points = points ? points : 1;
    wave_points = s_points;
}

uint16_t ACQ_GetPoints(void)
{
    return s_points;
}

HAL_StatusTypeDef ACQ_TimerStart(void)
{
    return HAL_TIM_Base_Start(&ACQ_TIM_HANDLE);
}

HAL_StatusTypeDef ACQ_TimerStop(void)
{
    return HAL_TIM_Base_Stop(&ACQ_TIM_HANDLE);
}
