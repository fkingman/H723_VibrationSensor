#include "app_acq.h"
#include "tim.h"      
#include <string.h>

extern uint16_t wave_points;

static uint32_t s_freq_hz  = 0;
static uint16_t s_points   = 0;

/* ---- 配置单个定时器 ---- */
static HAL_StatusTypeDef Config_Timer(TIM_HandleTypeDef *htim, uint32_t freq_hz)
{
    if (freq_hz == 0) return HAL_ERROR;

    uint32_t timclk = ACQ_TIM_CLK_HZ; // 200MHz
    uint32_t psc = 0;
    uint32_t arr = 0;

    // 1. 计算理想的总分频值 (Total Divider)
    // Target_Freq = TIM_CLK / (PSC+1) / (ARR+1)
    // Total_Div = (PSC+1) * (ARR+1) = TIM_CLK / Freq
    uint64_t total_div = ((uint64_t)timclk + freq_hz/2) / freq_hz; 
    
    // 2. 动态计算 PSC 和 ARR
    // 目标是让 ARR 不超过寄存器上限 (TIM2是32位, TIM3是16位)
    // 为了安全，统一按 16 位限制计算 (兼容 TIM3)，或者判断 Instance
    uint32_t max_arr = 0xFFFF; // 默认按16位限制，保证TIM3不溢出
    
    if (htim->Instance == TIM2) {
        max_arr = 0xFFFFFFFF; // TIM2 是32位
    }

    // 如果 total_div 超过了当前的 ARR 上限，就需要增加 PSC
    // (PSC+1) = Total_Div / (ARR_Max + 1)
		uint64_t limit_div = (uint64_t)max_arr + 1;
    uint32_t psc_plus_1 = (uint32_t)(total_div / limit_div) + 1;
    
    psc = psc_plus_1 - 1;
    // 限制 PSC 也是 16 位的
    if (psc > 0xFFFF) psc = 0xFFFF; 

    // 3. 反推 ARR
    // (ARR+1) = Total_Div / (PSC+1)
    uint64_t arr_plus_1 = total_div / (psc + 1);
    arr = (uint32_t)arr_plus_1 - 1;

    // 4. 写入寄存器
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_PRESCALER(htim, psc);
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COUNTER(htim, 0);
    htim->Instance->EGR = TIM_EGR_UG; // 刷新影子寄存器
    __HAL_TIM_ENABLE(htim);
    
    return HAL_OK;
}

/* ---- 对外API ---- */
HAL_StatusTypeDef ACQ_Init(uint32_t freq_hz, uint16_t points)
{
    s_points = points ? points : 1;
    wave_points = s_points;

    s_freq_hz = freq_hz ? freq_hz : 25600u; // 默认值

    // 同时初始化两个定时器
    HAL_StatusTypeDef st1 = Config_Timer(&htim2, s_freq_hz);
    HAL_StatusTypeDef st2 = Config_Timer(&htim3, s_freq_hz);

    if (st1 != HAL_OK || st2 != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef ACQ_SetFreqHz(uint32_t freq_hz)
{
    if (freq_hz == 0) return HAL_ERROR;
    
    // 同时更新两个定时器
    HAL_StatusTypeDef st1 = Config_Timer(&htim2, freq_hz);
    HAL_StatusTypeDef st2 = Config_Timer(&htim3, freq_hz);

    if (st1 == HAL_OK && st2 == HAL_OK) {
        s_freq_hz = freq_hz;
        return HAL_OK;
    }
    return HAL_ERROR;
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
    // 两个都要启动
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);
    return HAL_OK;
}

HAL_StatusTypeDef ACQ_TimerStop(void)
{
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop(&htim3);
    return HAL_OK;
}