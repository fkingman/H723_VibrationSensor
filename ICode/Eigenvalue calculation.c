#include "Eigenvalue calculation.h"
#include "arm_const_structs.h"
#include <string.h>  //  memcpy

static arm_rfft_fast_instance_f32 S_rfft;
static float32_t fftBuf[FFT_N_Z * 2]; // 复数运算缓冲区 (Z轴最长)

float g_z_offset_g  = 0.0f;   // 0g 偏移
float fr = 50;

//#define CFFT (&arm_cfft_sR_f32_len1024)

//计算初始化函数
void Calc_Init(void)
{
    arm_rfft_fast_init_f32(&S_rfft, FFT_N_Z);      // 初始化 FFT 结构体   
    //Vib_Filter_Init();     // 初始化滤波器
}

/*void Vib_Filter_Init(void)
{
    arm_biquad_cascade_df1_init_f32(&IIR_HP,    1, (float32_t*)hp10HzCoeff,    hpState);
    arm_biquad_cascade_df1_init_f32(&IIR_NOTCH, 1, (float32_t*)notch50HzCoeff, notchState);
}*/

//私有函数
static inline float Zcode_to_g(uint16_t code) 
{
    float vin = (float)code * Z_REF_VOLTAGE / Z_ADC_RESOLUTION;
    float g_raw = (vin - Z_REF_VOLTAGE_BIAS) / Z_SENSITIVITY;
		return g_raw - g_z_offset_g;     // 减掉偏移
}
static inline float XYcode_to_g(uint16_t code)
{
    float vin = (float)code * XY_REF_VOLTAGE /XY_ADC_RESOLUTION;
    return (vin - XY_REF_VOLTAGE_BIAS) / XY_SENSITIVITY;
}

//(uint16 -> float)
void Eigen_Separate_And_Convert(uint16_t *pZBuf, uint16_t *pXYBuf)
{
    // --- 处理 Z 轴 ---
    for (uint32_t i = 0; i < FFT_N_Z; i++) {
        g_data_z[i] = Zcode_to_g(pZBuf[i]);
    }

    // --- 处理 XY 轴 (交错拆分) ---
    for (uint32_t i = 0; i < FFT_N_XY; i++) {
        g_data_x[i] = XYcode_to_g(pXYBuf[2 * i]);
        g_data_y[i] = XYcode_to_g(pXYBuf[2 * i + 1]);
    }
}
//去直流
static void Remove_DC(float *data, uint32_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    float mean = sum / (float)len;

    for (uint32_t i = 0; i < len; i++) {
        data[i] -= mean;
    }
}

void Z_Calib_Z_Upright_Neg1G(float *gBuf, uint32_t N)
{
    float sum_g = 0.0f;

    for (uint32_t i = 0; i < N; ++i)
    {
        sum_g += gBuf[i];
    }

    float mean_current = sum_g / (float)N;

    const float target_g = -1.0f;
  
    g_z_offset_g += (mean_current - target_g);
}
/*
static inline CFFT_PTR_T pick_cfft_u32(uint32_t N)
{
    switch (N) {
        case 16:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len16;
        case 32:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len32;
        case 64:   return (CFFT_PTR_T)&arm_cfft_sR_f32_len64;
        case 128:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len128;
        case 256:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len256;
        case 512:  return (CFFT_PTR_T)&arm_cfft_sR_f32_len512;
        case 1024: return (CFFT_PTR_T)&arm_cfft_sR_f32_len1024;
        case 2048: return (CFFT_PTR_T)&arm_cfft_sR_f32_len2048;
        case 4096: return (CFFT_PTR_T)&arm_cfft_sR_f32_len4096;
        default:   return (CFFT_PTR_T)0;
    }
}*/
//时域特征计算 (Mean, RMS, PP, Kurt)
void Calc_TimeDomain_Only(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    float32_t sum = 0.0f;
    float32_t sumSq = 0.0f;
    float32_t minVal = data[0];
    float32_t maxVal = data[0];
    float32_t mean, rms, pp, kurt;

    for (uint32_t i = 0; i < len; i++) {
        float32_t val = data[i];
        sum += val;
        sumSq += val * val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    mean = sum / (float32_t)len;
    rms = sqrtf(sumSq / (float32_t)len); // 这里是包含直流分量的 RMS
    pp = maxVal - minVal;

    // 第二遍循环：计算峭度 (Kurtosis) 需要中心矩
    // Kurtosis = E[(x - u)^4] / (sigma^4)
    float32_t m2 = 0.0f; // 二阶中心矩 (方差 * N)
    float32_t m4 = 0.0f; // 四阶中心矩

    for (uint32_t i = 0; i < len; i++) {
        float32_t diff = data[i] - mean;
        float32_t diff2 = diff * diff;
        m2 += diff2;
        m4 += diff2 * diff2;
    }
    if (m2 < 1e-9f) {
        kurt = 0.0f;
    } else {
        // 公式调整：N * m4 / (m2^2)
        kurt = ((float32_t)len * m4) / (m2 * m2);
    }
    result->mean = mean;
    //result->rms  = rms;
    result->pp   = pp;
    result->kurt = kurt;
}
//积分速度mm/s
static void Integrate_Acc_To_Vel(float *data, uint32_t len)
{
    uint16_t freq = g_cfg_freq_hz; // 获取当前采样率
    if (freq == 0) return;

    float dt = 1.0f / (float)freq;
    float vel = 0.0f;
    float val_prev = data[0]; 
    
    // 重力加速度常数: 1g ≈ 9806.65 mm/s²
    const float G_TO_MM_S2 = 9806.65f; 

    for (uint32_t i = 0; i < len; i++) {
        float val_curr = data[i];
        
        // 梯形积分公式
        vel += (val_prev + val_curr) * 0.5f * dt * G_TO_MM_S2;
        val_prev = val_curr;
        
        data[i] = vel;
    }

    // 积分后必须再次去直流，消除积分漂移
    Remove_DC(data, len);
}
//速度rms
static float Calc_RMS_Only(float *data, uint32_t len, AxisFeatureValue *result)
{
    float sumSq = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        sumSq += data[i] * data[i];
    }
    result->rms = sqrtf(sumSq / (float)len);
}

//频域特征计算 (Z轴: PeakFreq, PeakAmp, 2xAmp)
// 替换整个 Calc_FreqDomain_Z 函数
void Calc_FreqDomain_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    uint16_t current_fs = g_cfg_freq_hz;
    
    // 1. 使用 RFFT (实数FFT)，直接使用实数输入，输出为压缩的复数格式
    arm_rfft_fast_f32(&S_rfft, data, fftBuf, 0);

    // 2. 计算幅值 (Modulus)
    // arm_rfft_fast_f32 的输出 fftBuf 布局如下：
    // [0]: 直流分量(DC)实部
    // [1]: 奈奎斯特分量实部
    // [2]: f1 实部, [3]: f1 虚部
    // [4]: f2 实部, [5]: f2 虚部 ...
    
    // 先计算从 index 2 开始的复数部分的模
    // 结果会直接存回 fftBuf[2], fftBuf[3]... 紧凑排列
    if (len > 2) {
        arm_cmplx_mag_f32(&fftBuf[2], &fftBuf[2], len / 2 - 1);
    }
    
    // 单独处理 DC 分量 (取绝对值)
    float32_t dc_val = fabsf(fftBuf[0]);
    
    // 3. 整理数组，使其变成标准的 [Amp_DC, Amp_f1, Amp_f2, ...]
    // 将计算好的交流幅值从 fftBuf[2] 搬移到 fftBuf[1]
    memmove(&fftBuf[1], &fftBuf[2], (len / 2 - 1) * sizeof(float32_t));
    fftBuf[0] = dc_val;
    
    // 4. 归一化
    // 实数 FFT 输出的幅值通常放大了，需要归一化
    fftBuf[0] /= (float32_t)len;            // 直流除以 N
    float32_t norm = 2.0f / (float32_t)len; // 交流除以 N/2
    
    for (uint32_t i = 1; i < len / 2; i++) {
        fftBuf[i] *= norm;
    }

    // 5. 寻找主峰 (逻辑保持不变，但循环上限确保正确)
    float32_t maxAmp = 0.0f;
    uint32_t maxIndex = 0;
    
    // 从索引 3 开始找，避开低频
    for (uint32_t i = 3; i < len / 2; i++) {
        if (fftBuf[i] > maxAmp) {
            maxAmp = fftBuf[i];
            maxIndex = i;
        }
    }
    
    // 计算物理频率
    float32_t freq_res = (float32_t)current_fs / (float32_t)len;
    float32_t peak_freq = (float32_t)maxIndex * freq_res;

    // 寻找 2x 主频幅值
    float32_t amp_2x = 0.0f;
    uint32_t index_2x = maxIndex * 2;
    
    if (index_2x < len / 2) { 
        amp_2x = fftBuf[index_2x];
    }

    result->peakFreq = peak_freq; 
    result->peakAmp  = maxAmp;    
    result->amp2x    = amp_2x;    
}

// 包络特征计算 (RFFT 优化版)
void Calc_Envelope_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    // 1. RFFT 正变换：计算 x(t) 的频谱
    // 输入：data (实数 x(t))
    // 输出：fftBuf (压缩格式的复数频谱)
    // S_rfft 已经在 Calc_Init 中初始化
    arm_rfft_fast_f32(&S_rfft, data, fftBuf, 0);

    // 2. 频域处理：构造希尔伯特变换的频谱
    // 希尔伯特变换在频域相当于所有正频率乘以 -j
    // RFFT 结果 fftBuf 布局：[DC, Nyquist, Re1, Im1, Re2, Im2, ...]
    
    // DC 和 Nyquist 分量的希尔伯特变换理论上为 0 (或不需要移相)
    // 这里简单清零以消除直流偏置影响
    fftBuf[0] = 0.0f; 
    fftBuf[1] = 0.0f;

    // 遍历所有交流分量 (k=1 到 N/2 - 1)
    // 索引从 2 开始，每次步进 2
    for (uint32_t i = 2; i < len; i += 2) {
        float32_t re = fftBuf[i];
        float32_t im = fftBuf[i+1];
        
        // 乘以 -j : (Re + jIm) * (-j) = Im - jRe
        fftBuf[i]     = im;   // 新实部 = 旧虚部
        fftBuf[i+1]   = -re;  // 新虚部 = -旧实部
    }

    // 3. RIFFT 逆变换：得到时域的希尔伯特变换信号 h(t)
    // 输入：fftBuf (修改后的频谱)
    // 输出：将结果存入 fftBuf 的后半段 (&fftBuf[len])，避免覆盖频谱
    // 注意：FFT/IFFT 后通常幅度会放大 N 倍，后面需要归一化
    arm_rfft_fast_f32(&S_rfft, fftBuf, &fftBuf[len], 1);

    // 4. 合成包络并计算特征值
    // Envelope = sqrt( x(t)^2 + h(t)^2 )
    float32_t sumSq = 0.0f;
    float32_t maxEnv = 0.0f;
    float32_t inv_len = 1.0f / (float32_t)len; // 用于 IFFT 结果的归一化

    for (uint32_t i = 0; i < len; i++) {
        float32_t x = data[i];                   // 原始信号 x(t)
        float32_t h = fftBuf[len + i] * inv_len; // 希尔伯特变换信号 h(t)，需归一化

        // 计算瞬时包络
        float32_t envVal = sqrtf(x * x + h * h);
        
        sumSq += envVal * envVal;
        if (envVal > maxEnv) maxEnv = envVal;
    }

    // 计算包络的有效值和峰值
    result->envelope_vrms = sqrtf(sumSq / (float32_t)len);
    result->envelope_peak = maxEnv;
}

	
void Process_Data(uint16_t *pZBuf, uint16_t *pXYBuf)
{	  
    Eigen_Separate_And_Convert(pZBuf, pXYBuf);

    Calc_TimeDomain_Only(g_data_x, FFT_N_XY, &X_data);
    if (FFT_N_XY <= FFT_N_Z * 2) { 
        memcpy(fftBuf, g_data_x, FFT_N_XY * sizeof(float)); 
        Remove_DC(fftBuf, FFT_N_XY);       
        Integrate_Acc_To_Vel(fftBuf, FFT_N_XY); 
        Calc_RMS_Only(fftBuf, FFT_N_XY, &X_data); 
    }

    Calc_TimeDomain_Only(g_data_y, FFT_N_XY, &Y_data);
    if (FFT_N_XY <= FFT_N_Z * 2) {
        memcpy(fftBuf, g_data_y, FFT_N_XY * sizeof(float));
        Remove_DC(fftBuf, FFT_N_XY);
        Integrate_Acc_To_Vel(fftBuf, FFT_N_XY);
        Calc_RMS_Only(fftBuf, FFT_N_XY, &Y_data); 
    }

    Calc_TimeDomain_Only(g_data_z, FFT_N_Z,  &Z_data);
    Calc_FreqDomain_Z(g_data_z, FFT_N_Z, &Z_data);
    Calc_Envelope_Z(g_data_z, FFT_N_Z, &Z_data);
    memcpy(fftBuf, g_data_z, FFT_N_Z * sizeof(float));
    Remove_DC(fftBuf, FFT_N_Z);
    Integrate_Acc_To_Vel(fftBuf, FFT_N_Z);
    Calc_RMS_Only(fftBuf, FFT_N_XY, &Z_data);

}






