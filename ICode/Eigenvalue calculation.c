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
    result->rms  = rms;
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
void Calc_FreqDomain_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    uint16_t current_fs = g_cfg_freq_hz;
    
    for (uint32_t i = 0; i < len; i++) 
    {
        fftBuf[2 * i]     = data[i];
        fftBuf[2 * i + 1] = 0.0f;
    }
    
    extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 0, 1);
    
    arm_cmplx_mag_f32(fftBuf, fftBuf, len);    // 4. 计算幅值 (Modulus)

    // 归一化
    // 直流分量除以 N，交流分量除以 N/2
    float32_t norm = 2.0f / (float32_t)len;
    fftBuf[0] /= (float32_t)len; 
    for (uint32_t i = 1; i < len / 2; i++) {
        fftBuf[i] *= norm;
    }

    //寻找主峰 (Max Peak) -> 认定为主频
    float32_t maxAmp = 0.0f;
    uint32_t maxIndex = 0;
    
    // 从索引 5 开始找（避开 0Hz 直流和极低频噪声，例如 5*分辨率 频率以下的）
    // 假设分辨率 6Hz，这就避开了 30Hz 以下。如果你的转速很慢，这里要改成 1
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
    
    if (index_2x < len / 2) { // 边界检查
        amp_2x = fftBuf[index_2x];
    }

    result->peakFreq = peak_freq; // 自动识别的主频
    result->peakAmp  = maxAmp;    // 主频幅值
    result->amp2x    = amp_2x;    // 2倍主频幅值
}

//包络特征计算
void Calc_Envelope_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    for (uint32_t i = 0; i < len; i++) {
        fftBuf[2 * i]     = data[i];
        fftBuf[2 * i + 1] = 0.0f;
    }

    // FFT
    extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 0, 1);

    // Hilbert 变换 
    // k=0, k=N/2 不变
    // k=1 ~ N/2-1 乘以 2
    // k=N/2+1 ~ N-1 归零
    for (uint32_t i = 1; i < len / 2; i++) {
        fftBuf[2 * i]     *= 2.0f;
        fftBuf[2 * i + 1] *= 2.0f;
    }
    for (uint32_t i = len / 2 + 1; i < len; i++) {
        fftBuf[2 * i]     = 0.0f;
        fftBuf[2 * i + 1] = 0.0f;
    }

    // IFFT 
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 1, 1);

    // 取模 
    float32_t sumSq = 0.0f;
    float32_t maxEnv = 0.0f;
    
    for (uint32_t i = 0; i < len; i++) {
        // IFFT 后需要除以 N 才能得到正确时域幅值吗？
        // CMSIS-DSP 的 IFFT 文档通常不带缩放，可能需要手动除以 len
        // 但为了包络检波，通常关心相对变化。标准公式通常要除以 N。
        float32_t re = fftBuf[2 * i] / (float32_t)len;
        float32_t im = fftBuf[2 * i + 1] / (float32_t)len;
        
        float32_t envVal = sqrtf(re * re + im * im);
        
        sumSq += envVal * envVal;
        if (envVal > maxEnv) maxEnv = envVal;
    }

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






