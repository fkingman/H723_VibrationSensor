#include "Eigenvalue calculation.h"
#include "arm_const_structs.h"

static arm_rfft_fast_instance_f32 S_rfft;
static float32_t fftBuf[FFT_N_Z * 2]; // 复数运算缓冲区 (Z轴最长)

float g_z_offset_g  = 0.0f;   // 0g 偏移
float fr = 50;

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
#define CFFT (&arm_cfft_sR_f32_len1024)

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
	return g_raw - g_z_offset_g ;     // 减掉偏移
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

void Z_Calib_Z_Upright_Neg1G(uint16_t *adcBuf, uint32_t N)
{
    float sum_g = 0.0f;

    for (uint32_t i = 0; i < N; ++i)
    {
        // 按“未校准”的方式算出当前点的 g（不减 offset）
        float vin   = (float)adcBuf[i] * Z_REF_VOLTAGE / Z_ADC_RESOLUTION;
        float g_raw = (vin - Z_REF_VOLTAGE_BIAS) / Z_SENSITIVITY;
        sum_g += g_raw;
    }

    float mean_g = sum_g / (float)N;

    // 你的机械姿态定义：正立 = -1g
    const float target_g = -1.0f;
    g_z_offset_g = mean_g - target_g;
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
//频域特征计算 (Z轴: PeakFreq, PeakAmp, 2xAmp)
void Calc_FreqDomain_Z(float32_t *data, uint32_t len, float rotation_speed)
{
	for (uint32_t i = 0; i < len; i++) //准备 FFT 输入
    {
        fftBuf[2 * i]     = data[i];
        fftBuf[2 * i + 1] = 0.0f;
    }
    extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 0, 1);
    arm_cmplx_mag_f32(fftBuf, fftBuf, len);//计算幅值

    float32_t norm = 2.0f / (float32_t)len;//归一化 (除以 N/2，直流除以 N)
    fftBuf[0] /= (float32_t)len; // DC
    for (uint32_t i = 1; i < len / 2; i++) {
        fftBuf[i] *= norm;
    }
    // 寻找主峰幅值 
    float32_t maxAmp = 0.0f;
    uint32_t maxIndex = 0;
    // 从索引 1 (分辨率) 开始找，避开 0Hz
    for (uint32_t i = 1; i < len / 2; i++) {
        if (fftBuf[i] > maxAmp) {
            maxAmp = fftBuf[i];
            maxIndex = i;
        }
    }
    //计算主频
    float32_t freq_res = (float32_t)Z_Sample_freq / (float32_t)len;
    float32_t peak_freq = (float32_t)maxIndex * freq_res;

    //寻找 2x 转频幅值
    float32_t amp_2x = 0.0f;
    if (rotation_speed > 0.1f) {
        float32_t target_freq = 2.0f * rotation_speed;
        uint32_t target_idx = (uint32_t)(target_freq / freq_res);
        if (target_idx < len / 2) {
            amp_2x = fftBuf[target_idx];
        }
    }
    Z_data.peakFreq = peak_freq;
    Z_data.peakAmp  = maxAmp;
    Z_data.amp2x    = amp_2x;
}

//包络特征计算
void Calc_Envelope_Z(float32_t *data, uint32_t len)
{
    // 1. 准备数据 (再次拷贝，因为之前的 fftBuf 被破坏了)
    for (uint32_t i = 0; i < len; i++) {
        fftBuf[2 * i]     = data[i];
        fftBuf[2 * i + 1] = 0.0f;
    }

    // 2. FFT
    extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len4096;
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 0, 1);

    // 3. Hilbert 变换 (频域操作)
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

    // 4. IFFT (逆变换)
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 1, 1);

    // 5. 取模 (得到包络信号) 并计算统计值
    float32_t sumSq = 0.0f;
    float32_t maxEnv = 0.0f;
    
    for (uint32_t i = 0; i < len; i++) {
        // IFFT 后需要除以 N 才能得到正确时域幅值吗？
        // CMSIS-DSP 的 IFFT 文档通常不带缩放，可能需要手动除以 len
        // 但为了包络检波，通常关心相对变化。标准公式通常要除以 N。
        // 此处假设库未缩放：
        float32_t re = fftBuf[2 * i] / (float32_t)len;
        float32_t im = fftBuf[2 * i + 1] / (float32_t)len;
        
        float32_t envVal = sqrtf(re * re + im * im);
        
        sumSq += envVal * envVal;
        if (envVal > maxEnv) maxEnv = envVal;
    }

    // --- 更新 Z_data 结构体 ---
    Z_data.envelope_vrms = sqrtf(sumSq / (float32_t)len);
    Z_data.envelope_peak = maxEnv;
}


void print_FEATURE(void)
{
    printf("========== Vibration Analysis Result ==========\r\n");
    printf("[X-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", X_data.mean, X_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   X_data.pp,   X_data.kurt);

    printf("[Y-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", Y_data.mean, Y_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   Y_data.pp,   Y_data.kurt);

    printf("[Z-Axis] Mean = %6.3f g   RMS = %6.3f g\r\n", Z_data.mean, Z_data.rms);
    printf("         P-P  = %6.3f g   Kurt= %6.3f\r\n",   Z_data.pp,   Z_data.kurt);
    printf("[Z-Freq] Main Freq = %5.1f Hz   Peak Amp = %.4f g\r\n", 
           Z_data.peakFreq, Z_data.peakAmp);
    printf("         2x Amp    = %.4f g\r\n", Z_data.amp2x);
    printf("[Z-Enve] Env Vrms  = %.3f g     Env Peak = %.3f g\r\n", 
           Z_data.envelope_vrms, Z_data.envelope_peak);

    printf("===============================================\r\n\n");
}
	
void Process_Data(uint16_t *pZBuf, uint16_t *pXYBuf)
{	  
    Eigen_Separate_And_Convert(ADC_Buffer_Z, ADC_Buffer_XY);

    Calc_TimeDomain_Only(g_data_x, FFT_N_XY, &X_data);
    Calc_TimeDomain_Only(g_data_y, FFT_N_XY, &Y_data);
    Calc_TimeDomain_Only(g_data_z, FFT_N_Z,  &Z_data);

    Calc_FreqDomain_Z(g_data_z, FFT_N_Z, fr);
    Calc_Envelope_Z(g_data_z, FFT_N_Z);
    Temp = Tempetature_Dis();
}


//Test
void print_adc_buffer_Z(uint16_t *buf, uint32_t N)
{
    for (uint32_t i = 0; i < N; i++) 
		{
			float g =  Zcode_to_g(buf[i]);
			printf("%.3f \n", g);
		}
}

void print_adc_buffer_X(uint16_t *buf, uint32_t N)
{
    for (uint32_t i = 0; i < N; i++)
    {
        float g = XYcode_to_g(buf[2 * i]);   // X 在偶数索引
        printf("%.3f\n", g);
    }
}
void print_adc_buffer_Y(uint16_t *buf, uint32_t N)
{
    for (uint32_t i = 0; i < N; i++)
    {
        float g = XYcode_to_g(buf[2 * i + 1]);  // Y 在奇数索引
        printf("%.3f\n", g);
    }
}

void print_TEMP()
{		
	    Temp = Tempetature_Dis();
		printf("Temp is: %.1f°C\n", Temp);
}


