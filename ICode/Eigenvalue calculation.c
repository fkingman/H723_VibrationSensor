#include "Eigenvalue calculation.h"
#include "arm_const_structs.h"
#include <string.h>  //  memcpy
#include <math.h>
#define MIN_VALID_PEAK_AMP  0.04f
// 1 kHz 是特征测量带宽上限，不应同时作为 Butterworth 的 -3 dB 截止点。
// 截止点放到 3 kHz，使默认 25.6 kHz 采样下 1 kHz 幅值衰减小于约 1%。
#define LPF_TARGET_HZ      3000.0f
#define LPF_BUTTERWORTH_Q  0.70710678f

static arm_rfft_fast_instance_f32 S_rfft;
static float32_t fftBuf[FFT_N_Z * 2]; // 复数运算缓冲区 (Z轴最长)
static float g_filter_workbuf[FFT_N_Z];

float g_z_offset_g  = 0.0f;   // 0g 偏移
static float g_last_z_mean_g = 0.0f;
static uint8_t g_last_z_mean_valid = 0U;

volatile uint8_t g_LongCaptureState = 0; // 0:空闲, 1:录制中, 2:录制完成
volatile uint8_t g_CaptureIndex = 0; // 当前段


// 低通系数和状态分开存，便于运行时按采样率重算真正的 1kHz 截止
typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} LPF_Coeff_t;

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
} LPF_State_t;

static LPF_Coeff_t g_lpf_coeff = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// 三轴独立低通状态，避免帧与帧之间串扰
static LPF_State_t lpf_state_x = {0.0f, 0.0f, 0.0f, 0.0f};
static LPF_State_t lpf_state_y = {0.0f, 0.0f, 0.0f, 0.0f};
static LPF_State_t lpf_state_z = {0.0f, 0.0f, 0.0f, 0.0f};

//计算初始化函数
void Calc_Init(void)
{
    arm_rfft_fast_init_f32(&S_rfft, FFT_N_Z);      // 初始化 FFT 结构体  
    //Vib_Filter_Init();     // 初始化滤波器
		Algo_Update_LPF_Coeff(g_cfg_freq_hz);
}

/*void Vib_Filter_Init(void)
{
    arm_biquad_cascade_df1_init_f32(&IIR_HP,    1, (float32_t*)hp10HzCoeff,    hpState);
    arm_biquad_cascade_df1_init_f32(&IIR_NOTCH, 1, (float32_t*)notch50HzCoeff, notchState);
}*/

void Create_Wave_Snapshot(void)
{
    // 进入临界区，防止拷贝到一半被算法任务打断
    //taskENTER_CRITICAL(); 
    //memcpy(g_WaveZ_Tx, g_WaveZ_Live, sizeof(g_WaveZ_Live));
    //taskEXIT_CRITICAL();
    g_CaptureIndex = 0;
    g_LongCaptureState = 1;
}

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
    float z_sum = 0.0f;

    // --- 处理 Z 轴 ---
    for (uint32_t i = 0; i < FFT_N_Z; i++) {
        g_data_z[i] = Zcode_to_g(pZBuf[i]);
        z_sum += g_data_z[i];
    }
    g_last_z_mean_g = z_sum / (float)FFT_N_Z;
    g_last_z_mean_valid = 1U;

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

void Z_Calib_Z_Upright_Neg1G(void)
{
    if (g_last_z_mean_valid != 0U) {
        const float target_g = -1.0f;
        g_z_offset_g += (g_last_z_mean_g - target_g);
        g_last_z_mean_valid = 0U;
    }
}

static float Calc_Mean(const float *data, uint32_t len)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum / (float)len;
}

static float Calc_PeakToPeak(const float *data, uint32_t len)
{
    float min_val = data[0];
    float max_val = data[0];

    for (uint32_t i = 1; i < len; ++i) {
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }
    return max_val - min_val;
}

// 低通系数更新：二阶 Butterworth；低采样率下自动限制到 0.45 * Fs。
void Algo_Update_LPF_Coeff(uint16_t sample_rate_hz)
{
    float fs;
    float fc;
    float max_fc;
    float w0;
    float cos_w0;
    float sin_w0;
    float alpha;
    float a0;

    if (sample_rate_hz == 0U) {
        return;
    }

    fs = (float)sample_rate_hz;
    fc = LPF_TARGET_HZ;

    // 截止频率必须小于奈奎斯特，采样率太低时自动压到安全范围
    max_fc = 0.45f * fs;
    if (fc > max_fc) {
        fc = max_fc;
    }
    if (fc < 1.0f) {
        g_lpf_coeff.b0 = 1.0f;
        g_lpf_coeff.b1 = 0.0f;
        g_lpf_coeff.b2 = 0.0f;
        g_lpf_coeff.a1 = 0.0f;
        g_lpf_coeff.a2 = 0.0f;
        memset(&lpf_state_x, 0, sizeof(lpf_state_x));
        memset(&lpf_state_y, 0, sizeof(lpf_state_y));
        memset(&lpf_state_z, 0, sizeof(lpf_state_z));
        return;
    }

    w0 = 2.0f * M_PI * fc / fs;
    cos_w0 = cosf(w0);
    sin_w0 = sinf(w0);
    alpha = sin_w0 / (2.0f * LPF_BUTTERWORTH_Q);
    a0 = 1.0f + alpha;

    g_lpf_coeff.b0 = ((1.0f - cos_w0) * 0.5f) / a0;
    g_lpf_coeff.b1 = (1.0f - cos_w0) / a0;
    g_lpf_coeff.b2 = g_lpf_coeff.b0;
    g_lpf_coeff.a1 = (-2.0f * cos_w0) / a0;
    g_lpf_coeff.a2 = (1.0f - alpha) / a0;

    memset(&lpf_state_x, 0, sizeof(lpf_state_x));
    memset(&lpf_state_y, 0, sizeof(lpf_state_y));
    memset(&lpf_state_z, 0, sizeof(lpf_state_z));
}

static void LowPassFilter(float *data, uint32_t len, LPF_State_t *state)
{
    float x0;
    float y0;

    for (uint32_t i = 0; i < len; ++i)
    {
        x0 = data[i];
        y0 = g_lpf_coeff.b0 * x0
           + g_lpf_coeff.b1 * state->x1
           + g_lpf_coeff.b2 * state->x2
           - g_lpf_coeff.a1 * state->y1
           - g_lpf_coeff.a2 * state->y2;

        state->x2 = state->x1;
        state->x1 = x0;
        state->y2 = state->y1;
        state->y1 = y0;
        data[i] = y0;
    }
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
    float32_t minVal = data[0];
    float32_t maxVal = data[0];
    float32_t mean, pp, kurt;

    for (uint32_t i = 0; i < len; i++) {
        float32_t val = data[i];
        sum += val;
        if (val < minVal) minVal = val;
        if (val > maxVal) maxVal = val;
    }
    mean = sum / (float32_t)len;
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
    result->pp   = pp;
    result->kurt = kurt;
}

// 在频域按 10~1000 Hz 积分，避免时域积分被低频偏置和帧边界放大。
static void Calc_Velocity_RMS(float *data, uint32_t len, AxisFeatureValue *result)
{
    const float g_to_mm_s2 = 9806.65f;
    const float fs = (float)g_cfg_freq_hz;
    float velocity_mean_square = 0.0f;

    if ((fs <= 0.0f) || (len == 0U) ||
        (len * 2U > sizeof(fftBuf) / sizeof(fftBuf[0]))) {
        result->rms = 0.0f;
        return;
    }

    memcpy(&fftBuf[len], data, len * sizeof(float));
    arm_rfft_fast_f32(&S_rfft, &fftBuf[len], fftBuf, 0);

    uint32_t first_bin = (uint32_t)ceilf(MIN_FREQ_HZ * (float)len / fs);
    uint32_t last_bin = (uint32_t)floorf(MAX_FREQ_HZ * (float)len / fs);
    if (first_bin < 1U) first_bin = 1U;
    if (last_bin >= len / 2U) last_bin = len / 2U - 1U;
    if (first_bin > last_bin) {
        result->rms = 0.0f;
        return;
    }

    for (uint32_t k = first_bin; k <= last_bin; ++k) {
        float re = fftBuf[2U * k];
        float im = fftBuf[2U * k + 1U];
        float acc_peak_g = (2.0f / (float)len) * sqrtf(re * re + im * im);
        float freq_hz = (float)k * fs / (float)len;
        float vel_peak = acc_peak_g * g_to_mm_s2 / (2.0f * M_PI * freq_hz);
        velocity_mean_square += 0.5f * vel_peak * vel_peak;
    }

    result->rms = sqrtf(velocity_mean_square);
}


void Calc_FreqDomain_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    uint16_t current_fs = g_cfg_freq_hz;
    float32_t window_sum = 0.0f;

    if ((current_fs == 0U) || (len < 2U)) {
        result->peakFreq = 0.0f;
        result->peakAmp = 0.0f;
        result->amp2x = 0.0f;
        return;
    }

    // 保护现场
    // arm_rfft_fast_f32 会破坏输入数据
    if (len * 2 > sizeof(fftBuf)/sizeof(float32_t)) {
         // 理论上不会发生，除非 len 传错了，加个保险
         return; 
    }
    // Hann 窗降低非整数周期截断造成的频谱泄漏；window_sum 用于恢复幅值。
    for (uint32_t i = 0; i < len; ++i) {
        float32_t w = 0.5f - 0.5f * cosf(2.0f * M_PI * (float32_t)i /
                                        (float32_t)(len - 1U));
        fftBuf[len + i] = data[i] * w;
        window_sum += w;
    }

    arm_rfft_fast_f32(&S_rfft, &fftBuf[len], fftBuf, 0);
 
    // arm_rfft_fast_f32 的输出 fftBuf 布局如下：
    // [0]: 直流分量(DC)实部
    // [1]: 奈奎斯特分量实部
    // [2]: f1 实部, [3]: f1 虚部 ...
    
    if (len > 2) {
        arm_cmplx_mag_f32(&fftBuf[2], &fftBuf[2], len / 2 - 1);
    }
    
    float32_t dc_val = fabsf(fftBuf[0]);
    
    memmove(&fftBuf[1], &fftBuf[2], (len / 2 - 1) * sizeof(float32_t));
    fftBuf[0] = dc_val;
    
    // 归一化
    fftBuf[0] /= window_sum;
    float32_t norm = 2.0f / window_sum;
    
    for (uint32_t i = 1; i < len / 2; i++) {
        fftBuf[i] *= norm;
    }

    // 寻找主峰
    float32_t maxAmp = 0.0f;
    uint32_t maxIndex = 0;
    
    uint32_t first_bin = (uint32_t)ceilf(MIN_FREQ_HZ * (float32_t)len /
                                         (float32_t)current_fs);
    uint32_t last_bin = (uint32_t)floorf(MAX_FREQ_HZ * (float32_t)len /
                                         (float32_t)current_fs);
    if (first_bin < 1U) first_bin = 1U;
    if (last_bin >= len / 2U) last_bin = len / 2U - 1U;

    for (uint32_t i = first_bin; i <= last_bin; i++) {
        if (fftBuf[i] > maxAmp) {
            maxAmp = fftBuf[i];
            maxIndex = i;
        }
    }
    
		if ((first_bin > last_bin) || (maxAmp < MIN_VALID_PEAK_AMP)) {
        // 如果最大值都没超过门限，说明是静置噪音
        result->peakFreq = 0.0f; // 强制置零
        result->peakAmp  = 0.0f; // 或者保留 maxAmp 作为底噪参考，看你需求
        result->amp2x    = 0.0f;
		} else {
    float32_t freq_res = (float32_t)current_fs / (float32_t)len;
    float32_t peak_freq = (float32_t)maxIndex * freq_res;

    float32_t amp_2x = 0.0f;
    uint32_t index_2x = maxIndex * 2;
    
		if (index_2x > 1 && index_2x < (len/2 - 1)) {
				float32_t a = fftBuf[index_2x - 1];
				float32_t b = fftBuf[index_2x];
				float32_t c = fftBuf[index_2x + 1];
				// 取三者最大
				amp_2x = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
		} else if (index_2x < len/2) {
				amp_2x = fftBuf[index_2x];
		}

    result->peakFreq = peak_freq; 
    result->peakAmp  = maxAmp;    
    result->amp2x    = amp_2x;    
		}
}

// 包络特征计算 (RFFT 优化版 - 修复输入被破坏 bug)
void Calc_Envelope_Z(float32_t *data, uint32_t len, AxisFeatureValue *result)
{
    // 先保护现场！
    // arm_rfft_fast_f32 会破坏输入数据
    // 我们把 data 拷贝到 fftBuf 的后半段 (&fftBuf[len]) 作为临时输入。
    if (len * 2 > sizeof(fftBuf)/sizeof(float32_t)) {
         return; // 越界保护
    }
    memcpy(&fftBuf[len], data, len * sizeof(float32_t));

    // 输入：&fftBuf[len] (这是 data 的副本，破坏了也没事)
    // 输出：fftBuf       (存放复数频谱)
    arm_rfft_fast_f32(&S_rfft, &fftBuf[len], fftBuf, 0);

    // 频域处理：构造希尔伯特变换的频谱
    fftBuf[0] = 0.0f; 
    fftBuf[1] = 0.0f;

    for (uint32_t i = 2; i < len; i += 2) {
        float32_t re = fftBuf[i];
        float32_t im = fftBuf[i+1];
        
        // 乘以 -j : (Re + jIm) * (-j) = Im - jRe
        fftBuf[i]     = im;   // 新实部 = 旧虚部
        fftBuf[i+1]   = -re;  // 新虚部 = -旧实部
    }

    // 3. RIFFT 逆变换：得到时域的希尔伯特变换信号 h(t)
    // 输入：fftBuf (修改后的频谱)
    // 输出：存入 &fftBuf[len] (也就是之前的副本位置，现在正好用来存结果)
    // 注意：这一步会覆盖掉 &fftBuf[len] 里的 data 副本，但没关系，因为我们有原始 data 指针
    arm_rfft_fast_f32(&S_rfft, fftBuf, &fftBuf[len], 1);

    // 4. 合成包络并计算特征值
    // Envelope = sqrt( x(t)^2 + h(t)^2 )
    float32_t sumSq = 0.0f;
    float32_t maxEnv = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        // x 从原始 data 读取 (现在它是干净的)
        float32_t x = data[i];     
        
        // h 从 RIFFT 结果读取 (在 fftBuf 后半段)              
        // CMSIS-DSP 的 RIFFT 已在内部完成归一化，不能再次除以 len。
        float32_t h = fftBuf[len + i];

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
    float mean_x;
    float mean_y;
    float mean_z;
    float raw_pp_x;
    float raw_pp_y;
    float raw_pp_z;

    Eigen_Separate_And_Convert(pZBuf, pXYBuf);

    // PP 必须反映真实采样波形，在任何低通处理之前计算。
    // 去直流只改变整体偏置，不改变峰峰值，因此它也与上传的去直流波形一致。
    raw_pp_x = Calc_PeakToPeak(g_data_x, FFT_N_XY);
    raw_pp_y = Calc_PeakToPeak(g_data_y, FFT_N_XY);
    raw_pp_z = Calc_PeakToPeak(g_data_z, FFT_N_Z);

    // 波形上传只保留原始 Z 轴数据的去直流结果，不再叠加中值/低通处理
    if (g_LongCaptureState == 1) {
        uint32_t offset = g_CaptureIndex * FFT_N_Z;
        memcpy(g_filter_workbuf, g_data_z, FFT_N_Z * sizeof(float));
        Remove_DC(g_filter_workbuf, FFT_N_Z);
        memcpy(&Tx_Wave_Buffer_Z[offset], g_filter_workbuf, FFT_N_Z * sizeof(float));
        g_CaptureIndex++;
        if (g_CaptureIndex >= LONG_WAVE_PKT)
        {
            g_CaptureIndex = 0;
            g_LongCaptureState = 0;
        }
    }

    mean_x = Calc_Mean(g_data_x, FFT_N_XY);
    Remove_DC(g_data_x, FFT_N_XY);             // 不做高通，只做去直流
    LowPassFilter(g_data_x, FFT_N_XY, &lpf_state_x);
    Calc_TimeDomain_Only(g_data_x, FFT_N_XY, &X_data);
    X_data.mean = mean_x;
    X_data.pp = raw_pp_x;
    Calc_Velocity_RMS(g_data_x, FFT_N_XY, &X_data);
		
    mean_y = Calc_Mean(g_data_y, FFT_N_XY);
    Remove_DC(g_data_y, FFT_N_XY);
    LowPassFilter(g_data_y, FFT_N_XY, &lpf_state_y);
    Calc_TimeDomain_Only(g_data_y, FFT_N_XY, &Y_data);
    Y_data.mean = mean_y;
    Y_data.pp = raw_pp_y;
    Calc_Velocity_RMS(g_data_y, FFT_N_XY, &Y_data);

    mean_z = Calc_Mean(g_data_z, FFT_N_Z);
    Remove_DC(g_data_z, FFT_N_Z);
    LowPassFilter(g_data_z, FFT_N_Z, &lpf_state_z);
    Calc_TimeDomain_Only(g_data_z, FFT_N_Z,  &Z_data);
    Z_data.mean = mean_z;
    Z_data.pp = raw_pp_z;
    Calc_FreqDomain_Z(g_data_z, FFT_N_Z, &Z_data);
    Calc_Envelope_Z(g_data_z, FFT_N_Z, &Z_data);
    Calc_Velocity_RMS(g_data_z, FFT_N_Z, &Z_data);

}






