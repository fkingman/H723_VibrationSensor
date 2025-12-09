#include "Eigenvalue calculation.h"
#include "arm_const_structs.h"

float g_z_offset_g  = 0.0f;   // 0g 偏移
uint32_t flag_cal_speed=0;
#define fr  50.0f   // 转速为 50 Hz 
#define CFFT_PTR_T  const arm_cfft_instance_f32*

static arm_cfft_instance_f32 cfftInst;
static arm_cfft_radix4_instance_f32 S_CFFT;
static arm_rfft_fast_instance_f32 S_rfft;
static uint8_t cfftInitDone = 0;
const float32_t hp10HzCoeff[5] = { 0.998266f ,-1.996532f,0.998266f, -1.996529f, 0.996535f };   
const float32_t notch50HzCoeff[5] = {  0.999796f,-1.999440f,0.999796f,-1.999440f,0.999591f };

static float32_t hpState[4];
static float32_t notchState[4];
static arm_biquad_casd_df1_inst_f32 IIR_HP, IIR_NOTCH;

float32_t hannWin[FFT_N_Z]; 

AxisFeatureValue X_data,Y_data,Z_data;

float32_t xBuf[FFT_N_XY];  // 存储 X 方向的加速度数据
float32_t yBuf[FFT_N_XY];  // 存储 Y 方向的加速度数据
float32_t zBuf[FFT_N_Z];  // 存储 Y 方向的加速度数据

static float32_t meanX, meanY, rmsX, rmsY, ppX, ppY, X_Vrms, Y_Vrms, kurtX, kurtY;// XY特征值	
static float32_t meanZ, rmsZ, ppZ, kurtZ, envelopeVrmsZ, envelopePeakZ;// Z特征值
float32_t dppZ;
float32_t peak_value,std, fgf, skewness_factor;
float32_t crest_factor, clearance_factor, shape_factor, impulse_factor;
static float32_t peakFreq = 0.0f, peakAmp = 0.0f, amp2x = 0.0f;

static float32_t timeBuf[FFT_N_Z];
static float32_t fftBuf[FFT_N_Z * 2];

float Temp;

extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len1024;
#define CFFT (&arm_cfft_sR_f32_len1024)
void FFT_Init(void)
{
    arm_rfft_fast_init_f32(&S_rfft, FFT_N_Z);  
}

float* getZBuf(void) 
{ 
	return zBuf; 
}


void Vib_Filter_Init(void)
{
    arm_biquad_cascade_df1_init_f32(&IIR_HP,    1, (float32_t*)hp10HzCoeff,    hpState);
    arm_biquad_cascade_df1_init_f32(&IIR_NOTCH, 1, (float32_t*)notch50HzCoeff, notchState);
}

void Vib_Filter_Run_Z(uint16_t *adcBuf, float32_t *outBuf, uint32_t len)
{  
    for(uint32_t i=0;i<len;i++)
        outBuf[i] = Zcode_to_g(adcBuf[i]);

    /* 高通陷波 */
//    arm_biquad_cascade_df1_f32(&IIR_HP,    outBuf, outBuf, len);
//    arm_biquad_cascade_df1_f32(&IIR_NOTCH, outBuf, outBuf, len);
}

float Zcode_to_g(uint16_t code)
{
    float vin = (float)code * Z_REF_VOLTAGE / Z_ADC_RESOLUTION;
    float g_raw = (vin - Z_REF_VOLTAGE_BIAS) / Z_SENSITIVITY;
		return g_raw - g_z_offset_g ;     // 减掉偏移
}

float XYcode_to_g(uint16_t code)
{
    float vin = (float)code * XY_REF_VOLTAGE /XY_ADC_RESOLUTION;
    return (vin - XY_REF_VOLTAGE_BIAS) / XY_SENSITIVITY;
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

    // TODO: 建议在这里把 g_z_offset_g 写入 Flash，开机时恢复
    // Flash_UpdateZOffset(g_z_offset_g);
}

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

void XY_Ha_Feature_Calc(uint16_t *XY_Data, uint32_t FFT_N)
{

    float32_t sumX = 0.0f, sumY = 0.0f;
    float32_t sumX2 = 0.0f, sumY2 = 0.0f;
		float32_t m2X = 0.0f, m4X = 0.0f;   // 二阶、四阶中心距累加器
		float32_t m2Y = 0.0f, m4Y = 0.0f;
	
    for (uint32_t i = 0; i < FFT_N; ++i)	
    {
        float32_t xVal = XYcode_to_g(XY_Data[2*i]);    // X 
        float32_t yVal = XYcode_to_g(XY_Data[2*i+1]);  // Y 

        xBuf[i] = xVal;
        yBuf[i] = yVal;

        sumX  += xVal;
        sumY  += yVal;
        sumX2 += xVal * xVal;
        sumY2 += yVal * yVal;
    }

    // Mean
    meanX = sumX / (float32_t)FFT_N;
    meanY = sumY / (float32_t)FFT_N;

    // RMS
    rmsX = sqrtf(sumX2 / (float32_t)FFT_N);
    rmsY = sqrtf(sumY2 / (float32_t)FFT_N);

    //PP
    float32_t maxX = -3.4e38f, minX =  3.4e38f;
    float32_t maxY = -3.4e38f, minY =  3.4e38f;
    for (uint32_t i = 0; i < FFT_N; ++i)
    {
        if (xBuf[i] > maxX) maxX = xBuf[i];
        if (xBuf[i] < minX) minX = xBuf[i];
        if (yBuf[i] > maxY) maxY = yBuf[i];
        if (yBuf[i] < minY) minY = yBuf[i];
			
				float32_t dx = xBuf[i] - meanX;
        float32_t dy = yBuf[i] - meanY;
        float32_t dx2 = dx * dx;
        float32_t dy2 = dy * dy;

        m2X += dx2;
        m2Y += dy2;
        m4X += dx2 * dx2;           /* (x-μ)^4 */
        m4Y += dy2 * dy2;
			
    }

    ppX = maxX - minX;
    ppY = maxY - minY;
		
    //Kurt
		float32_t denomX = m2X * m2X + 1e-12f;   
    float32_t denomY = m2Y * m2Y + 1e-12f;

    kurtX = ((float32_t)FFT_N) * m4X / denomX;
    kurtY = ((float32_t)FFT_N) * m4Y / denomY;
		
//		printf("meanX = %.3f g  meanY = %.3f g\r\n", meanX, meanY);
//		printf("RMS_X = %.3f g  RMS_Y = %.3f g\r\n", rmsX, rmsY);
//		printf("Peak-Peak X = %.3f g  Peak-Peak Y = %.3f g\r\n", ppX, ppY);
//		printf("Kurt_X = %.3f     Kurt_Y = %.3f\r\n", kurtX, kurtY);

}

/* 增量矩更新：同时维护 mean(M1)、M2、M3、M4（Pébay/Welford 扩展） */
static inline void moments_update_f32(float x, float n_prev,
                                      float *mean, float *M2, float *M3, float *M4)
{
    // n_prev = 之前的样本数 (>=0)，将更新到 n = n_prev + 1
    float n       = n_prev + 1.0f;
    float delta   = x - *mean;
    float delta_n = delta / n;
    float delta_n2= delta_n * delta_n;
    float term1   = delta * delta_n * n_prev;     // (x-μ_old)*(x-μ_new)*(n-1)

    float M2_old = *M2;
    float M3_old = *M3;

    *mean += delta_n;
    *M4   += term1 * delta_n2 * (n*n - 3.0f*n + 3.0f)
           + 6.0f * delta_n2 * M2_old
           + 4.0f * delta_n  * M3_old;            // 注意：这里是 +4*delta_n*M3_old
    *M3   += term1 * delta_n * (n - 2.0f)
           - 3.0f * delta_n * M2_old;
    *M2   += term1;
}

void Z_Ha_Feature_Calc(uint16_t *Data,uint32_t FFT_N)
{
	
		Vib_Filter_Run_Z(ADC_Buffer_Z, zBuf, FFT_N); //滤波2g
	
		float mean = 0.0f;
    float M2 = 0.0f, M3 = 0.0f, M4 = 0.0f;

    float min_v =  FLT_MAX;
    float max_v = -FLT_MAX;

    for (uint32_t i = 0; i < FFT_N; ++i)
    {
        const float x = zBuf[i];

        if (x < min_v) min_v = x;
        if (x > max_v) max_v = x;

        moments_update_f32(x, (float)i, &mean, &M2, &M3, &M4);
    }

    /* 3) 派生指标 */
    const float Nf   = (float)FFT_N;
    const float eps  = 1e-12f;
    meanZ = mean;                                  // 均值（g）
    rmsZ  = sqrtf( (M2 + eps) / Nf );              // 去均值 RMS（g）
    ppZ   = max_v - min_v;                         // 峰-峰（g）

    kurtZ = (Nf * M4) / ( (M2 * M2) + eps );       // 峭度 β2（非 excess）

//    printf("mean       = %.5f g\r\n", meanZ);
//    printf("RMS        = %.5f g\r\n", rmsZ);
//    printf("Peak-Peak  = %.5f g\r\n", ppZ);
//    printf("Kurtosis   = %.5f\r\n",   kurtZ); 
}

/* 计算 Z 轴振动烈度（RMS）和 Z 轴位移峰-峰值 */
void Freq_Feature_Calc(uint16_t *Data, uint32_t fs_Hz, uint32_t FFT_N)
{
    // 临时缓冲区用于存储 Z 轴加速度数据和计算结果
    float32_t fftBuf[2 * FFT_N];  // 用于 FFT 变换和包络提取
    float32_t displacement[FFT_N];  // 存储计算的位移

    // 1. 从 ADC_Buffer_Z 获取原始 Z 轴加速度数据并填充到 fftBuf 中
    for (uint32_t i = 0; i < FFT_N; ++i)
    {
        fftBuf[2*i] = (float32_t)ADC_Buffer_Z[i] * ADS_LSB_2_g;  // 转换为加速度单位（g）
        fftBuf[2*i+1] = 0.0f;  // 设置虚部为 0
    }

    // 2. 执行 FFT 变换，获取频域信号
    arm_cfft_f32(CFFT, fftBuf, 0, 1);  // 执行 FFT 变换

    // 3. 计算 Z 轴的振动烈度（RMS）
    float32_t sumSquares = 0.0f;
    for (uint32_t i = 0; i < FFT_N; ++i)
    {
        sumSquares += fftBuf[2*i] * fftBuf[2*i];  // 累加实部的平方
    }
    float32_t rms = sqrtf(sumSquares / (float32_t)FFT_N);  // 计算 RMS（振动烈度）

    // 4. 计算 Z 轴的位移
    float32_t velocity[FFT_N];  // 存储 Z 轴的速度
    for (uint32_t i = 0; i < FFT_N; ++i)
    {
        velocity[i] = fftBuf[2*i] * (1.0f / fs_Hz);  // 计算速度：速度 = 加速度 × 时间
    }

    // 5. 计算 Z 轴位移的峰-峰值（Peak-to-Peak）
    float32_t maxDisplacement = -3.4e38f;  // 最小值初始化为一个极小值
    float32_t minDisplacement = 3.4e38f;   // 最大值初始化为一个极大值

    // 计算位移
    for (uint32_t i = 1; i < FFT_N; ++i)
    {
        displacement[i] = displacement[i-1] + velocity[i] * (1.0f / fs_Hz);  // 位移 = 速度 × 时间（积分）
        
        // 计算位移峰-峰值
        if (displacement[i] > maxDisplacement) maxDisplacement = displacement[i];
        if (displacement[i] < minDisplacement) minDisplacement = displacement[i];
    }
    dppZ = maxDisplacement - minDisplacement;  // 位移的峰-峰值

    // 6. 打印结果
//    printf("Z Axis Vibration Intensity (RMS) = %.3f g\r\n", rms);  // 打印 Z 轴振动烈度（RMS）
//    printf("Z Axis Displacement Peak-to-Peak = %.3f mm\r\n", dppZ);  // 打印 Z 轴位移峰-峰值
}

static float make_hann_window(float *w, uint32_t N) {
    const float k = 2.0f * (float)M_PI / (float)(N - 1u);
    double sum = 0.0;
    for (uint32_t n = 0; n < N; ++n) {
        float wn = 0.5f * (1.0f - cosf(k * (float)n));
        w[n] = wn;
        sum += wn;
    }
    return (float)(sum / (double)N);  // coherent gain (≈0.5)
}

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
}

void Z_Freq_Feature_Calc(uint16_t *Data, uint32_t fs_Hz, uint32_t FFT_N, float32_t *peakFreq, float32_t *peakAmp, float32_t *amp2x, float32_t fr_Hz) 
{
    // 1. 量化并去除直流偏置
//    float32_t dcOffset = 0.0f;

    // 量化并计算直流偏置（DC偏置即平均值）
    for (uint32_t i = 0; i < FFT_N; ++i) {
        timeBuf[i] = Zcode_to_g(Data[i]);  
//				timeBuf[i] =((sin(2*PI*1000*i/Z_Sample_freq)+1)*1024+ (sin(2*PI*10000*i/Z_Sample_freq)+1)*1024)*3.3f/4095.0f;
//        dcOffset += timeBuf[i];  // 累加所有数据值
    }

    // 计算直流偏置（DC平均值）
//    dcOffset /= (float32_t)FFT_N; 

    // 去除直流偏置
//    for (uint32_t i = 0; i < FFT_N; ++i) {
//        timeBuf[i] -= dcOffset;  // 去除直流偏移
//    }
		
    // 2. 执行 FFT
		for(uint32_t i=0; i<FFT_N; i++)
		{
			  fftBuf[2*i] = timeBuf[i];
				fftBuf[2*i+1]=0; //虚部都放0
		}
		const arm_cfft_instance_f32 *S = pick_cfft_u32(FFT_N);
		float32_t mag[FFT_N];
		arm_cfft_f32(S,fftBuf,0,1);	  
		arm_cmplx_mag_f32(fftBuf, mag, FFT_N); 
    const float32_t norm = 2.0f / (float32_t)FFT_N;  // 归一化因子
		
//		for(uint16_t i=0;i<FFT_N;i++)
//		{
//				printf("%d,%f\r\n",i,mag[ i ]);	
//		}
    // 归一化每个频点的幅值
    for (uint32_t k = 0; k < FFT_N / 2 + 1 ; ++k) {
        mag[k] *= norm;  // 归一化幅值				 
    }

    // 4. 识别主峰幅值和主频率
    uint32_t peakBin = 1;
    *peakAmp = mag[1];  // 初始主频幅值为第一个频点的幅值

    // 找到最大幅值对应的频点
    for (uint32_t k = 2; k < FFT_N / 2 ; ++k) {
        if (mag[k] > *peakAmp) {
            *peakAmp = mag[k];
            peakBin = k;
        }
    }

    // 插值计算主频（提高频率计算精度）
    if (peakBin > 1 && peakBin < FFT_N / 2 - 1) {
        float32_t left = mag[peakBin - 1];
        float32_t peak = mag[peakBin];
        float32_t right = mag[peakBin + 1];
        
        // 使用三点插值计算偏移量
        float32_t offset = 0.5f * (right - left) / (left - 2.0f * peak + right);
        *peakFreq = (float32_t)(peakBin + offset) * fs_Hz / (float32_t)FFT_N;
    } else {
        *peakFreq = (float32_t)peakBin * fs_Hz / (float32_t)FFT_N;
    }

    // 5. 提取 2× 转频幅值（如果已知转速）
    *amp2x = 0.0f;
    if (fr_Hz > 0.1f) {  // 如果已知转速
        uint32_t k2 = (uint32_t)(2 * fr_Hz * FFT_N / fs_Hz);  // 计算 2x 转频对应的频点
        if (k2 < FFT_N / 2) {
            *amp2x = mag[k2];  // 如果 2x 转频的频点在有效范围内，获取对应的幅值
        }
    }

    // 6. 打印/返回特征值
//    printf("[Z] Main Frequency = %.1f Hz Peak Amp = %.4f g 2x Amp = %.4f g\n", *peakFreq, *peakAmp, *amp2x);
}

// 希尔伯特频域乘子
static void hilbert_freq_multiplier(float32_t *X, uint32_t N)
{
    // N 点复FFT，频谱为 X[0..N-1], 每点是复数 (Re,Im)
    // k=0 (DC) 与 k=N/2 (Nyquist) 保持
    // k=1..N/2-1：乘2
    // k=N/2+1..N-1：清零
    for (uint32_t k = 1; k < N/2; ++k) {
        X[2*k]   *= 2.0f;  // Re
        X[2*k+1] *= 2.0f;  // Im
    }
    for (uint32_t k = N/2+1; k < N; ++k) {
        X[2*k]   = 0.0f;
        X[2*k+1] = 0.0f;
    }
}

// Z  包络有效值（Vrms） 和 包络峰值（Peak）
void Enve_Feature_Calc(uint16_t *Data, uint32_t FFT_N)
{
    // 1) ADC→g，填充时域（每次都重写！）
    for (uint32_t n = 0; n < FFT_N; ++n) {
        timeBuf[n] = Zcode_to_g(Data[n]);
        fftBuf[2*n]   = timeBuf[n];
        fftBuf[2*n+1] = 0.0f;
    }
		const arm_cfft_instance_f32 *S = pick_cfft_u32(FFT_N);
		// --- 3) FFT ---
    arm_cfft_f32(S, fftBuf, 0, 1);

    // --- 4) Hilbert 频域乘子 ---
    hilbert_freq_multiplier(fftBuf, FFT_N);

    // --- 5) IFFT ---
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, fftBuf, 1, 1);
//    for (uint32_t n = 0; n < FFT_N; ++n) {
//        fftBuf[2*n]   /= FFT_N;
//        fftBuf[2*n+1] /= FFT_N;
//    }
    // --- 6) 取模求包络 ---
    float32_t sum2 = 0.0f, minE = 1e9f, maxE = -1e9f;
    for (uint32_t n = 0; n < FFT_N; ++n) {
        float32_t a = sqrtf(fftBuf[2*n]*fftBuf[2*n] + fftBuf[2*n+1]*fftBuf[2*n+1]);
        sum2 += a * a;
        if (a < minE) minE = a;
        if (a > maxE) maxE = a;
    }

     envelopeVrmsZ = sqrtf(sum2 / (float32_t)FFT_N);
     envelopePeakZ = maxE - minE;

//    printf("Envelope Vrms = %.3f g\r\n", envelopeVrmsZ);
//    printf("Envelope Peak = %.3f g\r\n", envelopePeakZ);
}

// 填充XY轴特征值数据
void Fill_XY_AxisFeatureValue(AxisFeatureValue *axis_data, float mean, float rms, float pp, float kurt)
{
    axis_data->mean = mean;             // 填充均值
    axis_data->rms = rms;               // 填充RMS
    axis_data->pp = pp;                 // 填充峰-峰值
    axis_data->kurt = kurt;             // 填充峭度
	
	axis_data->peakFreq = 0.0f;         // 无效
    axis_data->peakAmp = 0.0f;          
    axis_data->amp2x = 0.0f;              
    axis_data->envelope_vrms = 0.0f;    
    axis_data->envelope_peak = 0.0f;    
}

// 填充Z轴特征值数据
void Fill_Z_AxisFeatureValue(AxisFeatureValue *axis_data, float mean, float rms, float pp, float kurt, 
														float peakFreq, float peakAmp, float amp2x, float envelope_vrms, float envelope_peak)
{
    axis_data->mean = mean;             
    axis_data->rms = rms;              
    axis_data->pp = pp;                 
    axis_data->kurt = kurt;               
	axis_data->peakFreq = peakFreq;         
    axis_data->peakAmp = peakAmp;          
    axis_data->amp2x = amp2x;   
    axis_data->envelope_vrms = envelope_vrms;  
    axis_data->envelope_peak = envelope_peak;  
}

void print_FEATURE()
{
	printf("meanX = %.3f g  meanY = %.3f g\r\n", meanX, meanY);
	printf("RMS_X = %.3f g  RMS_Y = %.3f g\r\n", rmsX, rmsY);
	printf("Peak-Peak X = %.3f g  Peak-Peak Y = %.3f g\r\n", ppX, ppY);
	printf("Kurt_X = %.3f     Kurt_Y = %.3f\r\n", kurtX, kurtY);    
    printf("mean       = %.5f g\r\n", meanZ);
    printf("RMS        = %.5f g\r\n", rmsZ);
    printf("Peak-Peak  = %.5f g\r\n", ppZ);
    printf("Kurtosis   = %.5f\r\n",   kurtZ); 
    printf("[Z] Main Frequency = %.1f Hz Peak Amp = %.4f g 2x Amp = %.4f g\n", peakFreq, peakAmp, amp2x);
    printf("Envelope Vrms = %.3f g\r\n", envelopeVrmsZ);
    printf("Envelope Peak = %.3f g\r\n", envelopePeakZ);
}
	
void Process_Data()
{	  
	FFT_Init();
	// XY mean RMS Peak-Peak 
    XY_Ha_Feature_Calc((uint16_t *)ADC_Buffer_XY, FFT_N_XY);
	// Z  mean RMS Peak-Peak Kurtosis
	Z_Ha_Feature_Calc(ADC_Buffer_Z,FFT_N_Z);
    // Z peakFreq peakAmp amp2x 
	Z_Freq_Feature_Calc(ADC_Buffer_Z, Z_Sample_freq, FFT_N_Z, &peakFreq, &peakAmp, &amp2x, fr);
	// Z  包络有效值（Vrms） 和 包络峰值（Peak）
    Enve_Feature_Calc(ADC_Buffer_Z, FFT_N_Z);
	Temp = Tempetature_Dis();
	// 总值数据赋值
	Fill_XY_AxisFeatureValue(&X_data, meanX, rmsX, ppX, kurtX);
	Fill_XY_AxisFeatureValue(&Y_data, meanY, rmsY, ppY, kurtY);
	Fill_Z_AxisFeatureValue(&Z_data, meanZ, rmsZ, ppZ, kurtZ, peakFreq, peakAmp, amp2x, envelopeVrmsZ, envelopePeakZ);
}





