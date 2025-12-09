#ifndef EIGENVALUE_CALCULATION_H_
#define EIGENVALUE_CALCULATION_H_
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "main.h"
#include "arm_math.h"
#include <math.h>
#include "modbus_rtu.h"

#define ARM_MATH_CM7

#define UINT32_T_BYTE													 4
#define UINT16_T_BYTE													 2
#define UINT8_T_BYTE													 1
#define FLOAT_BYTE													   4

#ifndef M_PI
  #define M_PI 3.14159265358979323846f
#endif
#define Z_ADC_RESOLUTION           65535.0f   /* 0xFFFF */
#define Z_REF_VOLTAGE              3.30f      /* VREF  */
#define Z_REF_VOLTAGE_BIAS         1.65f      
#define Z_SENSITIVITY              0.03f    /* V / g  */
#define XY_ADC_RESOLUTION  				 65535.0f		
#define XY_REF_VOLTAGE             3.30f
#define XY_REF_VOLTAGE_BIAS        1.65f
#define XY_SENSITIVITY             0.063f     /* V/g */

#define MIN_FREQ_HZ  			 10.0f
#define MAX_FREQ_HZ        1000.0f
#ifndef ADS_LSB_2_g
  #define ADS_LSB_2_g   ( (Z_REF_VOLTAGE / 256.0f) / (32768.0f * Z_SENSITIVITY) )
#endif

// 用于存储每个轴的特征值
typedef struct
{
    float mean;                // 均值（4字节）
    float rms;                 // RMS（4字节）
    float pp;                  // 峰-峰值（4字节）
		float kurt;								 // 峭度（4字节）
		float peakFreq;						 // 主峰频率（4字节）
		float peakAmp;					   // 主频幅值（4字节）
		float amp2x;							 // 2x转频幅值（4字节）
    float envelope_vrms;       // 包络有效值（4字节）
    float envelope_peak;       // 包络峰值（4字节）
} AxisFeatureValue;
extern AxisFeatureValue X_data,Y_data,Z_data;
extern float Temp; 
extern float g_z_offset_g;
void Z_Calib_Z_Upright_Neg1G(uint16_t *adcBuf, uint32_t N);
float* getZBuf(void);
void Z_Freq_Feature_Calc_TEST(uint16_t *Data, uint32_t FFT_N);
void Test(void);
void Vib_Filter_Init(void);
float Zcode_to_g(uint16_t code);
float XYcode_to_g(uint16_t code);
void print_adc_buffer_Z(uint16_t *buf, uint32_t N);
void print_adc_buffer_X(uint16_t *buf, uint32_t N);
void print_adc_buffer_Y(uint16_t *buf, uint32_t N);
void print_TEMP();
void print_FEATURE();
void XY_Ha_Feature_Calc(uint16_t *XY_Data,uint32_t FFT_N);
void accelerate_XY_Vrms_calc(float32_t resolu);
void Z_Ha_Feature_Calc(uint16_t *Data,uint32_t FFT_N);
void Z_Freq_Feature_Calc(uint16_t *Data, uint32_t fs_Hz, uint32_t FFT_N, float32_t *peakFreq, float32_t *peakAmp, float32_t *amp2x, float32_t fr_Hz);
void Enve_Feature_Calc(uint16_t *Data, uint32_t FFT_N);
void freq_harmonic_amp_f_phase_calc(float rotate_speed_f, uint32_t fs_Hz, const uint32_t fft_points, 
                                     float32_t *z_mag, float32_t *z_phase);

void freq_bearing_failure_amp_f_phase_calc(float rotate_speed_f, uint32_t fs_Hz, uint32_t fft_points, 
                                           float32_t *z_mag, float32_t *z_phase);
void freq_gear_engagement_and_sideband_amp_f_phase_calc(float rotate_speed_f, uint32_t fs_Hz, uint32_t fft_points, 
                                                         float32_t *z_mag, float32_t *z_phase);
void vibration_intensity_calc(float32_t Z_rms_g);
void displacement_pp_value_calc(float32_t *z_mag, uint32_t fs_Hz, uint32_t fft_points);
void enve_harmonic_amp_f_phase_calc(float32_t *z_mag, float32_t *z_phase, 
                                     float rotate_speed, uint32_t fs_Hz, uint32_t fft_points);
void enve_bearing_failure_amp_f_phase_calc(float32_t *z_mag, float32_t *z_phase, 
                                           float rotate_speed, uint32_t fs_Hz, uint32_t fft_points);
void Fill_XY_AxisFeatureValue(AxisFeatureValue *axis_data, float mean, float rms, float pp, float kurt);
void Fill_Z_AxisFeatureValue(AxisFeatureValue *axis_data, float mean, float rms, float pp, float kurt, 
														float peakFreq, float peakAmp, float amp2x, float envelope_vrms, float envelope_peak);
															void Process_Data();

#endif /* EIGENVALUE_CALCULATION_H_ */
