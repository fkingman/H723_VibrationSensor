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
extern float g_z_offset_g;

void Calc_Init(void);// 用于在上电时调用一次，负责 FFT 表初始化和滤波器初始化
void Process_Data(uint16_t *pZBuf, uint16_t *pXYBuf);
void Eigen_Separate_And_Convert(uint16_t *pZBuf, uint16_t *pXYBuf);//转g

//void Calc_TimeDomain_Only(float32_t *data, uint32_t len, AxisFeatureValue *result);
//void Calc_FreqDomain_Z(float32_t *data, uint32_t len);
//void Calc_Envelope_Z(float32_t *data, uint32_t len);
void Z_Calib_Z_Upright_Neg1G(float *gBuf, uint32_t N);
float Tempetature_Dis(void);
void print_g_data(float *buf, uint32_t N);
void print_FEATURE();

															

#endif /* EIGENVALUE_CALCULATION_H_ */
