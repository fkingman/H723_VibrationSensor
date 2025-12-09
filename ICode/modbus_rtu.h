#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <stdbool.h>

/*
XY: mean RMS PP 
Z:	mean RMS PP Displacement_PP Envelope_Vrms Envelope_Peak
*/

/* ────────── Command 定义 ────────── */
#define CMD_CONFIG       0x00     /* 配置请求  */
#define CMD_FEATURE      0x02     /* 特征值请求  */
#define CMD_WAVE_PACK    0x03     /* 波形包请求  */
#define CMD_WAVE         0x04     /* 波形请求    */
#define CMD_TEST         0x77     /* 测试请求    */
#define CMD_DISCOVER     0x41   	 /* 主站广播发现*/
#define CMD_SET_ADDR  	 0x42   	 /* 主站广播给某uid配置地址*/
#define CMD_CALIBRATION  0x60   	 /* 校准请求*/
#define CMD_WRONG        0x80     /* 错误 */

/* ────────── TEST Channel 定义 ────────── */
#define CH_X         		 		0x01
#define CH_Y          	 		  0x02
#define CH_Z         		 		0x03
#define CH_FEATURE                  0x04
#define CH_TEMP          		   0x05
#define CH_FEATURE_TEST            0x06
#define CH_X3         		 		0x07
#define CH_Y3         	 		   0x08
#define CH_Z3        		 		0x09
#define FIXED_THIRD_BYTE    0xFF
/* ────────── CONfIG Command 定义 ────────── */
#define FREQ          	 		0x01
#define PORINT         		 	0x02


extern volatile uint8_t g_tx_busy;
/* 上位机发来一帧后调用此函数，len=完整帧长度 */
void Protocol_HandleRxFrame(const uint8_t *rx, uint16_t len, uint8_t local_address);


#endif
