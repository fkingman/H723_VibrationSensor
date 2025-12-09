/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "modbus_rtu.h"
#include "Eigenvalue calculation.h"
#include "DS18B20.h"
#include "flash.h"
#include <stdbool.h>
#include "app_acq.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Temp_DS_Pin GPIO_PIN_8
#define Temp_DS_GPIO_Port GPIOB
#define X_ADC_Pin GPIO_PIN_0
#define X_ADC_GPIO_Port GPIOC
#define Z_ADC_Pin GPIO_PIN_0
#define Z_ADC_GPIO_Port GPIOA
#define Y_ADC_Pin GPIO_PIN_4
#define Y_ADC_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOE
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define RING           FFT_N_Z*2        	/* 2^n 最方便，≈80 KB */ 
#define FFT_N_Z       4096 			  									// 你需要采集的点数
#define FFT_N_XY  		 1024  							// 你需要采集的点数
#define Z_Sample_freq  25600								// Z轴采集频率
extern uint16_t  ADC_Buffer_Z[FFT_N_Z];    		// 用于存储Z轴数据
extern uint16_t ADC_Buffer_XY[FFT_N_XY*2];				// 用于存储XY轴数据

//dma和modbus的缓冲区,串口3
#define RX_DMA_BUF_SZ   256
#define RX_FRAME_MAX    256
extern uint8_t  rx_dma_buf[RX_DMA_BUF_SZ]; 
extern uint8_t  rx_frame_buf[RX_FRAME_MAX];
extern volatile uint16_t rx_frame_len;
extern volatile uint8_t  rx_frame_ready;

//系统信息结构体
typedef struct
{
	char szVersion[32];
	char szBuildDate[32];
	char szBuildTime[32];
} AppInfo_t;

//本机信息结构体
#define FLASH_CONFIG_ADDRESS 0x080E0000
typedef struct {
    uint8_t device_address;  // 设备地址
    uint16_t num_samples;     // 采集点数
    uint16_t sample_rate;     // 采集频率
} Config_t;

uint32_t get_wr_z(void);
uint32_t get_wr_xy(void);
uint32_t ring_distance(uint32_t wr, uint32_t rd);
void Uart3_RxStart(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
