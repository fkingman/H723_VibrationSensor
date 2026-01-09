/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//U3接收
//__attribute__((section(".ARM.__at_0x24010000"))) uint8_t rx_dma_buf[RX_DMA_BUF_SZ];
//__attribute__((section(".ARM.__at_0x24010200"))) uint8_t rx_frame_buf[RX_FRAME_MAX];
volatile uint16_t rx_frame_len = 0;
volatile uint8_t  rx_frame_ready = 0;

//FLAH
static uint8_t g_uid[12];
uint8_t LOCAL_DEVICE_ADDR = FLASH_CFG_DEFAULT_ADDR;
uint16_t g_cfg_freq_hz = FLASH_CFG_DEFAULT_FREQ;
uint16_t g_cfg_points  = FLASH_CFG_DEFAULT_POINTS;
uint16_t wave_points = FLASH_CFG_DEFAULT_POINTS;

//组态结构体
Config_t device_config;
//ADC
//__attribute__((section(".ARM.__at_0x24000000"))) uint16_t ADC_Buffer_Z[FFT_N_Z * 2];// Z轴8192点乒乓结构
//__attribute__((section(".ARM.__at_0x24004000"))) uint16_t ADC_Buffer_XY[FFT_N_XY * 2 * 2];// XY轴(1024点 * 2通道) * 2 = 4096点乒乓
//__attribute__((section(".ARM.__at_0x2400C000"))) float Tx_Wave_Buffer_Z[FFT_N_Z];// 专门用于发送的“冷数据”区
uint16_t Process_Buffer_Z[FFT_N_Z];                   // Z轴计算区
uint16_t Process_Buffer_XY[FFT_N_XY * 2];             // XY轴计算区

float32_t g_data_x[FFT_N_XY];  // 对应 X 轴点数
float32_t g_data_y[FFT_N_XY];  // 对应 Y 轴点数
float32_t g_data_z[FFT_N_Z];   // 对应 Z 轴点数

AxisFeatureValue X_data;
AxisFeatureValue Y_data;
AxisFeatureValue Z_data;

float Temp = 0.0f;
uint32_t temp_timer = 0;      // 用于计时
uint8_t  temp_state = 0;      // 状态机：0=需要启动转换, 1=等待转换完成

typedef enum {
    BUFFER_IDLE = 0,
    BUFFER_HALF_READY, // 前半段好了
    BUFFER_FULL_READY  // 后半段好了
} BufferState_t;

volatile BufferState_t z_buffer_state = BUFFER_IDLE;
volatile BufferState_t xy_buffer_state = BUFFER_IDLE;

volatile uint8_t z_data_ready_flag = 0;
volatile uint8_t xy_data_ready_flag = 0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void App_ConfigInit()
{
//    UID_Fill_BE_w0w1w2(uid_me); 
	
		uint8_t  addr;
    uint16_t freq;
    uint16_t points;
	
    /*从 Flash 载入设备地址，不合法则为 0x00(广播地址) */
		Flash_ReadConfig(&addr, &freq, &points);	
		if (addr == 0xFF) addr = 0x00;

		LOCAL_DEVICE_ADDR = addr;
    g_cfg_freq_hz     = freq;
    g_cfg_points      = points;
    wave_points       = points;
	
		ACQ_Init(g_cfg_freq_hz, g_cfg_points);
}

void Uart3_RxStart(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);          // 清一次 IDLE 残留 
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);    
		HAL_UART_Receive_DMA(&huart3, rx_dma_buf, RX_DMA_BUF_SZ);
}

void Start_ADC_DMA(void)
{
	  HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop(&htim3);
//		TIM2->CR2 &= ~TIM_CR2_MMS;       // 清除旧配置
//    TIM2->CR2 |= TIM_CR2_MMS_1;      // 设置为 010 (Update Event)
	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer_Z, FFT_N_Z * 2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Buffer_XY, FFT_N_XY * 2 * 2);
	  HAL_TIM_Base_Start(&htim2);
		HAL_TIM_Base_Start(&htim3);
		__NOP();
}

void Stop_ADC_DMA(void)
{
	  HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop(&htim3);	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	SCB->VTOR = 0x08020000;//中断向量表偏移
	//HAL_MPU_Disable();
	//SCB_DisableDCache();
	//MPU_Config();
	//SCB_DisableDCache();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
	Delay_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	//使能串口中断和接收
	__enable_irq();
	Uart3_RxStart();
	App_ConfigInit();
	Calc_Init();
	Ds18b20_Init();
  Start_ADC_DMA();
	rx_frame_ready = 0;
  /* USER CODE END 2 */
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if (rx_frame_ready) 
      {
        rx_frame_ready = 0;  
        Protocol_HandleRxFrame(rx_frame_buf, rx_frame_len, LOCAL_DEVICE_ADDR);
      }

    if (z_data_ready_flag && xy_data_ready_flag)
    {
        // 清除标志位
        __disable_irq();
        z_data_ready_flag = 0;
        xy_data_ready_flag = 0;
        __enable_irq();
        Process_Data(Process_Buffer_Z, Process_Buffer_XY);
    }
    if (temp_state == 0) 
      {
          if (now - temp_timer > 10000) // 每10秒发起一次
          {
              Ds18b20_Start(); // 发送 0x44 
              temp_timer = now; // 记录发起时间
              temp_state = 1;   // 切换到等待状态
          }
      }
    else if (temp_state == 1)
      {
          if (now - temp_timer > 800) // 留一点余量，给800ms
          {
              short raw_temp = Ds18b20_Read_Result(); 
              
              if((raw_temp & 0XF800) > 0) // 负温
              {
                  raw_temp = ~raw_temp + 1;
                  Temp = -(float)raw_temp / 16.0f;
              }
              else // 正温
              {
                  Temp = (float)(raw_temp & 0X07FF) / 16.0f;
              }          
              // printf("Async Temp: %.1f\r\n", Temp);
              temp_state = 0; 
          }
      }  
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 15;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 2950;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//半传输完成回调 BufferA
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Z 轴处理
    if (hadc->Instance == ADC1) 
    {
        //SCB_InvalidateDCache_by_Addr((uint32_t*)&ADC_Buffer_Z[0], FFT_N_Z * 2);

        // 把前半段 (0 ~ N-1) 拷贝到计算区
        memcpy(Process_Buffer_Z, &ADC_Buffer_Z[0], FFT_N_Z * sizeof(uint16_t));
        
        z_data_ready_flag = 1; 
    }
    // XY 轴处理
    else if (hadc->Instance == ADC2)
    {
        //SCB_InvalidateDCache_by_Addr((uint32_t*)&ADC_Buffer_XY[0], FFT_N_XY * 2 * 2);

        // XY 是交错的，前半段长度是 FFT_N_XY * 2
        memcpy(Process_Buffer_XY, &ADC_Buffer_XY[0], (FFT_N_XY * 2) * sizeof(uint16_t));
        
        xy_data_ready_flag = 1;
    }
}

// DMA填满了后半段 BufferB
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Z 轴处理
    if (hadc->Instance == ADC1) 
    {
        //SCB_InvalidateDCache_by_Addr((uint32_t*)&ADC_Buffer_Z[FFT_N_Z], FFT_N_Z * 2);

        // 把后半段 (N ~ 2N-1) 拷贝到计算区
        memcpy(Process_Buffer_Z, &ADC_Buffer_Z[FFT_N_Z], FFT_N_Z * sizeof(uint16_t));
        
        z_data_ready_flag = 1; 
    }
    // XY 轴处理
    else if (hadc->Instance == ADC2) 
    {
        //SCB_InvalidateDCache_by_Addr((uint32_t*)&ADC_Buffer_XY[FFT_N_XY * 2], FFT_N_XY * 2 * 2);

        // 源地址偏移 FFT_N_XY * 2
        memcpy(Process_Buffer_XY, &ADC_Buffer_XY[FFT_N_XY * 2], (FFT_N_XY * 2) * sizeof(uint16_t));
        
        xy_data_ready_flag = 1;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        g_tx_busy = 0; 
    }
}



/* USER CODE END 4 */

 /* MPU Configuration */

// Core/Src/main.c

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  // 1. 先禁止 MPU
  HAL_MPU_Disable();

  // 2. 配置 Region 0: 将 AXI SRAM 前 128KB 设为 "Non-Cacheable"
  // 范围：0x24000000 ~ 0x2401FFFF (刚好覆盖你的三个数组)
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB; // 覆盖 128KB 的数据区
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1; // TEX=1
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // 关键！关Cache
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE; // 关Buffer

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  // 3. 开启 MPU
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
