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
uint8_t  rx_dma_buf[RX_DMA_BUF_SZ]; 
uint8_t  rx_frame_buf[RX_FRAME_MAX];
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
//Z_ADC
__attribute__((section(".RAM_D2"))) uint16_t  ADC_Buffer_Z[FFT_N_Z];    		// 用于存储Z轴数据
volatile uint32_t write_index_Z = 0;  	// Z轴写指针
volatile uint32_t read_index_Z = 0;  		// Z轴读指针
volatile uint8_t flag_Z = 0;   					// Z轴标志位
//volatile uint16_t adc_ring_z[RING];  // Z 轴环形缓冲区
volatile uint32_t rd_z = 0;  // Z 轴读指针
volatile uint32_t wr_z = 0;  // Z 轴写指针
volatile uint32_t sampleCount = 0;  // 记录已采集的数据点数

//XY_ADC
//volatile uint16_t adc_ring_xy[RING]; // XY 轴环形缓冲区static uint32_t rd_z = 0;   
__attribute__((section(".RAM_D2"))) uint16_t ADC_Buffer_XY[FFT_N_XY*2];				// 用于存储XY轴数据
volatile uint32_t write_index_XY = 0;   // XY轴写指针
volatile uint32_t read_index_XY = 0;  	// XY轴读指针
volatile uint8_t flag_XY = 0;  					// XY轴标志位
volatile uint32_t rd_xy = 0; // XY 轴读指针
volatile uint32_t wr_xy = 0; // XY 轴写指针
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

uint32_t get_wr_z(void)
{
    return RING - __HAL_DMA_GET_COUNTER(&hdma_adc1);  // 获取 Z 轴的写指针
}

uint32_t get_wr_xy(void)
{
    return RING - __HAL_DMA_GET_COUNTER(&hdma_adc2);  // 获取 XY 轴的写指针
}

uint32_t ring_distance(uint32_t wr, uint32_t rd)
{
    return (wr >= rd) ? (wr - rd) : (RING - rd + wr);  // 返回可用数据点数
}


void Start_ADC_DMA(void)
{
	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
		HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Buffer_Z, FFT_N_Z);  // Z 轴
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_Buffer_XY, FFT_N_XY*2); // XY 轴
	  HAL_TIM_Base_Start(&htim2);
		HAL_TIM_Base_Start(&htim3);
}

void Stop_ADC_DMA(void)
{
	  HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop(&htim3);	
		flag_XY = 0;
		flag_Z  = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
	Uart3_RxStart();
	App_ConfigInit();
	Vib_Filter_Init();
	Ds18b20_Init();
	rx_frame_ready = 0;
	flag_XY = 0;
	flag_Z = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        if (rx_frame_ready) 
				{
					  rx_frame_ready = 0;  
						const uint8_t target_address = rx_frame_buf[0];
            const uint8_t cmd    = rx_frame_buf[1];
					  if (target_address == LOCAL_DEVICE_ADDR && (cmd == CMD_FEATURE || cmd == CMD_WAVE|| cmd == CMD_CALIBRATION||cmd == CMD_TEST )) 
						{
							Start_ADC_DMA();
							uint32_t t0 = HAL_GetTick();
							while (!(flag_XY && flag_Z)) 
							{
								if (HAL_GetTick() - t0 > 5000) // 5000ms 超时
								{   
									Stop_ADC_DMA();// 这里可以打印一条错误，或者设置一个 error_flag
									break;
								}
							}
							HAL_Delay(10);
							Stop_ADC_DMA();
							Process_Data();	
						}							
            Protocol_HandleRxFrame(rx_frame_buf, rx_frame_len, LOCAL_DEVICE_ADDR);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 5;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  PeriphClkInitStruct.PLL2.PLL2N = 16;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		if (hadc->Instance == ADC1) 
		{
        flag_Z = 1;

    } else if (hadc->Instance == ADC2) 
		{
        flag_XY = 1;
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        g_tx_busy = 0;
    }
}

void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma){
//    printf("DMA ERR: 0x%lx\r\n", (unsigned long)hdma->ErrorCode);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
//    printf("ADC ERR: 0x%lx\r\n", (unsigned long)hadc->ErrorCode);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
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
