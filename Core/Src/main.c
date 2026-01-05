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
#include "usart.h"
#include "gpio.h"
#define  TFLM_RUNTIME_USE_ALL_OPERATORS 1
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include  "network_tflite_data.h"
#include "../Inc/tflm_c.h"
#include "stdio.h"
#include "../Inc/entry_point.h"
#include "stdbool.h"
 void DebugLog(const char* s) {
  extern UART_HandleTypeDef huart3;
  HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t tensor_arena[TFLM_NETWORK_TENSOR_AREA_SIZE+32];


struct tflm_context {
  char *name;
  uint32_t hdl;
  uint32_t n_inputs;
  uint32_t n_outputs;
  struct tflm_c_tensor_info *inputs;
  struct tflm_c_tensor_info *outputs;
  struct tflm_c_tensor_info  arena;
  struct tflm_c_tensor_info  tflite;
  int error;
  bool debug;
} net_exec_ctx[1] = {0};


struct io_malloc {
  uint32_t cfg;           /* configuration and state */
  uint32_t alloc;         /* accumulated size of allocated memory */
  uint32_t free;          /* accumulated size of freed memory */
  uint32_t alloc_req;     /* number of requested alloc */
  uint32_t free_req;      /* number of requested free */
  uint32_t max;           /* maximum allocated memory */
  uint32_t used;          /* current allocated memory */
#if _IO_MALLOC_TRACK_MODE == 1
  void* a_ptr[_IO_MALLOC_TRACK_DEPTH_SIZE];
  size_t a_s[_IO_MALLOC_TRACK_DEPTH_SIZE];
  int a_idx;
  void *f_ptr[_IO_MALLOC_TRACK_DEPTH_SIZE];
  size_t f_s[_IO_MALLOC_TRACK_DEPTH_SIZE];
  int f_idx;
#endif
};


struct io_malloc io_malloc;
#define MON_ALLOC_RESET()\
_mon_alloc_reset()
#define MON_ALLOC_ENABLE() io_malloc.cfg |= 1UL

#define MON_ALLOC_DISABLE() io_malloc.cfg &= ~1UL
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void _mon_alloc_reset(void) {
  memset((void *)&io_malloc, 0, sizeof(struct io_malloc));
  /* force a call of wrap functions */
  free(malloc(10));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char t[100];

  TfLiteStatus res;
  struct tflm_c_version ver;
  uint32_t uaddr = (uint32_t)tensor_arena;
  uaddr = (uaddr + (16 - 1)) & (uint32_t)(-16);  // Round up to 16-byte boundary
  TfLiteStatus tflm_res;
  static struct tflm_c_tensor_info input_tensor;
  static struct tflm_c_tensor_info output_tensor;



  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  MON_ALLOC_RESET();
  MON_ALLOC_ENABLE();

  res = tflm_c_create(g_tflm_network_model_data, (uint8_t*)uaddr,
           TFLM_NETWORK_TENSOR_AREA_SIZE, &net_exec_ctx->hdl);

  MON_ALLOC_DISABLE();

  tflm_c_input(net_exec_ctx->hdl, 0, &input_tensor);
  tflm_c_output(net_exec_ctx->hdl, 0, &output_tensor);
  HAL_UART_Transmit(&huart3, (uint8_t*)"Init_success\r\n", strlen("Init_success\r\n"), HAL_MAX_DELAY);
  int32_t n_inputs = tflm_c_inputs_size(net_exec_ctx->hdl);
  // int32_t n_outputs = tflm_c_outputs_size(res);



  if (input_tensor.bytes != 16) {
    DebugLog("Input tensor size mismatch!\r\n");
    while(1);
  }

  float* in_data = (float*)input_tensor.data;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


      in_data[0] = 500.0f;
      in_data[1] = 480.0f;
      in_data[2] = 460.0f;
      in_data[3] = 440.0f;


    tflm_res = tflm_c_invoke(net_exec_ctx->hdl);

      if (tflm_res == kTfLiteOk) {

      float* out_data = (float*)output_tensor.data;
      sprintf(t, "%.4f ",out_data[0]);
      HAL_UART_Transmit(&huart3, (uint8_t*)t, strlen(t), HAL_MAX_DELAY);

    }

    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
