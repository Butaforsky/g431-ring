/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "dac.h"
#include "gpio.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "ucpd.h"
#include "usart.h"
#include "usb.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ring_buffer.h"
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
rb_t ring_buffer = {
    .huart = &huart2,
};
uint8_t flag = 0;
uint32_t random_num;

uint8_t min_index = 1;
uint8_t max_index = 109;
int8_t Y;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int interpret_num(int y);
int ipow(int base, int exp);
int eu_mod(int x, int y);
uint8_t mod(int8_t num)
{
  if (num < 0)
    return -num;
  return num;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_DAC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UCPD1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  ring_buf_init(&ring_buffer, &huart3);
  HAL_UART_Transmit(&huart3, (uint8_t *)"hello", 6, 1000);

  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  SystemClock_Config();
  HAL_ResumeTick();

  HAL_UART_Transmit(&huart3, (uint8_t *)"hello", 6, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_RNG_GenerateRandomNumber(&hrng, &random_num); // generate a random num in range [0 - 0xFFFF FFFF]
    random_num %= ((50 + 1 - 1) + 1);

    uint8_t data[random_num];
    for (uint8_t i = 0; i < random_num; i++)
    {
      data[i] = random_num >> 2;
      ring_buff_put(&ring_buffer, data[i]);
      HAL_Delay(random_num);
      if (ring_buf_get_empty_slots(&ring_buffer) == 0)
      {
        Y = ring_buf_get_head(&ring_buffer);
        int mod_y = mod(Y);
        if (mod_y < 10)
        {
          uint16_t num_of_six;
          uint8_t SSB;
          num_of_six = interpret_num(mod_y);
          SSB = (0xFF00 & num_of_six) >> 8;
          HAL_UART_Transmit(&huart3, (uint8_t *)SSB, 1, 100);
        }
        else
        {
          HAL_UART_Transmit(&huart3, (uint8_t *)Y, 1, 100);
        }
        break;
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

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == GPIO_PIN_4)
  {
    flag = 1;
  }
}

int interpret_num(int y)
{
  // count num of "6" in district with 10 in pow(y) buildings
  // available nums: [1 - 109]
  // num of "6": (11 + 1) for each range [1-109]
  uint32_t buildings_count; // as we can reach 10 in power of 9, use 32bit var;
  uint32_t num_of_ranges;
  uint16_t num_of_nums;
  uint16_t rest;
  uint16_t total;
  // calculate pow(10, Y), as we work with integer, we don't have to use
  // math.h pow func, that uses double
  buildings_count = ipow(10, y);
  // calc num of ranges[1-109]
  if (buildings_count == 1)
  {
    return 0;
  }
  num_of_ranges = buildings_count / 109;
  if (num_of_ranges <= 1)
  {
    num_of_ranges = buildings_count / 10;
    num_of_nums = 1;
    if (num_of_ranges > 6)
      num_of_nums = num_of_ranges + 1;
    return num_of_nums;
  }
  num_of_nums = num_of_ranges * 12;
  rest = eu_mod(buildings_count, 12);
  total = rest + num_of_nums;
  return total;
}

int ipow(int base, int exp)
{
  int result = 1;
  for (;;)
  {
    if (exp & 1)
      result *= base;
    exp >>= 1;
    if (!exp)
      break;
    base *= base;
  }

  return result;
}

// euclidian
int eu_mod(int x, int y)
{
  int r;
  r = x % y;
  if (r < 0)
    r += mod(y);
  return r;
}

/* USER CODE END 4 */

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
