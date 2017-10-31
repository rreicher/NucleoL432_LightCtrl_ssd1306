/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "binary.h"

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16

//static const unsigned char /*PROGMEM*/ logo16_glcd_bmp[] = {
//		B00000000, B11000000,
//		B00000001, B11000000,
//		B00000001, B11000000,
//		B00000011, B11100000,
//		B11110011, B11100000,
//		B11111110, B11111000,
//		B01111110, B11111111,
//		B00110011, B10011111,
//		B00011111, B11111100,
//		B00001101, B01110000,
//		B00011011, B10100000,
//		B00111111, B11100000,
//		B00111111, B11110000,
//		B01111100, B11110000,
//		B01110000, B01110000,
//		B00000000, B00110000
//};

#define printf(...)                                                            \
				do {                                                                   \
					uart2_transmit_complete = false;                                     \
					sprintf(vcpbuff, __VA_ARGS__);                                       \
					HAL_UART_Transmit_DMA(&huart2, (uint8_t *)vcpbuff, strlen(vcpbuff)); \
					while (!uart2_transmit_complete) __WFE();                            \
				} while (0)

#define uart_write_buffer(BUFFER, LENGTH)                                      \
				do {                                                                   \
					uart2_transmit_complete = false;                                     \
					HAL_UART_Transmit_DMA(&huart2, (uint8_t *)BUFFER, LENGTH);           \
					while (!uart2_transmit_complete) __WFE();                            \
				} while (0)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp1;

DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO bool uart2_transmit_complete = false;
__IO bool spi1_transmit_complete = false;
__IO bool comp1_trig_event = false;
static char vcpbuff[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP1_Init(void);
static void MX_DAC1_Init(void);
void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//void testdrawchar(void);
//void testdrawline(void);
//void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_COMP1_Init();
  MX_DAC1_Init();
//  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* Set DAC Channel to VDDA/2 */
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)2048);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_Delay(2000);

  /* Start Comparator */
  HAL_COMP_Start(&hcomp1);

  printf("Nucleo64 SSD1306\r\n");

  ssd1306_init_spi();
	ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, true);
	HAL_Delay(1000);

	ssd1306_display();
	HAL_Delay(3000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	ssd1306_clear_display();

  	/* Welcome message */
  	ssd1306_set_cursor(0, 0);
  	ssd1306_set_textcolor(WHITE);
  	ssd1306_set_textsize(2);
  	ssd1306_putstring("Light Ctrl");
  	ssd1306_display();
  	HAL_Delay(1000);

//		// draw a single pixel
//		ssd1306_draw_pixel(10, 10, WHITE);
//		ssd1306_display();
//		HAL_Delay(1000);
//
//		ssd1306_draw_circle(SSD1306_LCDWIDTH / 2, SSD1306_LCDHEIGHT / 2, 30, WHITE);
//		ssd1306_display();
//		HAL_Delay(1000);
//
//		testdrawchar();
//		HAL_Delay(1000);
//
//		ssd1306_clear_display();
//		ssd1306_display();
//		HAL_Delay(1000);
//
//		testdrawline();
//
//		ssd1306_clear_display();
//		ssd1306_draw_bitmap(30, 16,  logo16_glcd_bmp, 16, 16, 1);
//		ssd1306_display();
//		HAL_Delay(1000);
//
//		// draw a bitmap icon and 'animate' movement
//		testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* COMP1 init function */
static void MX_COMP1_Init(void)
{

  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INPUT_MINUS_DAC1_CH1;
  hcomp1.Init.NonInvertingInput = COMP_INPUT_PLUS_IO3;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp1.Init.Mode = COMP_POWERMODE_MEDIUMSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI1_NSS_Pin|SSD_DC_Pin|SSD_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI1_NSS_Pin SSD_DC_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin|SSD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SSD_RES_Pin */
  GPIO_InitStruct.Pin = SSD_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SSD_RES_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//void testdrawchar(void)
//{
//	ssd1306_clear_display();
//	ssd1306_set_textsize(1);
//	ssd1306_set_textcolor(WHITE);
//	ssd1306_set_cursor(0, 0);
//
//	for (uint8_t i = 0; i < 168; i++) {
//		if (i == '\n') continue;
//		ssd1306_write(i);
//		if ((i > 0) && (i % 21 == 0))
//			ssd1306_write('\n');
//	}
//	ssd1306_display();
//}
//
//
//void testdrawline(void)
//{
//	for (int16_t i = 0; i < ssd1306_width(); i += 4) {
//		ssd1306_draw_line(0, 0, i, ssd1306_height() - 1, WHITE);
//		ssd1306_display();
//	}
//	for (int16_t i = 0; i < ssd1306_height(); i += 4) {
//		ssd1306_draw_line(0, 0, ssd1306_width() - 1, i, WHITE);
//		ssd1306_display();
//	}
//	HAL_Delay(250);
//
//	ssd1306_clear_display();
//	for (int16_t i = 0; i < ssd1306_width(); i += 4) {
//		ssd1306_draw_line(0, ssd1306_height() - 1, i, 0, WHITE);
//		ssd1306_display();
//	}
//	for (int16_t i = ssd1306_height() - 1; i >= 0; i -= 4) {
//		ssd1306_draw_line(0, ssd1306_height() - 1, ssd1306_width() - 1, i, WHITE);
//		ssd1306_display();
//	}
//	HAL_Delay(250);
//
//	ssd1306_clear_display();
//	for (int16_t i = ssd1306_width() - 1; i >= 0; i -= 4) {
//		ssd1306_draw_line(ssd1306_width() - 1, ssd1306_height() - 1, i, 0, WHITE);
//		ssd1306_display();
//	}
//	for (int16_t i = ssd1306_height() - 1; i >= 0; i -= 4) {
//		ssd1306_draw_line(ssd1306_width() - 1, ssd1306_height() - 1, 0, i, WHITE);
//		ssd1306_display();
//	}
//	HAL_Delay(250);
//
//	ssd1306_clear_display();
//	for (int16_t i = 0; i < ssd1306_height(); i += 4) {
//		ssd1306_draw_line(ssd1306_width() - 1, 0, 0, i, WHITE);
//		ssd1306_display();
//	}
//	for (int16_t i = 0; i < ssd1306_width(); i += 4) {
//		ssd1306_draw_line(ssd1306_width() - 1, 0, i, ssd1306_height() - 1, WHITE);
//		ssd1306_display();
//	}
//	HAL_Delay(250);
//
//	ssd1306_display();
//	HAL_Delay(250);
//	ssd1306_clear_display();
//}
//
//
//void testdrawbitmap(const uint8_t *bitmap, uint8_t w, uint8_t h)
//{
//	#define NUMFLAKES 10
//	#define XPOS 0
//	#define YPOS 1
//	#define DELTAY 2
//
//	uint8_t icons[NUMFLAKES][3];
//
//	// initialize
//	for (uint8_t f = 0; f < NUMFLAKES; f++)
//	{
//		icons[f][XPOS] = rand() % ssd1306_width();
//		icons[f][YPOS] = 0;
//		icons[f][DELTAY] = (rand() % 5) + 1;
//	}
//
//	while (1)
//	{
//		// draw each icon
//		for (uint8_t f = 0; f < NUMFLAKES; f++)
//		{
//			ssd1306_draw_bitmap(icons[f][XPOS], icons[f][YPOS], logo16_glcd_bmp, w, h, WHITE);
//		}
//		ssd1306_display();
//		HAL_Delay(200);
//
//		// then erase it + move it
//		for (uint8_t f = 0; f < NUMFLAKES; f++)
//		{
//			ssd1306_draw_bitmap(icons[f][XPOS], icons[f][YPOS],  logo16_glcd_bmp, w, h, BLACK);
//			// move it
//			icons[f][YPOS] += icons[f][DELTAY];
//			// if its gone, reinit
//			if (icons[f][YPOS] > ssd1306_height()) {
//				icons[f][XPOS] = rand() % ssd1306_width();
//				icons[f][YPOS] = 0;
//				icons[f][DELTAY] = (rand() % 5) + 1;
//			}
//		}
//	}
//}

/**
 * @brief Tx Transfer completed callback.
 * @param huart: UART handle.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	uart2_transmit_complete = true;
}

/**
 * @brief Tx Transfer completed callback.
 * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	spi1_transmit_complete = true;
}

/**
  * @brief  Comparator callback.
  * @param  hcomp  COMP handle
  * @retval None
  */
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hcomp);

  comp1_trig_event = true;

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
