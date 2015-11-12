/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "vl6180x_api.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BUFF_SIZE 200

#define volSensor  0x52
#define freqSensor  0x62

#define VOL_RESET_PORT GPIOA
#define FREQ_RESET_PORT GPIOA
#define VOL_RESET_PIN GPIO_PIN_2
#define FREQ_RESET_PIN GPIO_PIN_1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void VL6180x_Reset(VL6180xDev_t dev, GPIO_PinState state);
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len);
void CUSTOM_GPIO_Init(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static volatile uint8_t buff_index = 0;
static volatile int volume = 255;
static uint16_t buff[2][BUFF_SIZE];

VL6180x_RangeData_t rangeV;
VL6180x_RangeData_t rangeF;


void 	prepareBuffer(uint16_t * buffer, uint16_t size, uint8_t volume)
{
	//Triangle
	uint32_t value = 0;
	uint32_t dv = 0x00FFFFFF * volume; 
  dv /= size;
	dv *= 2;
	
	for(int  i = 0; i < size/4;i++)
	{
		int v = value >> 20;
		buffer[i] = buffer[size/2 - 1 - i ] = v + 2048;
		buffer[i + size / 2] = buffer[size - 1 - i ] = 2048 - v;


//		buffer[i + size / 4] =  buffer[size /4 - 1 - i] = v;
//		buffer[i + size / 2] =  buffer[size - 1 - i] = v + 2048;
		value += dv;
	}
}
void 		delay(int ms)
{
	uint32_t v = HAL_GetTick() + ms;
	while(HAL_GetTick() < v){
		__nop();
	}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	prepareBuffer(buff[buff_index], BUFF_SIZE, volume);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
	CUSTOM_GPIO_Init();
  VL6180x_Reset(volSensor, GPIO_PIN_RESET);
  VL6180x_Reset(freqSensor, GPIO_PIN_RESET);
  delay(10);
	VL6180x_Reset(freqSensor, GPIO_PIN_SET);
  delay(1);

	VL6180x_WaitDeviceBooted(volSensor);
	VL6180x_DisableGPIOxOut(volSensor, 1);
	VL6180x_SetI2CAddress(volSensor, freqSensor);
	
  VL6180x_InitData(freqSensor);
	

	VL6180x_Reset(volSensor, GPIO_PIN_SET);
  delay(1);
  VL6180x_WaitDeviceBooted(volSensor);
	VL6180x_DisableGPIOxOut(volSensor, 1);
  VL6180x_InitData(volSensor);


    /* Enable Dmax calculation only if value is displayed (to save computation power) */
  VL6180x_DMaxSetState(volSensor, true);
  VL6180x_DMaxSetState(freqSensor, true);
	VL6180x_Prepare(volSensor);
	VL6180x_Prepare(freqSensor);

	
	htim2.Instance->CR1 |= TIM_CR1_ARPE;
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)buff[buff_index], BUFF_SIZE, DAC_ALIGN_12B_R);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		__HAL_DMA_GET_FLAG(&hdma1, DMA_FLAG_TC3);
		hdma_dac_ch1.Instance->CMAR;
		VL6180x_RangePollMeasurement(freqSensor, &rangeF);
		VL6180x_RangePollMeasurement(volSensor, &rangeV);
		buff_index = buff_index ==0 ? 1: 0;
		if(rangeV.errorStatus == 0) {
			volume = 255 - rangeV.range_mm * 2 / 3;
			if(volume < 0 ) volume = 0;
		} else {
			volume = 0;
		}
		if(rangeF.errorStatus == 0) {
			__HAL_TIM_SET_AUTORELOAD(&htim2, rangeF.range_mm *2);
		} else {
			volume = 0;
		}
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, volume * volume);
		prepareBuffer(buff[buff_index], BUFF_SIZE, volume);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL11;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC init function */
void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  HAL_DAC_Init(&hdac);

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00A0B0FE;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c2, I2C_ANALOGFILTER_ENABLED);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim2.Init.Period = 300;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void CUSTOM_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void VL6180x_Reset(VL6180xDev_t device, GPIO_PinState state)
{
	if(device == volSensor) {
		HAL_GPIO_WritePin(VOL_RESET_PORT, VOL_RESET_PIN, state);
	} else {
		HAL_GPIO_WritePin(FREQ_RESET_PORT, FREQ_RESET_PIN, state);
	}
}

/**
 * @brief       Write data buffer to VL6180x device via i2c
 * @param dev   The device to write to
 * @param buff  The data buffer
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup cci_i2c
 */
int  VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t  *buff, uint8_t len) {
	HAL_StatusTypeDef res =  HAL_I2C_Master_Transmit(&hi2c2, dev, buff,len, 20000);
	return res == HAL_OK ? 0 : -1;
}
	
/**
 *
 * @brief       Read data buffer from VL6180x device via i2c
 * @param dev   The device to read from
 * @param buff  The data buffer to fill
 * @param len   The length of the transaction in byte
 * @return      0 on success
 * @ingroup  cci_i2c
 */
int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
		HAL_StatusTypeDef res =  HAL_I2C_Master_Receive(&hi2c2, dev, buff, len, 20000);
	return res == HAL_OK ? 0 : -1;
}

void VL6180x_PollDelay(VL6180xDev_t dev) {}

/* USER CODE END 4 */

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
