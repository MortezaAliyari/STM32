/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "lwip.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_BUF_LEN3 256
#define ADC_BUF_LEN1 256


#define UDP_BUF_LEN3 2*ADC_BUF_LEN3
#define UDP_BUF_LEN1 2*ADC_BUF_LEN1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t pData3[ADC_BUF_LEN3];
uint16_t pData1[ADC_BUF_LEN1];

uint16_t i=0;
uint8_t data3[UDP_BUF_LEN3+2];
uint8_t data1[UDP_BUF_LEN1+2];
uint8_t dataH1[UDP_BUF_LEN1];
uint8_t dataH2[UDP_BUF_LEN1];

uint8_t AdcDmaFlagC=0;
uint8_t AdcDmaFlagH=0;
uint8_t AdcDmaFlagC3=0;
uint8_t AdcDmaFlagH3=0;
uint8_t AdcDmaFlagC1=0;
uint8_t AdcDmaFlagH1=0;
uint8_t AdcDmaFlagS3=0;
uint8_t AdcDmaFlagS1=0;
uint32_t count1=0,count3=0;

struct udp_pcb* upcb;
struct pbuf *p;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_LWIP_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
//HAL_TIM_Base_Start(&htim6);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t *)pData3,ADC_BUF_LEN3);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)pData1,ADC_BUF_LEN1);
  data3[0]=33;
  data3[1]=33;
  data1[0]=11;
  data1[1]=11;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

	if(hadc->Instance==ADC3){
	  for(uint16_t j=0;j<=ADC_BUF_LEN3/2;j++){
		 data3[2*j+2]  = pData3[j] & 0xff ;
		 data3[2*j+1+2]=(pData3[j] >> 8);
				}
	//AdcDmaFlagS3++;
	}

	if(hadc->Instance==ADC1){
		for(uint16_t j=0;j<=ADC_BUF_LEN1/2;j++){
			 data1[2*j+2]  = pData1[j] & 0xff;
			 data1[2*j+1+2]=(pData1[j] >> 8);
		}
	//AdcDmaFlagS1++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
		if(hadc->Instance==ADC3){
	        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3,SET);

			for(uint16_t j=0;j<ADC_BUF_LEN3/2;j++){
				 data3[2*j+ADC_BUF_LEN3+2]  = pData3[j+ADC_BUF_LEN3/2] & 0xff;
				 data3[2*j+ADC_BUF_LEN3+1+2]=(pData3[j+ADC_BUF_LEN3/2] >> 8);
		    	}
	        AdcDmaFlagS3++;
	        MX_LWIP_Process();
	        p = pbuf_alloc(PBUF_TRANSPORT,UDP_BUF_LEN3+2, PBUF_POOL);
			pbuf_take(p, (char*)data3, UDP_BUF_LEN3+2);
			udp_send(upcb,p);
			AdcDmaFlagS3=0;
			pbuf_free(p);
	        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3,RESET);
			}

		if(hadc->Instance==ADC1){
	        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,SET);
		  for(uint16_t j=0;j<ADC_BUF_LEN1/2;j++){
			 data1[2*j+ADC_BUF_LEN1+2]  = pData1[j+ADC_BUF_LEN1/2] & 0xff;
			 data1[2*j+ADC_BUF_LEN1+1+2]=(pData1[j+ADC_BUF_LEN1/2] >> 8);
			}
		    AdcDmaFlagS1++;
	        MX_LWIP_Process();
	        p = pbuf_alloc(PBUF_TRANSPORT,UDP_BUF_LEN1+2, PBUF_POOL);
			pbuf_take(p, (char*)data1, UDP_BUF_LEN1+2);
			udp_send(upcb,p);
			AdcDmaFlagS1=0;
			pbuf_free(p);
	        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2,RESET);

			}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
