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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Posicao{
	int linha;
	int coluna;
	char peca;
}Posicao;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OFF_LED 0x38 << 16u
#define RED_LED 0x08
#define GREEN_LED 0x10
#define BLUE_LED 0x20
#define WHITE_LED 0x38

#define TabStatus (GPIOB->IDR & GPIO_PIN_0)

#define LedON GPIOB->BSRR = GPIO_PIN_2
#define LedOFF GPIOB->BSRR = GPIO_PIN_2 << 16u

#define CounterON GPIOB->BSRR = GPIO_PIN_6
#define CounterOFF GPIOB->BSRR = GPIO_PIN_6 << 16u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char Tab [8][8] = {
	{'T','C', 'B', 'Q', 'K', 'B', 'C', 'T'},
	{'P','P', 'P', 'P', 'P', 'P', 'P', 'P'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'p','p', 'p', 'p', 'p', 'p', 'p', 'p'},
	{'t','c', 'b', 'q', 'k', 'b', 'c', 't'}
};

char TabAtual [8][8] = {
	{'T','C', 'B', 'Q', 'K', 'B', 'C', 'T'},
	{'P','P', 'P', 'P', 'P', 'P', 'P', 'P'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'p','p', 'p', 'p', 'p', 'p', 'p', 'p'},
	{'t','c', 'b', 'q', 'k', 'b', 'c', 't'}
};

uint32_t Led [8][8] = {
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED},
	{OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED, OFF_LED}
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
Posicao VerifyTab();
void AtualizaLed(Posicao p);
void LigaLed();
void SetTable();
void ClearLed();
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	Posicao posicao1;
	posicao1 = VerifyTab() ;
	if ( posicao1.peca != '@' ){
		AtualizaLed(posicao1);
		TabAtual[posicao1.linha][posicao1.coluna] = '-';
	}
	posicao1 = VerifyMov();

	LigaLed();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB3 PB4
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ClearLed() {
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			Led[j][i]= OFF_LED;
		}
	}
}

void AtualizaLed(Posicao p){
	Led[ p.linha ][ p.coluna ] = WHITE_LED;
}

void LigaLed(){
	CounterON;
	LedON;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			GPIOB->BSRR = Led[j][i];
			SetTable();
		}
	}
	CounterOFF;
	LedOFF;
}

/*
 * Virifica se uma casa teve uma peça removida do tabuleiro
 */
Posicao VerifyTab(){
	CounterON;
	Posicao posicao;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			if (!TabStatus && Tab[j][i] != '-' ){
				posicao.linha = j;
				posicao.coluna = i;
				posicao.peca = Tab[j][i];
				CounterOFF;
				return posicao;
			}
			SetTable();
		}
	}
	CounterOFF;
	posicao.linha=8;
	posicao.coluna=8;
	posicao.peca = '@';
	return posicao;
}

Posicao VerifyMov(){
	CounterON;
	Posicao posicao;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			if ((!TabStatus && TabAtual[j][i] != '-')||(TabStatus && TabAtual[j][i] == '-') ){
				posicao.linha = j;
				posicao.coluna = i;
				posicao.peca = Tab[j][i];
				CounterOFF;
				return posicao;
			}
			SetTable();
		}
	}
	CounterOFF;
	posicao.linha=8;
	posicao.coluna=8;
	posicao.peca = '@';
	return posicao;
}

/*
 * Altera a casa do tabuleiro apontada pelo contador
 */
void SetTable(){
	GPIOB->BSRR = GPIO_PIN_1;
	HAL_Delay(1);
	GPIOB->BSRR = GPIO_PIN_1 << 16u;
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
