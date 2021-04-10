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
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Posicao{
	int linha;
	int coluna;
}Posicao;

typedef struct Peca{
	Posicao posicao;
	char nome;
	uint32_t movimentos;
}Peca;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OFF_LED                 0x38 << 16u
#define RED_LED                 0x08
#define GREEN_LED               0x10
#define BLUE_LED                0x20
#define WHITE_LED               0x38

#define TabStatus               (GPIOB->IDR & GPIO_PIN_0)

#define LedON                   GPIOB->BSRR = GPIO_PIN_2
#define LedOFF                  GPIOB->BSRR = GPIO_PIN_2 << 16u

#define CounterON               GPIOB->BSRR = GPIO_PIN_6
#define CounterOFF              GPIOB->BSRR = GPIO_PIN_6 << 16u

/*Trabalhando com display*/
#define LCD_E                   GPIO_PIN_8
#define LCD_RW                  GPIO_PIN_9
#define LCD_RS                  GPIO_PIN_10
#define LCD_RS_LOW              GPIOA->BSRR = LCD_RS << 16u
#define LCD_RS_HIGH             GPIOA->BSRR = LCD_RS
#define LCD_E_LOW               GPIOA->BSRR = LCD_E << 16u
#define LCD_E_HIGH              GPIOA->BSRR = LCD_E
#define LCD_E_BLINK             LCD_E_HIGH; LCD_Delay_us(50); LCD_E_LOW; LCD_Delay_us(50)
/* Commands*/
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80
/* Flags for display entry mode */
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
/* Flags for display on/off control */
#define LCD_DISPLAYON           0x04
#define LCD_CURSORON            0x02
#define LCD_BLINKON             0x01
/* Flags for display/cursor shift */
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00
/* Flags for function set */
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t Turno = 0;

Peca Tabuleiro[8][8]= {
	{
		{.posicao.linha = 0, .posicao.coluna = 0, .nome = 'T', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 1, .nome = 'C', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 2, .nome = 'B', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 3, .nome = 'Q', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 4, .nome = 'K', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 5, .nome = 'B', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 6, .nome = 'C', .movimentos = 0},
		{.posicao.linha = 0, .posicao.coluna = 7, .nome = 'T', .movimentos = 0}
	},
	{
		{.posicao.linha = 1, .posicao.coluna = 0, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 1, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 2, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 3, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 4, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 5, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 6, .nome = 'P', .movimentos = 0},
		{.posicao.linha = 1, .posicao.coluna = 7, .nome = 'P', .movimentos = 0}
	},
	{
		{.posicao.linha = 2, .posicao.coluna = 0, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 1, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 2, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 3, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 4, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 5, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 6, .nome = '-', .movimentos = 0},
		{.posicao.linha = 2, .posicao.coluna = 7, .nome = '-', .movimentos = 0}
	},
	{
		{.posicao.linha = 3, .posicao.coluna = 0, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 1, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 2, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 3, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 4, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 5, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 6, .nome = '-', .movimentos = 0},
		{.posicao.linha = 3, .posicao.coluna = 7, .nome = '-', .movimentos = 0}
	},
	{
		{.posicao.linha = 4, .posicao.coluna = 0, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 1, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 2, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 3, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 4, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 5, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 6, .nome = '-', .movimentos = 0},
		{.posicao.linha = 4, .posicao.coluna = 7, .nome = '-', .movimentos = 0}
	},
	{
		{.posicao.linha = 5, .posicao.coluna = 0, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 1, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 2, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 3, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 4, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 5, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 6, .nome = '-', .movimentos = 0},
		{.posicao.linha = 5, .posicao.coluna = 7, .nome = '-', .movimentos = 0}
	},
	{
		{.posicao.linha = 6, .posicao.coluna = 0, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 1, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 2, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 3, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 4, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 5, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 6, .nome = 'p', .movimentos = 0},
		{.posicao.linha = 6, .posicao.coluna = 7, .nome = 'p', .movimentos = 0}
	},
	{
		{.posicao.linha = 7, .posicao.coluna = 0, .nome = 't', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 1, .nome = 'c', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 2, .nome = 'b', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 3, .nome = 'q', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 4, .nome = 'k', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 5, .nome = 'b', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 6, .nome = 'c', .movimentos = 0},
		{.posicao.linha = 7, .posicao.coluna = 7, .nome = 't', .movimentos = 0}
	}
};
uint16_t MovPos [8][8] = {
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0},
	{0,0, 0, 0, 0,0, 0, 0}
};



char TabAtual [8][8] = {
	{'T','C', 'B', 'Q', 'K', 'B', 'C', 'T'},
	{'P','P', 'P', 'P', 'P', 'P', 'P', 'P'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','-', '-', '-', '-', '-', '-', '-'},
	{'-','T', '-', '-', '-', '-', '-', '-'},
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


char* LCD_Texto= "  Smart Chaess  \n      Board     ";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
Peca* VerifyTab();
Peca* VerifyMov();
void Troca(Peca* p,Peca* p2);
void AtualizaLed(Peca* p);
void MovPossiveis(Peca* p);
void LigaLed();
void SetTable();
void ClearLed();
void  LCD_Delay_us(uint16_t  us);
static void LCD_Cmd(uint8_t cmd);
static void LCD_Data(uint8_t data);
void LCD_Init();
void LCD_Clear();
void LCD_Write(char* str);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(Turno==0){
		LCD_Write(" Jogador da vez:\n    Brancas     ");
	}
	else{
		LCD_Write(" Jogador da vez:\n    Pretas      ");
	}
	Peca* p;
	p = VerifyTab() ;
	if ( p != NULL ){
		if((islower(p->nome)&&Turno!=0)||(!islower(p->nome)&&Turno==0)){
			AtualizaLed(p);
			LigaLed();
			TabAtual[p->posicao.coluna][p->posicao.coluna ] = '-';
			Peca* p2 = VerifyMov();
			while(p2 == NULL){
				LigaLed();
				p2 = VerifyMov();
			}
			ClearLed();
			if(p2->nome=='-'){
				Troca(p,p2);
			}
			else{
				Led[p2->posicao.linha][p2->posicao.coluna] = WHITE_LED;
				TabAtual[p2->posicao.coluna][p2->posicao.coluna ] = '-';
				p2->nome = '-';
				for(int i=0;i<8;i++){
					for(int j=0;j<8;j++){
						MovPos[i][j]=0;
					}
				}
				MovPos[p2->posicao.linha][p2->posicao.coluna]=1;
				p2 = VerifyMov();
				while(p2 == NULL){
					LigaLed();
					p2 = VerifyMov();
				}

				Troca(p,p2);
			}
			Turno = (Turno+1)%2;
		}
		else{
			Led[p->posicao.linha][p->posicao.coluna] = RED_LED;
			LigaLed();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* Desliga todos os leds do tabuleiro */
void ClearLed() {
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			Led[i][j]= OFF_LED;
		}
	}
}

/* Define a iluminação do tabuleiro */
void AtualizaLed(Peca* p){
	MovPossiveis(p);
	for(int i=0;i<8;i++){
		for(int j=0;j<8;j++){
			if(MovPos[i][j]){
				Led[i][j] = WHITE_LED;
			}
			else{
				Led[j][i]= OFF_LED;
			}
		}
	}

	LigaLed();
}
void MovPossiveis(Peca* p){
	int i=0;
	Posicao posicao_aux = p->posicao;
	switch ( p->nome ){
	case'P':
		if ( (islower( p->nome ) != islower( TabAtual[posicao_aux.linha+1][posicao_aux.coluna+1])) && (posicao_aux.coluna+1<8) ){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna+1] = 1;}
		if(islower(p->nome) != islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna-1]) && posicao_aux.coluna-1>-1 ){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna-1] = 1;}
		if ( posicao_aux.linha==1){
			if( TabAtual[posicao_aux.linha+2][posicao_aux.coluna]=='-' ){

				MovPos[posicao_aux.linha+2][posicao_aux.coluna] = 1;
			}
		}
		if( TabAtual[posicao_aux.linha+1][posicao_aux.coluna]=='-' ){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna] = 1;
		}
		break;
	case'p':
		if ( islower(p->nome) != islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna-1]) && posicao_aux.coluna+1<8 ){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna-1] = 1;}
		if ( islower(p->nome) != islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna-1]) && posicao_aux.coluna-1>-1 ){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna-1] = 1;}
		if ( posicao_aux.linha==6){
			if( TabAtual[posicao_aux.linha-2][posicao_aux.coluna]=='-' ){

				MovPos[posicao_aux.linha-2][posicao_aux.coluna] = 1;
			}
		}
		if( TabAtual[posicao_aux.linha-1][posicao_aux.coluna]=='-' ){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna] = 1;
		}
		break;

	case 'T':
	case 't':
        while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna]) ) && posicao_aux.linha>-1){

		    MovPos[posicao_aux.linha+i][posicao_aux.coluna] = 1;
			i = i-1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna]) ) && posicao_aux.linha<8){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna] = 1;
			i = i+1;

		}
		i=0;
		while ( (TabAtual[posicao_aux.linha][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna+i]) ) && posicao_aux.coluna>-1){

			MovPos[posicao_aux.linha][posicao_aux.coluna+i] = 1;
			i = i-1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna+i]) ) && posicao_aux.linha<8){

			MovPos[posicao_aux.linha][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		break;
	case 'c':
	case 'C':
		if ( (TabAtual[posicao_aux.linha-2][posicao_aux.coluna+1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-2][posicao_aux.coluna+1]) ) && posicao_aux.linha-2>-1 && posicao_aux.coluna+1<8){

		    MovPos[posicao_aux.linha-2][posicao_aux.coluna+1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha-1][posicao_aux.coluna+2]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna+2]) ) && posicao_aux.linha-1>-1 && posicao_aux.coluna+2<8){

		    MovPos[posicao_aux.linha-1][posicao_aux.coluna+2] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+1][posicao_aux.coluna+2]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna+2]) ) && posicao_aux.linha+1<8 && posicao_aux.coluna+2<8){

		    MovPos[posicao_aux.linha+1][posicao_aux.coluna+2] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+2][posicao_aux.coluna+1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+2][posicao_aux.coluna+1]) ) && posicao_aux.linha+2<8 && posicao_aux.coluna+1<8){

		    MovPos[posicao_aux.linha+2][posicao_aux.coluna+1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+2][posicao_aux.coluna-1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+2][posicao_aux.coluna-1]) ) && posicao_aux.linha+2<8 && posicao_aux.coluna-1>-1){

		    MovPos[posicao_aux.linha+2][posicao_aux.coluna-1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+1][posicao_aux.coluna-2]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna-2]) ) && posicao_aux.linha+1<8 && posicao_aux.coluna-2>-1){

		    MovPos[posicao_aux.linha+1][posicao_aux.coluna-2] = 1;
		}
		if ( (TabAtual[posicao_aux.linha-1][posicao_aux.coluna-2]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna-2]) ) && posicao_aux.linha-1>-1 && posicao_aux.coluna-2>-1){

		    MovPos[posicao_aux.linha-1][posicao_aux.coluna-2] = 1;
		}
		if ( (TabAtual[posicao_aux.linha-2][posicao_aux.coluna-1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-2][posicao_aux.coluna-1]) ) && posicao_aux.linha-2>-1 && posicao_aux.coluna-1>-1){

		    MovPos[posicao_aux.linha-2][posicao_aux.coluna-1] = 1;
		}
		break;

	case'b':
	case'B':
		while ( (TabAtual[posicao_aux.linha-i][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-i][posicao_aux.coluna+i]) ) && posicao_aux.linha-i>-1 && posicao_aux.coluna+i<8){

			MovPos[posicao_aux.linha-i][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna+i]) ) && posicao_aux.linha+i<8 && posicao_aux.coluna+i<8){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna-i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna-i]) ) && posicao_aux.linha+i<8 && posicao_aux.coluna-i>-1){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna-i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha-i][posicao_aux.coluna-i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-i][posicao_aux.coluna-i]) ) && posicao_aux.linha-i>-1 && posicao_aux.coluna-i>-1){

			MovPos[posicao_aux.linha-i][posicao_aux.coluna-i] = 1;
			i = i+1;
		}
		i=0;
		break;
	case 'q':
	case 'Q':
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna]) ) && posicao_aux.linha>-1){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna] = 1;
			i = i-1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna]) ) && posicao_aux.linha<8){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna+i]) ) && posicao_aux.coluna>-1){

			MovPos[posicao_aux.linha][posicao_aux.coluna+i] = 1;
			i = i-1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna+i]) ) && posicao_aux.linha<8){

			MovPos[posicao_aux.linha][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		while ( (TabAtual[posicao_aux.linha-i][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-i][posicao_aux.coluna+i]) ) && posicao_aux.linha-i>-1 && posicao_aux.coluna+i<8){

			MovPos[posicao_aux.linha-i][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna+i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna+i]) ) && posicao_aux.linha+i<8 && posicao_aux.coluna+i<8){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna+i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha+i][posicao_aux.coluna-i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+i][posicao_aux.coluna-i]) ) && posicao_aux.linha+i<8 && posicao_aux.coluna-i>-1){

			MovPos[posicao_aux.linha+i][posicao_aux.coluna-i] = 1;
			i = i+1;
		}
		i=0;
		while ( (TabAtual[posicao_aux.linha-i][posicao_aux.coluna-i]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-i][posicao_aux.coluna-i]) ) && posicao_aux.linha-i>-1 && posicao_aux.coluna-i>-1){

			MovPos[posicao_aux.linha-i][posicao_aux.coluna-i] = 1;
			i = i+1;
			}
		i=0;
		break;
	case'K':
	case'k':
		if ( (TabAtual[posicao_aux.linha-1][posicao_aux.coluna+1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna+1]) ) && posicao_aux.linha-1>-1 && posicao_aux.coluna+1<8){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna+1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha-1][posicao_aux.coluna-1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna-1]) ) && posicao_aux.linha-1>-1 && posicao_aux.coluna-1>-1){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna-1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+1][posicao_aux.coluna+1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna+1]) ) && posicao_aux.linha+1<8 && posicao_aux.coluna+1<8){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna+1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+1][posicao_aux.coluna-1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna-1]) ) && posicao_aux.linha+1<8 && posicao_aux.coluna-1>-1){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna-1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha-1][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha-1][posicao_aux.coluna]) ) && posicao_aux.linha-1>-1){

			MovPos[posicao_aux.linha-1][posicao_aux.coluna] = 1;
		}
		if ( (TabAtual[posicao_aux.linha+1][posicao_aux.coluna]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha+1][posicao_aux.coluna]) ) && posicao_aux.linha+1<8){

			MovPos[posicao_aux.linha+1][posicao_aux.coluna] = 1;
		}
		if ( (TabAtual[posicao_aux.linha][posicao_aux.coluna+1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna+1]) ) && posicao_aux.coluna+1<8){

			MovPos[posicao_aux.linha][posicao_aux.coluna+1] = 1;
		}
		if ( (TabAtual[posicao_aux.linha][posicao_aux.coluna-1]=='-' || islower(p->nome)!=islower(TabAtual[posicao_aux.linha][posicao_aux.coluna-1]) ) && posicao_aux.coluna-1>-1){

			MovPos[posicao_aux.linha][posicao_aux.coluna-1] = 1;
		}

	default:
		break;
	}
}

/* Acende os leds do tabuleiro */
void LigaLed(){
	CounterON;
	LedON;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			GPIOB->BSRR = Led[j][i];
			HAL_Delay(5);
			GPIOB->BSRR = OFF_LED;
			SetTable();
		}
	}
	CounterOFF;
	LedOFF;
}

/* Virifica se uma casa teve uma peça removida do tabuleiro */
Peca* VerifyTab(){
	CounterON;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			if (!TabStatus && Tabuleiro[j][i].nome != '-' ){
				CounterOFF;
				return &Tabuleiro[j][i];
			}
			SetTable();
		}
	}
	CounterOFF;
	return NULL;
}


/* Verifica os movimentos realizados no tabuleiro */
Peca* VerifyMov(){
	CounterON;
	for(int i=0;i<8;i++){
		for (int j=0;j<8;j++){
			if ( ( (!TabStatus && TabAtual[j][i] != '-') || (TabStatus && TabAtual[j][i] == '-') ) && MovPos[j][i] ){
				CounterOFF;
				return &Tabuleiro[j][i];
			}
			SetTable();
		}
	}
	CounterOFF;
	return NULL;
}
void Troca(Peca* p,Peca* p2){
	Posicao aux = p->posicao;
	p->posicao  = p2->posicao;
	p2->posicao = aux;
	Tabuleiro[p->posicao.linha][p->posicao.coluna] = *p;
	Tabuleiro[p2->posicao.linha][p2->posicao.coluna] = *p2;
}

/* Altera a casa do tabuleiro apontada pelo contador */
void SetTable(){
	GPIOB->BSRR = GPIO_PIN_1;
	HAL_Delay(1);
	GPIOB->BSRR = GPIO_PIN_1 << 16u;
}




/* Funções do LCD
 * Adaptado de https://github.com/nimaltd/LCD-Character
 * */



void  LCD_Delay_us(uint16_t  us)
{
  uint32_t  Div = (SysTick->LOAD+1)/1000;
  uint32_t  StartMicros = HAL_GetTick()*1000 + (1000- SysTick->VAL/Div);
  while((HAL_GetTick()*1000 + (1000-SysTick->VAL/Div)-StartMicros < us));
}
static void LCD_Cmd(uint8_t cmd)
{
	GPIOA->ODR = cmd;
	LCD_RS_LOW;
	LCD_E_BLINK;
}
static void LCD_Data(uint8_t data)
{
	GPIOA->ODR = data;
	LCD_RS_HIGH;
	LCD_E_BLINK;
}
void LCD_Init()
{
	LCD_Cmd(LCD_FUNCTIONSET | LCD_8BITMODE | LCD_5x8DOTS | LCD_2LINE);
	HAL_Delay(5);
	LCD_Cmd(LCD_DISPLAYCONTROL | LCD_DISPLAYON  |LCD_BLINKON);
	HAL_Delay(5);
}
void LCD_Clear()
{
	LCD_Cmd(LCD_CLEARDISPLAY);
	HAL_Delay(5);
}
void LCD_Write(char* str)
{
	LCD_Cmd(LCD_CLEARDISPLAY);
	while (*str)
	{
		if (*str == '\n')
		{
			LCD_Cmd(LCD_SETDDRAMADDR | 0x40);
		}
		LCD_Data(*str);
		str++;
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
