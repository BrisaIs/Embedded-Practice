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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */
int USART6_write (int ch);
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART1_UART_Init(void);

void	I2C_Addr (unsigned char adr);
void	I2C_Start();
void	I2C_Stop (void);
void	I2C_Write (unsigned char c);
unsigned char	I2C_Read (int ack);

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
	uint8_t data_t[1];
	data_t[0]='1';
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /*Escribir un dato*/


	I2C_Start( );
	I2C_Addr((0xA0));
	I2C_Write(((0)&0xFFFF)>>8);
	I2C_Write(((0)&0xFF));
	I2C_Write(78);
	I2C_Stop( );
	HAL_Delay(1000);

	/*Leer un dato*/
	I2C_Start( );
	I2C_Addr((0xA0));
	I2C_Write(((0)&0xFFFF)>>8);
	I2C_Write(((0)&0xFF));
	I2C_Start( );
	I2C_Addr(((0xA1)));
	uint8_t u8i=I2C_Read(0); /*Envia NACK para avisar el fin*/
	I2C_Stop( );
	HAL_Delay(1000);
	/*Leer Pagina*/
	I2C_Start( );
	I2C_Addr((0xA0));
	I2C_Write(((0)&0xFFFF)>>8);
	I2C_Write(((0)&0xFF));
	I2C_Start( );
	I2C_Addr(((0xA1)));
	uint8_t u8Data[10];
	 for(unsigned int i=0; i<10; i++)
	 {
		 if(i<9)
		 {
			 u8Data[i]=I2C_Read(1);/*Envia ACK cada vez que recibe*/
		 }
		 else
		 {
			 u8Data[i]=I2C_Read(0);/*Envia NACK para avisar ultimo dato de lectura*/
		 }
	 }
	 I2C_Stop( );
	 HAL_Delay(1000);

	/*Escribir Paginas*/
	I2C_Start( );
	I2C_Addr((0xA0));
	I2C_Write(((0)&0xFFFF)>>8);
	I2C_Write(((0)&0xFF));
	uint8_t u8Data2[10]={9,8,7,6,5,4,3,2,1,0};
	 for(unsigned int i=0; i<10; i++)
	 {
		 I2C_Write(u8Data2[i]);
	 }
	I2C_Stop( );
	HAL_Delay(1000);
	  //		char res;
  while (1)
  {
    /* USER CODE END WHILE */
	  uint16_t u16ir=0;
	  uint16_t u16Data;
	  I2C_Start( );
      I2C_Addr((8));
	  I2C_Write('A');
	  I2C_Write('R');
	  I2C_Write('D');
	  I2C_Write('U');
	  I2C_Write('I');
	  I2C_Write('N');
	  I2C_Write('O');
	  I2C_Stop( );
		  HAL_Delay(100);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
#ifdef STM32F401xE
  RCC->AHB1ENR |= 2;
  RCC->APB1ENR |= 0x00200000;/*Habilita I2C1*/

  GPIOB->MODER |= 0xa0000;/*Habilita Modo Alternativa Funcion a Pin 8 y Pin 9*/
  GPIOB->AFR[1] |= 0x44;/*AF4 para los pines 8 y 9 conforme a la dathaseet  STM32F401xD  https://www.mouser.com/datasheet/2/389/stm32f401re-956236.pdf*/
  GPIOB->OTYPER |= 0x0300;/*Se configura Registro como Output open-Drain*/
  GPIOB->OSPEEDR|= 0xf0000;/*Se configura Very high speed*/
  GPIOB->PUPDR  |= 0x50000;/*Configuración como PULLUP*/


  I2C1->CR2 |= 0x002a; //42 MHZ == (pclk1/1000000 )
/*
 *        Tr(SCL) + Tw(SCLH)      1000  + 4000 ns           Ver la tabla I2C Characteristics, para obtener valores de Tr(SCL) y Tw(SCLH)
 * CCR=  ___________________  =   _______________ =  210
 *        TPCLK1                        23.809 ns                       TPCLK1 = 1/42Mhz
 * */
  I2C1->CCR |= 0xd2; /*Standar Mode and clock*/  //1/42MZ, 100kz, 5000ns
 // I2C1->OAR1 |= 0x4000;/*4 siempre se debe de mantener en 1 por software, */
  I2C1->TRISE |= 0x02c;
 /*
   *         Tr(SCL)              1000  ns
   * TRISE = _________ + 1 =    ___________ + 1 = 43 + 1
   *          TPCLK1              23.80 ns
   *
   * */

  I2C1->CR1 |= 0x001;/*Se habilita el periferico*/

#else if STM32F767

  RCC->AHB1ENR |= 2; /*Activa GPIOB */
  RCC->APB1ENR |= 1<<21; /*Habilita el I2C1*/

  I2C1
  I2C1->CR1 |=0x001; /*PE enable*/


#endif

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */


  RCC->APB2ENR |= RCC_APB2ENR_USART6EN;/*USART6*/
/*Puedes configurar USART6 con el puerto C o con el puerto A*/
/*GPIOA11 y GPIOA12*/
//  GPIOA->MODER |= 0x02800000;
//  GPIOA->AFR[1] |= 0x88000; /*Function Alternativa Pin 11 y 12*/

  /*GPIOC6 y GPIOC7*/
    GPIOC->MODER |= 0x00A000;
    GPIOC->AFR[0] |= 0x88000000; /*Function Alternativa Pin 6  y 7*/


  USART6->BRR =0x2d9; /*115200*/
  USART6->CR1|= 0xC; /*Hablita Transmision y Recepcion*/
  USART6->CR2=0;
  USART6->CR3=0;
  USART6->CR1|= 0x2000; /*Habiliata UART*/

}
static void MX_USART1_UART_Init(void)
{



  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;/*USART1*/
/*Puedes configurar USART1 con el puerto C o con el puerto A*/
/*GPIOA9 y GPIOA10*/
  GPIOA->MODER |= 0x0280000;
  GPIOA->AFR[1] |= 0x770; /*Function Alternativa Pin 11 y 12*/

  /*GPIOB6 y GPIOB7*/
//   GPIOB->MODER |= 0x00A000;
//   GPIOB->AFR[0] |= 0x77000000; /*Function Alternativa Pin 6 y 12*/


  USART1->BRR =0x2d9; /*115200*/
  USART1->CR1|= 0xC; /*Hablita Transmision y Recepcion*/
  USART1->CR2=0;
  USART1->CR3=0;
  USART1->CR1|= 0x2000; /*Habiliata UART*/

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

void USART6_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2, PA3 for USART2 TX, RX */
    GPIOA->AFR[0] &= ~0xFF00;
    GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */

    USART2->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

/* Write a character to USART2 */
int USART6_write (int ch) {
    while (!(USART1->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART1->DR = (ch & 0xFF);
    return ch;
}

/* Read a character from USART2 */
int USART6_read(void) {
    while (!(USART1->SR & 0x0020)) {}   // wait until char arrives
    return USART1->DR;
}


void	I2C_Write (unsigned char c)
{
	char res;
I2C1->DR = c;
int x=0;
while (!(I2C1->SR1 & (1<<7)) && 1000>x)
{
	x++;
}

}

unsigned char	I2C_Read (int ack)
{
	int x=0;
	if (ack)
	{
		I2C1->CR1 |= 0x0400;
	}
    else
    {
    	I2C1->CR1 &= ~0x0400;
    }
while (!(I2C1->SR1 & 0x00000040) && 1000 > x)
{
	x++;
}

return (I2C1->DR);
}
void	I2C_Addr (unsigned char adr)
{
uint8_t res;
int x=0;

I2C1->DR = adr | 0;
while (!(I2C1->SR1 & 0x0002) && 1000 > x)
{
	x++;
}
res = (I2C1->SR2);
}
void	I2C_Start() {
 I2C1->CR1 |= 1<<8;	//I2C Start
 int x=0;
 while (!(I2C1->SR1 & 0x0001) && 1000>x)
	 {
	 x++;
	 }
}
void	I2C_Stop (void) {
 I2C1->CR1 |= 0x0200;
 int x=0;
 while (I2C1->SR2 & 0x0002 && 1000>x)
 {
	 x++;
 }
}








/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
