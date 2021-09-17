/* USER CODE BEGIN Header */
/**
 ********************************************************************************
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
#include "string.h" // usarttan mesaj gönderirken strlen() fonksiyonunu kullanabilmek için bu kütüphaneyi ekliyoruz
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void Usart_Menu_Yazdir(void); // usart menü fonksiyonunu burada derleyiciye bildiriyoruz
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char mesaj1[] = "Motor Kontrol Menu\r\n0- Dc Ac\r\n1- Dc Kapat\r\n2- Dc Hiz+\r\n3- Dc Hiz-\r\n"; // usart kontrol menüsü karakter dizileri. \r\n alt satıra inmek için kullanılıyor
char mesaj2[] = "4- Step Ac\r\n5- Step Kapat\r\n6- Step Hiz+\r\n7- Step Hiz-\r\n";				// mesajı 3'e bölzmemizin sebebi proteus'un senkronizasyon probleminden kaynaklı
char mesaj3[] = "8- Servo Aci+\r\n9- Servo Aci-\r\n";											// bütün mesajı tek seferde göndermeye kalkınca belli bir yerden sonra yazmıyor
char usartgelen;
uint8_t servo_pwm = 5; // servonun başlangıç pwm değeri
uint8_t dc_pwm = 10;	//dc motorun başlangıç pwm değeri
uint32_t step_hiz = 20;	// step motorun başlangıç bekleme değeri

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {						// Usarttan veri geldiğinde çalışan kesme fonksiyonu
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);
	/* NOTE: This function should not be modified, when the callback is needed,
	 the HAL_UART_RxCpltCallback could be implemented in the user file
	 */

	//switch case döngüsü ile gelen veriye göre işlem yaptırılıyor
	switch (usartgelen) {
	case '0':
		HAL_GPIO_WritePin(DC_Motor_GPIO_Port, DC_Motor_Pin, GPIO_PIN_SET); // eğer 0 geldi ise dc motoru çalıştır
		break;

	case '1':
		HAL_GPIO_WritePin(DC_Motor_GPIO_Port, DC_Motor_Pin, GPIO_PIN_RESET); // eğer 1 geldi ise dc motoru durdur
		break;

	case '2':																// eğer 2 geldi ise dc motorun hızını arttır
		dc_pwm +=10;	//dc motorun hızını 10 birim arttır
		if(dc_pwm >= 100) dc_pwm = 100; // eğer hız değişkeni 100'den büyük olursa 100'e sabitle
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dc_pwm); // yeni hız değerini pwm kanalına gönder
		break;

	case '3':																// eğer 3 geldi ise dc motorun hızını azalt
		dc_pwm -=10;	// dc motorun hızını 10 birim azalt
		if(dc_pwm <= 10) dc_pwm = 10;	// eğer hız değişkeni 10'dan küçükse 10'a sabitle
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dc_pwm); // yeni hız değerini pwm kanalına gönder
		break;

	case '4':
		HAL_GPIO_WritePin(Step_En_GPIO_Port, Step_En_Pin, GPIO_PIN_SET);	// eğer 4 geldi ise step motoru çalıştır
		break;

	case '5':
		HAL_GPIO_WritePin(Step_En_GPIO_Port, Step_En_Pin, GPIO_PIN_RESET);	// eğer 5 geldi ise step motoru durdur
		break;

	case '6':																// eğer 6 geldi ise step motorun hızını arttır
		step_hiz -=4; // step motorun hız değişkenini(bekleme değerini) 4 birim azalt, böylelikle kare dalganın frekansı ve step motorun hızı artacak
		if(step_hiz <= 5) step_hiz = 5;	// eğer hız değeri 5'den küçükse 5'e sabitle
		break;

	case '7':																// eğer 7 geldi ise step motorun hızını azalt
		step_hiz +=4;// step motorun hız değişkenini(bekleme değerini) 4 birim arttır, böylelikle kare dalganın frekansı ve step motorun hızı azalacak
		if(step_hiz >= 20) step_hiz = 20; // eğer hız değeri 20'den büyükse 20'ye sabitle
		break;

	case '8':																// eğer 8 geldi ise servo motorun açısını arttır
		servo_pwm +=1; // servo motorun açsını arttırmak için pwm değerini 1 arttırıyoruz
		if(servo_pwm >= 10) servo_pwm = 10;	// eğer pwm değeri 10'dan büyükse(maks açı değeri +90 derece) pwm değerini 10'a sabitle
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,servo_pwm); // yeni açı değerini pwm kanalına gönder
		break;

	case '9':																// eğer 9 geldi ise servo motorun açısını arttır
		servo_pwm -=1; // servo motorun açsını azaltmak için pwm değerini 1 azaltyoruz
		if(servo_pwm <= 5) servo_pwm = 5; //  eğer pwm değeri 5'ten küçükse(min açı değeri -90 derece) pwm değerini 5'e sabitle
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,servo_pwm); // yeni açı değerini pwm kanalına gönder
		break;
	}

}


void Usart_Menu_Yazdir(void){
	HAL_UART_Transmit(&huart1, (uint8_t*) mesaj1, strlen(mesaj1), 100); // 1. mesajı usarttan yazdır
	HAL_Delay(10);														// 10ms bekle
	HAL_UART_Transmit(&huart1, (uint8_t*) mesaj2, strlen(mesaj2), 100);	// 2. mesajı usarttan yazdır
	HAL_Delay(10);														// 10ms bekle
	HAL_UART_Transmit(&huart1, (uint8_t*) mesaj3, strlen(mesaj3), 100); // 3. mesajı usarttan yazdır
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
  MX_USART1_UART_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE); // Usart kesmesi aktif ediliyor
	Usart_Menu_Yazdir();	//Kontrol menüsü usart'tan yazdırılıyor


	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);	// Servo ve DC motor için pwm kanalları başlatılıyor. 1. Kanal Dc, 2. Kanal Servo motor için.
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, servo_pwm); // Servo kanallarına başlangıç değerleri çıkış olarak veriliyor
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, dc_pwm);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		HAL_UART_Receive_IT(&huart1, &usartgelen, 1); // Usarttan gelen veriyi interrupt yöntemi ile işlemciye alıyoruz ve sonra kesme fonksiyonuna gidiyor

		if(HAL_GPIO_ReadPin(Step_En_GPIO_Port, Step_En_Pin)){	// eğer step motor açılmışsa, yani enable pini 1 olmuşsa step motorun hareket edebilmesi için kare dalga üretiliyor
			HAL_GPIO_TogglePin(Step_Clk_GPIO_Port, Step_Clk_Pin);	// clock frekansı üretmek için clock pini ayarlanan süre kadar toggle yapılıyor
			HAL_Delay(step_hiz);									// kare dalga çıkışı elde etmek için belirlenen süre kadar bekleme yapılıyor
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Step_Clk_Pin|Step_En_Pin|DC_Motor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : Step_Clk_Pin Step_En_Pin DC_Motor_Pin */
  GPIO_InitStruct.Pin = Step_Clk_Pin|Step_En_Pin|DC_Motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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
