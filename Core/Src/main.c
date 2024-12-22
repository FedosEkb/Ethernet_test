/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUFF_LEN 100
typedef struct {
    ETH_BufferTypeDef *AppBuff;
    uint8_t buffer[BUFF_LEN]__ALIGNED(32);
} ETH_AppBuff;
typedef struct {
    uint8_t dest_mac[6];
    uint8_t src_mac[6];
    uint8_t type[2];
    uint8_t payload[BUFF_LEN];
} ethernet_frame_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEGUG_MESSAGE_ON


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ETH_Init(void);
/* USER CODE BEGIN PFP */
int32_t ETH_PHY_INTERFACE_Init(void);
int32_t ETH_PHY_INTERFACE_DeInit(void);
int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr,
		uint32_t *pRegVal);
int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr,
		uint32_t RegVal);
int32_t ETH_PHY_INTERFACE_GetTick(void);
void ETH_StartLink();
void ETH_ConstructEthernetFrame(ethernet_frame_t *frame, uint8_t *dest_mac,
		uint8_t *src_mac, uint8_t *type, uint8_t *payload, uint16_t payload_len);

/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_INTERFACE_Init,
                                  ETH_PHY_INTERFACE_DeInit,
                                  ETH_PHY_INTERFACE_WriteReg,
                                  ETH_PHY_INTERFACE_ReadReg,
                                  ETH_PHY_INTERFACE_GetTick};
static int inc = 0; // NOTE для счетчика принятых/отправленных кадров
ETH_BufferTypeDef *frame_Rx=NULL;

/**
  * @brief Overload callback for
  * @retval None
  */
int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	ETH_BufferTypeDef TxBuffer;
	ethernet_frame_t frame;
	uint8_t dest_mac[] = { 0x00, 0x80, 0xE1, 0x00, 0x00, 0x10 }; // Destination MAC Address
	uint8_t src_mac[] = { 0x00, 0x80, 0xE1, 0x00, 0x00, 0x00 }; // Source MAC Address
	uint8_t type[] = { 0x08, 0x00 }; // EtherType set to IPV4 packet
	uint8_t payload[] = { 0x54, 0x65, 0x73, 0x74, 0x69, 0x6e, 0x67, 0x20, 0x45,
			0x74, 0x68, 0x65, 0x72, 0x6e, 0x65, 0x74, 0x20, 0x6f, 0x6e, 0x20,
			0x53, 0x54, 0x4d, 0x33, 0x32 };
	uint16_t payload_len = sizeof(payload);

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
  MX_TIM6_Init();
  MX_ETH_Init();
  /* USER CODE BEGIN 2 */
  printf("Program started!\n\r");
  fflush(0);
//  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_OnePulse_Start_IT(&htim6, TIM_CHANNEL_ALL);

  ETH_ConstructEthernetFrame(&frame, dest_mac, src_mac, type, payload, payload_len);
  TxBuffer.buffer = (uint8_t *)&frame;
  TxBuffer.len = sizeof(dest_mac) + sizeof(src_mac) + sizeof(type) + payload_len;
  TxBuffer.next = NULL;
  TxConfig.TxBuffer = &TxBuffer;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */
	/* Set PHY IO functions */
	LAN8742_RegisterBusIO(&LAN8742, &LAN8742_IOCtx);
	/* Initialize the LAN8742 ETH PHY */
	LAN8742_Init(&LAN8742);
	/* Initialize link speed negotiation and start Ethernet peripheral */
	ETH_StartLink();

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 65535;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim6, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */



  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void ETH_ConstructEthernetFrame(ethernet_frame_t * frame, uint8_t * dest_mac, uint8_t * src_mac, uint8_t * type, uint8_t * payload, uint16_t payload_len)
{
  // Copy the destination MAC address
  memcpy(frame -> dest_mac, dest_mac, 6);
  // Copy the source MAC address
  memcpy(frame -> src_mac, src_mac, 6);
  // Set the Ethernet type field
  memcpy(frame -> type, type, 2);
  // Copy the payload data
  memcpy(frame -> payload, payload, payload_len);
}

/*************PHY_INIT_START**********************/

int32_t ETH_PHY_INTERFACE_Init(void)
{
  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&heth);
  return 0;
}
int32_t ETH_PHY_INTERFACE_DeInit (void)
{
  return 0;
}
int32_t ETH_PHY_INTERFACE_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }
  return 0;
}
int32_t ETH_PHY_INTERFACE_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }
  return 0;
}
int32_t ETH_PHY_INTERFACE_GetTick(void)
{
  return HAL_GetTick();
}


void ETH_StartLink() {
	ETH_MACConfigTypeDef MACConf = { 0 };
	int32_t PHYLinkState = LAN8742_GetLinkState(&LAN8742);
	if (PHYLinkState <= LAN8742_STATUS_LINK_DOWN)
	{
		HAL_ETH_Stop(&heth);
	}
	else if (PHYLinkState > LAN8742_STATUS_LINK_DOWN)
	{
		uint32_t linkchanged = 0U;
		uint32_t speed = 0U;
		uint32_t duplex = 0U;
		switch (PHYLinkState) {
		case LAN8742_STATUS_100MBITS_FULLDUPLEX:
			duplex = ETH_FULLDUPLEX_MODE;
			speed = ETH_SPEED_100M;
			linkchanged = 1;
			break;
		case LAN8742_STATUS_100MBITS_HALFDUPLEX:
			duplex = ETH_HALFDUPLEX_MODE;
			speed = ETH_SPEED_100M;
			linkchanged = 1;
			break;
		case LAN8742_STATUS_10MBITS_FULLDUPLEX:
			duplex = ETH_FULLDUPLEX_MODE;
			speed = ETH_SPEED_10M;
			linkchanged = 1;
			break;
		case LAN8742_STATUS_10MBITS_HALFDUPLEX:
			duplex = ETH_HALFDUPLEX_MODE;
			speed = ETH_SPEED_10M;
			linkchanged = 1;
			break;
		default:
			break;
		}
		if (linkchanged) {
			HAL_ETH_GetMACConfig(&heth, &MACConf);
			MACConf.DuplexMode = duplex;
			MACConf.Speed = speed;
			MACConf.DropTCPIPChecksumErrorPacket = DISABLE;
			MACConf.ForwardRxErrorPacket = ENABLE;   //FEP Bit
			MACConf.ForwardRxUndersizedGoodPacket = ENABLE;
			MACConf.LoopbackMode= DISABLE;
			HAL_ETH_SetMACConfig(&heth, &MACConf);
			HAL_ETH_Start_IT(&heth);
		}
	}
}

/*************PHY_INIT_STOP**********************/


/*************CALBACK_OVERLOAD_START**********************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);

		// Clear interrupt bit in SR that was set as a side effect of generating an update event in TIM_Base_SetConfig
		__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
		// enable timer update interrupt
		__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
		__HAL_TIM_ENABLE(&htim6);

	    HAL_ETH_Transmit_IT( & heth, & TxConfig);
	    HAL_ETH_ReleaseTxPacket( & heth);
	    HAL_ETH_ReadData( & heth, (void ** ) & frame_Rx);

		HAL_GPIO_TogglePin(GPIOD,
				GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
#ifdef DEGUG_MESSAGE_ON
		printf("Button was been pushed!\n\r");
		fflush(0);
#endif /*DEGUG_MESSAGE_ON*/
//      HAL_ETH_Transmit_IT(&heth, &TxConfig);
//      HAL_ETH_ReleaseTxPacket(&heth);

	} else {
		__NOP();
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim6) // check source
		{
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0); // NOTE отчистим предыдущие значение если оно там было
		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#ifdef DEGUG_MESSAGE_ON
		printf("Timer interrupt was accrued !\n\r");
		fflush(0);
#endif /*DEGUG_MESSAGE_ON*/

	} else {
		__NOP();
	}
}

/*************ETHERNET_CALBACK_OVERLOAD_START**********************/

/**
  * @brief  Rx Allocate callback.
  * @param  buff: pointer to allocated buffer
  * @retval None
  */
void HAL_ETH_RxAllocateCallback(uint8_t ** buff) {
  ETH_BufferTypeDef * p = malloc(BUFF_LEN * sizeof(uint8_t));
  if (p)
  {
    * buff = (uint8_t * ) p + offsetof(ETH_AppBuff, buffer);
    p -> next = NULL;
    p -> len = BUFF_LEN;
  } else {
    * buff = NULL;
  }
}

/**
  * @brief  Rx Link callback.
  * @param  pStart: pointer to packet start
  * @param  pStart: pointer to packet end
  * @param  buff: pointer to received data
  * @param  Length: received data length
  * @retval None
  */
void HAL_ETH_RxLinkCallback(void ** pStart, void ** pEnd, uint8_t * buff, uint16_t Length)
{
  ETH_BufferTypeDef ** ppStart = (ETH_BufferTypeDef ** ) pStart;
  ETH_BufferTypeDef ** ppEnd = (ETH_BufferTypeDef ** ) pEnd;
  ETH_BufferTypeDef * p = NULL;
  p = (ETH_BufferTypeDef * )(buff - offsetof(ETH_AppBuff, buffer));
  p -> next = NULL;
  p -> len = BUFF_LEN;
  if (! * ppStart)
  {
    * ppStart = p;
  } else
  {
    ( * ppEnd) -> next = p;
  }
  * ppEnd = p;
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef * heth)
{
  printf("Packet Transmitted successfully!\r\n");
  fflush(0);
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  heth: pointer to a ETH_HandleTypeDef structure that contains
  *         the configuration information for ETHERNET module
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef * heth)
{
  printf("Packet Received successfully!\r\n");
  fflush(0);
}

/*************ETHERNET_CALBACK_OVERLOAD_END**********************/

/*************CALBACK_OVERLOAD_STOP**********************/


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
