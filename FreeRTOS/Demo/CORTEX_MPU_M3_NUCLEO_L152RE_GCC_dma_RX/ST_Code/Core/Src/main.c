/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "FreeRTOS.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#if (configUSE_TRACE_FACILITY ==1)
traceHandle USARTHandle;
traceHandle DMAHandle;
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
__IO ITStatus UartReady = RESET;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

#if ADCBENCHMARK>0
static void MX_ADC1_Init_ll(void);
#endif

#if USARTBENCHMARK>0
static void MX_USART2_UART_Init_ll(void);
#endif

#if SPIBENCHMARK>0
static void MX_SPI1_Init_ll(void);
#endif

#if I2CBENCHMARK>0
static void MX_I2C2_Init_ll(void);
#endif

static void MX_UART4_UART_Init_ll(void);


void usart_send_string(const char* str);
/* USER CODE BEGIN PFP */

//This is the definition of Pines per Board
const GpioDefinition_t BoardGPIO [portTOTAL_NUM_GPIO_CS] = {
		{ (uint32_t)CS_FRAM_Port,  (uint32_t)CS_FRAM_Pin   },
		{ (uint32_t)CS_RS485_Port, (uint32_t)CS_RS485_Pin  },

		};


const uint32_t f = ADC_CHANNEL_0 | ADC_CHANNEL_4; // channel 0 and 4 configured



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA**** ";
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
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if (configUSE_TRACE_FACILITY ==1)

  vTraceEnable(TRC_START);
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

#if (USARTBENCHMARK == 1 || USARTBENCHMARK == 2)
   MX_USART2_UART_Init_ll();
#endif

#if (SPIBENCHMARK == 1 || SPIBENCHMARK == 2 )
   MX_SPI1_Init_ll();
#endif

#if ( I2CBENCHMARK == 1 || I2CBENCHMARK == 2 )
  MX_I2C2_Init_ll();
#endif

#if ADCBENCHMARK == 1
  MX_ADC1_Init_ll();
#endif


#if ( USARTBENCHMARK  ||   SPIBENCHMARK  || I2CBENCHMARK  ||  ADCBENCHMARK )
  MX_UART4_UART_Init_ll();
#endif


  /* Call our entry point. */
  app_main();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}




/**
  * @brief This function configures SPI1 for DMA full-duplex communication.
  * @note  This function is used to :
  *        -1- Enables GPIO clock and configures the SPI1 pins.
  *        -2- Configure SPI1 functional parameters.
  *        -3- Configure DMA channels
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
#if SPIBENCHMARK >0
static void MX_SPI1_Init_ll(void)
{
	/* (1) Enables GPIO clock and configures the SPI1 pins ********************/
	  /* Enable the peripheral clock of GPIOB */
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	  /* Configure SCK Pin connected to pin 31 of CN10 connector (PB3) */
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_5);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_HIGH);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_3, LL_GPIO_PULL_DOWN);

	  /* Configure MISO Pin connected to pin 27 of CN10 connector (PB4) */
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_5);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_DOWN);

	  /* Configure MOSI Pin connected to pin 29 of CN10 connector (PB5) */
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_5);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_DOWN);

	  /* (2) Configure SPI1 functional parameters ********************************/
	  /* Enable the peripheral clock of GPIOB */
	  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

	  /* Configure SPI1 communication */
	  LL_SPI_SetBaudRatePrescaler(SPI1, LL_SPI_BAUDRATEPRESCALER_DIV128);
	  //LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
	  LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
	  LL_SPI_SetClockPhase(SPI1, LL_SPI_PHASE_1EDGE);
	  LL_SPI_SetClockPolarity(SPI1, LL_SPI_POLARITY_LOW);
	  /* Reset value is LL_SPI_MSB_FIRST */
	  //LL_SPI_SetTransferBitOrder(SPI1, LL_SPI_MSB_FIRST);
	  LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
	  LL_SPI_SetNSSMode(SPI1, LL_SPI_NSS_SOFT);
	//#ifdef MASTER_BOARD
	  LL_SPI_SetMode(SPI1, LL_SPI_MODE_MASTER);
	//#else
	  /* Reset value is LL_SPI_MODE_SLAVE */
	  //LL_SPI_SetMode(SPI1, LL_SPI_MODE_SLAVE);
	//#endif /* MASTER_BOARD */
	  LL_SPI_DisableCRC(SPI1);

	  //Enable SPI IRQ
	  NVIC_SetPriority(SPI1_IRQn, 5);
	  NVIC_EnableIRQ(SPI1_IRQn);

	  /* Configure SPI1 DMA transfer interrupts */
	  /* Enable DMA RX Interrupt */
	  //LL_SPI_EnableDMAReq_RX(SPI1);
	  /* Enable DMA TX Interrupt */
	  // LL_SPI_EnableDMAReq_TX(SPI1);


	  /* (3) Configure DMA channels ******************************* */

	  /* Enable the clock of DMA1  */
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	  /* Configure NVIC for DMA transfer complete/error interrupts */
	  NVIC_SetPriority(DMA1_Channel2_IRQn, 5);
	  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	  NVIC_SetPriority(DMA1_Channel3_IRQn, 5);
	  NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	  /*  Configure the DMA1_Channel2 functional parameters */

	  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2,
	                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY | LL_DMA_PRIORITY_LOW| LL_DMA_MODE_NORMAL |
	                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
	                        LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);

	  /* this should be configured in the driver
	  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), (uint32_t)aRxBuffer,
	                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
	  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, ubNbDataToReceive);
	  */

	  /* Configure the DMA1_Channel3 functional parameters */

	  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_3,
	                        LL_DMA_DIRECTION_MEMORY_TO_PERIPH | LL_DMA_PRIORITY_LOW | LL_DMA_MODE_NORMAL |
	                        LL_DMA_PERIPH_NOINCREMENT | LL_DMA_MEMORY_INCREMENT |
	                        LL_DMA_PDATAALIGN_BYTE | LL_DMA_MDATAALIGN_BYTE);

	  /* this should be configured in the driver
	  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)aTxBuffer, LL_SPI_DMA_GetRegAddr(SPI1),
	                         LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
	  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, ubNbDataToTransmit);
	   */

	  /*  Enable DMA interrupts complete/error */
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
	  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);
	  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
	  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);


}

#endif





/**
  * @brief Low level initialization of USART and DMA
  * @param None
  * @retval None
  **/
#if USARTBENCHMARK >0
static void MX_USART2_UART_Init_ll(void)
{
	LL_USART_InitTypeDef USART_InitStruct;
	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);


    /*
     * USART2 GPIO Configuration
     *
     * PA2   ------> USART2_TX
     * PA3   ------> USART2_RX
     */
    GPIO_InitStruct.Pin = USART_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_GPIO_SetAFPin_0_7(GPIOA, USART_TX_Pin, LL_GPIO_AF_7); //set AF to USART2_TX

    GPIO_InitStruct.Pin = USART_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    LL_GPIO_SetAFPin_0_7(GPIOA, USART_RX_Pin, LL_GPIO_AF_7); //set AF to USART2_RX


    /* USART2 DMA Init */
    /* USART2_RX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);


    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)&USART2->DR);
    // This must be done in the driver per transfer
    // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, (uint32_t)ucSharedMemory);
    // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, ARRAY_LEN(ucSharedMemory));

    /* Enable TC interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);


    /* DMA1_Channel6_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);

    /* USART2_TX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_7, LL_DMA_MDATAALIGN_BYTE);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)&USART2->DR);
    // This must be done in the driver per transfer
    // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, (uint32_t)ucSharedMemory);
    // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7, ARRAY_LEN(ucSharedMemory));
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_7);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_7);
    /* DMA1_Channel7_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);


    /* USART configuration */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);

    /* USART interrupt */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART2_IRQn);
    /* Enable USART*/
    LL_USART_Enable(USART2);

}
#endif


static void MX_UART4_UART_Init_ll(void)
{

	LL_GPIO_InitTypeDef GPIO_InitStruct;

	/* Peripheral clock enable */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);


    /*
     * USART2 GPIO Configuration
     *
     * PC10   ------> UART4_TX
     * PC11   ------> UART4_RX
     */
    GPIO_InitStruct.Pin = UART4_TX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LL_GPIO_SetAFPin_8_15(GPIOC, UART4_TX_Pin, UART4_TX_AF); //set AF to UART4_TX

    GPIO_InitStruct.Pin = UART4_RX_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    LL_GPIO_SetAFPin_8_15(GPIOC, UART4_RX_Pin, UART4_RX_AF); //set AF to UART4_RX



    /* USART configuration */
    /*
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(UART4, &USART_InitStruct);
    */

    LL_USART_SetTransferDirection(UART4, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(UART4, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(UART4, SystemCoreClock, LL_USART_OVERSAMPLING_16, 115200);
    //LL_USART_ConfigAsyncMode(UART4);
    NVIC_SetPriority(UART4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(UART4_IRQn);
    LL_USART_DisableIT_IDLE(UART4);
    LL_USART_DisableIT_TXE(UART4);
    LL_USART_DisableIT_TC(UART4);
    LL_USART_DisableIT_ERROR(UART4);

    /* Enable USART*/
    LL_USART_Enable(UART4);

}

#if I2CBENCHMARK >0
static void MX_I2C2_Init_ll(void)
{
	/**** GPIO and I2C configuration ****/
	  LL_RCC_ClocksTypeDef rcc_clocks;

	  /* (1) Enables GPIO clock and configures the I2C2 pins **********************/
	  /*    (SCL on PB.10, SDA on PB.11)                     **********************/

	  /* Enable the peripheral clock of GPIOB */
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

	  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10 | LL_GPIO_PIN_11);

	  /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
	  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_4);
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);

	  /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
	  LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_4);
	  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
	  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
	  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
	  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);

	  /* (2) Enable the I2C2 peripheral clock *************************************/

	  /* Enable the peripheral clock for I2C2 */
	  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);

	  /* (3) Configure NVIC for I2C2 **********************************************/

	  /* Configure Event IT:
	   *  - Set priority for I2C2_EV_IRQn
	   *  - Enable I2C2_EV_IRQn
	   */
	  NVIC_SetPriority(I2C2_EV_IRQn, 6);
	  NVIC_EnableIRQ(I2C2_EV_IRQn);

	  /* Configure Error IT:
	   *  - Set priority for I2C2_ER_IRQn
	   *  - Enable I2C2_ER_IRQn
	   */
	  NVIC_SetPriority(I2C2_ER_IRQn, 6);
	  NVIC_EnableIRQ(I2C2_ER_IRQn);

	  /* (4) Configure I2C2 functional parameters ********************************/

	  /* Disable I2C2 prior modifying configuration registers */
	  LL_I2C_Disable(I2C2);

	  /* Retrieve Clock frequencies */
	  LL_RCC_GetSystemClocksFreq(&rcc_clocks);

	  /* Configure the SCL Clock Speed */
	  LL_I2C_ConfigSpeed(I2C2, rcc_clocks.PCLK1_Frequency, I2C_SPEEDCLOCK, I2C_DUTYCYCLE);


	  /**** DMA configuration TX ****/


	  /* (1) Enable the clock of DMA1 */
	   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	   /* (2) Configure NVIC for DMA1_Channel4 and DMA1_Channel5 */
	   NVIC_SetPriority(DMA1_Channel4_IRQn, 6);
	   NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	   NVIC_SetPriority(DMA1_Channel5_IRQn, 6);
	   NVIC_EnableIRQ(DMA1_Channel5_IRQn);


	   /* (3) Configure the DMA functional parameters for Master Transmit */
	   LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | \
	                                                 LL_DMA_PRIORITY_HIGH              | \
	                                                 LL_DMA_MODE_NORMAL                | \
	                                                 LL_DMA_PERIPH_NOINCREMENT         | \
	                                                 LL_DMA_MEMORY_INCREMENT           | \
	                                                 LL_DMA_PDATAALIGN_BYTE            | \
	                                                 LL_DMA_MDATAALIGN_BYTE);
	   //LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)(*pMasterTransmitBuffer), (uint32_t)LL_I2C_DMA_GetRegAddr(I2C2), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

	   /* (4) Configure the DMA functional parameters for Master Receive */
	    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | \
	                                                  LL_DMA_PRIORITY_HIGH              | \
	                                                  LL_DMA_MODE_NORMAL                | \
	                                                  LL_DMA_PERIPH_NOINCREMENT         | \
	                                                  LL_DMA_MEMORY_INCREMENT           | \
	                                                  LL_DMA_PDATAALIGN_BYTE            | \
	                                                  LL_DMA_MDATAALIGN_BYTE);
	   // LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C2), (uint32_t)&(aMasterReceiveBuffer), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));

	   /* (5) Enable DMA1 interrupts complete/error */
	   LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
	   LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);
	   LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
	   LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);

}

#endif

/**
  * @brief Basic board GPIO Initialization Function
  * @param None
  * @retval None
 **/

#if ADCBENCHMARK
static void MX_ADC1_Init_ll(void)
{
	/* Enable GPIO Clock */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

	/* Configure GPIO in analog mode to be used as ADC input */
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_ANALOG);

	/*## Configuration of NVIC #################################################*/
	/* Configure NVIC to enable ADC1 interruptions */
	//NVIC_SetPriority(ADC1_IRQn, 5); /* ADC IRQ greater priority than DMA IRQ */
	//NVIC_EnableIRQ(ADC1_IRQn);

	/* Enable ADC clock (core clock) */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

	if(__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE() == 0)
	{
	    /* Set ADC clock (conversion clock) common to several ADC instances */
	    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_ASYNC_DIV2);
	}

	if (LL_ADC_IsEnabled(ADC1) == 0)
	{

		/* Set Set ADC sequencers scan mode, for all ADC groups                   */
		/* (group regular, group injected).                                       */
		LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_ENABLE);  //comment this to disable the sequence scan mode

		/* Set ADC group regular trigger source */
	    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);

	    /* Set ADC group regular continuous mode */
	    //LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_SINGLE);
	    LL_ADC_REG_SetContinuousMode(ADC1, LL_ADC_REG_CONV_CONTINUOUS); //trigger is automatic after the first trigger

	    /* Set ADC group regular conversion data transfer */
	    LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED);
	    //LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);

	    /* Specify which ADC flag between EOC (end of unitary conversion)         */
	    /* or EOS (end of sequence conversions) is used to indicate               */
	    /* the end of conversion.                                                 */

	    LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_SEQUENCE_CONV);

	    /* Set ADC group regular sequencer */
	    /* Set ADC group regular sequencer length and scan direction */
	    //LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);			// check if this is configured in the DMA request
	    //LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS);

	    /* Set ADC group regular sequencer discontinuous mode */
	    // LL_ADC_REG_SetSequencerDiscont(ADC1, LL_ADC_REG_SEQ_DISCONT_DISABLE);

	    /* Set ADC group regular sequence: channel on the selected sequence rank. */
	    // LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);  // check if this is configured in the DMA request
	    // LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_0);

	    //LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_48CYCLES);  //this should be done per each enabled channel
	    //LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_48CYCLES);
	}

    /*## Configuration of ADC interruptions ####################################*/
	/* Enable interruption ADC group regular overrun */
	//LL_ADC_EnableIT_OVR(ADC1);


	/**** DMA configuration ****/
	/* Configure NVIC to enable DMA interruptions */
	NVIC_SetPriority(DMA1_Channel1_IRQn, 6); /* DMA IRQ lower priority than ADC IRQ */
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	/*## Configuration of DMA ##################################################*/
	/* Enable the peripheral clock of DMA */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* Configure the DMA transfer */
	/*  - DMA transfer in circular mode to match with ADC configuration:        */
	/*    DMA unlimited requests.                                               */
	/*  - DMA transfer from ADC without address increment.                      */
	/*  - DMA transfer to memory with address increment.                        */
	/*  - DMA transfer from ADC by half-word to match with ADC configuration:   */
	/*    ADC resolution 12 bits.                                               */
	/*  - DMA transfer to memory by half-word to match with ADC conversion data */
	/*    buffer variable type: half-word.                                      */
	LL_DMA_ConfigTransfer(DMA1,
	                     LL_DMA_CHANNEL_1,
	                     LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
						 LL_DMA_MODE_NORMAL                |
	                     LL_DMA_PERIPH_NOINCREMENT         |
	                     LL_DMA_MEMORY_INCREMENT           |
	                     LL_DMA_PDATAALIGN_HALFWORD        |
	                     LL_DMA_MDATAALIGN_HALFWORD        |
	                     LL_DMA_PRIORITY_HIGH               );


	/* Enable DMA transfer interruption: transfer complete */
	LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_1 );

	/* Enable DMA transfer interruption: transfer error */
	LL_DMA_EnableIT_TE( DMA1,LL_DMA_CHANNEL_1);

	/*## Activation of DMA #####################################################*/
	/* Enable the DMA transfer */


}
#endif

/**
  * @brief Basic board GPIO Initialization Function
  * @param None
  * @retval None
  **/
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO CS: FRAM */
  HAL_GPIO_WritePin(CS_FRAM_Port, CS_FRAM_Pin, GPIO_PIN_SET); // we need to set the CS to disable the FRAM
  GPIO_InitStruct.Pin = CS_FRAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_FRAM_Port, &GPIO_InitStruct);


}




/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}





/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
