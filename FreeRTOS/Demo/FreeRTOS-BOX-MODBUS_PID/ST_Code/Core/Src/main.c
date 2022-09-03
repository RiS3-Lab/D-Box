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
#define ARM_MATH_CM3
#include "arm_math.h"


#define ModbusDATA ((modbusData_t *)xModbusShareRegion)->xModbusDATA
#define is_fisrt_capture ((modbusData_t *)xModbusShareRegion)->xis_fisrt_capture
#define vcap1 ((modbusData_t *)xModbusShareRegion)->xvcap1
#define vcap2 ((modbusData_t *)xModbusShareRegion)->xvcap2
#define speed ((modbusData_t *)xModbusShareRegion)->xspeed
#define frequency ((modbusData_t *)xModbusShareRegion)->xfrequency
#define htim3 ((modbusData_t *)xModbusShareRegion)->xhtim3
//#define htim4 ((modbusData_t *)xModbusShareRegion)->xhtim4
//#define htim7 ((modbusData_t *)xModbusShareRegion)->xhtim7

#define duty  ((modbusData_t *)xModbusShareRegion)->xduty
#define duty_pulse ((modbusData_t *)xModbusShareRegion)->xduty_pulse
#define pid_error ((modbusData_t *)xModbusShareRegion)->xpid_error


	//float duty;
	//float duty_pulse;
	//float pid_error;


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

__IO ITStatus UartReady = RESET;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init_ll(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI1_Init_ll(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void user_pwm_setvalue(uint16_t value,  uint32_t channel, uint32_t polarity);

//TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;




void usart_send_string(const char* str);
static void prvPLCTask( void * pvParameters );
/* USER CODE BEGIN PFP */
#define PID_PARAM_KP        0.1           /* Proportional */
#define PID_PARAM_KI        0.025        /* Integral */
#define PID_PARAM_KD        0.2            /* Derivative */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t xModbusShareRegion[ MSR_SIZE ] __attribute__( ( aligned( MSR_SIZE  ) ) );
//uint8_t aTxBuffer[] = " ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA****  ****UART_TwoBoards communication based on DMA**** ";
static volatile uint8_t ucROTaskFaultTracker[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( SHARED_MEMORY_SIZE ) ) ) = { 0 };

const GpioDefinition_t BoardGPIO [portTOTAL_NUM_GPIO_CS] = {
		{ (uint32_t)CS_TERMO_Port,  (uint32_t)CS_TERMO_Pin   },
		{ (uint32_t)CS_RS485_Port, (uint32_t)CS_RS485_Pin  },

		};

static StackType_t xPLCTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );

//PRIVILEGED_CONSTANT capabilities
static const PeripheralPermission_t  xPermissionRO[portTOTAL_NUM_PERMISSIONS] =
{
		{ (uint32_t *)USART2	, (eRead | eWrite) 	, CS_RS485_ }, // working
		{ (uint32_t *)SPI1	, (eRead | eWrite | eFullDuplex), CS_TERMO_Pin }, // working
		{ (uint32_t *)I2C2	, (eRead | eWrite | eFullDuplex) ,	4	}, // working
		{ (uint32_t *)NULL, eRead, (uint32_t)NULL}
};


TaskParameters_t xPLCTaskParameters =
{
	.pvTaskCode		= prvPLCTask,
	.pcName			= "PLCTask",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= 10,
	.puxStackBuffer	= xPLCTaskStack,
	.xRegions		=	{
								{  xModbusShareRegion,	MSR_SIZE,	portMPU_REGION_READ_WRITE },
								{ (uint32_t *)  TIM3,	512,	portMPU_REGION_READ_WRITE},
								{0,	0,	0}
							},

	.pxPeripheralPermissions = (PeripheralPermission_t *)  xPermissionRO
};


#define SCANCYCLE 10
extern BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);
static void prvPLCTask( void * pvParameters )
{
	 arm_pid_instance_f32 PID;
	 uint16_t setpoint;
	 Peripheral_Request_t xPeripheralRequest;
	 BaseType_t xNotificationValue;
	 uint8_t uRawTemperature[2];
	 uint16_t uTemperature;
	 float fTemperature;

	 uint16_t udelayTermo =0 ; // the time of conversion is 0.22 seconds, we read only one sample every 500 ms


	 user_pwm_setvalue((uint32_t)0, TIM_CHANNEL_1, TIM_OCPOLARITY_HIGH);


	 PID.Kp = PID_PARAM_KP;        /* Proportional */
	 PID.Ki = PID_PARAM_KI;        /* Integral */
	 PID.Kd = PID_PARAM_KD;        /* Derivative */

	 ModbusDATA[1] = (uint16_t)(PID_PARAM_KP*100);
     ModbusDATA[2] = (uint16_t)(PID_PARAM_KI*100);
	 ModbusDATA[3] = (uint16_t)(PID_PARAM_KD*100);

	 arm_pid_init_f32(&PID, 1);

	 for( ; ; )
	 {

		 if(udelayTermo>= 500/SCANCYCLE)
		 {
			 udelayTermo = 0;
			 xPeripheralRequest.pulPeripheral =  (uint32_t *)SPI1; //peripheral for operation
			 xPeripheralRequest.ucOperation = eRead; // writing operation
			 xPeripheralRequest.ulAddress = (uint32_t)uRawTemperature; // source address
			 xPeripheralRequest.ulSize = 2;  //size of transfer
			 xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
			 xPeripheralRequest.ulOption1 = CS_TERMO_; // this is mandatory when no CS is used


			 if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
			 {
				while(1);
			 }

			 xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			 if( ( xNotificationValue & 0xff )== pdFREERTOS_ERRNO_ETIMEDOUT)
			 {
						//error
			    while(1);
			 }

			 uTemperature = uRawTemperature[0]<<8;
			 uTemperature |= uRawTemperature[1];

			 if (uTemperature & 0x04)
			 {
				 ModbusDATA[4] = 0; //error Thermocouple disconnected
			 }
			 else
			 {
				 fTemperature = (uTemperature>>3) * 0.25 ;
				 ModbusDATA[4] = (uTemperature>>3) * 0.25;
			 }


		 }

         // update constants from Modbus shared region
		 PID.Kp = (float)ModbusDATA[1]/100.0;        /* Proportional */
		 PID.Ki = (float)ModbusDATA[2]/100.0;        /* Integral */
		 PID.Kd = (float)ModbusDATA[3]/100.0;        /* Derivative */




		 setpoint =  ModbusDATA[0];
	     if(setpoint>200)
	     {
	    	 setpoint = 200;
	    	 ModbusDATA[0] = 200;
	     }
		 if(setpoint<0)setpoint = 0;
	     pid_error =  setpoint - speed;
		 /* Calculate PID here, argument is the error */
		 /* Output data will be returned, we will use it as duty cycle parameter */
		 duty = arm_pid_f32(&PID, pid_error);
		 duty_pulse =duty;
	 	 if (duty_pulse > 1000)
	 	 {
	 		duty_pulse = 1000;
		 }
		 else if (duty_pulse < 0)
		 {
			 duty_pulse = 0;
		 }
	 	 if(setpoint == 0 && duty_pulse>0 && pid_error == 0 ) duty_pulse=0; // Do a complete stop

		 user_pwm_setvalue((uint32_t)duty_pulse, TIM_CHANNEL_1, TIM_OCPOLARITY_HIGH);

		 ModbusDATA[5] = setpoint;
		 ModbusDATA[6] = speed;
		 ModbusDATA[7] = duty_pulse;


		 vTaskDelay(SCANCYCLE);

		 udelayTermo++;

	 }

}



void user_pwm_setvalue(uint16_t value, uint32_t channel, uint32_t polarity)
{
    TIM_OC_InitTypeDef sConfigOC;
    //the resolution of PWm is 1 - 1000 according to configuration
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = polarity;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, channel);
    HAL_TIM_PWM_Start(&htim3, channel);
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
  //Call first HAL initialization
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_SPI1_Init_ll();
  MX_GPIO_Init();
  MX_USART2_UART_Init_ll();

  // Initialization encoder and timer interrupts for speed
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

#if configUSE_TRACE_FACILITY ==1
  vTraceEnable(TRC_START);
#endif

  /* USER CODE END SysInit */
  is_fisrt_capture = 1;
  /* Initialize all configured GPIO */




  /* USER CODE BEGIN 2 */
  ModbusH->uiModbusType = SLAVE_RTU;
  ModbusH->port = &huart2;// This is the UART port connected to STLINK in the NUCLEO F429
  ModbusH->u8id = 1; //slave ID
  ModbusH->u16timeOut = 1000;
  ModbusH->EN_Port = NULL; // No RS485
   //ModbusH2.EN_Port = LD2_GPIO_Port; // RS485 Enable
   //ModbusH2.EN_Pin = LD2_Pin; // RS485 Enable
  ModbusH->u32overTime = 0;
  ModbusH->au16regs = ModbusDATA;  //((modbusData_t *)xModbusShareRegion)->ModbusDATA;
  ModbusH->u16regsize= sizeof(ModbusDATA)/sizeof(ModbusDATA[0]);
   //Initialize Modbus library
  ModbusInit(ModbusH);
  //Start capturing traffic on serial Port
  ModbusStart(ModbusH);
  /* USER CODE END 2 */

  xTaskCreateRestricted( &( xPLCTaskParameters ), NULL );




  /* Start the scheduler. */
  vTaskStartScheduler();


  /* Call our entry point. */

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

    /* Enable HT & TC interrupts */
    //LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_6);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);

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
    /* DMA1_Channel7_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);


    /* USART configuration */
    USART_InitStruct.BaudRate = 9600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);
    // This must be done in the driver per transfer
    // LL_USART_EnableDMAReq_RX(USART2);
    // LL_USART_EnableDMAReq_TX(USART2);
    // LL_USART_EnableIT_IDLE(USART2);


    /* USART interrupt */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART2_IRQn);

    /* Enable USART and DMA RX*/
    //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6); // Check if we activate here or in the request
    //LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7); // Check if we activate here or in the request
    LL_USART_Enable(USART2);

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_TERMO */
  GPIO_InitStruct.Pin = CS_TERMO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_TERMO_Port, &GPIO_InitStruct);



}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of DMA Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
  /* Set transmission flag: transfer complete */
//  UartReady = SET;
//}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */



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

/* USER CODE END 4 */

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
  /* USER CODE BEGIN Callback 1 */
 if ( htim == &htim7)  // if interrput source is channel 1
 	{
    		if(is_fisrt_capture)
    		{
    			vcap1 = TIM4->CNT;
    			is_fisrt_capture = 0;
    		}
    		else if(!is_fisrt_capture)
    		{
    			vcap2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    			vcap2 = TIM4->CNT;
    			uint16_t speed_aux;
    			speed_aux = vcap2>vcap1 ? (vcap2 - vcap1) : (vcap1 - vcap2);
    			if(speed_aux>300)
    			{
    				speed = 0xffff - speed_aux +1;
    			}
    			else
    			{
    				speed = speed_aux;
    			}
    			frequency = speed *20;
    			is_fisrt_capture = 1;
    		}
   	}
  /* USER CODE END Callback 1 */
}




void usart_rx_check(void) {
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(ucSharedMemory) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6);
    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&ucSharedMemory[old_pos], pos - old_pos);
        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&ucSharedMemory[old_pos], ARRAY_LEN(ucSharedMemory) - old_pos);
            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&ucSharedMemory[0], pos);
            }
        }
    }
    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(ucSharedMemory)) {
        old_pos = 0;
    }
}


/**
 * \brief           Process received data over UART
 * \note            Either process them directly or copy to other bigger buffer
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void usart_process_data(const void* data, size_t len) {
	 const uint8_t* d = data;

	    /*
	     * This function is called on DMA TC and HT events, as well as on UART IDLE (if enabled) line event.
	     *
	     * For the sake of this example, function does a loop-back data over UART in polling mode.
	     * Check ringbuff RX-based example for implementation with TX & RX DMA transfer.
	     */

	    for (; len > 0; --len, ++d) {
	        LL_USART_TransmitData8(USART2, *d);
	        while (!LL_USART_IsActiveFlag_TXE(USART2)) {}
	    }
	    while (!LL_USART_IsActiveFlag_TC(USART2)) {}
}


/**
 * \brief           Send string to USART
 * \param[in]       str: String to send
 */
/*
void usart_send_string(const char* str) {
    usart_process_data(str, strlen(str));
}
*/
/* Interrupt handlers here */




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

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
	function will automatically get called if a task overflows its stack. */
	( void ) pxTask;
	( void ) pcTaskName;
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* If configUSE_MALLOC_FAILED_HOOK is set to 1 then this function will
	be called automatically if a call to pvPortMalloc() fails.  pvPortMalloc()
	is called automatically when a task, queue or semaphore is created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/


portDONT_DISCARD void vHandleMemoryFault( uint32_t * pulFaultStackAddress )
{
	uint32_t ulPC;
	uint16_t usOffendingInstruction;
    ulPC = pulFaultStackAddress[ 6 ];
	/* Is this an expected fault? */
	if( ucROTaskFaultTracker[ 0 ] == 1 )
	{
		/* Read program counter. */
		ulPC = pulFaultStackAddress[ 6 ];

		/* Read the offending instruction. */
		usOffendingInstruction = *( uint16_t * )ulPC;

		/* From ARM docs:
		 * If the value of bits[15:11] of the halfword being decoded is one of
		 * the following, the halfword is the first halfword of a 32-bit
		 * instruction:
		 * - 0b11101.
		 * - 0b11110.
		 * - 0b11111.
		 * Otherwise, the halfword is a 16-bit instruction.
		 */

		/* Extract bits[15:11] of the offending instruction. */
		usOffendingInstruction = usOffendingInstruction & 0xF800;
		usOffendingInstruction = ( usOffendingInstruction >> 11 );

		/* Determine if the offending instruction is a 32-bit instruction or
		 * a 16-bit instruction. */
		if( usOffendingInstruction == 0x001F ||
			usOffendingInstruction == 0x001E ||
			usOffendingInstruction == 0x001D )
		{
			/* Since the offending instruction is a 32-bit instruction,
			 * increment the program counter by 4 to move to the next
			 * instruction. */
			ulPC += 4;
		}
		else
		{
			/* Since the offending instruction is a 16-bit instruction,
			 * increment the program counter by 2 to move to the next
			 * instruction. */
			ulPC += 2;
		}

		/* Save the new program counter on the stack. */
		pulFaultStackAddress[ 6 ] = ulPC;

		/* Mark the fault as handled. */
		ucROTaskFaultTracker[ 0 ] = 0;
	}
	else
	{
		/* This is an unexpected fault - loop forever. */
		for( ; ; )
		{
		}
	}
}



static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}


/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 64-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period =50000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}





/**
* @brief TIM_OC MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_oc: TIM_OC handle pointer
* @retval None
*/
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
    /* TIM3 interrupt Init */
    //HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    //HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }

}

/**
* @brief TIM_Encoder MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_encoder->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}
/**
* @brief TIM_OC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_oc: TIM_OC handle pointer
* @retval None
*/
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* htim_oc)
{
  if(htim_oc->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }

}

/**
* @brief TIM_Encoder MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_encoder: TIM_Encoder handle pointer
* @retval None
*/
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{
  if(htim_encoder->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }

}



void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}



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
