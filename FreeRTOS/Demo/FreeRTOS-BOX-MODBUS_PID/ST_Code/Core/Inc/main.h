/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "stm32l1xx_ll_dma.h"
#include "stm32l1xx_ll_rcc.h"
#include "stm32l1xx_ll_bus.h"
#include "stm32l1xx_ll_system.h"
#include "stm32l1xx_ll_exti.h"
#include "stm32l1xx_ll_adc.h"
#include "stm32l1xx_ll_i2c.h"
#include "stm32l1xx_ll_cortex.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_spi.h"
#include "stm32l1xx.h"
#include "stm32l1xx_ll_gpio.h"
#include "FreeRTOS.h"
#include "Modbus.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()


/* Definition for USARTx Pins */
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USARTx_TX_AF	GPIO_AF7_USART2
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define USARTx_RX_AF    GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel6


/* Definition for USARTx's NVIC */
//#define USARTx_DMA_TX_IRQn                DMA1_Channel7_IRQn
//#define USARTx_DMA_RX_IRQn                DMA1_Channel6_IRQn
//#define USARTx_DMA_TX_IRQHandler          DMA1_Channel7_IRQHandler
//#define USARTx_DMA_RX_IRQHandler          DMA1_Channel6_IRQHandler
//#define USARTx_DMA_PreemptPriority		  5
//#define USARTx_PreemptPriority		      5

void DMA1_Channel6_IRQHandler(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);


#define MSR_SIZE 256
#define SHARED_MEMORY_SIZE 256
extern uint8_t xModbusShareRegion[ MSR_SIZE ];
extern uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ];
#define BOX 1
#define PRIV 0
#define MPU_EN 1
extern const GpioDefinition_t BoardGPIO [portTOTAL_NUM_GPIO_CS];
#define CS_FRAM_Pin  GPIO_PIN_10
#define CS_FRAM_Port GPIOA

#define CS_TERMO_Pin  GPIO_PIN_8
#define CS_TERMO_Port GPIOA

#define CS_RS485_Pin  GPIO_PIN_7
#define CS_RS485_Port GPIOC
#define CS_FRAM_  0
#define CS_TERMO_  0
#define CS_RS485_ 1

#define huart2 ((modbusData_t *)xModbusShareRegion)->huart
#define ModbusH ((modbusHandler_t *)ucSharedMemory)

typedef struct
{
	UART_HandleTypeDef huart;
	uint16_t xModbusDATA[16];

	int8_t   xis_fisrt_capture;
	uint16_t xvcap1, xvcap2, xspeed ;
	float    xfrequency;

	TIM_HandleTypeDef xhtim3;

	float xduty;
	float xduty_pulse;
	float xpid_error;


	//TIM_HandleTypeDef xhtim4;
	//TIM_HandleTypeDef xhtim7;

}
modbusData_t;


/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Exported functions ------------------------------------------------------- */


extern TIM_HandleTypeDef htim6;

/* This define the USART peripherals for STM32L152RE*/
#define USART_PERIPERALS (uint32_t) USART1: \
		case (uint32_t) USART2: \
		case (uint32_t) USART3: \
		case (uint32_t) UART4: \
		case (uint32_t) UART5 \

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
