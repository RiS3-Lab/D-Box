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
#include "stm32l1xx_ll_cortex.h"
#include "stm32l1xx_ll_utils.h"
#include "stm32l1xx_ll_pwr.h"
#include "stm32l1xx_ll_usart.h"
#include "stm32l1xx_ll_spi.h"
#include "stm32l1xx_ll_i2c.h"
#include "stm32l1xx_ll_adc.h"
#include "stm32l1xx_ll_dac.h"

#include "stm32l1xx.h"
#include "stm32l1xx_ll_gpio.h"
#include "FreeRTOS.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#if (configUSE_TRACE_FACILITY ==1)
extern traceHandle USARTHandle;
extern traceHandle DMAHandle;
#endif
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


/**
  * @brief I2C devices settings
  */
/* I2C SPEEDCLOCK define to max value: 400 KHz */
#define I2C_SPEEDCLOCK                  400000
#define I2C_DUTYCYCLE                   LL_I2C_DUTYCYCLE_2

/**
  * @brief Slave settings
  */
#define SLAVE_OWN_ADDRESS                       0x5A /* This value is a left shift of a real 7 bits of a slave address
                                                        value which can find in a Datasheet as example: b0101101
                                                        mean in uint8_t equivalent at 0x2D and this value can be
                                                        seen in the OAR1 register in bits ADD[1:7] */

/**
  * @brief Master Transfer Request Direction
  */
//#define I2C_REQUEST_WRITE                       0x00
//#define I2C_REQUEST_READ                        0x01




/* USER CODE BEGIN Private defines */
#define CS_FRAM_Pin  GPIO_PIN_10
#define CS_FRAM_Port GPIOA
#define CS_RS485_Pin  GPIO_PIN_7
#define CS_RS485_Port GPIOC

#define CS_FRAM_  0
#define CS_RS485_ 1

extern const GpioDefinition_t BoardGPIO [portTOTAL_NUM_GPIO_CS];
extern const uint32_t ulADCchannelsActivated;

/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for UART4 Pins */
#define UART4_TX_Pin GPIO_PIN_10
#define UART4_TX_GPIO_Port GPIOC
#define UART4_TX_AF	GPIO_AF8_UART4
#define UART4_RX_Pin GPIO_PIN_11
#define UART4_RX_GPIO_Port GPIOC
#define UART4_RX_AF GPIO_AF8_UART4



/* Definition for USART2 Pins */
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USARTx_TX_AF	GPIO_AF7_USART2
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define USARTx_RX_AF    GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL             DMA1_Channel7
#define USARTx_RX_DMA_CHANNEL             DMA1_Channel6

#define SPIx_TX_DMA_CHANNEL				  DMA1_Channel3
#define SPIx_RX_DMA_CHANNEL				  DMA1_Channel2

#define I2Cx_TX_DMA_CHANNEL				  DMA1_Channel5
#define I2Cx_RX_DMA_CHANNEL				  DMA1_Channel4


//This is the definition of ADC channels enabled per Board
#define ADC_CHANNEL_0 0x00000001
#define ADC_CHANNEL_1 0x00000002
#define ADC_CHANNEL_2 0x00000004
#define ADC_CHANNEL_3 0x00000008
#define ADC_CHANNEL_4 0x00000010
#define ADC_CHANNEL_5 0x00000020
#define ADC_CHANNEL_6 0x00000040
#define ADC_CHANNEL_7 0x00000080



void DMA1_Channel6_IRQHandler(void);
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);


#define SHARED_MEMORY_SIZE 32
extern uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ];

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
extern UART_HandleTypeDef huart2;

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
