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
#include "task.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Definition for UART4 Pins */
#define UART4_TX_Pin GPIO_PIN_10
#define UART4_TX_GPIO_Port GPIOC
#define UART4_TX_AF	GPIO_AF8_UART4
#define UART4_RX_Pin GPIO_PIN_11
#define UART4_RX_GPIO_Port GPIOC
#define UART4_RX_AF GPIO_AF8_UART4

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#if (configUSE_TRACE_FACILITY ==1)
extern traceHandle USARTHandle;
extern traceHandle DMAHandle;
extern traceHandle SPIHandle;
extern traceHandle I2CHandle;
extern traceHandle ADCHandle;
#endif

extern uint8_t ucSharedPeripheral[ 512 ] __attribute__( ( aligned( 512 ) ) );

//extern UART_HandleTypeDef huart2;


typedef struct __xPeripheralShared_t
{
	TaskHandle_t xTaskToNotify1;

	UART_HandleTypeDef huartX;

	ADC_HandleTypeDef hadcX;

	I2C_HandleTypeDef hi2c2X;

	SPI_HandleTypeDef hspi1X;
}xPeripheralShared_t;


#define hadc1 ((xPeripheralShared_t *)ucSharedPeripheral)->hadcX
#define hi2c2 ((xPeripheralShared_t *)ucSharedPeripheral)->hi2c2X
#define hspi1 ((xPeripheralShared_t *)ucSharedPeripheral)->hspi1X
#define huart2  ((xPeripheralShared_t *)ucSharedPeripheral)->huartX
#define xTaskToNotify ((xPeripheralShared_t *)ucSharedPeripheral)->xTaskToNotify1


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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
