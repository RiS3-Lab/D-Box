/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32l1xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_COMP_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  static DMA_HandleTypeDef hdma_tx;
  static DMA_HandleTypeDef hdma_rx;

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
 	   /*##-1- Enable peripherals and GPIO Clocks #################################*/
	   /* Enable GPIO TX/RX clock */
	   USARTx_TX_GPIO_CLK_ENABLE();
	   USARTx_RX_GPIO_CLK_ENABLE();

	   /* Enable USARTx clock */
	   USARTx_CLK_ENABLE();

	   /* Enable DMA clock */
	   DMAx_CLK_ENABLE();

	   /*##-2- Configure peripheral GPIO ##########################################*/
	   /* UART TX GPIO pin configuration  */
	   GPIO_InitStruct.Pin       = USART_TX_Pin;
	   GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	   GPIO_InitStruct.Pull      = GPIO_PULLUP;
	   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	   GPIO_InitStruct.Alternate = USARTx_TX_AF;

	   HAL_GPIO_Init(USART_TX_GPIO_Port, &GPIO_InitStruct);

	   /* UART RX GPIO pin configuration  */
	   GPIO_InitStruct.Pin = USART_RX_Pin;
	   GPIO_InitStruct.Alternate = USARTx_RX_AF;

	   HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

	   /*##-3- Configure the DMA ##################################################*/
	   /* Configure the DMA handler for Transmission process */
	   hdma_tx.Instance                 = USARTx_TX_DMA_CHANNEL;
	   hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	   hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	   hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	   hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	   hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	   hdma_tx.Init.Mode                = DMA_NORMAL;
	   hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

	   HAL_DMA_Init(&hdma_tx);

	   /* Associate the initialized DMA handle to the UART handle */
	   __HAL_LINKDMA(huart, hdmatx, hdma_tx);

	   /* Configure the DMA handler for reception process */
	   hdma_rx.Instance                 = USARTx_RX_DMA_CHANNEL;
	   hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	   hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
	   hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
	   hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	   hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	   hdma_rx.Init.Mode                = DMA_NORMAL;
	   hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

	   HAL_DMA_Init(&hdma_rx);

	   /* Associate the initialized DMA handle to the the UART handle */
	   __HAL_LINKDMA(huart, hdmarx, hdma_rx);

	   /*##-4- Configure the NVIC for DMA #########################################*/
	   /* NVIC configuration for DMA transfer complete interrupt (USART1_TX) */
	   HAL_NVIC_SetPriority(USARTx_DMA_TX_IRQn, USARTx_DMA_PreemptPriority, 1);
	   HAL_NVIC_EnableIRQ(USARTx_DMA_TX_IRQn);

	   /* NVIC configuration for DMA transfer complete interrupt (USART1_RX) */
	   HAL_NVIC_SetPriority(USARTx_DMA_RX_IRQn, USARTx_DMA_PreemptPriority, 0);
	   HAL_NVIC_EnableIRQ(USARTx_DMA_RX_IRQn);

	   /* NVIC for USART, to catch the TX complete */
	   HAL_NVIC_SetPriority(USARTx_IRQn, USARTx_PreemptPriority, 1);
	   HAL_NVIC_EnableIRQ(USARTx_IRQn);
  }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, USART_TX_Pin|USART_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
