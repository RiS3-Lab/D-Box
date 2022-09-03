/*
 * UARTCallback.c
 *
 *  Created on: May 27, 2020
 *      Author: Alejandro Mera
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "Modbus.h"


/**
 * @brief
 * This is the callback for HAL interrupts of UART TX used by Modbus library.
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
#if (configUSE_TRACE_FACILITY ==1)
extern traceHandle USARTHandle;
#endif


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Modbus RTU callback BEGIN */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if (configUSE_TRACE_FACILITY ==1)
//vTraceStoreISRBegin(USARTHandle);
#endif


	int i;
	for (i = 0; i < numberHandlers; i++ )
	{
	   	if (mHandlers[i]->port == huart )
	   	{
	   		xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
	   		break;
	   	}
	}
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );



	/* Modbus RTU callback END */

	/*
	 * Here you should implement the callback code for other UARTs not used by Modbus
	 *
	 * */

#if (configUSE_TRACE_FACILITY ==1)
//vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif

}


/**
 * @brief
 * This is the callback for HAL interrupt of UART RX
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
vTraceStoreISRBegin(USARTHandle);
#endif

	/* Modbus RTU RX callback BEGIN */
    int i;
    for (i = 0; i < numberHandlers; i++ )
    {
    	if (mHandlers[i]->port == UartHandle  )
    	{
      RingAdd(&mHandlers[i]->xBufferRX, mHandlers[i]->dataRX);
    		HAL_UART_Receive_IT(mHandlers[i]->port, &mHandlers[i]->dataRX, 1);
    		xTimerResetFromISR(mHandlers[i]->xTimerT35, &xHigherPriorityTaskWoken);
    		break;
    	}
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

#if (configUSE_TRACE_FACILITY ==1)
vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}
