/*
 * FreeRTOS V202011.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "queue.h"
#include "string.h"
#include "printf.h"

/** ARMv7 MPU Details:
 *
 * - ARMv7 MPU requires that the size of a MPU region is a power of 2.
 * - Smallest supported region size is 32 bytes.
 * - Start address of a region must be aligned to an integer multiple of the
 *   region size. For example, if the region size is 4 KB(0x1000), the starting
 *   address must be N x 0x1000, where N is an integer.
 */

/**
 * @brief Size of the shared memory region.
 */





#define STACK_SIZE 200

/* Structure that will hold the TCB of the task being created. */
StaticTask_t xTaskBuffer;

/* Buffer that the task being created will use as its stack.  Note this is
an array of StackType_t variables.  The size of StackType_t is dependent on
the RTOS port. */
StackType_t xStack[ STACK_SIZE ];





/**
 * @brief Memory region shared between two tasks.
 */
//static uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( SHARED_MEMORY_SIZE ) ) );
uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( SHARED_MEMORY_SIZE ) ) );


static StackType_t xTestTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );

/**
 * @brief Memory region used to track Memory Fault intentionally caused by the
 * RO Access task.
 *
 * RO Access task sets ucROTaskFaultTracker[ 0 ] to 1 before accessing illegal
 * memory. Illegal memory access causes Memory Fault and the fault handler
 * checks ucROTaskFaultTracker[ 0 ] to see if this is an expected fault. We
 * recover gracefully from an expected fault by jumping to the next instruction.
 *
 * @note We are declaring a region of 32 bytes even though we need only one.
 * The reason is that the smallest supported MPU region size is 32 bytes.
 */
static volatile uint8_t ucROTaskFaultTracker[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( SHARED_MEMORY_SIZE ) ) ) = { 0 };
/*-----------------------------------------------------------*/

/*
 * @brief Peripheral Permissions for task prvTestTask
 * The permissions can be either allocated in privileged RAM, or
 * privileged Flash. This is part of the container project.
 */

PRIVILEGED_CONSTANT
static const PeripheralPermission_t  xPermissionRO[portTOTAL_NUM_PERMISSIONS] =
{
		{ (uint32_t *)USART2	, (eRead | eWrite) 	, CS_RS485_ }, // working
		{ (uint32_t *)SPI1	, (eRead | eWrite | eFullDuplex), CS_FRAM_ }, // working
		{ (uint32_t *)I2C2	, (eRead | eWrite | eFullDuplex) ,	4	}, // working
		{ (uint32_t *)ADC1	, (eRead),	(ADC_CHANNEL_0 | ADC_CHANNEL_4)	} // working
};




/* @brief Implements a task for testing the DMA driver of the container
* @param pvParameters[in] Parameters as passed during task creation.
*/
static void prvTestTask( void * pvParameters );

extern BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);
extern void MPU_xContainerEnableDWTcounter();
extern void MPU_xContainerSetDWTcounter(uint32_t val);
extern uint32_t MPU_xContainerGetDWTcounter();

static void prvTestTask( void * pvParameters )
{
	Peripheral_Request_t xPeripheralRequest;

	// cStringTest variable is allocated in the stack

#if ADCBENCHMARK == 1
	 uint8_t cStringTest[]= "IxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySBIxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySB";
#else
	 uint8_t cStringTest[]="IxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySB";

#endif



    uint8_t ucSize = 1;
    uint64_t uLTotalCycles = 0;
    uint8_t ucIterations = 0;


    uint32_t ulCycles;
    uint32_t bytesRX;

	//uint8_t ucIndex;
	BaseType_t xNotificationValue;

	LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);   //signal the data capturing interface for power measuring
	vTaskDelay(5);                                  //if power is not measured these lines can be ignored
	LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

	for( ; ; )
	{

#if USARTBENCHMARK ==1

#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
#endif
				// Structure initialization for D-Box request
		        xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral reference
				xPeripheralRequest.ucOperation = eWrite; // writing operation
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				xPeripheralRequest.ulSize = ucSize;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER; // region number should contain cStringTest address
				xPeripheralRequest.ulOption1 = OPT_NULL;


				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
					while(1);
				}

				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);;

#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();

#endif

#endif


//This is a special case RX is essentially asynchronous so measures are not deterministic
#if USARTBENCHMARK == 2
#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
#endif

				xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral for operation
				xPeripheralRequest.ucOperation = eRead;
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				//ucSize = 20;
				xPeripheralRequest.ulSize = ucSize;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = OPT_NULL;

				printf("*%d\n", ucSize);


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

				if(ucSize != xNotificationValue>>8)
				{
					//error
					//while(1);
					xNotificationValue = xNotificationValue>>8;
				}


#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif


#if (SPIBENCHMARK == 1 || SPIBENCHMARK == 2 )
#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
#endif

				xPeripheralRequest.pulPeripheral =  (uint32_t *)SPI1; //peripheral for operation

#if (SPIBENCHMARK == 1) // TX
				xPeripheralRequest.ucOperation = eWrite; // writing operation
#endif

#if (SPIBENCHMARK == 2) // RX
				xPeripheralRequest.ucOperation = eRead; // read operation
#endif

				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				xPeripheralRequest.ulSize = ucSize;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = CS_FRAM_;

				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
					while(1);
				}

				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif



#if (I2CBENCHMARK == 1 || I2CBENCHMARK == 2)
#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
#endif
		        // this benchmark requires a dummy slave, we provide some sample code for an Arduino sketch
		        xPeripheralRequest.pulPeripheral =  (uint32_t *)I2C2;
#if (I2CBENCHMARK == 1 ) // TX
		        xPeripheralRequest.ucOperation = eWrite;
#endif
#if (I2CBENCHMARK == 2) // RX
		        xPeripheralRequest.ucOperation = eRead;
#endif
		        xPeripheralRequest.ulAddress = (uint32_t)cStringTest;
		        xPeripheralRequest.ulSize = ucSize;
		        //xPeripheralRequest.ulSize = 1;
		        xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
		        xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;
		        xPeripheralRequest.ulOption1 = 4; // This is the slave address

		        if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
		        {
		        		while(1);
		        }

		        xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();
#endif
#endif

#if ADCBENCHMARK ==1
#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
#endif


				xPeripheralRequest.pulPeripheral =  (uint32_t *)ADC1; //peripheral for operation
				xPeripheralRequest.ucOperation = eRead; // reading operation
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				xPeripheralRequest.ulSize = ucSize;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = ADC_CHANNEL_0; // for ADC option 1 represents the channels
				//xPeripheralRequest.ulOption1 = ADC_CHANNEL_0 | ADC_CHANNEL_4;

				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
					while(1);
				}
				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();
#endif
#endif



		        if( ( xNotificationValue & 0xff )== pdFREERTOS_ERRNO_ETIMEDOUT)
				{
						//error
						while(1);
				}
		        bytesRX = xNotificationValue>>8;

		        //ulCycles = ulCyclesEnd - ulCycles;
 				uLTotalCycles = uLTotalCycles + ulCycles;
 				ucIterations++;

 				if(ucIterations == 10)
 				{
 					uLTotalCycles = uLTotalCycles/10;
 					printf("%u, %llu \n", ucSize,   uLTotalCycles);
 					uLTotalCycles = 0;
 					ucSize++;
 					ucIterations = 0;

 				}

 				if(ucSize>10)
 				{

 					ucSize = 1;
 				}

				vTaskDelay(5);

	}


}
/*-----------------------------------------------------------*/


void vStartMPUDemo( void )
{
/**
 * Since stack of a task is protected using MPU, it must satisfy MPU
 * requirements as mentioned at the top of this file.
 */





TaskParameters_t xTestTaskParameters =
{
	.pvTaskCode		= prvTestTask,
	.pcName			= "TestTask",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= 10,
	.puxStackBuffer	= xTestTaskStack,
	.xRegions		=	{
							{  ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE },
							{   UART4,	256,	portMPU_REGION_READ_WRITE},
							{ LD2_GPIO_Port,	256,	portMPU_REGION_READ_WRITE}
						},
	.pxPeripheralPermissions = (PeripheralPermission_t *)  xPermissionRO
};

    xTaskCreateRestricted( &( xTestTaskParameters ), NULL );


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
		//D-Box: to do handle these exceptions


	}
}

/*-----------------------------------------------------------*/

void _putchar(char character)
{
  LL_USART_TransmitData8(UART4, character);
  while(!LL_USART_IsActiveFlag_TXE(UART4));
}

