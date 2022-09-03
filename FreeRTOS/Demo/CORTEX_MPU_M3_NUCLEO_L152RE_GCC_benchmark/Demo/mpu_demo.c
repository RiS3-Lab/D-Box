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
#define SHARED_MEMORY_SIZE 32


//static TaskHandle_t xTaskToNotify = NULL;

/**
 * @brief Memory region shared between two tasks.
 */
static uint8_t ucSharedMemory[ SHARED_MEMORY_SIZE ] __attribute__( ( aligned( SHARED_MEMORY_SIZE ) ) );

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




/* @brief Implements a task for benchmark
* @param pvParameters[in] Parameters as passed during task creation.
*/
static void prvTestTask( void * pvParameters );

/**
 * @brief Implements the task which has Read Only access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvROAccessTask( void * pvParameters );

/**
 * @brief Implements the task which has Read Write access to the memory region
 * ucSharedMemory.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvRWAccessTask( void * pvParameters );

/*-----------------------------------------------------------*/

static void prvROAccessTask( void * pvParameters )
{
uint8_t ucVal;

	/* Unused parameters. */
	( void ) pvParameters;

	for( ; ; )
	{
		/* This task has RO access to ucSharedMemory and therefore it can read
		 * it but cannot modify it. */
		ucVal = ucSharedMemory[ 0 ];

		/* Silent compiler warnings about unused variables. */
		( void ) ucVal;
		/* Since this task has Read Only access to the ucSharedMemory region,
		 * writing to it results in Memory Fault. Set ucROTaskFaultTracker[ 0 ]
		 * to 1 to tell the Memory Fault Handler that this is an expected fault.
		 * The handler will recover from this fault gracefully by jumping to the
		 * next instruction. */
		ucROTaskFaultTracker[ 0 ] = 1;

		/* Illegal access to generate Memory Fault. */
		ucSharedMemory[ 0 ] = 0;

		/* Ensure that the above line did generate MemFault and the fault
		 * handler did clear the  ucROTaskFaultTracker[ 0 ]. */
		configASSERT( ucROTaskFaultTracker[ 0 ] == 0 );

		#if( configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY == 1 )
		{
			/* Generate an SVC to raise the privilege. Since privilege
			 * escalation is only allowed from kernel code, this request must
			 * get rejected and the task must remain unprivileged. As a result,
			 * trying to write to ucSharedMemory will still result in Memory
			 * Fault. */
			portRAISE_PRIVILEGE();

			/* Set ucROTaskFaultTracker[ 0 ] to 1 to tell the Memory Fault
			 * Handler that this is an expected fault. The handler will then be
			 * able to recover from this fault gracefully by jumping to the
			 * next instruction.*/
			ucROTaskFaultTracker[ 0 ] = 1;

			/* The following must still result in Memory Fault since the task
			 * is still running unprivileged. */
			ucSharedMemory[ 0 ] = 0;

			/* Ensure that the above line did generate MemFault and the fault
		 	 * handler did clear the  ucROTaskFaultTracker[ 0 ]. */
			configASSERT( ucROTaskFaultTracker[ 0 ] == 0 );
		}
		#else
		{
			/* Generate an SVC to raise the privilege. Since
			 * configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY is not enabled, the
			 * task will be able to escalate privilege. */
			portRAISE_PRIVILEGE();

			/* At this point, the task is running privileged. The following
			 * access must not result in Memory Fault. If something goes
			 * wrong and we do get a fault, the execution will stop in fault
			 * handler as ucROTaskFaultTracker[ 0 ] is not set (i.e.
			 * un-expected fault). */
			ucSharedMemory[ 0 ] = 0;

			/* Lower down the privilege. */
			portSWITCH_TO_USER_MODE();

			/* Now the task is running unprivileged and therefore an attempt to
			 * write to ucSharedMemory will result in a Memory Fault. Set
			 * ucROTaskFaultTracker[ 0 ] to 1 to tell the Memory Fault Handler
			 * that this is an expected fault. The handler will then be able to
			 * recover from this fault gracefully by jumping to the next
			 * instruction.*/
			ucROTaskFaultTracker[ 0 ] = 1;

			/* The following must result in Memory Fault since the task is now
			 * running unprivileged. */
			ucSharedMemory[ 0 ] = 0;

			/* Ensure that the above line did generate MemFault and the fault
			 * handler did clear the  ucROTaskFaultTracker[ 0 ]. */
			configASSERT( ucROTaskFaultTracker[ 0 ] == 0 );
		}
		#endif /* #if( configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY == 1 ) */

		/* Wait for a second. */
		vTaskDelay( pdMS_TO_TICKS( 1000 ) );
	}
}
/*-----------------------------------------------------------*/

static void prvRWAccessTask( void * pvParameters )
{
	/* Unused parameters. */
	( void ) pvParameters;

	for( ; ; )
	{
		/* This task has RW access to ucSharedMemory and therefore can write to
		 * it. */
		ucSharedMemory[ 0 ] = 0;

		/* Wait for a second. */
		vTaskDelay( pdMS_TO_TICKS( 1000 ) );
	}
}
/*-----------------------------------------------------------*/




extern void MPU_xContainerEnableDWTcounter();
extern void MPU_xContainerSetDWTcounter(uint32_t val);
extern uint32_t MPU_xContainerGetDWTcounter();

static void prvTestTask( void * pvParameters )
{
	//Peripheral_Request_t xPeripheralRequest;
	//MB85RS64V_t xMB85;


	uint32_t ulNotificationValue;
	static const MemoryRegion_t xAltRegions[ portNUM_CONFIGURABLE_REGIONS ] =
	{
	    /* Base address    Length    Parameters */
			{  ucSharedHALGlobal,	32,	portMPU_REGION_READ_WRITE },
			{  UART4,	256,	portMPU_REGION_READ_WRITE},
			{ ucSharedPeripheral,	512,	portMPU_REGION_READ_WRITE},
	};



	static const MemoryRegion_t xOriRegions[ portNUM_CONFIGURABLE_REGIONS ] =
	{
		    /* Base address    Length    Parameters */
				{ ucSharedHALGlobal,	32,	portMPU_REGION_READ_WRITE  },
				{  0,	0,	0},
				{ 0,	0,	0},
	};


   #if (ADCBENCHMARK == 1 || ADCBENCHMARK == 2)
	 uint16_t cStringTest[100];
    #else
	 uint8_t cStringTest[]="IxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySB";
    #endif

    uint8_t ucSize = 1;
    uint64_t uLTotalCycles = 0;
    uint8_t ucIterations = 0;


    //uint8_t cStringTestRX[sizeof(cStringTest)];
    //uint16_t wADCdata[4];
    //int8_t cDataFRAMwrite[sizeof(cStringTest)];
    //int8_t cDataFRAM[sizeof(cStringTest)];

    uint32_t ulCycles,ulCyclesEND;



	//uint8_t ucIndex;
	//BaseType_t xNotificationValue, xNumberOfBytes;

    //MB85R_Init( &xMB85, (uint32_t *)SPI1, CS_FRAM_ ); // FRAM initialization


    LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);   //signal the data capturing interface
    vTaskDelay(5);
    LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

	for( ; ; )
	{



#if USARTBENCHMARK == 2

#if (configUSE_TRACE_FACILITY == 0)

		MPU_xContainerSetDWTcounter(0);
#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();

				xTaskToNotify =  xTaskGetCurrentTaskHandle();

				/* The following line is for TX */
				//HAL_UART_Transmit_IT(&huart2, cStringTest, ucSize);

				/* The following 2 lines are for RX */
				printf("*%d\n",ucSize);
				HAL_UART_Receive_IT(&huart2, cStringTest, ucSize);



				ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );

				if( ulNotificationValue == 1 )
				{
				        // The transmission ended as expected.
				}
				else
				{
				        // The call to ulTaskNotifyTake() timed out.
					while(1);
				}

				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif

#if USARTBENCHMARK == 1

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);

#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();

				/* The following line is for TX */
				//HAL_UART_Transmit(&huart2, cStringTest, ucSize,200);

				/* The following 2 lines are for RX */
				printf("*%d\n",ucSize);
				if (HAL_UART_Receive(&huart2, cStringTest, ucSize, 200)!= HAL_OK)
				{
					while(1);
				}

				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif



#if SPIBENCHMARK == 2 //IT

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();



				xTaskToNotify =  xTaskGetCurrentTaskHandle();

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				/* The following line is for TX */
			    //HAL_SPI_Transmit_IT(&hspi1, cStringTest, ucSize);

				/* The following 2 lines are for RX */
				HAL_SPI_Receive_IT(&hspi1, cStringTest, ucSize);
				//printf("*%d\n",ucSize);


				ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );

				if( ulNotificationValue == 1 )
				{
				        // The transmission ended as expected.
				}
				else
				{
				        // The call to ulTaskNotifyTake() timed out.
					while(1);
				}

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif




#if SPIBENCHMARK == 1 //pooling

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

				/* The following line is for TX */
			    //HAL_SPI_Transmit(&hspi1, cStringTest, ucSize, 200);
			    HAL_SPI_Receive(&hspi1, cStringTest, ucSize, 200);

			    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif



#if I2CBENCHMARK == 1 //pooling

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();

				//HAL_I2C_Master_Receive(&hi2c2, 8, cStringTest, ucSize, 200);
				HAL_I2C_Master_Transmit(&hi2c2, 8, cStringTest, ucSize, 200);

				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif


#if I2CBENCHMARK == 2 //IT

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();



				xTaskToNotify =  xTaskGetCurrentTaskHandle();

				/* The following line is for RX */
			    //HAL_I2C_Master_Receive_IT(&hi2c2, 8, cStringTest, ucSize);

				HAL_I2C_Master_Transmit_IT(&hi2c2, 8, cStringTest, ucSize);


				ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );

				if( ulNotificationValue == 1 )
				{
				        // The transmission ended as expected.
				}
				else
				{
				        // The call to ulTaskNotifyTake() timed out.
					while(1);
				}

				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();


#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif



#if ADCBENCHMARK == 1 //pooling

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif
				int i;
				ADC_ChannelConfTypeDef sConfig = {0};

				vTaskAllocateMPURegions( NULL, xAltRegions );
				portYIELD();
/*
				if (ucSize%2 == 0 )
				{


				}
				*/

				for(i=0; i < ucSize/2; i++)
				{
					    sConfig.Channel = ADC_CHANNEL_0;
						sConfig.Rank = ADC_REGULAR_RANK_1;
						sConfig.SamplingTime = ADC_SAMPLETIME_48CYCLES;
						if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
						{
						   Error_Handler();
					    }

					    HAL_ADC_Start(&hadc1);    // start a conversion of a regular group

						if (HAL_ADC_PollForConversion(&hadc1, 200) == HAL_TIMEOUT)
						{
						   while(1);
						}
						cStringTest[i*2] = HAL_ADC_GetValue(&hadc1);

						sConfig.Channel = ADC_CHANNEL_4;
						sConfig.Rank = ADC_REGULAR_RANK_1;
						sConfig.SamplingTime = ADC_SAMPLETIME_48CYCLES;
						if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
						{
						   Error_Handler();
						}

						HAL_ADC_Start(&hadc1);    // start a conversion of a regular group

						if (HAL_ADC_PollForConversion(&hadc1, 200) == HAL_TIMEOUT)
						{
							while(1);
						}
   					    cStringTest[i*2+1] = HAL_ADC_GetValue(&hadc1);
  					    HAL_ADC_Stop(&hadc1);
				}

				if (ucSize%2)
				{

					 sConfig.Channel = ADC_CHANNEL_0;
					 sConfig.Rank = ADC_REGULAR_RANK_1;
					 sConfig.SamplingTime = ADC_SAMPLETIME_48CYCLES;
					 if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
					 {
						   Error_Handler();
					 }

					 HAL_ADC_Start(&hadc1);    // start a conversion of a regular group
					 if (HAL_ADC_PollForConversion(&hadc1, 200) == HAL_TIMEOUT)
					 {
					    while(1);
					 }
					 cStringTest[i*2] = HAL_ADC_GetValue(&hadc1);

					 HAL_ADC_Stop(&hadc1);
				}




				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();

#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif


#if ADCBENCHMARK == 2 //pooling

#if (configUSE_TRACE_FACILITY == 0)

				MPU_xContainerSetDWTcounter(0);
#endif

	int i;

	            vTaskAllocateMPURegions( NULL, xAltRegions );
	    		portYIELD();
	            xTaskToNotify =  xTaskGetCurrentTaskHandle();


				for(i=0; i < ucSize/2; i++)
				{
					HAL_ADC_Start_IT(&hadc1);    // start a conversion of a regular group
					ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );
					if( ulNotificationValue == 1 )
					{
									        // The transmission ended as expected.
					}
					else
					{
					    // The call to ulTaskNotifyTake() timed out.
						while(1);
					}
					cStringTest[i*2] = HAL_ADC_GetValue(&hadc1);

					HAL_ADC_Start_IT(&hadc1);    // start a conversion of a regular group
					ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );
					if( ulNotificationValue == 1 )
					{
					     // The transmission ended as expected.
					}
					else
					{
					    // The call to ulTaskNotifyTake() timed out.
					   	while(1);
					}
   					cStringTest[i*2+1] = HAL_ADC_GetValue(&hadc1);

				}

				if (ucSize%2)
				{
					 HAL_ADC_Start_IT(&hadc1);    // start a conversion of a regular group
					 ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, 200 );
					 if( ulNotificationValue == 1 )
					 {
					      // The transmission ended as expected.
					 }
					 else
					 {
					     // The call to ulTaskNotifyTake() timed out.
					     while(1);
					 }

					 cStringTest[i*2] = HAL_ADC_GetValue(&hadc1);
				}


				vTaskAllocateMPURegions( NULL, xOriRegions );
				portYIELD();

#if (configUSE_TRACE_FACILITY == 0)
				ulCycles = MPU_xContainerGetDWTcounter();
#endif

#endif


				//ulCycles = ulCyclesEND - ulCycles;
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

 				if(ucSize>100)
 				{

 					ucSize = 1;
 				}

				vTaskDelay(5);

	}
}




void vStartMPUDemo( void )
{
/**
 * Since stack of a task is protected using MPU, it must satisfy MPU
 * requirements as mentioned at the top of this file.
 */
static StackType_t xROAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
static StackType_t xRWAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
static StackType_t xTestTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );

TaskParameters_t xROAccessTaskParameters =
{
	.pvTaskCode		= prvROAccessTask,
	.pcName			= "ROAccess",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xROAccessTaskStack,
	.xRegions		=	{
							{ ucSharedMemory,					SHARED_MEMORY_SIZE,	portMPU_REGION_PRIVILEGED_READ_WRITE_UNPRIV_READ_ONLY | portMPU_REGION_EXECUTE_NEVER	},
							{ ( void * ) ucROTaskFaultTracker,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER								},
							{ 0,								0,					0																						},
						}
};
TaskParameters_t xRWAccessTaskParameters =
{
	.pvTaskCode		= prvRWAccessTask,
	.pcName			= "RWAccess",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xRWAccessTaskStack,
	.xRegions		=	{
							{ ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER},
							{ 0,				0,					0														},
							{ 0,				0,					0														},
						}
};


TaskParameters_t xTestTaskParameters =
{
	.pvTaskCode		= prvTestTask,
	.pcName			= "TestTask",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= 10,
	//.uxPriority		= 5 | portPRIVILEGE_BIT ,
	.puxStackBuffer	= xTestTaskStack,
	.xRegions		=	{
							{  0,	0,	0},
							{  0,	0,	0},
							{  0,	0,	0},
						}
};



	/* Create an unprivileged task with RO access to ucSharedMemory. */
	//xTaskCreateRestricted( &( xROAccessTaskParameters ), NULL );

	/* Create an unprivileged task with RW access to ucSharedMemory. */
	//xTaskCreateRestricted( &( xRWAccessTaskParameters ), NULL );

    xTaskCreateRestricted( &( xTestTaskParameters ), NULL );
}
/*-----------------------------------------------------------*/

portDONT_DISCARD void vHandleMemoryFault( uint32_t * pulFaultStackAddress )
{
uint32_t ulPC;
uint16_t usOffendingInstruction;

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


void _putchar(char character)
{

LL_USART_TransmitData8(UART4, character);
while(!LL_USART_IsActiveFlag_TXE(UART4));
}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(USARTHandle);
#endif

	if( huart == &huart2 )
	{

		vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

#if (configUSE_TRACE_FACILITY ==1)
  vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( huart == &huart2 )
	{

		vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( hspi == &hspi1)
	{

		vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if( hspi == &hspi1 )
	{

		vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if( hi2c == &hi2c2 )
		{

			vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

		}

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if( hi2c == &hi2c2 )
		{

			vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

		}

		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(xTaskToNotify)
	{
	   if( hadc == &hadc1 )
	   {
		  vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );
	   }
	}

	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}









/*-----------------------------------------------------------*/
