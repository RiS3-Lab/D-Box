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
#include "MB85RS64V.h"
#include "printf.h"
//#include "application_defined_privileged_functions.h"

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

static StackType_t xRWPrivilegedTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
static StackType_t xTestTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
static StackType_t xROAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
static StackType_t xRWAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );

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



/* @brief Implements a task in privileged mode, it has access to all memory
* @param pvParameters[in] Parameters as passed during task creation.
*/
static void prvPrivilegedTask( void * pvParameters );

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

/**
 * @brief Implements a task wihtout restricted regions
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvUnrestrictedTask( void * pvParameters );


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
			 * that this is an expected fault. MPU_xContainerSendRequest
			 * The handler will then be able to
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


extern BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);
extern void MPU_xContainerEnableDWTcounter();
extern void MPU_xContainerSetDWTcounter(uint32_t val);
extern uint32_t MPU_xContainerGetDWTcounter();

static void prvTestTask( void * pvParameters )
{
	Peripheral_Request_t xPeripheralRequest;
	//MB85RS64V_t xMB85;
#if ADCBENCHMARK == 1
	 uint8_t cStringTest[]= "IxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySBIxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySB";
#else
	 uint8_t cStringTest[]="IxQxebhGMz4cI9JdRjsP9rCjGIvAMewsJqohA3KxyV4lmFzWuXVsSbnKA4LggQJKLCJW0ueyh0H4Z8ToLYaDvQ0SUK7vhWsfAySB";
	 //uint8_t cStringTest[100];
#endif



    uint8_t ucSize = 1;
    uint64_t uLTotalCycles = 0;
    uint8_t ucIterations = 0;
    //uint8_t cStringTestRX[sizeof(cStringTest)];
    //uint16_t wADCdata[4];
    //int8_t cDataFRAMwrite[sizeof(cStringTest)];
    //int8_t cDataFRAM[sizeof(cStringTest)];

    uint32_t ulCycles;
    uint32_t bytesRX;

	//uint8_t ucIndex;
	BaseType_t xNotificationValue, xNumberOfBytes;

    //MB85R_Init( &xMB85, (uint32_t *)SPI1, CS_FRAM_ ); // FRAM initialization
	LL_GPIO_SetOutputPin(LD2_GPIO_Port, LD2_Pin);   //signal the data capturing interface
	vTaskDelay(5);
	LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

	for( ; ; )
	{

		        /* reading  from i2c to a variable in stack */
/*
		        xPeripheralRequest.pulPeripheral =  (uint32_t *)I2C2;
				xPeripheralRequest.ucOperation = eWrite;
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest;
				xPeripheralRequest.ulSize = sizeof(cStringTest);
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = 4;  // This is the slave address

				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
						while(1);
				}

				//For read operation the 3 upper bytes of notification value are the actual number of bytes read, between 0 and ulSize,
				 // the lower byte represents operation errors

				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

				if( (xNotificationValue & 0xff) != 0 )
				{
					//error
					while(1);
				}
*/

				//vTaskDelay(10);


				/* Writing/reading  from i2c to variables in stack */
/*
				xPeripheralRequest.pulPeripheral =  (uint32_t *)I2C2;
				xPeripheralRequest.ucOperation = eFullDuplex;
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest;
				xPeripheralRequest.ulSize = sizeof(cStringTest);
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulAddressSec = (uint32_t)cStringTestRX;
				xPeripheralRequest.ulSizeSec = 6;
				xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = 4; // This is the slave address
                //xContainerGPIOwrite( (uint8_t)xPeripheralRequest.ulOption1, 1);
				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
						while(1);
				}


				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

				if( (xNotificationValue & 0xff) != 0 )
				{
					//error
					while(1);
				}

*/
/*

				xPeripheralRequest.pulPeripheral =  (uint32_t *)I2C2;
				xPeripheralRequest.ucOperation = eRead;
				xPeripheralRequest.ulAddress = (uint32_t)cStringTestRX;
				xPeripheralRequest.ulSize = 6;
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = 4; // This is the slave address

				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
						while(1);
				}

				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

				if( (xNotificationValue & 0xff) != 0 )
				{
					//error
					while(1);
				}

*/

/*
				xPeripheralRequest.pulPeripheral =  (uint32_t *)ADC1; //peripheral for operation
				xPeripheralRequest.ucOperation = eRead; // reading operation
				xPeripheralRequest.ulAddress = (uint32_t)wADCdata; // source address
				xPeripheralRequest.ulSize = 3;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				//xPeripheralRequest.ulOption1 = ADC_CHANNEL_0; // for ADC option 1 represents the channels
				xPeripheralRequest.ulOption1 = ADC_CHANNEL_0 | ADC_CHANNEL_4;

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
*/


				/* writing to serial port from a variable in the first xRegion 1 */
/*
		        xPeripheralRequest.pulPeripheral =  (uint32_t *)SPI1; //peripheral for operation
				xPeripheralRequest.ucOperation = eFullDuplex; // writing operation
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				xPeripheralRequest.ulSize = 2;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;

				xPeripheralRequest.ulAddressSec = (uint32_t)cStringTestRX; // source address
				xPeripheralRequest.ulSizeSec = 2;  //size of transfer
				xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;

				xPeripheralRequest.ulOption1 = CS_FRAM_; // this is mandatory when no CS is used

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

				float temp;
				float *p;
				uint16_t  tempInt;
				tempInt = (uint16_t) cStringTestRX[0]<<8 | (uint16_t) cStringTestRX[1];
				tempInt = tempInt>>3;
				temp = ( tempInt ) * 0.25;
				p = (float *) &ucSharedMemory[0];
				*p = temp;

				cStringTestRX[2] = '\n';
*/

				/* writing to serial port from a variable in the first xRegion 1 */
/*
				xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral for operation
				xPeripheralRequest.ucOperation = eRead; // writing operation
				xPeripheralRequest.ulAddress = (uint32_t)cStringTestRX; // source address
				xPeripheralRequest.ulSize = 3;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = OPT_NULL; // this is mandatory when no CS is used

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

*/

		       /* reading  from serial port to a variable in stack */
			/*
				xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2;
				xPeripheralRequest.ucOperation = eRead;
				xPeripheralRequest.ulAddress = (uint32_t)cStringTestRX;
				xPeripheralRequest.ulSize = sizeof(cStringTestRX);
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = OPT_NULL;

				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
						while(1);
				}
*/
				/* For read operation the 3 upper bytes of notification value are the actual number of bytes read, between 0 and ulSize,
				 * the lower byte represents operation errors */
				/*
				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

				if( (xNotificationValue & 0xff) != 0 )
				{
					//error
					while(1);
				}
*/
				//memset(&cDataFRAM, '0', sizeof(cDataFRAM));

		        //xNotificationValue =1024;

				//MB85R_Write_Data( &xMB85, 0x00, (uint8_t *) &cStringTestRX, portSTACK_REGION_NUMBER, xNotificationValue>>8 );

				//MB85R_Read_Data( &xMB85, 0x00, (uint8_t *) &cDataFRAM, portSTACK_REGION_NUMBER, xNotificationValue>>8 );


				//MB85R_Write_Data(MB85RS64V_t *mb85H_v, uint16_t address, uint8_t *data, uint8_t ucDataRegion, uint16_t size)


				/* writing to serial port from a variable in the first xRegion 1 */

#if USARTBENCHMARK ==1

#if configUSE_TRACE_FACILITY == 0
		        MPU_xContainerSetDWTcounter(0);
		//ulCycles = MPU_xContainerGetDWTcounter();
#endif
				xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral for operation
				xPeripheralRequest.ucOperation = eWrite; // writing operation
				xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
				xPeripheralRequest.ulSize = ucSize;  //size of transfer
				xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
				xPeripheralRequest.ulOption1 = OPT_NULL;


				if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
				{
					while(1);
				}

				xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);;

#if configUSE_TRACE_FACILITY == 0
				ulCycles = MPU_xContainerGetDWTcounter();
				//ulCyclesEnd = MPU_xContainerGetDWTcounter();

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


 				if(ucSize>100)
 				{

 					ucSize = 1;
 				}

				vTaskDelay(5);

	}


}
/*-----------------------------------------------------------*/


static void prvPrivilegedTask( void * pvParameters )
{
	/* Unused parameters. */
	( void ) pvParameters;

	uint32_t ulAuxVal;
	( void )ulAuxVal;

	static const MemoryRegion_t xAltRegions[ portNUM_CONFIGURABLE_REGIONS ] =
	{
	    /* Base address    Length    Parameters */
	    //{ xTestTaskStack,   sizeof( xTestTaskStack ),     portMPU_REGION_READ_WRITE }, // test for overlapped stack
		{ (uint32_t*) 0x40026000, (uint32_t) (0x400263FF - 0x40026000),     portMPU_REGION_READ_WRITE }, // test for region accessing DMA
	    { 0,               0,        0 },
	    { 0,               0,        0 }
	};

	for( ; ; )
	{
		/* This task is privileged and has access to all memory areas
		 * lets read from kernel space, this should not trigger an exception this time
		 */

		//ulAuxVal = *(uint32_t *)(0x20000000);  //read from kernel space

		/*let's prepare an exception  and lower privileges so next time we attempt
		 * to read from kernel, after lowering privileges,
		 *  it will trigger an exception*/
		//ucROTaskFaultTracker[ 0 ] = 1; //prepare exception so handler is aware of the coming exception
		//vResetTaskPrivilege();	//reset privileges of this task
		//ulAuxVal = *(uint32_t *)(0x20000000);  //read from kernel space

		/* end of this part of the demo */
		//while(1)
		ucSharedMemory[0]=1;


		//vTaskDelay( pdMS_TO_TICKS( 1000 ) );
		//vTaskAllocateMPURegions( NULL, xAltRegions );
		vTaskDelay( pdMS_TO_TICKS( 100 ) );

	}
}
/*-----------------------------------------------------------*/


static void prvUnrestrictedTask( void * pvParameters )
{
    ( void ) pvParameters;
    uint8_t uLocalVar = 100;

    for( ; ; )
    {
    	uLocalVar++;
    	vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
}


void vStartMPUDemo( void )
{
/**
 * Since stack of a task is protected using MPU, it must satisfy MPU
 * requirements as mentioned at the top of this file.
 */





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
	.puxStackBuffer	= xRWAccessTaskStack,xRWPrivilegedTaskStack,
	.xRegions		=	{
							{ ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER},
							{ 0,				0,					0														},
							{ 0,				0,					0														},
						},
	.pxPeripheralPermissions = (PeripheralPermission_t *) xPermissionRO
};


TaskParameters_t xRWPrivilegedTaskParameters =
{
	.pvTaskCode		= prvPrivilegedTask,
	.pcName			= "Privileged",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY ,
	.puxStackBuffer	= xRWPrivilegedTaskStack,
	.xRegions		=	{
			                { ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE },
							{ 0,	0,	0},
							{ 0,	0,	0},
						},

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
							{  ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE },
							{   UART4,	256,	portMPU_REGION_READ_WRITE},
							{ LD2_GPIO_Port,	256,	portMPU_REGION_READ_WRITE}
						},
	.pxPeripheralPermissions = (PeripheralPermission_t *)  xPermissionRO
};



	/* Create an unprivileged task with RO access to ucSharedMemory. */

	//xTaskCreateRestricted( &( xROAccessTaskParameters ), NULL );

	/* Create an unprivileged task with RW access to ucSharedMemory. */

	//xTaskCreateRestricted( &( xRWAccessTaskParameters ), NULL );

	/* Create privileged task*/

	//xTaskCreateRestricted( &( xRWPrivilegedTaskParameters ), NULL );

    /* Create privileged task*/
	// *(uint32_t *)0xE0001004 = 0;
    xTaskCreateRestricted( &( xTestTaskParameters ), NULL );


/*
	xTaskCreate(prvUnrestrictedTask,
	                            "UnresTask",
								configMINIMAL_STACK_SIZE,
	                            NULL,
								tskIDLE_PRIORITY,
	                            NULL
	                          );
*/

  //  xTaskCreateStatic(
  //  		          prvUnrestrictedTask,       /* Function that implements the task. */
  //                    "StaticTask",          /* Text name for the task. */
  //                    STACK_SIZE,      /* Number of indexes in the xStack array. */
  //                    ( void * ) 1,    /* Parameter passed into the task. */
  //                    tskIDLE_PRIORITY,/* Priority at which the task is created. */
  //                    xStack,          /* Array to use as the task's stack. */
  //                    &xTaskBuffer );  /* Variable to hold the task's data structure. */
  //
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
		//for( ; ; )
		//{
		//}


	}
}

/*-----------------------------------------------------------*/

void _putchar(char character)
{
  LL_USART_TransmitData8(UART4, character);
  while(!LL_USART_IsActiveFlag_TXE(UART4));
}

