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
//static StackType_t xROAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );
//static StackType_t xRWAccessTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );

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

PRIVILEGED_CONSTANT static const PeripheralPermission_t  xPermissionRO[portTOTAL_NUM_PERMISSIONS] = {
		{(uint32_t *)USART2	, (eRead | eWrite) 	}, // working
		{(uint32_t *)SPI1	, (eRead | eWrite) 	}, // not really working right now
		{ 0					, 0					},
		{ 0					, 0					}
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


extern BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);

static void prvTestTask( void * pvParameters )
{
	Peripheral_Request_t xPeripheralRequest;

    int8_t cStringTest[] = "1. Hi from DMA driver!! \n";
    int8_t cStringTestRX[sizeof(cStringTest)];
	//uint8_t ucIndex;
	BaseType_t xNotificationValue, xNumberOfBytes;


	for( ; ; )
	{

		vTaskDelay(100);

		/* reading  from serial port to a variable in stack */
		xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2;
		xPeripheralRequest.ucOperation = eRead;
		xPeripheralRequest.ulAddress = (uint32_t)cStringTestRX;
		xPeripheralRequest.ulSize = sizeof(cStringTestRX);
		xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;

		if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
		{
				while(1);
		}

		/* For read operation the 3 upper bytes of notification value are the actual number of bytes read, between 0 and ulSize,
		 * the lower byte represents operation errors */
		xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if( (xNotificationValue & 0xff) != 0 )
		{
			//error
			while(1);
		}
		else
		{
			/* no error so lets copy the received data to TX buffer in stack*/
			xNumberOfBytes = (xNotificationValue >> 8) & 0xffffff;
			memset(cStringTest, ' ', sizeof(cStringTest));
			memcpy(cStringTest, cStringTestRX, xNumberOfBytes);

			/* also copy to shared region */
			memset(ucSharedMemory, ' ', sizeof(ucSharedMemory));
			memcpy(ucSharedMemory, cStringTestRX, xNumberOfBytes);
		}

		/* writing to serial port from a variable in stack */
		xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral for operation
		xPeripheralRequest.ucOperation = eWrite; // writing operation
		xPeripheralRequest.ulAddress = (uint32_t)cStringTest; // source address
		xPeripheralRequest.ulSize = xNumberOfBytes;  //size of transfer
		xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;

		if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
		{
			// the operation is not permitted
			while(1);
		}

		xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if( ( xNotificationValue & 0xff )== pdFREERTOS_ERRNO_ETIMEDOUT)
		{
			//error
			while(1);
		}


		/* writing to serial port from a variable in the first xRegion 1 */
		xPeripheralRequest.pulPeripheral =  (uint32_t *)USART2; //peripheral for operation
		xPeripheralRequest.ucOperation = eWrite; // writing operation
		xPeripheralRequest.ulAddress = (uint32_t)ucSharedMemory; // source address
		xPeripheralRequest.ulSize = xNumberOfBytes;  //size of transfer
		xPeripheralRequest.ulRegionNumber = portFIRST_REGION_NUMBER;

		if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
		{
			// the operation is not permitted
			while(1);
		}

		xNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if( ( xNotificationValue & 0xff )== pdFREERTOS_ERRNO_ETIMEDOUT)
		{
			//error
			while(1);
		}

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



		vTaskDelay( pdMS_TO_TICKS( 1000 ) );
		//vTaskAllocateMPURegions( NULL, xAltRegions );
		vTaskDelay( pdMS_TO_TICKS( 1000 ) );

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



/*

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
	.puxStackBuffer	= xRWAccessTaskStack,xRWPrivilegedTaskStack
	.xRegions		=	{
							{ ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE | portMPU_REGION_EXECUTE_NEVER},
							{ 0,				0,					0														},
							{ 0,				0,					0														},
						},
	.pxPeripheralPermissions = (PeripheralPermission_t *) xPermissionRO
};
*/

TaskParameters_t xRWPrivilegedTaskParameters =
{
	.pvTaskCode		= prvPrivilegedTask,
	.pcName			= "Privileged",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xRWPrivilegedTaskStack,
	.xRegions		=	{
							{ 0,	0,	0},
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
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xTestTaskStack,
	.xRegions		=	{
							{  ucSharedMemory,	SHARED_MEMORY_SIZE,	portMPU_REGION_READ_WRITE },
							{ 0,	0,	0},
							{ 0,	0,	0},
						},
	.pxPeripheralPermissions = (PeripheralPermission_t *)  xPermissionRO
};



	/* Create an unprivileged task with RO access to ucSharedMemory. */
	//xTaskCreateRestricted( &( xROAccessTaskParameters ), NULL );

	/* Create an unprivileged task with RW access to ucSharedMemory. */
	//xTaskCreateRestricted( &( xRWAccessTaskParameters ), NULL );

	/* Create privileged task*/
	xTaskCreateRestricted( &( xRWPrivilegedTaskParameters ), NULL );

    /* Create privileged task*/
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
		for( ; ; )
		{
		}
	}
}
/*-----------------------------------------------------------*/
