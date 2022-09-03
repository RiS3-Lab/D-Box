/*
 * FreeRTOS Kernel V10.4.2
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
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */


#ifndef PORTMACRO_H
    #define PORTMACRO_H

    #ifdef __cplusplus
        extern "C" {
    #endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
    #define portCHAR          char
    #define portFLOAT         float
    #define portDOUBLE        double
    #define portLONG          long
    #define portSHORT         short
    #define portSTACK_TYPE    uint32_t
    #define portBASE_TYPE     long

    typedef portSTACK_TYPE   StackType_t;
    typedef long             BaseType_t;
    typedef unsigned long    UBaseType_t;

    #if ( configUSE_16_BIT_TICKS == 1 )
        typedef uint16_t     TickType_t;
        #define portMAX_DELAY              ( TickType_t ) 0xffff
    #else
        typedef uint32_t     TickType_t;
        #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
 * not need to be guarded with a critical section. */
        #define portTICK_TYPE_IS_ATOMIC    1
    #endif
/*-----------------------------------------------------------*/

/* MPU specific constants. */
    #define portUSING_MPU_WRAPPERS                                   1
    #define portPRIVILEGE_BIT                                        ( 0x80000000UL )

    #define portMPU_REGION_NO_ACCESS                                 ( 0x00UL << 24UL )
    #define portMPU_REGION_READ_WRITE                                ( 0x03UL << 24UL )
    #define portMPU_REGION_PRIVILEGED_READ_ONLY                      ( 0x05UL << 24UL )
    #define portMPU_REGION_READ_ONLY                                 ( 0x06UL << 24UL )
    #define portMPU_REGION_PRIVILEGED_READ_WRITE                     ( 0x01UL << 24UL )
    #define portMPU_REGION_PRIVILEGED_READ_WRITE_UNPRIV_READ_ONLY    ( 0x02UL << 24UL )
    #define portMPU_REGION_CACHEABLE_BUFFERABLE                      ( 0x07UL << 16UL )
    #define portMPU_REGION_EXECUTE_NEVER                             ( 0x01UL << 28UL )



#if (configUSE_DMA_CONTAINER == 1 )
    #define portSYSCALLS_REGION                                      ( 0UL )
    #define portPRIVILEGED_FLASH_REGION                              ( 6UL )
    #define portPRIVILEGED_RAM_REGION                                ( 7UL )


    #define portUNPRIVILEGED_FLASH_REGION                            ( 1UL )
    #define portSTACK_REGION                                         ( 2UL )
    #define portFIRST_CONFIGURABLE_REGION                            ( 3UL )
    #define portLAST_CONFIGURABLE_REGION                             ( 5UL )
    #define portNUM_CONFIGURABLE_REGIONS                             ( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 + 1) /* Plus one to make space for the task code region */
    #define portTOTAL_NUM_REGIONS                                    ( portNUM_CONFIGURABLE_REGIONS + 1  ) /* Plus one to make space for the stack region. */

    /* index of the Regions in the xRegion  array*/
    #define portSTACK_REGION_NUMBER								 	 (0UL)
	#define portFIRST_REGION_NUMBER						     		 (2UL)
	#define portSECOND_REGION_NUMBER						         (3UL)
	#define portTHIRD_REGION_NUMBER						     		 (4UL)
	#define portLAST_REGION_NUMBER						     		 (4UL)

#else

           #define portUNPRIVILEGED_FLASH_REGION                            ( 0UL )
           #define portPRIVILEGED_FLASH_REGION                              ( 1UL )
           #define portPRIVILEGED_RAM_REGION                                ( 2UL )
           #define portGENERAL_PERIPHERALS_REGION                           ( 3UL )
           #define portSTACK_REGION                                         ( 4UL )
           #define portFIRST_CONFIGURABLE_REGION                            ( 5UL )
           #define portLAST_CONFIGURABLE_REGION                             ( 7UL )
           #define portNUM_CONFIGURABLE_REGIONS                             ( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )
           #define portTOTAL_NUM_REGIONS                                    ( portNUM_CONFIGURABLE_REGIONS + 1 )
          /* Plus one to make space for the stack region. */
#endif

    #define portSWITCH_TO_USER_MODE()    __asm volatile ( " mrs r0, control \n orr r0, #1 \n msr control, r0 " ::: "r0", "memory" )

    typedef struct MPU_REGION_REGISTERS
    {
        uint32_t ulRegionBaseAddress;
        uint32_t ulRegionAttribute;
    } xMPU_REGION_REGISTERS;

/* Plus 1 to create space for the stack region. */
    typedef struct MPU_SETTINGSCS_MB85_GPIO_Port
    {
        xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
    } xMPU_SETTINGS;


    /* Containers definitions */
    #if (configUSE_DMA_CONTAINER == 1 )

        #define portTOTAL_NUM_PERMISSIONS 4
		#define portDMA_START_ADDRESS  	0x40026000
		#define portDMA_END_ADDRESS		0x400267FF
		#define portTOTAL_NUM_GPIO_CS	2

    	typedef enum
    	{
        eRead = 0x01,
		eWrite = 0x02,
		eFullDuplex = 0x04, // special case for I2C WriteRead
		eNone = 0xff
    	} ePeripheralOperation;


        typedef struct PERIPHERAL_PERMISSION
        {
            uint32_t *pulPeripheral;
            ePeripheralOperation  ucOperation;
            uint32_t ucOption; // this specific to each peripheral
        } PeripheralPermission_t;

		#define OPT_NULL  		0xFF  		// this is the value used by default for ucOption
		#define OPT_CS_KEEP_ON 	0x80 		// special option for maintaining CS ON

        typedef struct  PERIPHERAL_REQUEST
        {
        	uint32_t *pulPeripheral;   				//Peripheral for this request
        	uint32_t ulSize;						//Size of the transfer
        	uint32_t ulSizeSec;						//Secondary size of the transfer
        	uint32_t ulAddress;						//Primary Physical address in RAM passed by user to read/write, or Full-duplex Read
        	uint32_t ulAddressSec;					//Secondary Physical address in RAM passed by user only for Full-duplex Write
        	void *ulTask;							//Task requesting the transfer
        	ePeripheralOperation ucOperation;		//Read or write operation
        	uint8_t ulRegionNumber;    			    //Primary MPU region number used for DMA transfers
        	uint8_t ulRegionNumberSec;    			//Secondary MPU region number used for DMA transfers
        	uint32_t ulOption1;      				//Option1 used for EAR extensible access rights
        	//uint8_t ucOption2;					//Option2 reserved for future extension

        }Peripheral_Request_t;


        typedef struct STACK_REGION
        {
        	        uint32_t *ulTCB;
        			uint32_t *ulStackAddress;
                    uint32_t ulStackSize;
        } StackRegion_t;

        typedef struct GPIO_DEFINITION
        {
        	uint32_t ulPort;
			uint32_t ulPin;
        }GpioDefinition_t;


    #endif


/* Architecture specifics. */
    #define portSTACK_GROWTH      ( -1 )
    #define portTICK_PERIOD_MS    ( ( TickType_t ) 1000 / configTICK_RATE_HZ )
    #define portBYTE_ALIGNMENT    8
    #define portDONT_DISCARD      __attribute__( ( used ) )
/*-----------------------------------------------------------*/

/* SVC numbers for various services. */
    #define portSVC_START_SCHEDULER    0
    #define portSVC_YIELD              1
    #define portSVC_RAISE_PRIVILEGE    2

/* Scheduler utilities. */

    #define portYIELD()    __asm volatile ( "	SVC	%0	\n"::"i" ( portSVC_YIELD ) : "memory" )
    #define portYIELD_WITHIN_API()                      \
    {                                                   \
        /* Set a PendSV to request a context switch. */ \
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT; \
                                                        \
        /* Barriers are normally not required but do ensure the code is completely \
         * within the specified behaviour for the architecture. */ \
        __asm volatile ( "dsb" ::: "memory" );                     \
        __asm volatile ( "isb" );                                  \
    }

    #define portNVIC_INT_CTRL_REG     ( *( ( volatile uint32_t * ) 0xe000ed04 ) )
    #define portNVIC_PENDSVSET_BIT    ( 1UL << 28UL )
    #define portEND_SWITCHING_ISR( xSwitchRequired )    if( xSwitchRequired ) portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT
    #define portYIELD_FROM_ISR( x )                     portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/

/* Critical section management. */
    extern void vPortEnterCritical( void );
    extern void vPortExitCritical( void );
    #define portSET_INTERRUPT_MASK_FROM_ISR()         ulPortRaiseBASEPRI()
    #define portCLEAR_INTERRUPT_MASK_FROM_ISR( x )    vPortSetBASEPRI( x )
    #define portDISABLE_INTERRUPTS()                  vPortRaiseBASEPRI()
    #define portENABLE_INTERRUPTS()                   vPortSetBASEPRI( 0 )
    #define portENTER_CRITICAL()                      vPortEnterCritical()
    #define portEXIT_CRITICAL()                       vPortExitCritical()

/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
 * not necessary for to use this port.  They are defined so the common demo files
 * (which build with all the ports) will build. */
    #define portTASK_FUNCTION_PROTO( vFunction, pvParameters )    void vFunction( void * pvParameters )
    #define portTASK_FUNCTION( vFunction, pvParameters )          void vFunction( void * pvParameters )
/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
    #ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
        #define configUSE_PORT_OPTIMISED_TASK_SELECTION    1
    #endif

    #if configUSE_PORT_OPTIMISED_TASK_SELECTION == 1

/* Generic helper function. */
        __attribute__( ( always_inline ) ) static inline uint8_t ucPortCountLeadingZeros( uint32_t ulBitmap )
        {
            uint8_t ucReturn;

            __asm volatile ( "clz %0, %1" : "=r" ( ucReturn ) : "r" ( ulBitmap ) : "memory" );

            return ucReturn;
        }

/* Check the configuration. */
        #if ( configMAX_PRIORITIES > 32 )
            #error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
        #endif

/* Store/clear the ready priorities in a bit map. */
        #define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities )    ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
        #define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities )     ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

/*-----------------------------------------------------------*/

        #define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities )    uxTopPriority = ( 31UL - ( uint32_t ) ucPortCountLeadingZeros( ( uxReadyPriorities ) ) )

    #endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */

/*-----------------------------------------------------------*/

    #ifdef configASSERT
        void vPortValidateInterruptPriority( void );
        #define portASSERT_IF_INTERRUPT_PRIORITY_INVALID()    vPortValidateInterruptPriority()
    #endif

/* portNOP() is not required by this port. */
    #define portNOP()

    #define portINLINE              __inline

    #ifndef portFORCE_INLINE
        #define portFORCE_INLINE    inline __attribute__( ( always_inline ) )
    #endif
/*-----------------------------------------------------------*/

    extern BaseType_t xIsPrivileged( void );
    extern void vResetPrivilege( void );
    extern void vResetTaskPrivilege( void );

/**
 * @brief Checks whether or not the processor is privileged.
 *
 * @return 1 if the processor is already privileged, 0 otherwise.
 */
    #define portIS_PRIVILEGED()      xIsPrivileged()


#if (configPATCH_PRIVILEGE_ELEVATION == 1 )
extern BaseType_t xIsUsingMSP( void );
 /**
     * @brief Checks whether or not the processor is using MSP.
     *
     * @return 1 if the processor is using MSP, 0 otherwise.
 */
#define portIS_USING_MSP()      xIsUsingMSP()

#endif

/**
 * @brief Raise an SVC request to raise privilege.
 */
#define portRAISE_PRIVILEGE()    __asm volatile ( "svc %0 \n" ::"i" ( portSVC_RAISE_PRIVILEGE ) : "memory" );

/**
 * @brief Lowers the privilege level by setting the bit 0 of the CONTROL
 * register.
 */

#define portRESET_PRIVILEGE()    vResetPrivilege()
#define portRESET_TASK_PRIVILEGE()    vResetTaskPrivilege()
/*-----------------------------------------------------------*/

    portFORCE_INLINE static BaseType_t xPortIsInsideInterrupt( void )
    {
        uint32_t ulCurrentInterrupt;
        BaseType_t xReturn;

        /* Obtain the number of the currently executing interrupt. */
        __asm volatile ( "mrs %0, ipsr" : "=r" ( ulCurrentInterrupt )::"memory" );

        if( ulCurrentInterrupt == 0 )
        {
            xReturn = pdFALSE;
        }
        else
        {
            xReturn = pdTRUE;
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    portFORCE_INLINE static void vPortRaiseBASEPRI( void )
    {
        uint32_t ulNewBASEPRI;

        __asm volatile
        (
            "	mov %0, %1												\n"\
            "	msr basepri, %0											\n"\
            "	isb														\n"\
            "	dsb														\n"\
            : "=r" ( ulNewBASEPRI ) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
        );
    }

/*-----------------------------------------------------------*/

    portFORCE_INLINE static uint32_t ulPortRaiseBASEPRI( void )
    {
        uint32_t ulOriginalBASEPRI, ulNewBASEPRI;

        __asm volatile
        (
            "	mrs %0, basepri											\n"\
            "	mov %1, %2												\n"\
            "	msr basepri, %1											\n"\
            "	isb														\n"\
            "	dsb														\n"\
            : "=r" ( ulOriginalBASEPRI ), "=r" ( ulNewBASEPRI ) : "i" ( configMAX_SYSCALL_INTERRUPT_PRIORITY ) : "memory"
        );

        /* This return will not be reached but is necessary to prevent compiler
         * warnings. */
        return ulOriginalBASEPRI;
    }
/*-----------------------------------------------------------*/

    portFORCE_INLINE static void vPortSetBASEPRI( uint32_t ulNewMaskValue )
    {
        __asm volatile
        (
            "	msr basepri, %0	"::"r" ( ulNewMaskValue ) : "memory"
        );
    }
/*-----------------------------------------------------------*/

    #define portMEMORY_BARRIER()    __asm volatile ( "" ::: "memory" )

    #ifndef configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY
        #warning "configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY is not defined. We recommend defining it to 1 in FreeRTOSConfig.h for better security. https://www.FreeRTOS.org/FreeRTOS-V10.3.x.html"
        #define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY    0
    #endif
/*-----------------------------------------------------------*/
    #ifdef __cplusplus
        }
    #endif

#endif /* PORTMACRO_H */
