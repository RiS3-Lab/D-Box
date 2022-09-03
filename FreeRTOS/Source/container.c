/*
 * container.c
 *
 *  Created on: Dec 11, 2020
 *      Author: alejandro
 */




/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* FreeRTOS includes. */
#include "FreeRTOS.h"

#if configUSE_DMA_CONTAINER == 1
#include "main.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "container.h"
#include "stdbool.h"
#include "timers.h"


#define TIMEOUT_NOTIFICATION		300
#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01
#define TIMEOUT_USART2_RX		5   //number of ticks

/* Lint e9021, e961 and e750 are suppressed as a MISRA exception justified
 * because the MPU ports require MPU_WRAPPERS_INCLUDED_FROM_API_FILE to be defined
 * for the header files above, but not in this file, in order to generate the
 * correct privileged Vs unprivileged linkage and placement. */
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE /*lint !e961 !e750 !e9021. */

#define NBYTE(v, n)  ((uint8_t) (((uint32_t) (v)) >> (8 * n)))
#define QUEUE_DMA_LENGTH    10
#define ITEM_SIZE sizeof( Peripheral_Request_t )

PRIVILEGED_DATA static StaticQueue_t xStaticQueue;
PRIVILEGED_DATA uint8_t ucQueueStorageArea[ QUEUE_DMA_LENGTH * ITEM_SIZE ];
PRIVILEGED_DATA static QueueHandle_t xQueueDMA;
PRIVILEGED_DATA static TaskHandle_t xDMAdriverTaskHandle = NULL;                          /*< Holds the handle of the DMA driver.  The DMA driver task is created automatically when the scheduler is started. */
PRIVILEGED_DATA static TimerHandle_t xTimerUSART2;
PRIVILEGED_DATA StackRegion_t xSTackRegions[8];
PRIVILEGED_DATA uint32_t ulNumberRestrictedTask = 0;
PRIVILEGED_DATA uint8_t  ucSlaveOwnAddress;         // I2C slave address
PRIVILEGED_DATA uint8_t ucMasterRequestDirection;   // I2C Read/Write direction
PRIVILEGED_DATA uint16_t usMasterNbDataToReceive;   // I2C Number of bytes to receive
PRIVILEGED_DATA bool ucGenerateStop;             // I2C Flag to control the Stop condition
PRIVILEGED_DATA uint8_t ucCSSP1;

PRIVILEGED_DATA static TaskHandle_t xTaskToNotify = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskUSART2NotifyTX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskUSART2NotifyRX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskSPI1NotifyRX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskSPI1NotifyTX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskI2C2NotifyRX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskI2C2NotifyTX = NULL;
PRIVILEGED_DATA static TaskHandle_t xTaskADC1Notify = NULL;


PRIVILEGED_DATA static uint32_t ulUSART2SizeRX;

#ifdef STM32L475xx
PRIVILEGED_DATA static TaskHandle_t xTaskUART4NotifyRX = NULL;
PRIVILEGED_DATA static uint32_t ulUART4SizeRX;
#endif

//static uint32_t xContainerWaitNotification(void *ulTask);
static void xContainerGPIOwrite(uint8_t ucCSnum, uint8_t ucVal);

extern PeripheralPermission_t * xTaskAdditionsGetPermissions(void);
extern xMPU_REGION_REGISTERS * xTaskAdditionsGetMemoryRegion(BaseType_t xRegionNumber);

#define PRIO_ISR_USART2 5

#if (configUSE_TRACE_FACILITY ==1)
traceHandle USARTHandle;
traceHandle DMAHandle;
traceHandle SPIHandle;
traceHandle ADCHandle;
traceHandle I2CHandle;
#endif


#ifdef STM32L152xE

static void SPI1_TransferError_Callback();
static void USART2_TransferError_Callback();
void vTimerUSART2Callback( TimerHandle_t xTimer );

#endif


static const uint32_t ulRanksADC[] ={
		LL_ADC_REG_RANK_1,
		LL_ADC_REG_RANK_2,
		LL_ADC_REG_RANK_3,
		LL_ADC_REG_RANK_4,
		LL_ADC_REG_RANK_5,
		LL_ADC_REG_RANK_6,
		LL_ADC_REG_RANK_7,
		LL_ADC_REG_RANK_8,
		LL_ADC_REG_RANK_9,
		LL_ADC_REG_RANK_10,
		LL_ADC_REG_RANK_11,
		LL_ADC_REG_RANK_12,
		LL_ADC_REG_RANK_13,
		LL_ADC_REG_RANK_14,
		LL_ADC_REG_RANK_15,
		LL_ADC_REG_RANK_16
};


static const uint32_t ulChannelsADC[] ={
		LL_ADC_CHANNEL_0,
		LL_ADC_CHANNEL_1,
		LL_ADC_CHANNEL_2,
		LL_ADC_CHANNEL_3,
		LL_ADC_CHANNEL_4,
		LL_ADC_CHANNEL_5,
		LL_ADC_CHANNEL_6,
		LL_ADC_CHANNEL_7,
		LL_ADC_CHANNEL_8,
		LL_ADC_CHANNEL_9,
		LL_ADC_CHANNEL_10,
		LL_ADC_CHANNEL_11,
		LL_ADC_CHANNEL_12,
		LL_ADC_CHANNEL_13,
		LL_ADC_CHANNEL_14,
		LL_ADC_CHANNEL_15

};

static const uint32_t ulNumberOfRanks[] ={
LL_ADC_REG_SEQ_SCAN_DISABLE,
LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_4RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_6RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_7RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_9RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_10RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_11RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_12RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_13RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_14RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_15RANKS,
LL_ADC_REG_SEQ_SCAN_ENABLE_16RANKS
};



BaseType_t xContainerSendRequest(Peripheral_Request_t xRequest, BaseType_t xCallerIsPrivileged)
{
	PeripheralPermission_t *PeripheralPermission = (PeripheralPermission_t *) xTaskAdditionsGetPermissions();
    uint8_t uIndex;
    ePeripheralOperation xPeripheraloperation = eNone;
    xMPU_REGION_REGISTERS *xMPU_region, *xMPU_regionSec;
    uint32_t ulBaseAddress, ulBaseAddressSec;
	uint32_t ulRegionSize, ulRegionSizeSec;
	uint8_t ucAccessPermission, ucAccessPermissionSec;
	uint8_t ucDMATransferDataAlign = 1; //each DMA operation is 1 byte length
	bool ucValidated;


    /*Check access permission for peripheral*/
	ucValidated = false;
	for(uIndex = 0; uIndex < portTOTAL_NUM_PERMISSIONS && (ucValidated == false) ; uIndex++  )
	{
		/* Verify if the current task has access to peripheral */
		if (PeripheralPermission->pulPeripheral == xRequest.pulPeripheral)
		{
			/* Verify if requested operation is a Read or write */
			if(xRequest.ucOperation == eWrite ||  xRequest.ucOperation == eRead   ||   xRequest.ucOperation == eFullDuplex )
			{
				/* Verify if the the requested operation is included in the application capabilities*/
				xPeripheraloperation= xRequest.ucOperation;

				if(xPeripheraloperation & PeripheralPermission->ucOperation)
				{

					switch ((uint32_t) xRequest.pulPeripheral) {
						case (uint32_t) USART1:
						case (uint32_t) USART2:
						case (uint32_t) USART3:
						case (uint32_t) SPI1:
						case (uint32_t) SPI2:
						case (uint32_t) SPI3:
							/* For these peripherals, specifically SPI the size for tx and rx must be the same, so we make sure to use the same size here for the next validations */
							xRequest.ulSizeSec = xRequest.ulSize;

							/* verify that the permission includes a valid chip select GPIO, useful for SPI or RS485 transceivers (i.e., EAR validation) */
							if( ((uint8_t)xRequest.ulOption1 & 0x7F )>= 0 && ( (uint8_t)xRequest.ulOption1 & 0x7F ) < portTOTAL_NUM_GPIO_CS   )
							{
								if( ((uint8_t)xRequest.ulOption1 & 0x7F)  == (uint8_t)PeripheralPermission->ucOption )
								{
									ucValidated = true;
									break;
								}


							}
							else if( (uint8_t)xRequest.ulOption1 == OPT_NULL )
							{
								ucValidated = true;
								break; // finish the loop continue with the next validation of MPU region
							}

							break;
						case (uint32_t) I2C1:
						case (uint32_t) I2C2:
							if( (uint8_t)xRequest.ulOption1 == (uint8_t)PeripheralPermission->ucOption ) // for I2C the option is the address of the slave device (i.e., EAR validation)
							{
								ucValidated = true;
								break;
							}
							break;
						case (uint32_t) ADC1:
								if( (uint32_t)xRequest.ulOption1 && (uint32_t)PeripheralPermission->ucOption ) // for I2C the option is the address of the slave device
								{
									ucValidated = true;
									ucDMATransferDataAlign = 2; // DMA size operation 2 bytes
									break;
								}
								break;
						default:

							break;
					}



				}


			}

		}
		PeripheralPermission++;

	}

	if (ucValidated == false)
	{
		return pdFREERTOS_ERRNO_EACCES;
	}

	/* Check for boundaries of the transfer*/

	if ( (xRequest.ulRegionNumber < portFIRST_REGION_NUMBER  || xRequest.ulRegionNumber > portLAST_REGION_NUMBER)  &&  xRequest.ulRegionNumber != portSTACK_REGION_NUMBER )
	{
		return pdFREERTOS_ERRNO_EACCES; //invalid region number
	}

	xMPU_region = xTaskAdditionsGetMemoryRegion(xRequest.ulRegionNumber);
	/* Check if the region has valid parameters*/
	if (xMPU_region->ulRegionAttribute == 0 || xMPU_region->ulRegionBaseAddress == 0 )
	{
		return pdFREERTOS_ERRNO_EACCES; // the region is deactivated or invalid
	}

	/* Check if size and start of the transfer is between of MPU region boundaries*/
	ulRegionSize = ((xMPU_region->ulRegionAttribute)>>1) & 0x1f;
	ulRegionSize = 2<<ulRegionSize;
	ulBaseAddress = (xMPU_region->ulRegionBaseAddress & 0xFFFFFFE0);

	if((xRequest.ulAddress + xRequest.ulSize) > (ulBaseAddress +   ulRegionSize) )
	{
		return pdFREERTOS_ERRNO_EACCES; // transfer exceeds the boundaries of the MPU  region
	}

	/* Check if the task has access to read/write to the region used by the DMA transfer */
	ucAccessPermission =  NBYTE(xMPU_region->ulRegionAttribute, 3) & 0x03; //MPU_RASR byte[3] contains the permissions
	if(ucAccessPermission == 0x04 || ucAccessPermission == 0x00 )
	{
		return pdFREERTOS_ERRNO_EACCES; // 0x00 = no access PU, 0x04 = reserved
	}


	if( xRequest.ucOperation == eFullDuplex ) //extra verifications when using full Duplex
	{
		if ( (xRequest.ulRegionNumberSec < portFIRST_REGION_NUMBER  || xRequest.ulRegionNumberSec > portLAST_REGION_NUMBER)  &&  xRequest.ulRegionNumberSec != portSTACK_REGION_NUMBER )
		{
			return pdFREERTOS_ERRNO_EACCES; //invalid region number
		}
		xMPU_regionSec = xTaskAdditionsGetMemoryRegion(xRequest.ulRegionNumberSec);
		if ( xMPU_regionSec->ulRegionAttribute == 0 || xMPU_regionSec->ulRegionBaseAddress == 0 )
		{
				return pdFREERTOS_ERRNO_EACCES; // the region is deactivated or invalid
		}

		/* Check size and start of the transfer is inside of MPU region boundaries*/
		ulRegionSizeSec = ((xMPU_regionSec->ulRegionAttribute)>>1) & 0x1f;
		ulRegionSizeSec = 2<<ulRegionSizeSec;
		ulBaseAddressSec = (xMPU_regionSec->ulRegionBaseAddress & 0xFFFFFFE0);

		if( (xRequest.ulAddressSec + (xRequest.ulSizeSec*ucDMATransferDataAlign) ) > (ulBaseAddressSec +   ulRegionSizeSec) )
		{
			return pdFREERTOS_ERRNO_EACCES; // transfer exceeds the boundaries of the MPU  region
		}
		ucAccessPermissionSec =  NBYTE(xMPU_regionSec->ulRegionAttribute, 3) & 0x03; //MPU_RASR byte[3] contains the permissions
		if(ucAccessPermissionSec == 0x04 || ucAccessPermissionSec == 0x00 )
		{
			return pdFREERTOS_ERRNO_EACCES; // 0x00 = no access PU, 0x04 = reserved
		}
	}

	// Get the handler of the Task to notify
	xRequest.ulTask = xTaskGetCurrentTaskHandle();

	/* Check permissions for reading/writing on the MPU regions */
	if( xRequest.ucOperation == eRead )
	{

		if( xCallerIsPrivileged || ( ucAccessPermission & 0x02) ) // Read in primary region PU
		{
			//send read request
			return xQueueSend(xQueueDMA, (void * ) &xRequest, ( TickType_t ) 10 );
		}

		else
		{
			return pdFREERTOS_ERRNO_EACCES;
		}

	}
	else if( xRequest.ucOperation == eWrite )
	{

		if((xCallerIsPrivileged && ( ucAccessPermission <= 0x03)) || // write in primary region P
		    (!xCallerIsPrivileged  && (ucAccessPermission == 0x03))) // write in primary region U
		{
			//send read request
			return xQueueSend(xQueueDMA, (void * ) &xRequest, ( TickType_t ) 10 );
		}
		else
		{
			return pdFREERTOS_ERRNO_EACCES;
		}

	}
	else if( xRequest.ucOperation == eFullDuplex )
	{
		if( ( xCallerIsPrivileged ||( ucAccessPermission & 0x02) ) &&  // Read in primary region PU
		    ( ( xCallerIsPrivileged && ( ucAccessPermissionSec <= 0x03 ) ) ||  // write in secondary region P
		      (!xCallerIsPrivileged  && (ucAccessPermissionSec == 0x03 ) )))  // write in secondary region U
		{
			//send read request
			return xQueueSend(xQueueDMA, (void * ) &xRequest, ( TickType_t ) 10 );
		}
		else
		{
			return pdFREERTOS_ERRNO_EACCES;
		}

	}

	return pdFREERTOS_ERRNO_EACCES;
}



//static TaskHandle_t xTaskToNotify = NULL;


/* The index within the target task's array of task notifications
to use. */
//const UBaseType_t xArrayIndex = 1;



static void prvDMATask( void * pvParameters );
static void prvDMATask( void * pvParameters )
{
	Peripheral_Request_t xPeripheralRequest;
	uint32_t ulNotificationValue;
	//uint32_t ulDMAchannel;
	//I2C_TypeDef * pxI2C ;


	for( ; ; )
			{
			    /* wait for a request forever */
				if( xQueueReceive( xQueueDMA, &(xPeripheralRequest), portMAX_DELAY ) == pdPASS )
			    {
					 xTaskToNotify = xTaskGetCurrentTaskHandle();


					 if (xPeripheralRequest.ucOperation == eWrite)
					 {

						 switch ((uint32_t)xPeripheralRequest.pulPeripheral)
						 {
						     case (uint32_t) USART2:


						    	 if(xTaskUSART2NotifyTX != NULL)
						    	 {
						    		 xQueueSend(xQueueDMA, &(xPeripheralRequest), ( TickType_t ) 10 );
						    		 break;
						    	 }

						         LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_7, xPeripheralRequest.ulAddress);  // set address
						         LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_7,  xPeripheralRequest.ulSize);		// set size
						         LL_USART_EnableDMAReq_TX(USART2); // Enable TX request of USART2
						         LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_7);
						         xTaskUSART2NotifyTX = xPeripheralRequest.ulTask;
						         //xContainerWaitNotification(xPeripheralRequest.ulTask);


								break;

						     case (uint32_t) UART4:
								 #ifdef STM32L475xx

						    	 LL_DMA_SetPeriphAddress(DMA2, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT));  // set address
						    	 LL_DMA_SetMemoryAddress(DMA2, LL_DMA_CHANNEL_3, xPeripheralRequest.ulAddress);  // set address
						     	 LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_3,  xPeripheralRequest.ulSize);		// set size
						     	 LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_3, LL_DMA_REQUEST_2);             //request from uart4 TX
						     	 LL_USART_EnableDMAReq_TX(UART4); // Enable TX request of USART2

						     	 LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_3);
						     	 xContainerWaitNotification(xPeripheralRequest.ulTask);

						     	 #endif
						     	 break;

						     case  (uint32_t) SPI1:
						         LL_SPI_SetTransferDirection(SPI1,LL_SPI_HALF_DUPLEX_TX);
					     	     LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, xPeripheralRequest.ulAddress, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
						     	 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, xPeripheralRequest.ulSize);
						     	 LL_SPI_EnableDMAReq_TX(SPI1);
						     	 xContainerGPIOwrite( (uint8_t)xPeripheralRequest.ulOption1, 0);
						     	 ucCSSP1 = (uint8_t)xPeripheralRequest.ulOption1;
						         xTaskSPI1NotifyTX = xPeripheralRequest.ulTask;
						     	 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
						     	 LL_SPI_Enable(SPI1);

						         break;

							 #ifdef STM32L152xE
						     case (uint32_t) I2C2:
							    	 /* (1) Enable I2C2 **********************************************************/
							    	 LL_I2C_Enable(I2C2);
							    	 LL_I2C_EnableIT_EVT(I2C2);
							    	 LL_I2C_EnableIT_ERR(I2C2);

							    	 LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, LL_I2C_DMA_GetRegAddr(I2C2));
							    	 LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, xPeripheralRequest.ulAddress);
							    	 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, xPeripheralRequest.ulSize);
							    	 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

							    	 /*  Initiate a Start condition to the Slave device ***********************/
							    	 /* Master Request direction WRITE */
							    	 ucMasterRequestDirection = I2C_REQUEST_WRITE;
							    	 /* Set Address to WRITE */
							    	 ucSlaveOwnAddress = (uint8_t)xPeripheralRequest.ulOption1 <<1; // option1 is the slave address, it is shifted left per I2C address + read/write schema

							    	 ucGenerateStop = true; // this flag must be true to generate a stop condition in the TXE event

							    	 /* Master Generate Start condition */
							    	 LL_I2C_GenerateStartCondition(I2C2);
							    	 //xContainerWaitNotification(xPeripheralRequest.ulTask);
							    	 xTaskI2C2NotifyTX = xPeripheralRequest.ulTask;
							    	 // The stop condition is generated in the I2C TXE event

						    	break;
							 #endif

						 	 default:
								break;
						}

					 }
					 else if(xPeripheralRequest.ucOperation == eRead)
					 {
						 switch ((uint32_t)xPeripheralRequest.pulPeripheral)
								{
								     case (uint32_t) USART2:
								    		 xTaskUSART2NotifyRX =  xPeripheralRequest.ulTask;
								    		 ulUSART2SizeRX = xPeripheralRequest.ulSize;
								     	 	 LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_6, xPeripheralRequest.ulAddress);  // set address
								             LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6,  xPeripheralRequest.ulSize);		// set peripheral address
								             LL_USART_EnableDMAReq_RX(USART2); // Enable RX request of USART2
								     	 	 LL_DMA_ClearFlag_TC6(DMA1);
								             LL_USART_ClearFlag_IDLE(USART2);
								             LL_USART_EnableIT_IDLE(USART2);
								     	 	 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
								     	 	 /*
								     	 	  * Note the notification for this request is send directly to the requesting task
								     	 	  * otherwise waiting for incoming unknown size messages is not possible
								     	 	  * */
								     	 	 break;


								     case (uint32_t) SPI1:
								    		 LL_SPI_Disable(SPI1);
								    		 LL_SPI_SetTransferDirection(SPI1,LL_SPI_SIMPLEX_RX);
								             LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), xPeripheralRequest.ulAddress, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
								    		 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, xPeripheralRequest.ulSize);
								    		 LL_SPI_EnableDMAReq_RX(SPI1);

								    		 while(LL_SPI_IsActiveFlag_RXNE(SPI1))
								    		 				LL_SPI_ReceiveData8(SPI1);
								    		 xContainerGPIOwrite( (uint8_t)xPeripheralRequest.ulOption1, 0 );
								    		 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
								    		 ucCSSP1 = (uint8_t)xPeripheralRequest.ulOption1;
								    		 xTaskSPI1NotifyRX = xPeripheralRequest.ulTask ;
								    		 LL_SPI_Enable(SPI1);


								    		 break;
									 #ifdef STM32L152xE
								     case (uint32_t) I2C2:


								    		 LL_I2C_Enable(I2C2);
								    		 LL_I2C_EnableIT_EVT(I2C2);
								    		 LL_I2C_EnableIT_ERR(I2C2);
								    		 LL_I2C_DisableDMAReq_RX(I2C2);

								    		 LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, LL_I2C_DMA_GetRegAddr( I2C2 ));
								    		 LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, xPeripheralRequest.ulAddress);
								    		 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, xPeripheralRequest.ulSize);
								    		 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

								    		 /* (6) Prepare acknowledge for Master data reception ************************/
								    		 LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);
   							    			 /* (7) Initiate a ReStart condition to the Slave device *********************/
								    		 /* Master Request direction READ */
								    		 ucMasterRequestDirection = I2C_REQUEST_READ;
								    		 ucSlaveOwnAddress = (uint8_t)xPeripheralRequest.ulOption1<<1;
								    		 usMasterNbDataToReceive = xPeripheralRequest.ulSize;

								    		 /* Master Generate ReStart condition, i.e. we did not generate a stop condition previously */
								    		 ucGenerateStop = true; // we need a stop condition after the transfer finishes
								    		 LL_I2C_GenerateStartCondition( I2C2 ); // the restart condition is required before enabling the interrupt again
								    		 //xContainerWaitNotification( xPeripheralRequest.ulTask );
								    		 xTaskI2C2NotifyRX =   xPeripheralRequest.ulTask;

								    		 break;
									 #endif


								     case (uint32_t) ADC1:

								    		 // Configure DMA
								    		 LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr( ADC1, LL_ADC_DMA_REG_REGULAR_DATA ));
								    		 LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_1, xPeripheralRequest.ulAddress );
								    		 LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_1, xPeripheralRequest.ulSize );


								    		 // Configure Scan sequence
								    		 uint8_t ucRank  = 0;
								    		 uint32_t ulChannel = 1;

								    		 for(int i = 0; i< 16 ; i++ ) // only 16 channels for this prototype, we consider all channels are in the same bank
								    		 {
								    			 if( ulChannel & xPeripheralRequest.ulOption1 )
								    			 {
								    				 LL_ADC_REG_SetSequencerRanks(ADC1,  ulRanksADC[ucRank], ulChannelsADC[i]);
													 #ifdef STM32L152xE
								    				 LL_ADC_SetChannelSamplingTime(ADC1,  ulChannelsADC[i], LL_ADC_SAMPLINGTIME_48CYCLES);
													 #endif
								    				 ucRank++;
								    			 }
								    			 ulChannel = ulChannel<<1;

								    		 }

								    		 LL_ADC_REG_SetSequencerLength( ADC1, ulNumberOfRanks[ucRank-1] );

								    		 /* Activate HSI if not already activated */
								    		 // We can activate the ADC early in the board initialization, but we keep here to save power
								    		 if (LL_RCC_HSI_IsReady() == 0)
								    		 {
								    			 LL_RCC_HSI_Enable();
								    			 while(LL_RCC_HSI_IsReady() != 1)
								    			 {}
								    		 }
								    		 if (LL_ADC_IsEnabled(ADC1) == 0)
								    		 {
								    		    /* Enable ADC */
								    		    LL_ADC_Enable(ADC1);

								    		 }

											 #ifdef STM32L152xE
								    		 //LL_ADC_EnableIT_EOCS(ADC1);
								    		 // Enable the DMA channel and start the conversion
								    		 LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);    // first reset the DMA bit and set again, this is required according to data-sheet and verified here
								    		 LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED); // without reseting and setting this bit ADC does not accept new starting triggers
								    		 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
								    		 LL_ADC_REG_StartConversionSWStart(ADC1);
								    		 xTaskADC1Notify = xPeripheralRequest.ulTask;

								    		 #endif


								     default:
								    	 break;
								}

					 }
					 else if(xPeripheralRequest.ucOperation == eFullDuplex)
					 {
						 switch ((uint32_t)xPeripheralRequest.pulPeripheral) {
							case (uint32_t) SPI1:

									 LL_SPI_Disable(SPI1); //we need to disable SPI before setting transfer direction
									 LL_SPI_SetTransferDirection(SPI1,LL_SPI_FULL_DUPLEX);
									 //RX
									 LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, LL_SPI_DMA_GetRegAddr(SPI1), xPeripheralRequest.ulAddressSec, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));
									 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, xPeripheralRequest.ulSize);
									 //TX
									 LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, xPeripheralRequest.ulAddress, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
									 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, xPeripheralRequest.ulSize);

									 // RX interrupt

					                 LL_SPI_EnableDMAReq_RX(SPI1);
					                 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

					                 // TX interrupt

					                 LL_SPI_EnableDMAReq_TX(SPI1);
					                 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

					                 xContainerGPIOwrite((uint8_t)xPeripheralRequest.ulOption1, 0);
					                 xTaskSPI1NotifyTX = xPeripheralRequest.ulTask;

					                 LL_SPI_Enable(SPI1);


								break;
							#ifdef STM32L152xE
							case (uint32_t) I2C2:
							    	 /* (1) Enable I2C2 **********************************************************/

							    	 LL_I2C_Enable(I2C2);
							    	 LL_I2C_EnableIT_EVT(I2C2);
							    	 LL_I2C_EnableIT_ERR(I2C2);
							    	 LL_I2C_DisableDMAReq_TX(I2C2);

							    	 LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4, LL_I2C_DMA_GetRegAddr(I2C2));
							    	 LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_4, xPeripheralRequest.ulAddress);
							    	 LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, xPeripheralRequest.ulSize);

							    	 LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);

							    	 /*  Initiate a Start condition to the Slave device ***********************/
							    	 /* Master Request direction WRITE */
							    	 ucMasterRequestDirection = I2C_REQUEST_WRITE;
							    	 ucSlaveOwnAddress = (uint8_t)xPeripheralRequest.ulOption1<<1; // option1 is the slave address, it is shifted left per I2C address + read/write schema

							    	 /* Master Generate Start condition */
							    	 ucGenerateStop = false; // we will keep control of the bus so we don't need a stop condition, we will rather use a restart
							    	 LL_I2C_GenerateStartCondition(I2C2);
							    	 xTaskI2C2NotifyTX = xTaskToNotify;
							    	 ulNotificationValue = ulTaskNotifyTakeIndexed( 0, pdFALSE, TIMEOUT_NOTIFICATION );

							    	 if( ulNotificationValue == 1 )
							    	 {
							    		 LL_I2C_DisableDMAReq_TX(I2C2);
							    		 xTaskToNotify = xTaskGetCurrentTaskHandle();
							    	    /* The TX ended as expected.  Lets continue with RX */
							    		//Configure DMA
							    		LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_5, LL_I2C_DMA_GetRegAddr( I2C2 ));
							    		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_5, xPeripheralRequest.ulAddressSec);
							    		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, xPeripheralRequest.ulSizeSec);
							    		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

							    		/* (6) Prepare acknowledge for Master data reception ************************/
							    		LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_ACK);

							    		/* (7) Initiate a ReStart condition to the Slave device *********************/
							    		/* Master Request direction READ */
							    		ucMasterRequestDirection = I2C_REQUEST_READ;
							    		ucSlaveOwnAddress = (uint8_t)xPeripheralRequest.ulOption1<<1;
							    		usMasterNbDataToReceive = xPeripheralRequest.ulSizeSec;

							    		/* Master Generate ReStart condition, i.e. we did not generate a stop condition previously */
							    		ucGenerateStop = true; // we need a stop condition after the transfer finishes
							    		LL_I2C_GenerateStartCondition( I2C2 ); // the restart condition is required before enabling the interrupt again

							    		LL_I2C_EnableIT_EVT(I2C2);
							    		LL_I2C_EnableIT_ERR(I2C2);

							    		xTaskI2C2NotifyRX =  xPeripheralRequest.ulTask;

							    	 }

							    	 break;
							#endif

							default:
								break;
						}
					 }


			    }
			}
}


void vStartDMATask( void )
{

	static StackType_t xDMATaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( configMINIMAL_STACK_SIZE * sizeof( StackType_t ) ) ) );


	TaskParameters_t xDMATaskParameters =
	{
		.pvTaskCode		= prvDMATask,
		.pcName			= "DMAtask",
		.usStackDepth	= configMINIMAL_STACK_SIZE,
		.pvParameters	= NULL,
		.uxPriority		= 10 | portPRIVILEGE_BIT,
		.puxStackBuffer	= xDMATaskStack,
		.xRegions		=	{
								{ 0, 0, 0 },
								{ 0, 0, 0 },
								{ 0, 0, 0 },
							}
	};


	/* Create a restricted, but privileged task  to handle the DMA driver*/
	xDMAdriverTaskHandle = (TaskHandle_t) xTaskCreateRestricted( &( xDMATaskParameters ), NULL );

	/* xDMAdriverTaskHandle should not be NULL. */
	configASSERT( xDMAdriverTaskHandle );

    /* Create a queue capable of containing 10 Peripheral_Request_t structures. */
	xQueueDMA = xQueueCreateStatic( QUEUE_DMA_LENGTH,
                             ITEM_SIZE,
                             ucQueueStorageArea,
                             &xStaticQueue );


	xTaskUSART2NotifyTX = NULL;

	/*Create a timer to handle timeout of USART, without autoload*/
	xTimerUSART2 = xTimerCreate("TimerUSART2", TIMEOUT_USART2_RX, pdFALSE, ( void * ) 0, vTimerUSART2Callback  );


    /* xQueueDMA should not be NULL. */
    configASSERT( xQueueDMA );
#if (configUSE_TRACE_FACILITY ==1)
    USARTHandle =  xTraceSetISRProperties("ISR_USART2", PRIO_ISR_USART2);
    DMAHandle =  xTraceSetISRProperties("ISR_DMA", PRIO_ISR_USART2);
    SPIHandle =  xTraceSetISRProperties("ISR_SPI", PRIO_ISR_USART2);
    I2CHandle =  xTraceSetISRProperties("ISR_I2C", PRIO_ISR_USART2);
    ADCHandle =  xTraceSetISRProperties("ISR_ADC", PRIO_ISR_USART2);

#endif
}




void vTimerUSART2Callback( TimerHandle_t xTimer )
{

	if(xTaskUSART2NotifyRX)
	{
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_6);
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
	/* Notify the DMA task that the RX line is idle */
	xTaskNotify(xTaskUSART2NotifyRX,  ( ulUSART2SizeRX -  LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6) ) << 8,  eSetValueWithoutOverwrite);
	xTaskUSART2NotifyRX = NULL;
	}

}



static void xContainerGPIOwrite(uint8_t ucCSnum, uint8_t ucVal)
{
	uint8_t ucKeepEnabled;

	ucKeepEnabled = ucCSnum & OPT_CS_KEEP_ON;

	ucCSnum = ucCSnum & 0x7f;


	if( ucCSnum != 0xff )
	{
		if( !(ucKeepEnabled) && ucVal ) //CS disable in HI
	    {
			LL_GPIO_SetOutputPin((GPIO_TypeDef *) BoardGPIO[ucCSnum].ulPort, BoardGPIO[ucCSnum].ulPin);
		}
		else
		{
			LL_GPIO_ResetOutputPin((GPIO_TypeDef *) BoardGPIO[ucCSnum].ulPort, BoardGPIO[ucCSnum].ulPin);
		}
	}

}



// This section manages specific interrupt service routines for for STM32l152xE
#ifdef STM32L152xE

//SPI RX
void DMA1_Channel2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif

    if(LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_2) && LL_DMA_IsActiveFlag_TC2(DMA1))
    {
    	//Disable SPI immediately to stop clocking out more bytes from slave
    	//Last byte will not be corrupted since it is already received in SPI buffer and copied by DMA,
    	//for TX operations extra consideration should be taken since the last byte can be corrupted.
    	//Also forcing  TX mode can stop the clock more information here: https://electronics.stackexchange.com/questions/130435/stm32-spi-half-duplex-1-wire-bidirectional-problem

    	if(LL_SPI_GetTransferDirection(SPI1) ==  LL_SPI_SIMPLEX_RX)
    	{

    		LL_DMA_ClearFlag_TC2(DMA1);
			LL_SPI_DisableDMAReq_RX(SPI1);
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2); //disable RX DMA channel
			LL_SPI_Disable(SPI1); // stop the clock

			xContainerGPIOwrite(ucCSSP1, 1);

			configASSERT( xTaskSPI1NotifyRX != NULL);

			/* Notify the task that the transmission is complete. */
			vTaskNotifyGiveIndexedFromISR( xTaskSPI1NotifyRX, 0, &xHigherPriorityTaskWoken );

			/* There are no transmissions in progress, so no tasks to notify. */
			xTaskSPI1NotifyRX = NULL;

			/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
					should be performed to ensure the interrupt returns directly to the highest
					priority task.  The macro used for this purpose is dependent on the port in
					use and may be called portEND_SWITCHING_ISR(). */
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    	}
    	else //in full duplex wait for TX before disabling the SPI and notifying, here disable RX
    	{
    		LL_DMA_ClearFlag_TC2(DMA1);
    		LL_SPI_DisableDMAReq_RX(SPI1);
    		LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2); //disable RX DMA channel
    	}

    }
    if(LL_DMA_IsEnabledIT_TE(DMA1, LL_DMA_CHANNEL_2) && LL_DMA_IsActiveFlag_TE2(DMA1))
    {
     /* Call Error function */
        SPI1_TransferError_Callback();
    }

#if (configUSE_TRACE_FACILITY ==1)
    vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}


void SPI1_IRQHandler(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(SPIHandle);
#endif

	if( LL_SPI_IsEnabledIT_TXE(SPI1) && LL_SPI_IsActiveFlag_TXE(SPI1)  )
	{
		LL_SPI_DisableIT_TXE(SPI1);
		while(LL_SPI_IsActiveFlag_BSY(SPI1));
		configASSERT( xTaskSPI1NotifyTX != NULL);

		/* Notify the DMA task that the transmission is complete. */
		//vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

		xContainerGPIOwrite(ucCSSP1, 1);
		vTaskNotifyGiveIndexedFromISR( xTaskSPI1NotifyTX, 0, &xHigherPriorityTaskWoken );

		/* There are no transmissions in progress, so no tasks to notify. */
		xTaskSPI1NotifyTX = NULL;

		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
				should be performed to ensure the interrupt returns directly to the highest
				priority task.  The macro used for this purpose is dependent on the port in
				use and may be called portEND_SWITCHING_ISR(). */
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}
#if (configUSE_TRACE_FACILITY ==1)
    vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif

}


//SPI TX
void DMA1_Channel3_IRQHandler(void)
{
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif

    if(LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_3) && LL_DMA_IsActiveFlag_TC3(DMA1) )
    {
        // For TX is safe to keep the SPI enabled since no more clocks are sent to slave
    	// but we cannot disable SPI immediately since it will corrupt the last byte
    	LL_DMA_ClearFlag_TC3(DMA1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3); //disable TX DMA channel

        LL_SPI_EnableIT_TXE(SPI1); // we need to catch the end of transmission of the last byte before disabling the SPI

    }
    if(LL_DMA_IsEnabledIT_TE(DMA1, LL_DMA_CHANNEL_3) && LL_DMA_IsActiveFlag_TE3(DMA1))
    {
     /* Call Error function */
    //   SPI1_TransferError_Callback();
    	while(1);
    }

#if (configUSE_TRACE_FACILITY ==1)
    vTraceStoreISREnd(0);
#endif
}


//USART TX
void DMA1_Channel7_IRQHandler(void) {
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif
	/* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_7) && LL_DMA_IsActiveFlag_TC7(DMA1))
    {
        LL_DMA_ClearFlag_TC7(DMA1);             /* Clear transfer complete flag */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_7); //disable TX DMA channel
        LL_USART_ClearFlag_TC(USART2);
        LL_USART_EnableIT_TC(USART2);			/* enable TC interrupt for USART2
        we need to catch the end of the last byte */

    }

    if (LL_DMA_IsEnabledIT_TE(DMA1, LL_DMA_CHANNEL_7) && LL_DMA_IsActiveFlag_TE7(DMA1))
    {
    	USART2_TransferError_Callback();
    }
#if (configUSE_TRACE_FACILITY ==1)
    vTraceStoreISREnd(0);
#endif
    /* Implement other events when needed */
}


//USART RX
void DMA1_Channel6_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif
    /*RX Check transfer-complete interrupt  */
    if (LL_DMA_IsEnabledIT_TC(DMA1, LL_DMA_CHANNEL_6) && LL_DMA_IsActiveFlag_TC6(DMA1))
    {

    	if(xTimerIsTimerActive(xTimerUSART2))
    	{
    		xTimerStopFromISR(xTimerUSART2, &xHigherPriorityTaskWoken);
    		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    	}


    	LL_DMA_ClearFlag_TC6(DMA1);             /* Clear transfer complete flag */
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6); //disable RX DMA channel
        LL_USART_DisableIT_IDLE(USART2);       // disable the idle interrupt of USART2


    	configASSERT( xTaskUSART2NotifyRX != NULL);
    			/* Notify the DMA task that the RX line is idle */
    			//vTaskNotifyGiveIndexedFromISR( xTaskUSART2NotifyRX, 0, &xHigherPriorityTaskWoken );

    	xTaskNotifyFromISR( xTaskUSART2NotifyRX,
    						( ulUSART2SizeRX - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6) ) << 8,
    						eSetValueWithOverwrite,
    						&xHigherPriorityTaskWoken);
    	/* There are no transmissions in progress, so no tasks to notify. */
    	xTaskUSART2NotifyRX = NULL;

    	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    					should be performed to ensure the interrupt returns directly to the highest
    					priority task.  The macro used for this purpose is dependent on the port in
    					use and may be called portEND_SWITCHING_ISR(). */
    	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

    }

    if (LL_DMA_IsEnabledIT_TE(DMA1, LL_DMA_CHANNEL_6) && LL_DMA_IsActiveFlag_TE6(DMA1))
    {
       	USART2_TransferError_Callback();
    }
#if (configUSE_TRACE_FACILITY ==1)
    vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
    /* Implement other events when needed */
}


void USART2_IRQHandler(void)
{

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(USARTHandle);
#endif

	if(LL_USART_IsEnabledIT_TC(USART2) &&  LL_USART_IsActiveFlag_TC(USART2) ) //This is for safely TX of the last byte
	{
		LL_USART_ClearFlag_TC(USART2);
		LL_USART_DisableIT_TC(USART2);
		//configASSERT( xTaskToNotify != NULL);
		configASSERT( xTaskUSART2NotifyTX != NULL);

		/* Notify the DMA task that the transmission is complete. */
		//vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );
		vTaskNotifyGiveIndexedFromISR( xTaskUSART2NotifyTX, 0, &xHigherPriorityTaskWoken );
		/* There are no transmissions in progress, so no tasks to notify. */
		//xTaskToNotify = NULL;
		xTaskUSART2NotifyTX = NULL;


		/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
			should be performed to ensure the interrupt returns directly to the highest
			priority task.  The macro used for this purpose is dependent on the port in
			use and may be called portEND_SWITCHING_ISR(). */
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	}

	if(LL_USART_IsEnabledIT_IDLE(USART2) &&  LL_USART_IsActiveFlag_IDLE(USART2) ) //This is for RX idle detection
	{

		LL_USART_DisableIT_IDLE(USART2);
		LL_USART_ClearFlag_IDLE(USART2);


		if (LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6) == 0)
		{


			LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_6);
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

			configASSERT( xTaskUSART2NotifyRX != NULL);
					/* Notify the DMA task that the RX line is idle */
			vTaskNotifyGiveIndexedFromISR( xTaskUSART2NotifyRX, 0, &xHigherPriorityTaskWoken );

			xTaskNotifyFromISR( xTaskUSART2NotifyRX,
					                        ( ulUSART2SizeRX -  LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_6) ) << 8,
											eSetValueWithOverwrite,
											&xHigherPriorityTaskWoken);
					/* There are no transmissions in progress, so no tasks to notify. */
			xTaskUSART2NotifyRX = NULL;

					/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
							should be performed to ensure the interrupt returns directly to the highest
							priority task.  The macro used for this purpose is dependent on the port in
							use and may be called portEND_SWITCHING_ISR(). */
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

		}
		else
		{

			xTimerStartFromISR( xTimerUSART2, &xHigherPriorityTaskWoken ); // start the timer
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
		}





	}
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}

/**
  * Brief   This function handles I2C2 (Master) interrupt request.
  * Param   None
  * Retval  None
  */
void I2C2_EV_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(I2CHandle);
#endif

  if(LL_I2C_IsEnabledIT_TX(I2C2) && LL_I2C_IsEnabledIT_BUF(I2C2))
  {
	  // This interrupt occurs when the last byte has been TX and DR was not written with a new byte
	  if(LL_I2C_IsActiveFlag_TXE(I2C2) && LL_I2C_IsActiveFlag_BTF(I2C2))
	  {
		  LL_I2C_DisableIT_BUF(I2C2);
		  LL_I2C_DisableIT_TX(I2C2);
		  LL_I2C_DisableIT_EVT(I2C2);

		  if(ucGenerateStop) {
			  LL_I2C_GenerateStopCondition(I2C2);
		  }

		  //Notify the end of transmission
		  //vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );

		  configASSERT( xTaskI2C2NotifyTX != NULL);

		  vTaskNotifyGiveIndexedFromISR( xTaskI2C2NotifyTX, 0, &xHigherPriorityTaskWoken );

		  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

	  }
  }
  else if(LL_I2C_IsActiveFlag_SB(I2C2))  /* Check SB flag value in ISR register */
  {
    /* Send Slave address with a 7-Bit SLAVE_OWN_ADDRESS for a ubMasterRequestDirection request */
    LL_I2C_TransmitData8(I2C2, ucSlaveOwnAddress | ucMasterRequestDirection);
  }
  /* Check ADDR flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_ADDR(I2C2))
  {
    /* Verify the transfer direction */
    if(LL_I2C_GetTransferDirection(I2C2) == LL_I2C_DIRECTION_READ)
    {

      if(usMasterNbDataToReceive == 1)
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

        /* Enable DMA transmission requests */
        LL_I2C_EnableDMAReq_RX(I2C2);
      }
      else
      {
        /* Prepare the generation of a Non ACKnowledge condition after next received byte */
        //LL_I2C_AcknowledgeNextData(I2C2, LL_I2C_NACK);

        /* Enable Pos */
        //LL_I2C_EnableBitPOS(I2C2);

        /* Enable DMA transmission requests */
        //LL_I2C_EnableDMAReq_RX(I2C2);
    	  LL_I2C_EnableLastDMA(I2C2);
    	  LL_I2C_EnableDMAReq_RX(I2C2);
      }

    }
    else
    {
      //ubMasterXferDirection = LL_I2C_DIRECTION_WRITE;

      /* Enable DMA transmission requests */
      LL_I2C_EnableDMAReq_TX(I2C2);
    }

    /* Clear ADDR flag value in ISR register */
    LL_I2C_ClearFlag_ADDR(I2C2);
  }

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}

/** I2C TX
  * @brief  This function handles DMA1_Channel4 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler(void)
{
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif

  if(LL_DMA_IsActiveFlag_TC4(DMA1))
  {
    LL_DMA_ClearFlag_GI4(DMA1);
    LL_I2C_DisableIT_EVT(I2C2);
    LL_I2C_DisableIT_ERR(I2C2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    LL_I2C_DisableDMAReq_TX(I2C2);


    // Activate the TXE and BUF interrupts to catch the events of the last bytes
    LL_I2C_EnableIT_BUF(I2C2);
   	LL_I2C_EnableIT_TX(I2C2);
   	LL_I2C_EnableIT_EVT(I2C2);

  }
  else if(LL_DMA_IsActiveFlag_TE4(DMA1))
  {
    while(1);
  }
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(0);
#endif

}

/** I2C RX
  * @brief  This function handles DMA1_Channel5 interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif

  if(LL_DMA_IsActiveFlag_TC5(DMA1))
  {
    LL_DMA_ClearFlag_GI5(DMA1);
    LL_I2C_DisableIT_EVT(I2C2);
    LL_I2C_DisableIT_ERR(I2C2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
    LL_I2C_DisableDMAReq_RX(I2C2);

    //RX event is simpler and stop condition and notification can be generated here

    LL_I2C_GenerateStopCondition(I2C2);
    //Notify the end of RX

    configASSERT(xTaskI2C2NotifyRX != NULL);

    //vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );
    vTaskNotifyGiveIndexedFromISR( xTaskI2C2NotifyRX, 0, &xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  }
  else if(LL_DMA_IsActiveFlag_TE5(DMA1))
  {
    while(1);
  }

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif

}


/**
  * Brief   This function handles I2C2 (Master) error interrupt request.
  * Param   None
  * Retval  None
  */
void I2C2_ER_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(I2CHandle);
#endif


  /* Call Error function */
  //Error_Callback();
  LL_I2C_GenerateStopCondition(I2C2);
  LL_I2C_Disable(I2C2);

  if(LL_I2C_IsEnabledDMAReq_RX(I2C2))
	  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);

  if(LL_I2C_IsEnabledDMAReq_TX(I2C2))
  	  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

  //vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif

}




// ADC1
void DMA1_Channel1_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(DMAHandle);
#endif

  if(LL_DMA_IsActiveFlag_TC1(DMA1))
  {
    //LL_DMA_ClearFlag_GI1(DMA1);
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    //vTaskNotifyGiveIndexedFromISR( xTaskToNotify, 0, &xHigherPriorityTaskWoken );
    vTaskNotifyGiveIndexedFromISR( xTaskADC1Notify, 0, &xHigherPriorityTaskWoken );
    LL_ADC_Disable(ADC1);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

  }
  else if(LL_DMA_IsActiveFlag_TE1(DMA1))
  {
    while(1);
  }

#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(xHigherPriorityTaskWoken);
#endif
}

/**
  * @brief  This function handles ADC1 interrupt request.
  * @param  None
  * @retval None
  */
void ADC1_IRQHandler(void)
{
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISRBegin(ADCHandle);
#endif

  /* Check whether ADC group regular overrun caused the ADC interruption */
  if(LL_ADC_IsActiveFlag_OVR(ADC1) != 0)
  {
    /* Clear flag ADC group regular overrun */
    LL_ADC_ClearFlag_OVR(ADC1);

    /* Call interruption treatment function */
    while(1);
  }
  if(LL_ADC_IsActiveFlag_EOCS(ADC1) != 0)
  {
	  LL_ADC_ClearFlag_EOCS(ADC1);
	  //while(1);
  }
#if (configUSE_TRACE_FACILITY ==1)
	vTraceStoreISREnd(0);
#endif
}


static void SPI1_TransferError_Callback()
{
while(1);
}


static void USART2_TransferError_Callback()
{
while(1);
}


#endif

#endif
