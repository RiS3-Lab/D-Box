/*
 * MB85RS64V.c
 *
 *  Created on: May 13, 2020
 *      Author: Alejandro Mera
 */

#include "MB85RS64V.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "container.h"

extern BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);

static  uint32_t waitNotification();





int8_t MB85R_Init(MB85RS64V_t *mb85H_v, uint32_t *pulPeripheral, uint8_t ucCSpin)
{
	if(mb85H_v == NULL || ucCSpin == 0xff ) return -1;

	mb85H_v->spiH = pulPeripheral;
	mb85H_v->ucCSpin = ucCSpin;
	//LL_GPIO_SetOutputPin((GPIO_TypeDef *) BoardGPIO[ucCSpin].ulPort, BoardGPIO[ucCSpin].ulPin); //Set the pin to disable CS
	return 0;

}



uint8_t MB85R_Read_SR(MB85RS64V_t *mb85H_v)
{

	Peripheral_Request_t xPeripheralRequest;
	uint8_t tx[2], rx[2];
	tx[0] = RDSR;
	tx[1] = DUMMY;

	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(mb85H_v->spiH,(uint8_t *) &tx, (uint8_t *) &rx, 2, 100);

	xPeripheralRequest.pulPeripheral = mb85H_v->spiH; //peripheral for operation
    xPeripheralRequest.ucOperation = eFullDuplex; // writing operation
	xPeripheralRequest.ulAddress = (uint32_t)tx; // source address
	xPeripheralRequest.ulAddressSec = (uint32_t)rx; // source address
	xPeripheralRequest.ulSize = 2;  //size of transfer
	xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
	xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;
    xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin; //CS_FRAM_; // CS option

    if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
  	{
  	    	while(1);
  	}

    waitNotification();


	return rx[1];

}


uint32_t MB85R_Read_ID(MB85RS64V_t *mb85H_v)
{

	Peripheral_Request_t xPeripheralRequest;
	uint8_t rx[5], tx[5];
	bytesFields aux;


	tx[0] = RDID;
	tx[1] = DUMMY;
	tx[2] = DUMMY;
	tx[3] = DUMMY;
	tx[4] = DUMMY;

	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(mb85H_v->spiH,(uint8_t *) &tx, (uint8_t *) &rx, 5, 100);
	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_SET);

	xPeripheralRequest.pulPeripheral =  mb85H_v->spiH; //peripheral for operation
	xPeripheralRequest.ucOperation = eFullDuplex; // writing operation
	xPeripheralRequest.ulAddress = (uint32_t)tx; // source address
	xPeripheralRequest.ulAddressSec = (uint32_t)rx; // source address
	xPeripheralRequest.ulSize = 5;  //size of transfer
	xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
	xPeripheralRequest.ulRegionNumberSec = portSTACK_REGION_NUMBER;
	xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin; // CS option

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}
	waitNotification();

	aux.u8[0] = rx[1];
	aux.u8[1] = rx[2];
	aux.u8[2] = rx[3];
	aux.u8[3] = rx[4];

	return aux.u32;

}

void MB85R_Write_EN(MB85RS64V_t *mb85H_v)
{
	Peripheral_Request_t xPeripheralRequest;
	uint8_t tx[1];
	tx[0] = WREN;

	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(mb85H_v->spiH,(uint8_t *) &tx, 1, 10);
	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_SET);
	xPeripheralRequest.pulPeripheral =  mb85H_v->spiH; //peripheral for operation
    xPeripheralRequest.ucOperation = eWrite; // writing operation
    xPeripheralRequest.ulAddress = (uint32_t)tx; // source address
    xPeripheralRequest.ulSize = 1;  //size of transfer
    xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
    xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin; // CS option

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}
	waitNotification();



}

void MB85R_WriteProtectBlock(MB85RS64V_t *mb85H_v, uint8_t block)
{
	MB85R_Write_EN(mb85H_v); //Enable Write

}

void MB85R_WriteUnProtectBlock(MB85RS64V_t *mb85H_v, uint8_t block)
{
	MB85R_Write_EN(mb85H_v); //Enable Write


}

uint32_t MB85R_Write_Data(MB85RS64V_t *mb85H_v, uint16_t address, uint8_t *data, uint8_t ucDataRegion, uint16_t size)
{
	Peripheral_Request_t xPeripheralRequest;
	uint8_t tx[3];
	bytesFields aux;

	aux.u16[0] = address;

	tx[0] = WRITE;
	tx[1] = aux.u8[1];
	tx[2] = aux.u8[0];

	MB85R_Write_EN(mb85H_v); //Enable Write

	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(mb85H_v->spiH, (uint8_t *)&tx, 3, 10);
	//HAL_SPI_Transmit(mb85H_v->spiH, data, size, 100);
	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_SET);
	//return mb85H_v->spiH->ErrorCode;

	xPeripheralRequest.pulPeripheral =  mb85H_v->spiH; //peripheral for operation
    xPeripheralRequest.ucOperation = eWrite; // writing operation
    xPeripheralRequest.ulAddress = (uint32_t)tx; // source address
    xPeripheralRequest.ulSize = 3;  //size of transfer
    xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
    xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin | OPT_CS_KEEP_ON; // CS option keep ON

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}
	waitNotification();

	xPeripheralRequest.pulPeripheral =  mb85H_v->spiH; //peripheral for operation
	xPeripheralRequest.ucOperation = eWrite; // writing operation
	xPeripheralRequest.ulAddress = (uint32_t)data; // source address
	xPeripheralRequest.ulSize = size;  //size of transfer
	xPeripheralRequest.ulRegionNumber = ucDataRegion;
	xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin;

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}
	return waitNotification();

	//return (uint32_t) ulTaskNotifyTake(pdTRUE, 10);


}

uint32_t MB85R_Read_Data(MB85RS64V_t *mb85H_v, uint16_t address, uint8_t *data, uint8_t ucDataRegion, uint16_t size)
{
	Peripheral_Request_t xPeripheralRequest;
	uint8_t tx[3];
	bytesFields aux;

	aux.u16[0] = address;

	tx[0] = READ;
	tx[1] = aux.u8[1];
	tx[2] = aux.u8[0];

	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit(mb85H_v->spiH, (uint8_t *)&tx, 3, 10);
	//HAL_SPI_Receive(mb85H_v->spiH, data, size, 100);
	//HAL_GPIO_WritePin(mb85H_v->CS_Port, mb85H_v->CS_Pin, GPIO_PIN_SET);
	//return mb85H_v->spiH->ErrorCode;

	xPeripheralRequest.pulPeripheral = mb85H_v->spiH; //peripheral for operation
	xPeripheralRequest.ucOperation = eWrite; // writing operation
	xPeripheralRequest.ulAddress = (uint32_t)tx; // destination address
	xPeripheralRequest.ulSize = 3;  //size of transfer
	xPeripheralRequest.ulRegionNumber = portSTACK_REGION_NUMBER;
	xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin | OPT_CS_KEEP_ON; // CS option keep ON

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}
	waitNotification();


	xPeripheralRequest.pulPeripheral =  mb85H_v->spiH; //peripheral for operation
	xPeripheralRequest.ucOperation = eRead; // writing operation
	xPeripheralRequest.ulAddress = (uint32_t)data; // source address
	xPeripheralRequest.ulSize = size;  //size of transfer
	xPeripheralRequest.ulRegionNumber = ucDataRegion;
	xPeripheralRequest.ulOption1 = mb85H_v->ucCSpin;

	if( MPU_xContainerSendRequest(xPeripheralRequest) == pdFREERTOS_ERRNO_EACCES  )
	{
	   	while(1);
	}

	return (uint32_t) ulTaskNotifyTake(pdTRUE, 100);

}


static uint32_t  waitNotification()
{

	uint32_t xNotificationValue;

	xNotificationValue = ulTaskNotifyTake(pdTRUE, 100);
	if( ( xNotificationValue & 0xff )== pdFREERTOS_ERRNO_ETIMEDOUT)
	{
			//error
	   while(1);
	}
	return xNotificationValue;

}
