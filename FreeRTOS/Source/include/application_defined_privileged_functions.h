/*
 * application_defined_privileged_functions.h
 *
 *  Created on: Dec 12, 2020
 *      Author: alejandro
 */

#ifndef FREERTOS_INCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS_H_
#define FREERTOS_INCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS_H_

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h must appear in source files before include task.h"
#endif

#include "container.h"


/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */

   #if (configUSE_DMA_CONTAINER == 1 )
    BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest);
    BaseType_t MPU_xContainerSendRequest(Peripheral_Request_t xRequest)
    {
    	BaseType_t xReturn;
    	BaseType_t xRunningPrivileged = xPortRaisePrivilege();

    	xReturn =  xContainerSendRequest(xRequest, xRunningPrivileged);

    	vPortResetPrivilege( xRunningPrivileged);
    	return xReturn;
    }

    #endif
    void MPU_xContainerSetDWTcounter(uint32_t val);
    void MPU_xContainerSetDWTcounter(uint32_t val)
    {
        	BaseType_t xRunningPrivileged = xPortRaisePrivilege();
            DWT->CYCCNT = val;
        	vPortResetPrivilege( xRunningPrivileged);
    }

    uint32_t MPU_xContainerGetDWTcounter();
    uint32_t MPU_xContainerGetDWTcounter()
    {
    	    uint32_t xReturn;
    	    BaseType_t xRunningPrivileged = xPortRaisePrivilege();

           	xReturn = DWT->CYCCNT;

           	vPortResetPrivilege( xRunningPrivileged);
           	return xReturn;

    }

    void MPU_xContainerEnableDWTcounter();
    void MPU_xContainerEnableDWTcounter()
    {
    	BaseType_t xRunningPrivileged = xPortRaisePrivilege();
    	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
       	vPortResetPrivilege( xRunningPrivileged);
    }



/* *INDENT-OFF* */
#ifdef __cplusplus
        }
#endif
/* *INDENT-ON* */


#endif /* FREERTOS_INCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS_H_ */
