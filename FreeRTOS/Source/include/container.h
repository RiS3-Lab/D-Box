/*
 * container.h
 *
 *  Created on: Dec 8, 2020
 *      Author: Alejandro Mera
 */

#ifndef FREERTOS_INCLUDE_CONTAINER_H_
#define FREERTOS_INCLUDE_CONTAINER_H_

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h must appear in source files before include task.h"
#endif

#include "queue.h"

/* *INDENT-OFF* */
#ifdef __cplusplus
    extern "C" {
#endif
/* *INDENT-ON* */


#if (configUSE_DMA_CONTAINER == 1)
extern uint32_t ulNumberRestrictedTask;
extern StackRegion_t xSTackRegions[8];



BaseType_t xContainerSendRequest(Peripheral_Request_t xRequest, BaseType_t xCallerIsPrivileged) PRIVILEGED_FUNCTION;


void vStartDMATask( void );

#endif
/* *INDENT-OFF* */
#ifdef __cplusplus
    }
#endif
/* *INDENT-ON* */

#endif /* FREERTOS_INCLUDE_CONTAINER_H_ */
