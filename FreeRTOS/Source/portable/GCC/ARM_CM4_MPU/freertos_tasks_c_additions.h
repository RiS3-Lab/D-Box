/*
 * freertos_tasks_c_additions.h
 *
 *  Created on: Dec 8, 2020
 *      Author: alejandro
 */

#ifndef FREERTOS_PORTABLE_GCC_ARM_CM3_MPU_FREERTOS_TASKS_C_ADDITIONS_H_
#define FREERTOS_PORTABLE_GCC_ARM_CM3_MPU_FREERTOS_TASKS_C_ADDITIONS_H_

#ifndef INC_FREERTOS_H
    #error "include FreeRTOS.h must appear in source files before include task.h"
#endif



 uint32_t *pTaskAdditionsGetPreviousTCBControl( void );
 uint32_t *pTaskAdditionsGetPreviousTCBControl( void )
 {
     if (pxCurrentTCB == NULL)
     {
    	 return NULL;
     }
     else
     {
    	 return (uint32_t*) &(pxCurrentTCB->ulPreviousControl);
     }
 }



 BaseType_t xTaskAdditionsIsTCBValid(void);
 BaseType_t xTaskAdditionsIsTCBValid(void)
 {
     if (pxCurrentTCB == NULL)
     {
    	 return pdFALSE;
     }
     else
     {
    	 return pdTRUE;
     }

 }



 PeripheralPermission_t * xTaskAdditionsGetPermissions(void);
 PeripheralPermission_t * xTaskAdditionsGetPermissions(void)
 {
     if (pxCurrentTCB )
     {
    	 return pxCurrentTCB->pxPeripheralPermissions;
     }
     else
     {
    	 return NULL;
     }

 }


 xMPU_REGION_REGISTERS * xTaskAdditionsGetMemoryRegion(BaseType_t xRegionNumber);
 xMPU_REGION_REGISTERS * xTaskAdditionsGetMemoryRegion(BaseType_t xRegionNumber)
 {
     if (pxCurrentTCB)
     {
    	 if(pxCurrentTCB->xMPUSettings.xRegion)
    	 {
    		 return ( xMPU_REGION_REGISTERS *) &(pxCurrentTCB->xMPUSettings.xRegion[xRegionNumber]);
    	 }

     }

     return NULL;

 }


#endif /* FREERTOS_PORTABLE_GCC_ARM_CM3_MPU_FREERTOS_TASKS_C_ADDITIONS_H_ */
