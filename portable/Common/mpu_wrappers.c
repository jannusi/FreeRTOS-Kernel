/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
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
 */

/*
 * Implementation of the wrapper functions used to raise the processor privilege
 * before calling a standard FreeRTOS API function.
 */

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "stream_buffer.h"
#include "mpu_prototypes.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE
/*-----------------------------------------------------------*/

#if ( portUSING_MPU_WRAPPERS == 1 )

    #ifndef configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE
        #error configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE must be defined to maximum number of kernel objects in the application.
    #endif

    /**
     * @brief Offset added to the index before returning to the user.
     *
     * If the actual handle is stored at index i, ( i + INDEX_OFFSET )
     * is returned to the user.
     */
    #define INDEX_OFFSET    1

    /**
     * @brief Opaque type used to define kernel object handle pool.
     */
    struct KernelObject;
    typedef struct KernelObject * KernelObjectHandle_t;

    /**
     * @brief Get the index of a free slot in the kernel object handle pool.
     *
     * @return Index of a free slot is returned, if a free slot is
     * found. Otherwise -1 is returned.
     */
    static int32_t MPU_GetFreeIndexInHandlePool( void ) PRIVILEGED_FUNCTION;

    /**
     * @brief Set the given index as free in the kernel object handle pool.
     *
     * @param lIndex The index to set as free.
     */
    static void MPU_SetIndexFreeInHandlePool( int32_t lIndex ) PRIVILEGED_FUNCTION;

    #if ( configUSE_QUEUE_SETS == 1 )

        /**
         * @brief Get the index at which a given kernel object handle is stored.
         *
         * @param xHandle The given kernel object handle.
         *
         * @return Index at which the kernel object is stored if it is a valid
         *         handle, -1 otherwise.
         */
        static int32_t MPU_GetIndexForHandle( KernelObjectHandle_t xHandle ) PRIVILEGED_FUNCTION;
    #endif

    /**
     * @brief Store the given kernel object handle at the given index in the handle pool.
     *
     * @param lIndex Index to store the given handle at.
     * @param xHandle Kernel object handle to store.
     */
    static void MPU_StoreHandleAtIndex( int32_t lIndex,
                                        KernelObjectHandle_t xHandle ) PRIVILEGED_FUNCTION;

    /**
     * @brief Get the kernel object handle at the given index from the handle pool.
     *
     * @param lIndex Index at which to get the kernel object handle.
     *
     * @return The kernel object handle at the index.
     */
    static KernelObjectHandle_t MPU_GetHandleAtIndex( int32_t lIndex ) PRIVILEGED_FUNCTION;

    /*
     * Wrappers to keep all the casting in one place.
     */
    #define MPU_StoreQueueHandleAtIndex( lIndex, xHandle )                 MPU_StoreHandleAtIndex( lIndex, ( KernelObjectHandle_t ) xHandle )
    #define MPU_GetQueueHandleAtIndex( lIndex )                            ( QueueHandle_t ) MPU_GetHandleAtIndex( lIndex )

    #if ( configUSE_QUEUE_SETS == 1 )
        #define MPU_StoreQueueSetHandleAtIndex( lIndex, xHandle )          MPU_StoreHandleAtIndex( lIndex, ( KernelObjectHandle_t ) xHandle )
        #define MPU_GetQueueSetHandleAtIndex( lIndex )                     ( QueueSetHandle_t ) MPU_GetHandleAtIndex( lIndex )
        #define MPU_StoreQueueSetMemberHandleAtIndex( lIndex, xHandle )    MPU_StoreHandleAtIndex( lIndex, ( KernelObjectHandle_t ) xHandle )
        #define MPU_GetQueueSetMemberHandleAtIndex( lIndex )               ( QueueSetHandle_t ) MPU_GetHandleAtIndex( lIndex )
        #define MPU_GetIndexForQueueSetMemberHandle( xHandle )             MPU_GetIndexForHandle( ( KernelObjectHandle_t ) xHandle )
    #endif

    /*
     * Wrappers to keep all the casting in one place for Task APIs.
     */
    #define MPU_StoreTaskHandleAtIndex( lIndex, xHandle )    MPU_StoreHandleAtIndex( lIndex, ( KernelObjectHandle_t ) xHandle )
    #define MPU_GetTaskHandleAtIndex( lIndex )               ( TaskHandle_t ) MPU_GetHandleAtIndex( lIndex )
    #define MPU_GetIndexForTaskHandle( xHandle )             MPU_GetIndexForHandle( ( KernelObjectHandle_t ) xHandle )

    /*
     * Wrappers to keep all the casting in one place for Event Group APIs.
     */
    #define MPU_StoreEventGroupHandleAtIndex( lIndex, xHandle )    MPU_StoreHandleAtIndex( lIndex, ( KernelObjectHandle_t ) xHandle )
    #define MPU_GetEventGroupHandleAtIndex( lIndex )               ( EventGroupHandle_t ) MPU_GetHandleAtIndex( lIndex )
    #define MPU_GetIndexForEventGroupHandle( xHandle )             MPU_GetIndexForHandle( ( KernelObjectHandle_t ) xHandle )

/*-----------------------------------------------------------*/

    /**
     * @brief Kernel object handle pool.
     */
    PRIVILEGED_DATA static KernelObjectHandle_t xKernelObjectHandlePool[ configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE ] = { NULL };
/*-----------------------------------------------------------*/

    static int32_t MPU_GetFreeIndexInHandlePool( void ) /* PRIVILEGED_FUNCTION */
    {
        int32_t i, lFreeIndex = -1;

        /* This function is called only from resource create APIs
         * which are not supposed to be called from ISRs. Therefore,
         * we only need to suspend the scheduler and do not require
         * critical section. */
        vTaskSuspendAll();
        {
            for( i = 0; i < configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE; i++ )
            {
                if( xKernelObjectHandlePool[ i ] == NULL )
                {
                    /* Mark this index as not free. */
                    xKernelObjectHandlePool[ i ] = ( KernelObjectHandle_t ) ( ~0 );
                    lFreeIndex = i;
                    break;
                }
            }
        }
        xTaskResumeAll();

        return lFreeIndex;
    }
/*-----------------------------------------------------------*/

    static void MPU_SetIndexFreeInHandlePool( int32_t lIndex ) /* PRIVILEGED_FUNCTION */
    {
        configASSERT( ( lIndex >= 0 ) && ( lIndex < configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE ) );

        taskENTER_CRITICAL();
        {
            xKernelObjectHandlePool[ lIndex ] = NULL;
        }
        taskEXIT_CRITICAL();
    }
/*-----------------------------------------------------------*/

    #if ( configUSE_QUEUE_SETS == 1 )

        static int32_t MPU_GetIndexForHandle( KernelObjectHandle_t xHandle ) /* PRIVILEGED_FUNCTION */
        {
            int32_t i, lIndex = -1;

            configASSERT( xHandle != NULL );

            for( i = 0; i < configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE; i++ )
            {
                if( xKernelObjectHandlePool[ i ] == xHandle )
                {
                    lIndex = i;
                    break;
                }
            }

            return lIndex;
        }

    #endif /* #if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

    static void MPU_StoreHandleAtIndex( int32_t lIndex,
                                        KernelObjectHandle_t xHandle )                 /* PRIVILEGED_FUNCTION */
    {
        configASSERT( ( lIndex >= 0 ) && ( lIndex < configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE ) );
        xKernelObjectHandlePool[ lIndex ] = xHandle;
    }
/*-----------------------------------------------------------*/

    static KernelObjectHandle_t MPU_GetHandleAtIndex( int32_t lIndex ) /* PRIVILEGED_FUNCTION */
    {
        configASSERT( ( lIndex >= 0 ) && ( lIndex < configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE ) );
        return xKernelObjectHandlePool[ lIndex ];
    }
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        BaseType_t MPU_xTaskCreate( TaskFunction_t pvTaskCode,
                                    const char * const pcName,
                                    uint16_t usStackDepth,
                                    void * pvParameters,
                                    UBaseType_t uxPriority,
                                    TaskHandle_t * pxCreatedTask ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                uxPriority = uxPriority & ~( portPRIVILEGE_BIT );
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xReturn = xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, &xInternalTaskHandle );

                    if( xInternalTaskHandle != NULL )
                    {
                        MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                        *pxCreatedTask = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xReturn = xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, &xInternalTaskHandle );

                    if( xInternalTaskHandle != NULL )
                    {
                        MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                        *pxCreatedTask = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xReturn;
        }
    #endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        TaskHandle_t MPU_xTaskCreateStatic( TaskFunction_t pxTaskCode,
                                            const char * const pcName,
                                            const uint32_t ulStackDepth,
                                            void * const pvParameters,
                                            UBaseType_t uxPriority,
                                            StackType_t * const puxStackBuffer,
                                            StaticTask_t * const pxTaskBuffer ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHandle_t xExternalTaskHandle = NULL;
            TaskHandle_t xInternalTaskHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                uxPriority = uxPriority & ~( portPRIVILEGE_BIT );
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalTaskHandle = xTaskCreateStatic( pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, puxStackBuffer, pxTaskBuffer );

                    if( xInternalTaskHandle != NULL )
                    {
                        MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                        xExternalTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalTaskHandle = xTaskCreateStatic( pxTaskCode, pcName, ulStackDepth, pvParameters, uxPriority, puxStackBuffer, pxTaskBuffer );

                    if( xInternalTaskHandle != NULL )
                    {
                        MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                        xExternalTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalTaskHandle;
        }
    #endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )

        BaseType_t MPU_xTaskCreateRestricted( const TaskParameters_t * const pxTaskDefinition,
                                              TaskHandle_t * pxCreatedTask ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;


            lIndex = MPU_GetFreeIndexInHandlePool();

            if( lIndex != -1 )
            {
                xReturn = xTaskCreateRestricted( pxTaskDefinition, &xInternalTaskHandle );

                if( xInternalTaskHandle != NULL )
                {
                    MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                    *pxCreatedTask = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                }
                else
                {
                    MPU_SetIndexFreeInHandlePool( lIndex );
                }
            }

            return xReturn;
        }
    #endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        BaseType_t MPU_xTaskCreateRestrictedStatic( const TaskParameters_t * const pxTaskDefinition,
                                                    TaskHandle_t * pxCreatedTask )  /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;


            lIndex = MPU_GetFreeIndexInHandlePool();

            if( lIndex != -1 )
            {
                xReturn = xTaskCreateRestrictedStatic( pxTaskDefinition, &xInternalTaskHandle );

                if( xInternalTaskHandle != NULL )
                {
                    MPU_StoreTaskHandleAtIndex( lIndex, xInternalTaskHandle );
                    *pxCreatedTask = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                }
                else
                {
                    MPU_SetIndexFreeInHandlePool( lIndex );
                }
            }

            return xReturn;
        }
    #endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/

    void MPU_vTaskAllocateMPURegions( TaskHandle_t xTaskToModify,
                                      const MemoryRegion_t * const xRegions ) /* FREERTOS_SYSTEM_CALL */
    {
        if( xTaskToModify != NULL )
        {
            TaskHandle_t xInternalTaskHandle = NULL;
            int32_t lIndex;

            lIndex = ( int32_t ) xTaskToModify;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                vTaskAllocateMPURegions( xInternalTaskHandle, xRegions );
            }
        }
        else
        {
            vTaskAllocateMPURegions( xTaskToModify, xRegions );
        }
    }
/*-----------------------------------------------------------*/

    #if ( INCLUDE_vTaskDelete == 1 )
        void MPU_vTaskDelete( TaskHandle_t pxTaskToDelete ) /* FREERTOS_SYSTEM_CALL */
        {
            if( pxTaskToDelete != NULL )
            {
                TaskHandle_t xInternalTaskHandle = NULL;
                int32_t lIndex;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTaskToDelete;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskDelete( xInternalTaskHandle );
                        MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTaskToDelete;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskDelete( xInternalTaskHandle );
                        MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskDelete( pxTaskToDelete );
                    portMEMORY_BARRIER();

                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskDelete( pxTaskToDelete );
                }
            }
        }
    #endif /* #if ( INCLUDE_vTaskDelete == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_xTaskDelayUntil == 1 )
        BaseType_t MPU_xTaskDelayUntil( TickType_t * const pxPreviousWakeTime,
                                        TickType_t xTimeIncrement ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTaskDelayUntil( pxPreviousWakeTime, xTimeIncrement );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTaskDelayUntil( pxPreviousWakeTime, xTimeIncrement );
            }

            return xReturn;
        }
    #endif /* if ( INCLUDE_xTaskDelayUntil == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_xTaskAbortDelay == 1 )
        BaseType_t MPU_xTaskAbortDelay( TaskHandle_t xTask ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( xTask != NULL )
            {
                TaskHandle_t xInternalTaskHandle = NULL;
                int32_t lIndex;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskAbortDelay( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskAbortDelay( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xTaskAbortDelay( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xTaskAbortDelay( xTask );
                }
            }

            return xReturn;
        }
    #endif /* if ( INCLUDE_xTaskAbortDelay == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_vTaskDelay == 1 )
        void MPU_vTaskDelay( TickType_t xTicksToDelay ) /* FREERTOS_SYSTEM_CALL */
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                vTaskDelay( xTicksToDelay );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                vTaskDelay( xTicksToDelay );
            }
        }
    #endif /* if ( INCLUDE_vTaskDelay == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_uxTaskPriorityGet == 1 )
        UBaseType_t MPU_uxTaskPriorityGet( const TaskHandle_t pxTask ) /* FREERTOS_SYSTEM_CALL */
        {
            UBaseType_t uxReturn;

            if( pxTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskPriorityGet( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskPriorityGet( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    uxReturn = uxTaskPriorityGet( pxTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    uxReturn = uxTaskPriorityGet( pxTask );
                }
            }

            return uxReturn;
        }
    #endif /* if ( INCLUDE_uxTaskPriorityGet == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_vTaskPrioritySet == 1 )
        void MPU_vTaskPrioritySet( TaskHandle_t pxTask,
                                   UBaseType_t uxNewPriority ) /* FREERTOS_SYSTEM_CALL */
        {
            if( pxTask != NULL )
            {
                TaskHandle_t xInternalTaskHandle = NULL;
                int32_t lIndex;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskPrioritySet( xInternalTaskHandle, uxNewPriority );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskPrioritySet( xInternalTaskHandle, uxNewPriority );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskPrioritySet( pxTask, uxNewPriority );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskPrioritySet( pxTask, uxNewPriority );
                }
            }
        }
    #endif /* if ( INCLUDE_vTaskPrioritySet == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_eTaskGetState == 1 )
        eTaskState MPU_eTaskGetState( TaskHandle_t pxTask ) /* FREERTOS_SYSTEM_CALL */
        {
            eTaskState eReturn = eInvalid;

            if( pxTask != NULL )
            {
                TaskHandle_t xInternalTaskHandle = NULL;
                int32_t lIndex;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        eReturn = eTaskGetState( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        eReturn = eTaskGetState( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    eReturn = eTaskGetState( pxTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    eReturn = eTaskGetState( pxTask );
                }
            }

            return eReturn;
        }
    #endif /* if ( INCLUDE_eTaskGetState == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TRACE_FACILITY == 1 )
        void MPU_vTaskGetInfo( TaskHandle_t xTask,
                               TaskStatus_t * pxTaskStatus,
                               BaseType_t xGetFreeStackSpace,
                               eTaskState eState ) /* FREERTOS_SYSTEM_CALL */
        {
            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskGetInfo( xInternalTaskHandle, pxTaskStatus, xGetFreeStackSpace, eState );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskGetInfo( xInternalTaskHandle, pxTaskStatus, xGetFreeStackSpace, eState );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskGetInfo( xTask, pxTaskStatus, xGetFreeStackSpace, eState );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskGetInfo( xTask, pxTaskStatus, xGetFreeStackSpace, eState );
                }
            }
        }
    #endif /* if ( configUSE_TRACE_FACILITY == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_xTaskGetIdleTaskHandle == 1 )
        TaskHandle_t MPU_xTaskGetIdleTaskHandle( void ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHandle_t xInternalTaskHandle = NULL;
            TaskHandle_t xSelectedIdleTaskHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xInternalTaskHandle = xTaskGetIdleTaskHandle();

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xSelectedIdleTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xInternalTaskHandle = xTaskGetIdleTaskHandle();
                lIndex = ( int32_t ) xInternalTaskHandle;

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xSelectedIdleTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                }
            }

            return xSelectedIdleTaskHandle;
        }
    #endif /* if ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_vTaskSuspend == 1 )
        void MPU_vTaskSuspend( TaskHandle_t pxTaskToSuspend ) /* FREERTOS_SYSTEM_CALL */
        {
            if( pxTaskToSuspend != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTaskToSuspend;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSuspend( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTaskToSuspend;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSuspend( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskSuspend( pxTaskToSuspend );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskSuspend( pxTaskToSuspend );
                }
            }
        }
    #endif /* if ( INCLUDE_vTaskSuspend == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_vTaskSuspend == 1 )
        void MPU_vTaskResume( TaskHandle_t pxTaskToResume ) /* FREERTOS_SYSTEM_CALL */
        {
            if( pxTaskToResume != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) pxTaskToResume;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskResume( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) pxTaskToResume;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskResume( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskResume( pxTaskToResume );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskResume( pxTaskToResume );
                }
            }
        }
    #endif /* if ( INCLUDE_vTaskSuspend == 1 ) */
/*-----------------------------------------------------------*/

    void MPU_vTaskSuspendAll( void ) /* FREERTOS_SYSTEM_CALL */
    {
        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            vTaskSuspendAll();
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            vTaskSuspendAll();
        }
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xTaskResumeAll( void ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xTaskResumeAll();
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xTaskResumeAll();
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    TickType_t MPU_xTaskGetTickCount( void ) /* FREERTOS_SYSTEM_CALL */
    {
        TickType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xTaskGetTickCount();
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xTaskGetTickCount();
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    UBaseType_t MPU_uxTaskGetNumberOfTasks( void ) /* FREERTOS_SYSTEM_CALL */
    {
        UBaseType_t uxReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            uxReturn = uxTaskGetNumberOfTasks();
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            uxReturn = uxTaskGetNumberOfTasks();
        }

        return uxReturn;
    }
/*-----------------------------------------------------------*/

    char * MPU_pcTaskGetName( TaskHandle_t xTaskToQuery ) /* FREERTOS_SYSTEM_CALL */
    {
        char * pcReturn;

        if( xTaskToQuery != NULL )
        {
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xTaskToQuery;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    pcReturn = pcTaskGetName( xInternalTaskHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xTaskToQuery;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    pcReturn = pcTaskGetName( xInternalTaskHandle );
                }
            }
        }
        else
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                pcReturn = pcTaskGetName( xTaskToQuery );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                pcReturn = pcTaskGetName( xTaskToQuery );
            }
        }

        return pcReturn;
    }
/*-----------------------------------------------------------*/

    #if ( INCLUDE_xTaskGetHandle == 1 )
        TaskHandle_t MPU_xTaskGetHandle( const char * pcNameToQuery ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHandle_t xInternalTaskHandle = NULL;
            TaskHandle_t xSelectedTaskHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xInternalTaskHandle = xTaskGetHandle( pcNameToQuery );

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xSelectedTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xInternalTaskHandle = xTaskGetHandle( pcNameToQuery );

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xSelectedTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                }
            }

            return xSelectedTaskHandle;
        }
    #endif /* if ( INCLUDE_xTaskGetHandle == 1 ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        void MPU_vTaskList( char * pcWriteBuffer ) /* FREERTOS_SYSTEM_CALL */
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                vTaskList( pcWriteBuffer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                vTaskList( pcWriteBuffer );
            }
        }
    #endif /* if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        void MPU_vTaskGetRunTimeStats( char * pcWriteBuffer ) /* FREERTOS_SYSTEM_CALL */
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                vTaskGetRunTimeStats( pcWriteBuffer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                vTaskGetRunTimeStats( pcWriteBuffer );
            }
        }
    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/
    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimeCounter( const TaskHandle_t xTask ) /*FREERTOS_SYSTEM_CALL */
        {
            configRUN_TIME_COUNTER_TYPE xReturn;

            if( xTask != NULL )
            {
                int32_t lindex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = ulTaskGetRunTimeCounter( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = ulTaskGetRunTimeCounter( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = ulTaskGetRunTimeCounter( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = ulTaskGetRunTimeCounter( xTask );
                }
            }

            return xReturn;
        }
    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetRunTimePercent( const TaskHandle_t xTask ) /*FREERTOS_SYSTEM_CALL */
        {
            configRUN_TIME_COUNTER_TYPE xReturn;

            if( xTask != NULL )
            {
                int32_t lindex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = ulTaskGetRunTimePercent( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = ulTaskGetRunTimePercent( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = ulTaskGetRunTimePercent( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = ulTaskGetRunTimePercent( xTask );
                }
            }

            return xReturn;
        }
    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) )
        configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimePercent( void ) /* FREERTOS_SYSTEM_CALL */
        {
            configRUN_TIME_COUNTER_TYPE xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = ulTaskGetIdleRunTimePercent();
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = ulTaskGetIdleRunTimePercent();
            }

            return xReturn;
        }
    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) )
        configRUN_TIME_COUNTER_TYPE MPU_ulTaskGetIdleRunTimeCounter( void ) /* FREERTOS_SYSTEM_CALL */
        {
            configRUN_TIME_COUNTER_TYPE xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = ulTaskGetIdleRunTimeCounter();
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = ulTaskGetIdleRunTimeCounter();
            }

            return xReturn;
        }
    #endif /* if ( ( configGENERATE_RUN_TIME_STATS == 1 ) && ( INCLUDE_xTaskGetIdleTaskHandle == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        void MPU_vTaskSetApplicationTaskTag( TaskHandle_t xTask,
                                             TaskHookFunction_t pxTagValue ) /* FREERTOS_SYSTEM_CALL */
        {
            if( xTask != NULL )
            {
                TaskHandle_t xInternalTaskHandle = NULL;
                int32_t lIndex;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSetApplicationTaskTag( xInternalTaskHandle, pxTagValue );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSetApplicationTaskTag( xInternalTaskHandle, pxTagValue );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskSetApplicationTaskTag( xTask, pxTagValue );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskSetApplicationTaskTag( xTask, pxTagValue );
                }
            }
        }
    #endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        TaskHookFunction_t MPU_xTaskGetApplicationTaskTag( TaskHandle_t xTask ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHookFunction_t xReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGetApplicationTaskTag( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGetApplicationTaskTag( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xTaskGetApplicationTaskTag( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xTaskGetApplicationTaskTag( xTask );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )
        void MPU_vTaskSetThreadLocalStoragePointer( TaskHandle_t xTaskToSet,
                                                    BaseType_t xIndex,
                                                    void * pvValue ) /* FREERTOS_SYSTEM_CALL */
        {
            if( xTaskToSet != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTaskToSet;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSetThreadLocalStoragePointer( xInternalTaskHandle, xIndex, pvValue );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTaskToSet;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        vTaskSetThreadLocalStoragePointer( xInternalTaskHandle, xIndex, pvValue );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    vTaskSetThreadLocalStoragePointer( xTaskToSet, xIndex, pvValue );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    vTaskSetThreadLocalStoragePointer( xTaskToSet, xIndex, pvValue );
                }
            }
        }
    #endif /* if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 ) */
/*-----------------------------------------------------------*/

    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 )
        void * MPU_pvTaskGetThreadLocalStoragePointer( TaskHandle_t xTaskToQuery,
                                                       BaseType_t xIndex ) /* FREERTOS_SYSTEM_CALL */
        {
            void * pvReturn;

            if( xTaskToQuery != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTaskToQuery;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        pvReturn = pvTaskGetThreadLocalStoragePointer( xInternalTaskHandle, xIndex );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTaskToQuery;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        pvReturn = pvTaskGetThreadLocalStoragePointer( xInternalTaskHandle, xIndex );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    pvReturn = pvTaskSetThreadLocalStoragePointer( xTaskToSet, xIndex );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    pvReturn = pvTaskSetThreadLocalStoragePointer( xTaskToSet, xIndex );
                }
            }

            return pvReturn;
        }
    #endif /* if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS != 0 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        BaseType_t MPU_xTaskCallApplicationTaskHook( TaskHandle_t xTask,
                                                     void * pvParameter ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskCallApplicationTaskHook( xInternalTaskHandle, pvParameter );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskCallApplicationTaskHook( xInternalTaskHandle, pvParameter );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xTaskCallApplicationTaskHook( xTask, pvParameter );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xTaskCallApplicationTaskHook( xTask, pvParameter );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_APPLICATION_TASK_TAG == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t MPU_uxTaskGetSystemState( TaskStatus_t * pxTaskStatusArray,
                                              UBaseType_t uxArraySize,
                                              configRUN_TIME_COUNTER_TYPE * pulTotalRunTime ) /* FREERTOS_SYSTEM_CALL */
        {
            UBaseType_t uxReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                uxReturn = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, pulTotalRunTime );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                uxReturn = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, pulTotalRunTime );
            }

            return uxReturn;
        }
    #endif /* if ( configUSE_TRACE_FACILITY == 1 ) */
/*-----------------------------------------------------------*/

    BaseType_t MPU_xTaskCatchUpTicks( TickType_t xTicksToCatchUp ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xTaskCatchUpTicks( xTicksToCatchUp );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xTaskCatchUpTicks( xTicksToCatchUp );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    #if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 )
        UBaseType_t MPU_uxTaskGetStackHighWaterMark( TaskHandle_t xTask ) /* FREERTOS_SYSTEM_CALL */
        {
            UBaseType_t uxReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskGetStackHighWaterMark( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskGetStackHighWaterMark( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    uxReturn = uxTaskGetStackHighWaterMark( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    uxReturn = uxTaskGetStackHighWaterMark( xTask );
                }
            }

            return uxReturn;
        }
    #endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark == 1 ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 )
        configSTACK_DEPTH_TYPE MPU_uxTaskGetStackHighWaterMark2( TaskHandle_t xTask ) /* FREERTOS_SYSTEM_CALL */
        {
            configSTACK_DEPTH_TYPE uxReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskGetStackHighWaterMark2( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskGetStackHighWaterMark2( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    uxReturn = uxTaskGetStackHighWaterMark2( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    uxReturn = uxTaskGetStackHighWaterMark2( xTask );
                }
            }

            return uxReturn;
        }
    #endif /* if ( INCLUDE_uxTaskGetStackHighWaterMark2 == 1 ) */
/*-----------------------------------------------------------*/

    #if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) )
        TaskHandle_t MPU_xTaskGetCurrentTaskHandle( void ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHandle_t xInternalTaskHandle = NULL;
            TaskHandle_t xCurrentTaskHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xInternalTaskHandle = xTaskGetCurrentTaskHandle();

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xCurrentTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xInternalTaskHandle = xTaskGetCurrentTaskHandle();

                if( xInternalTaskHandle != NULL )
                {
                    lIndex = MPU_GetIndexForTaskHandle( xInternalTaskHandle );
                    xCurrentTaskHandle = ( TaskHandle_t ) ( lIndex + INDEX_OFFSET );
                }
            }

            return xCurrentTaskHandle;
        }
    #endif /* if ( ( INCLUDE_xTaskGetCurrentTaskHandle == 1 ) || ( configUSE_MUTEXES == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( INCLUDE_xTaskGetSchedulerState == 1 )
        BaseType_t MPU_xTaskGetSchedulerState( void ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTaskGetSchedulerState();
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTaskGetSchedulerState();
            }

            return xReturn;
        }
    #endif /* if ( INCLUDE_xTaskGetSchedulerState == 1 ) */
/*-----------------------------------------------------------*/

    void MPU_vTaskSetTimeOutState( TimeOut_t * const pxTimeOut ) /* FREERTOS_SYSTEM_CALL */
    {
        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            vTaskSetTimeOutState( pxTimeOut );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            vTaskSetTimeOutState( pxTimeOut );
        }
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xTaskCheckForTimeOut( TimeOut_t * const pxTimeOut,
                                         TickType_t * const pxTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xTaskCheckForTimeOut( pxTimeOut, pxTicksToWait );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xTaskCheckForTimeOut( pxTimeOut, pxTicksToWait );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        BaseType_t MPU_xTaskGenericNotify( TaskHandle_t xTaskToNotify,
                                           UBaseType_t uxIndexToNotify,
                                           uint32_t ulValue,
                                           eNotifyAction eAction,
                                           uint32_t * pulPreviousNotificationValue ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;


            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskGenericNotify( xInternalTaskHandle, uxIndexToNotify, ulValue, eAction, pulPreviousNotificationValue );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskGenericNotify( xInternalTaskHandle, uxIndexToNotify, ulValue, eAction, pulPreviousNotificationValue );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        BaseType_t MPU_xTaskGenericNotifyWait( UBaseType_t uxIndexToWaitOn,
                                               uint32_t ulBitsToClearOnEntry,
                                               uint32_t ulBitsToClearOnExit,
                                               uint32_t * pulNotificationValue,
                                               TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTaskGenericNotifyWait( uxIndexToWaitOn, ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTaskGenericNotifyWait( uxIndexToWaitOn, ulBitsToClearOnEntry, ulBitsToClearOnExit, pulNotificationValue, xTicksToWait );
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        uint32_t MPU_ulTaskGenericNotifyTake( UBaseType_t uxIndexToWaitOn,
                                              BaseType_t xClearCountOnExit,
                                              TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
        {
            uint32_t ulReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                ulReturn = ulTaskGenericNotifyTake( uxIndexToWaitOn, xClearCountOnExit, xTicksToWait );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                ulReturn = ulTaskGenericNotifyTake( uxIndexToWaitOn, xClearCountOnExit, xTicksToWait );
            }

            return ulReturn;
        }
    #endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        BaseType_t MPU_xTaskGenericNotifyStateClear( TaskHandle_t xTask,
                                                     UBaseType_t uxIndexToClear ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGenericNotifyStateClear( xInternalTaskHandle, uxIndexToClear );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGenericNotifyStateClear( xInternalTaskHandle, uxIndexToClear );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xTaskGenericNotifyStateClear( xTask, uxIndexToClear );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xTaskGenericNotifyStateClear( xTask, uxIndexToClear );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        uint32_t MPU_ulTaskGenericNotifyValueClear( TaskHandle_t xTask,
                                                    UBaseType_t uxIndexToClear,
                                                    uint32_t ulBitsToClear ) /* FREERTOS_SYSTEM_CALL */
        {
            uint32_t ulReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        ulReturn = ulTaskGenericNotifyValueClear( xInternalTaskHandle, uxIndexToClear, ulBitsToClear );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        ulReturn = ulTaskGenericNotifyValueClear( xInternalTaskHandle, uxIndexToClear, ulBitsToClear );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    ulReturn = ulTaskGenericNotifyValueClear( xTask, uxIndexToClear, ulBitsToClear );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    ulReturn = ulTaskGenericNotifyValueClear( xTask, uxIndexToClear, ulBitsToClear );
                }
            }

            return ulReturn;
        }
    #endif /* if ( configUSE_TASK_NOTIFICATIONS == 1 ) */
/*-----------------------------------------------------------*/

/* Privileged only wrappers for Task APIs. These are needed so that
 * the application can use opaque handles maintained in mpu_wrappers.c
 * with all the APIs. */
/*-----------------------------------------------------------*/
    #if ( INCLUDE_uxTaskPriorityGet == 1 )
        UBaseType_t MPU_uxTaskPriorityGetFromISR( const TaskHandle_t xTask ) /* PRIVILEGED_FUNCTION */
        {
            UBaseType_t uxReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskPriorityGetFromISR( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        uxReturn = uxTaskPriorityGetFromISR( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    uxReturn = uxTaskPriorityGetFromISR( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    uxReturn = uxTaskPriorityGetFromISR( xTask );
                }
            }

            return uxReturn;
        }
    #endif /* #if ( INCLUDE_uxTaskPriorityGet == 1 ) */
/*-----------------------------------------------------------*/

    #if ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) ) /* PRIVILEGED_FUNCTION */
        BaseType_t MPU_xTaskResumeFromISR( TaskHandle_t xTaskToResume )
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xTaskToResume;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskResumeFromISR( xInternalTaskHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xTaskToResume;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskResumeFromISR( xInternalTaskHandle );
                }
            }

            return xReturn;
        }
    #endif /* #if ( ( INCLUDE_xTaskResumeFromISR == 1 ) && ( INCLUDE_vTaskSuspend == 1 ) )*/
/*---------------------------------------------------------------------------------------*/

    TickType_t MPU_xTaskGetTickCountFromISR( void ) /* PRIVILEGED_FUNCTION */
    {
        TickType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xTaskGetTickCountFromISR();
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xTaskGetTickCountFromISR();
        }

        return xReturn;
    }
/*---------------------------------------------------------------------------------------*/

    #if ( configUSE_APPLICATION_TASK_TAG == 1 ) /* PRIVILEGED_FUNCTION */
        TaskHookFunction_t MPU_xTaskGetApplicationTaskTagFromISR( TaskHandle_t xTask )
        {
            TaskHookFunction_t xReturn;

            if( xTask != NULL )
            {
                int32_t lIndex;
                TaskHandle_t xInternalTaskHandle = NULL;

                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGetApplicationTaskTagFromISR( xInternalTaskHandle );
                        portMEMORY_BARRIER();
                    }

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    lIndex = ( int32_t ) xTask;

                    if( ( lIndex >= INDEX_OFFSET ) &&
                        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                    {
                        xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                        xReturn = xTaskGetApplicationTaskTagFromISR( xInternalTaskHandle );
                    }
                }
            }
            else
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xTaskGetApplicationTaskTagFromISR( xTask );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xTaskGetApplicationTaskTagFromISR( xTask );
                }
            }

            return xReturn;
        }
    #endif /*#if ( configUSE_APPLICATION_TASK_TAG == 1 )*/
/*---------------------------------------------------------------------------------------*/
    #if ( configUSE_TASK_NOTIFICATIONS == 1 ) /* PRIVILEGED_FUNCTION */

        BaseType_t MPU_xTaskGenericNotifyFromISR( TaskHandle_t xTaskToNotify,
                                                  UBaseType_t uxIndexToNotify,
                                                  uint32_t ulValue,
                                                  eNotifyAction eAction,
                                                  uint32_t * pulPreviousNotificationValue,
                                                  BaseType_t * pxHigherPriorityTaskWoken )
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskGenericNotifyFromISR( xInternalTaskHandle, uxIndexToNotify, ulValue, eAction, pulPreviousNotificationValue, pxHigherPriorityTaskWoken );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xTaskGenericNotifyFromISR( xInternalTaskHandle, uxIndexToNotify, ulValue, eAction, pulPreviousNotificationValue, pxHigherPriorityTaskWoken );
                }
            }

            return xReturn;
        }
    #endif /*#if ( configUSE_TASK_NOTIFICATIONS == 1 )*/
/*---------------------------------------------------------------------------------------*/

    #if ( configUSE_TASK_NOTIFICATIONS == 1 ) /* PRIVILEGED_FUNCTION */

        void MPU_vTaskGenericNotifyGiveFromISR( TaskHandle_t xTaskToNotify,
                                                UBaseType_t uxIndexToNotify,
                                                BaseType_t * pxHigherPriorityTaskWoken )
        {
            int32_t lIndex;
            TaskHandle_t xInternalTaskHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    vTaskGenericNotifyGiveFromISR( xInternalTaskHandle, uxIndexToNotify, pxHigherPriorityTaskWoken );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xTaskToNotify;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalTaskHandle = MPU_GetTaskHandleAtIndex( lIndex - INDEX_OFFSET );
                    vTaskGenericNotifyGiveFromISR( xInternalTaskHandle, uxIndexToNotify, pxHigherPriorityTaskWoken );
                }
            }
        }
    #endif /*#if ( configUSE_TASK_NOTIFICATIONS == 1 )*/
/*---------------------------------------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        QueueHandle_t MPU_xQueueGenericCreate( UBaseType_t uxQueueLength,
                                               UBaseType_t uxItemSize,
                                               uint8_t ucQueueType ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueGenericCreate( uxQueueLength, uxItemSize, ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueGenericCreate( uxQueueLength, uxItemSize, ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        QueueHandle_t MPU_xQueueGenericCreateStatic( const UBaseType_t uxQueueLength,
                                                     const UBaseType_t uxItemSize,
                                                     uint8_t * pucQueueStorage,
                                                     StaticQueue_t * pxStaticQueue,
                                                     const uint8_t ucQueueType ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueGenericCreateStatic( uxQueueLength, uxItemSize, pucQueueStorage, pxStaticQueue, ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueGenericCreateStatic( uxQueueLength, uxItemSize, pucQueueStorage, pxStaticQueue, ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( configSUPPORT_STATIC_ALLOCATION == 1 ) */
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueGenericReset( QueueHandle_t xQueue,
                                       BaseType_t xNewQueue ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueGenericReset( xInternalQueueHandle, xNewQueue );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( uint32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueGenericReset( xInternalQueueHandle, xNewQueue );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue,
                                      const void * const pvItemToQueue,
                                      TickType_t xTicksToWait,
                                      BaseType_t xCopyPosition ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueGenericSend( xInternalQueueHandle, pvItemToQueue, xTicksToWait, xCopyPosition );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueGenericSend( xInternalQueueHandle, pvItemToQueue, xTicksToWait, xCopyPosition );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    UBaseType_t MPU_uxQueueMessagesWaiting( const QueueHandle_t pxQueue ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        UBaseType_t uxReturn = 0;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) pxQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                uxReturn = uxQueueMessagesWaiting( xInternalQueueHandle );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) pxQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                uxReturn = uxQueueMessagesWaiting( xInternalQueueHandle );
            }
        }

        return uxReturn;
    }
/*-----------------------------------------------------------*/

    UBaseType_t MPU_uxQueueSpacesAvailable( const QueueHandle_t xQueue ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        UBaseType_t uxReturn = 0;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                uxReturn = uxQueueSpacesAvailable( xInternalQueueHandle );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                uxReturn = uxQueueSpacesAvailable( xInternalQueueHandle );
            }
        }

        return uxReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueReceive( QueueHandle_t pxQueue,
                                  void * const pvBuffer,
                                  TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) pxQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueReceive( xInternalQueueHandle, pvBuffer, xTicksToWait );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) pxQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueReceive( xInternalQueueHandle, pvBuffer, xTicksToWait );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueuePeek( QueueHandle_t xQueue,
                               void * const pvBuffer,
                               TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueuePeek( xInternalQueueHandle, pvBuffer, xTicksToWait );
                portMEMORY_BARRIER();
            }

            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueuePeek( xInternalQueueHandle, pvBuffer, xTicksToWait );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueSemaphoreTake( QueueHandle_t xQueue,
                                        TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;
        BaseType_t xReturn = pdFAIL;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueSemaphoreTake( xInternalQueueHandle, xTicksToWait );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueSemaphoreTake( xInternalQueueHandle, xTicksToWait );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    #if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )
        TaskHandle_t MPU_xQueueGetMutexHolder( QueueHandle_t xSemaphore ) /* FREERTOS_SYSTEM_CALL */
        {
            void * xReturn = NULL;
            int32_t lIndex;
            QueueHandle_t xInternalQueueHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xSemaphore;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueGetMutexHolder( xInternalQueueHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xSemaphore;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueGetMutexHolder( xInternalQueueHandle );
                }
            }

            return xReturn;
        }
    #endif /* if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        QueueHandle_t MPU_xQueueCreateMutex( const uint8_t ucQueueType ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateMutex( ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateMutex( ucQueueType );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
        QueueHandle_t MPU_xQueueCreateMutexStatic( const uint8_t ucQueueType,
                                                   StaticQueue_t * pxStaticQueue ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateMutexStatic( ucQueueType, pxStaticQueue );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateMutexStatic( ucQueueType, pxStaticQueue );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( ( configUSE_MUTEXES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        QueueHandle_t MPU_xQueueCreateCountingSemaphore( UBaseType_t uxCountValue,
                                                         UBaseType_t uxInitialCount ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateCountingSemaphore( uxCountValue, uxInitialCount );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateCountingSemaphore( uxCountValue, uxInitialCount );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )

        QueueHandle_t MPU_xQueueCreateCountingSemaphoreStatic( const UBaseType_t uxMaxCount,
                                                               const UBaseType_t uxInitialCount,
                                                               StaticQueue_t * pxStaticQueue ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueHandle_t xInternalQueueHandle = NULL;
            QueueHandle_t xExternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateCountingSemaphoreStatic( uxMaxCount, uxInitialCount, pxStaticQueue );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueHandle = xQueueCreateCountingSemaphoreStatic( uxMaxCount, uxInitialCount, pxStaticQueue );

                    if( xInternalQueueHandle != NULL )
                    {
                        MPU_StoreQueueHandleAtIndex( lIndex, xInternalQueueHandle );
                        xExternalQueueHandle = ( QueueHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueHandle;
        }
    #endif /* if ( ( configUSE_COUNTING_SEMAPHORES == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_RECURSIVE_MUTEXES == 1 )
        BaseType_t MPU_xQueueTakeMutexRecursive( QueueHandle_t xMutex,
                                                 TickType_t xBlockTime ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            QueueHandle_t xInternalQueueHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xMutex;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueTakeMutexRecursive( xInternalQueueHandle, xBlockTime );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xMutex;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueTakeMutexRecursive( xInternalQueueHandle, xBlockTime );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_RECURSIVE_MUTEXES == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_RECURSIVE_MUTEXES == 1 )
        BaseType_t MPU_xQueueGiveMutexRecursive( QueueHandle_t xMutex ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            int32_t lIndex;
            QueueHandle_t xInternalQueueHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xMutex;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueGiveMutexRecursive( xInternalQueueHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xMutex;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = xQueueGiveMutexRecursive( xInternalQueueHandle );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_RECURSIVE_MUTEXES == 1 ) */
/*-----------------------------------------------------------*/

    #if ( ( configUSE_QUEUE_SETS == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) )
        QueueSetHandle_t MPU_xQueueCreateSet( UBaseType_t uxEventQueueLength ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueSetHandle_t xInternalQueueSetHandle = NULL;
            QueueSetHandle_t xExternalQueueSetHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueSetHandle = xQueueCreateSet( uxEventQueueLength );

                    if( xInternalQueueSetHandle != NULL )
                    {
                        MPU_StoreQueueSetHandleAtIndex( lIndex, xInternalQueueSetHandle );
                        xExternalQueueSetHandle = ( QueueSetHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1 )
                {
                    xInternalQueueSetHandle = xQueueCreateSet( uxEventQueueLength );

                    if( xInternalQueueSetHandle != NULL )
                    {
                        MPU_StoreQueueSetHandleAtIndex( lIndex, xInternalQueueSetHandle );
                        xExternalQueueSetHandle = ( QueueSetHandle_t ) ( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalQueueSetHandle;
        }
    #endif /* if ( ( configUSE_QUEUE_SETS == 1 ) && ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_QUEUE_SETS == 1 )
        QueueSetMemberHandle_t MPU_xQueueSelectFromSet( QueueSetHandle_t xQueueSet,
                                                        TickType_t xBlockTimeTicks ) /* FREERTOS_SYSTEM_CALL */
        {
            QueueSetHandle_t xInternalQueueSetHandle = NULL;
            QueueSetMemberHandle_t xSelectedMemberInternal = NULL;
            QueueSetMemberHandle_t xSelectedMemberExternal = NULL;
            int32_t lIndexQueueSet, lIndexSelectedMember;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndexQueueSet = ( int32_t ) xQueueSet;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xSelectedMemberInternal = xQueueSelectFromSet( xInternalQueueSetHandle, xBlockTimeTicks );

                    if( xSelectedMemberInternal != NULL )
                    {
                        lIndexSelectedMember = MPU_GetIndexForQueueSetMemberHandle( xSelectedMemberInternal );
                        xSelectedMemberExternal = ( QueueSetMemberHandle_t ) ( lIndexSelectedMember + INDEX_OFFSET );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndexQueueSet = ( int32_t ) xQueueSet;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xSelectedMemberInternal = xQueueSelectFromSet( xInternalQueueSetHandle, xBlockTimeTicks );

                    if( xSelectedMemberInternal != NULL )
                    {
                        lIndexSelectedMember = MPU_GetIndexForQueueSetMemberHandle( xSelectedMemberInternal );
                        xSelectedMemberExternal = ( QueueSetMemberHandle_t ) ( lIndexSelectedMember + INDEX_OFFSET );
                    }
                }
            }

            return xSelectedMemberExternal;
        }
    #endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_QUEUE_SETS == 1 )
        BaseType_t MPU_xQueueAddToSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                       QueueSetHandle_t xQueueSet ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            QueueSetMemberHandle_t xInternalQueueSetMemberHandle = NULL;
            QueueSetHandle_t xInternalQueueSetHandle;
            int32_t lIndexQueueSet, lIndexQueueSetMember;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndexQueueSet = ( int32_t ) xQueueSet;
                lIndexQueueSetMember = ( int32_t ) xQueueOrSemaphore;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) &&
                    ( lIndexQueueSetMember >= INDEX_OFFSET ) &&
                    ( lIndexQueueSetMember < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xInternalQueueSetMemberHandle = MPU_GetQueueSetMemberHandleAtIndex( lIndexQueueSetMember - INDEX_OFFSET );
                    xReturn = xQueueAddToSet( xInternalQueueSetMemberHandle, xInternalQueueSetHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndexQueueSet = ( int32_t ) xQueueSet;
                lIndexQueueSetMember = ( int32_t ) xQueueOrSemaphore;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) &&
                    ( lIndexQueueSetMember >= INDEX_OFFSET ) &&
                    ( lIndexQueueSetMember < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xInternalQueueSetMemberHandle = MPU_GetQueueSetMemberHandleAtIndex( lIndexQueueSetMember - INDEX_OFFSET );
                    xReturn = xQueueAddToSet( xInternalQueueSetMemberHandle, xInternalQueueSetHandle );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_QUEUE_SETS == 1 )
        BaseType_t MPU_xQueueRemoveFromSet( QueueSetMemberHandle_t xQueueOrSemaphore,
                                            QueueSetHandle_t xQueueSet ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn = pdFAIL;
            QueueSetMemberHandle_t xInternalQueueSetMemberHandle = NULL;
            QueueSetHandle_t xInternalQueueSetHandle;
            int32_t lIndexQueueSet, lIndexQueueSetMember;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndexQueueSet = ( int32_t ) xQueueSet;
                lIndexQueueSetMember = ( int32_t ) xQueueOrSemaphore;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) &&
                    ( lIndexQueueSetMember >= INDEX_OFFSET ) &&
                    ( lIndexQueueSetMember < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xInternalQueueSetMemberHandle = MPU_GetQueueSetMemberHandleAtIndex( lIndexQueueSetMember - INDEX_OFFSET );
                    xReturn = xQueueRemoveFromSet( xInternalQueueSetMemberHandle, xInternalQueueSetHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndexQueueSet = ( int32_t ) xQueueSet;
                lIndexQueueSetMember = ( int32_t ) xQueueOrSemaphore;

                if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                    ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) &&
                    ( lIndexQueueSetMember >= INDEX_OFFSET ) &&
                    ( lIndexQueueSetMember < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                    xInternalQueueSetMemberHandle = MPU_GetQueueSetMemberHandleAtIndex( lIndexQueueSetMember - INDEX_OFFSET );
                    xReturn = xQueueRemoveFromSet( xInternalQueueSetMemberHandle, xInternalQueueSetHandle );
                }
            }

            return xReturn;
        }
    #endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

    #if configQUEUE_REGISTRY_SIZE > 0
        void MPU_vQueueAddToRegistry( QueueHandle_t xQueue,
                                      const char * pcName ) /* FREERTOS_SYSTEM_CALL */
        {
            int32_t lIndex;
            QueueHandle_t xInternalQueueHandle = NULL;


            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    vQueueAddToRegistry( xInternalQueueHandle, pcName );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    vQueueAddToRegistry( xInternalQueueHandle, pcName );
                }
            }
        }
    #endif /* if configQUEUE_REGISTRY_SIZE > 0 */
/*-----------------------------------------------------------*/

    #if configQUEUE_REGISTRY_SIZE > 0
        void MPU_vQueueUnregisterQueue( QueueHandle_t xQueue ) /* FREERTOS_SYSTEM_CALL */
        {
            int32_t lIndex;
            QueueHandle_t xInternalQueueHandle = NULL;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    vQueueUnregisterQueue( xInternalQueueHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    vQueueUnregisterQueue( xInternalQueueHandle );
                }
            }
        }
    #endif /* if configQUEUE_REGISTRY_SIZE > 0 */
/*-----------------------------------------------------------*/

    #if configQUEUE_REGISTRY_SIZE > 0
        const char * MPU_pcQueueGetName( QueueHandle_t xQueue ) /* FREERTOS_SYSTEM_CALL */
        {
            const char * pcReturn;
            QueueHandle_t xInternalQueueHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    pcReturn = pcQueueGetName( xInternalQueueHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xQueue;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                    pcReturn = pcQueueGetName( xInternalQueueHandle );
                }
            }

            return pcReturn;
        }
    #endif /* if configQUEUE_REGISTRY_SIZE > 0 */
/*-----------------------------------------------------------*/

    void MPU_vQueueDelete( QueueHandle_t xQueue ) /* FREERTOS_SYSTEM_CALL */
    {
        QueueHandle_t xInternalQueueHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                vQueueDelete( xInternalQueueHandle );
                MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xQueue;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                vQueueDelete( xInternalQueueHandle );
                MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
            }
        }
    }
/*-----------------------------------------------------------*/

/* Privileged only wrappers for Queue APIs. These are needed so that
 * the application can use opaque handles maintained in mpu_wrappers.c
 * with all the APIs. */
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueGenericSendFromISR( QueueHandle_t xQueue,
                                             const void * const pvItemToQueue,
                                             BaseType_t * const pxHigherPriorityTaskWoken,
                                             const BaseType_t xCopyPosition ) /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueueGenericSendFromISR( xInternalQueueHandle, pvItemToQueue, pxHigherPriorityTaskWoken, xCopyPosition );
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueGiveFromISR( QueueHandle_t xQueue,
                                      BaseType_t * const pxHigherPriorityTaskWoken ) /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueueGiveFromISR( xInternalQueueHandle, pxHigherPriorityTaskWoken );
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueuePeekFromISR( QueueHandle_t xQueue,
                                      void * const pvBuffer ) /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueuePeekFromISR( xInternalQueueHandle, pvBuffer );
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueReceiveFromISR( QueueHandle_t xQueue,
                                         void * const pvBuffer,
                                         BaseType_t * const pxHigherPriorityTaskWoken ) /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueueReceiveFromISR( xInternalQueueHandle, pvBuffer, pxHigherPriorityTaskWoken );
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue ) /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueueIsQueueEmptyFromISR( xInternalQueueHandle );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xQueueIsQueueFullFromISR( const QueueHandle_t xQueue )  /* PRIVILEGED_FUNCTION */
    {
        BaseType_t xReturn = pdFAIL;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xQueueIsQueueFullFromISR( xInternalQueueHandle );
        }

        return xReturn;
    }

/*-----------------------------------------------------------*/

    UBaseType_t MPU_uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue ) /* PRIVILEGED_FUNCTION */
    {
        UBaseType_t uxReturn = 0;
        int32_t lIndex;
        QueueHandle_t xInternalQueueHandle = NULL;

        lIndex = ( int32_t ) xQueue;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalQueueHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
            uxReturn = uxQueueMessagesWaitingFromISR( xInternalQueueHandle );
        }

        return uxReturn;
    }

/*-----------------------------------------------------------*/

    #if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) )
        TaskHandle_t MPU_xQueueGetMutexHolderFromISR( QueueHandle_t xSemaphore ) /* PRIVILEGED_FUNCTION */
        {
            TaskHandle_t xReturn = NULL;
            int32_t lIndex;
            QueueHandle_t xInternalSemaphoreHandle = NULL;

            lIndex = ( int32_t ) xSemaphore;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalSemaphoreHandle = MPU_GetQueueHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xQueueGetMutexHolder( xInternalSemaphoreHandle );
            }

            return xReturn;
        }
    #endif /* #if ( ( configUSE_MUTEXES == 1 ) && ( INCLUDE_xSemaphoreGetMutexHolder == 1 ) ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_QUEUE_SETS == 1 )
        QueueSetMemberHandle_t MPU_xQueueSelectFromSetFromISR( QueueSetHandle_t xQueueSet,
                                                               TickType_t xBlockTimeTicks ) /* PRIVILEGED_FUNCTION */
        {
            QueueSetHandle_t xInternalQueueSetHandle = NULL;
            QueueSetMemberHandle_t xSelectedMemberInternal = NULL;
            QueueSetMemberHandle_t xSelectedMemberExternal = NULL;
            int32_t lIndexQueueSet, lIndexSelectedMember;

            lIndexQueueSet = ( int32_t ) xQueueSet;

            if( ( lIndexQueueSet >= INDEX_OFFSET ) &&
                ( lIndexQueueSet < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalQueueSetHandle = MPU_GetQueueSetHandleAtIndex( lIndexQueueSet - INDEX_OFFSET );
                xSelectedMemberInternal = xQueueSelectFromSetFromISR( xInternalQueueSetHandle );

                if( xSelectedMemberInternal != NULL )
                {
                    lIndexSelectedMember = MPU_GetIndexForQueueSetMemberHandle( xSelectedMemberInternal );
                    xSelectedMemberExternal = ( QueueSetMemberHandle_t ) ( lIndexSelectedMember + INDEX_OFFSET );
                }
            }

            return xSelectedMemberExternal;
        }
    #endif /* if ( configUSE_QUEUE_SETS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        void * MPU_pvTimerGetTimerID( const TimerHandle_t xTimer ) /* FREERTOS_SYSTEM_CALL */
        {
            void * pvReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                pvReturn = pvTimerGetTimerID( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                pvReturn = pvTimerGetTimerID( xTimer );
            }

            return pvReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        void MPU_vTimerSetTimerID( TimerHandle_t xTimer,
                                   void * pvNewID ) /* FREERTOS_SYSTEM_CALL */
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                vTimerSetTimerID( xTimer, pvNewID );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                vTimerSetTimerID( xTimer, pvNewID );
            }
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        BaseType_t MPU_xTimerIsTimerActive( TimerHandle_t xTimer ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTimerIsTimerActive( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTimerIsTimerActive( xTimer );
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        TaskHandle_t MPU_xTimerGetTimerDaemonTaskHandle( void ) /* FREERTOS_SYSTEM_CALL */
        {
            TaskHandle_t xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTimerGetTimerDaemonTaskHandle();
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTimerGetTimerDaemonTaskHandle();
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        void MPU_vTimerSetReloadMode( TimerHandle_t xTimer,
                                      const UBaseType_t uxAutoReload ) /* FREERTOS_SYSTEM_CALL */
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                vTimerSetReloadMode( xTimer, uxAutoReload );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                vTimerSetReloadMode( xTimer, uxAutoReload );
            }
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        UBaseType_t MPU_uxTimerGetReloadMode( TimerHandle_t xTimer )
        {
            UBaseType_t uxReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                uxReturn = uxTimerGetReloadMode( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                uxReturn = uxTimerGetReloadMode( xTimer );
            }

            return uxReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        const char * MPU_pcTimerGetName( TimerHandle_t xTimer ) /* FREERTOS_SYSTEM_CALL */
        {
            const char * pcReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                pcReturn = pcTimerGetName( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                pcReturn = pcTimerGetName( xTimer );
            }

            return pcReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        TickType_t MPU_xTimerGetPeriod( TimerHandle_t xTimer ) /* FREERTOS_SYSTEM_CALL */
        {
            TickType_t xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTimerGetPeriod( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTimerGetPeriod( xTimer );
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        TickType_t MPU_xTimerGetExpiryTime( TimerHandle_t xTimer ) /* FREERTOS_SYSTEM_CALL */
        {
            TickType_t xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTimerGetExpiryTime( xTimer );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTimerGetExpiryTime( xTimer );
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configUSE_TIMERS == 1 )
        BaseType_t MPU_xTimerGenericCommand( TimerHandle_t xTimer,
                                             const BaseType_t xCommandID,
                                             const TickType_t xOptionalValue,
                                             BaseType_t * const pxHigherPriorityTaskWoken,
                                             const TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
        {
            BaseType_t xReturn;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = xTimerGenericCommand( xTimer, xCommandID, xOptionalValue, pxHigherPriorityTaskWoken, xTicksToWait );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = xTimerGenericCommand( xTimer, xCommandID, xOptionalValue, pxHigherPriorityTaskWoken, xTicksToWait );
            }

            return xReturn;
        }
    #endif /* if ( configUSE_TIMERS == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        EventGroupHandle_t MPU_xEventGroupCreate( void ) /* FREERTOS_SYSTEM_CALL */
        {
            EventGroupHandle_t xInternalEventGroupHandle = NULL;
            EventGroupHandle_t xExternalEventGroupHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1)
                {
                    xInternalEventGroupHandle = xEventGroupCreate();
                    if( xInternalEventGroupHandle != NULL )
                    {
                        MPU_StoreEventGroupHandleAtIndex( lIndex, xInternalEventGroupHandle );
                        xExternalEventGroupHandle = ( EventGroupHandle_t )( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }

                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1)
                {
                    xInternalEventGroupHandle = xEventGroupCreate();
                    if( xInternalEventGroupHandle != NULL )
                    {
                        MPU_StoreEventGroupHandleAtIndex( lIndex, xInternalEventGroupHandle );
                        xExternalEventGroupHandle = ( EventGroupHandle_t )( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalEventGroupHandle;
        }
    #endif /* if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 ) */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        EventGroupHandle_t MPU_xEventGroupCreateStatic( StaticEventGroup_t * pxEventGroupBuffer ) /* FREERTOS_SYSTEM_CALL */
        {
            EventGroupHandle_t xInternalEventGroupHandle = NULL;
            EventGroupHandle_t xExternalEventGroupHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1)
                {
                    xInternalEventGroupHandle = xEventGroupCreateStatic( pxEventGroupBuffer );
                    if( xInternalEventGroupHandle != NULL )
                    {
                        MPU_StoreEventGroupHandleAtIndex( lIndex, xInternalEventGroupHandle );
                        xExternalEventGroupHandle = ( EventGroupHandle_t )( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                    portMEMORY_BARRIER();
                }
                
                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = MPU_GetFreeIndexInHandlePool();

                if( lIndex != -1)
                {
                    xInternalEventGroupHandle = xEventGroupCreateStatic( pxEventGroupBuffer );
                    if( xInternalEventGroupHandle != NULL )
                    {
                        MPU_StoreEventGroupHandleAtIndex( lIndex, xInternalEventGroupHandle );
                        xExternalEventGroupHandle = ( EventGroupHandle_t )( lIndex + INDEX_OFFSET );
                    }
                    else
                    {
                        MPU_SetIndexFreeInHandlePool( lIndex );
                    }
                }
            }

            return xExternalEventGroupHandle;
        }
    #endif /* if ( configSUPPORT_STATIC_ALLOCATION == 1 ) */
/*-----------------------------------------------------------*/

    EventBits_t MPU_xEventGroupWaitBits( EventGroupHandle_t xEventGroup,
                                         const EventBits_t uxBitsToWaitFor,
                                         const BaseType_t xClearOnExit,
                                         const BaseType_t xWaitForAllBits,
                                         TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        EventBits_t xReturn;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupWaitBits( xInternalEventGroupHandle, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait );
                portMEMORY_BARRIER();
            }
            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupWaitBits( xInternalEventGroupHandle, uxBitsToWaitFor, xClearOnExit, xWaitForAllBits, xTicksToWait );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    EventBits_t MPU_xEventGroupClearBits( EventGroupHandle_t xEventGroup,
                                          const EventBits_t uxBitsToClear ) /* FREERTOS_SYSTEM_CALL */
    {
        EventBits_t xReturn;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupClearBits( xInternalEventGroupHandle, uxBitsToClear );
                portMEMORY_BARRIER();
            }
            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupClearBits( xInternalEventGroupHandle, uxBitsToClear );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                        const EventBits_t uxBitsToSet ) /* FREERTOS_SYSTEM_CALL */
    {
        EventBits_t xReturn;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupSetBits( xInternalEventGroupHandle, uxBitsToSet );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupSetBits( xInternalEventGroupHandle, uxBitsToSet );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    EventBits_t MPU_xEventGroupSync( EventGroupHandle_t xEventGroup,
                                     const EventBits_t uxBitsToSet,
                                     const EventBits_t uxBitsToWaitFor,
                                     TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        EventBits_t xReturn;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupSync( xInternalEventGroupHandle, uxBitsToSet, uxBitsToWaitFor, xTicksToWait );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                xReturn = xEventGroupSync( xInternalEventGroupHandle, uxBitsToSet, uxBitsToWaitFor, xTicksToWait );
            }
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    void MPU_vEventGroupDelete( EventGroupHandle_t xEventGroup ) /* FREERTOS_SYSTEM_CALL */
    {
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                vEventGroupDelete( xInternalEventGroupHandle );
                MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                vEventGroupDelete( xInternalEventGroupHandle );
                MPU_SetIndexFreeInHandlePool( lIndex - INDEX_OFFSET );
            }
        }
    }
/*-----------------------------------------------------------*/
#if ( configUSE_TRACE_FACILITY == 1 )
    UBaseType_t MPU_uxEventGroupGetNumber( void * xEventGroup ) /* FREERTOS_SYSTEM_CALL */
    {
        UBaseType_t xReturn ;

        if( xEventGroup != NULL )
        {
            EventGroupHandle_t xInternalEventGroupHandle = NULL;
            int32_t lIndex;

            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                lIndex = ( int32_t ) xEventGroup;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = uxEventGroupGetNumber( xInternalEventGroupHandle );
                    portMEMORY_BARRIER();
                }

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                lIndex = ( int32_t ) xEventGroup;

                if( ( lIndex >= INDEX_OFFSET ) &&
                    ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
                {
                    xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                    xReturn = uxEventGroupGetNumber( xInternalEventGroupHandle );
                }
            }

        }
        else
        {
            if( portIS_PRIVILEGED() == pdFALSE )
            {
                portRAISE_PRIVILEGE();
                portMEMORY_BARRIER();

                xReturn = uxEventGroupGetNumber( xEventGroup );
                portMEMORY_BARRIER();

                portRESET_PRIVILEGE();
                portMEMORY_BARRIER();
            }
            else
            {
                xReturn = uxEventGroupGetNumber( xEventGroup );
            }
        }

        return xReturn;
    }
#endif /*( configUSE_TRACE_FACILITY == 1 )*/
/*-----------------------------------------------------------*/
#if ( configUSE_TRACE_FACILITY == 1 )
    void MPU_vEventGroupSetNumber( void * xEventGroup,
                               UBaseType_t uxEventGroupNumber ) /* FREERTOS_SYSTEM_CALL */
    {
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                vEventGroupSetNumber( xInternalEventGroupHandle, uxEventGroupNumber);
                portMEMORY_BARRIER();
            }

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            lIndex = ( int32_t ) xEventGroup;

            if( ( lIndex >= INDEX_OFFSET ) &&
                ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
            {
                xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
                vEventGroupSetNumber( xInternalEventGroupHandle, uxEventGroupNumber);
            }
        }
    }
#endif /*( configUSE_TRACE_FACILITY == 1 )*/
/*-----------------------------------------------------------*/
#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )
    BaseType_t MPU_xEventGroupClearBitsFromISR( EventGroupHandle_t xEventGroup,
                                                const EventBits_t uxBitsToClear ) /*PRIVILEGED_FUNCTION*/
    {
        BaseType_t xReturn = pdFALSE;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        lIndex = ( int32_t ) xEventGroup;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xEventGroupClearBitsFromISR( xInternalEventGroupHandle, uxBitsToClear );
        }
        return xReturn;
    }

/*-----------------------------------------------------------*/

    BaseType_t MPU_xEventGroupSetBitsFromISR( EventGroupHandle_t xEventGroup,
                                            const EventBits_t uxBitsToSet,
                                            BaseType_t * pxHigherPriorityTaskWoken ) /*PRIVILEGED_FUNCTION*/
    {
        BaseType_t xReturn = pdFALSE;
        EventGroupHandle_t xInternalEventGroupHandle = NULL;
        int32_t lIndex;

        lIndex = ( int32_t ) xEventGroup;

        if( ( lIndex >= INDEX_OFFSET ) &&
            ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
        {
            xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
            xReturn = xEventGroupSetBitsFromISR( xInternalEventGroupHandle, uxBitsToSet, pxHigherPriorityTaskWoken );
        }
        return xReturn;
    }

#endif /*( ( configUSE_TRACE_FACILITY == 1 ) && ( INCLUDE_xTimerPendFunctionCall == 1 ) && ( configUSE_TIMERS == 1 ) )*/

/*-----------------------------------------------------------*/

EventBits_t MPU_xEventGroupGetBitsFromISR( EventGroupHandle_t xEventGroup ) /*PRIVILEGED_FUNCTION*/
{
    EventBits_t xReturn;
    EventGroupHandle_t xInternalEventGroupHandle = NULL;
    int32_t lIndex;

    lIndex = ( int32_t ) xEventGroup;

    if( ( lIndex >= INDEX_OFFSET ) &&
        ( lIndex < ( configPROTECTED_KERNEL_OBJECT_HANDLE_POOL_SIZE + INDEX_OFFSET ) ) )
    {
        xInternalEventGroupHandle = MPU_GetEventGroupHandleAtIndex( lIndex - INDEX_OFFSET );
        xReturn = xEventGroupGetBitsFromISR( xInternalEventGroupHandle );
    }
    return xReturn;
}
/*-----------------------------------------------------------*/

    size_t MPU_xStreamBufferSend( StreamBufferHandle_t xStreamBuffer,
                                  const void * pvTxData,
                                  size_t xDataLengthBytes,
                                  TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        size_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferSend( xStreamBuffer, pvTxData, xDataLengthBytes, xTicksToWait );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferSend( xStreamBuffer, pvTxData, xDataLengthBytes, xTicksToWait );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    size_t MPU_xStreamBufferNextMessageLengthBytes( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        size_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferNextMessageLengthBytes( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferNextMessageLengthBytes( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    size_t MPU_xStreamBufferReceive( StreamBufferHandle_t xStreamBuffer,
                                     void * pvRxData,
                                     size_t xBufferLengthBytes,
                                     TickType_t xTicksToWait ) /* FREERTOS_SYSTEM_CALL */
    {
        size_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferReceive( xStreamBuffer, pvRxData, xBufferLengthBytes, xTicksToWait );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferReceive( xStreamBuffer, pvRxData, xBufferLengthBytes, xTicksToWait );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    void MPU_vStreamBufferDelete( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            vStreamBufferDelete( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            vStreamBufferDelete( xStreamBuffer );
        }
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xStreamBufferIsFull( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferIsFull( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferIsFull( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xStreamBufferIsEmpty( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferIsEmpty( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferIsEmpty( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xStreamBufferReset( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferReset( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferReset( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    size_t MPU_xStreamBufferSpacesAvailable( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        size_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();
            xReturn = xStreamBufferSpacesAvailable( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferSpacesAvailable( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    size_t MPU_xStreamBufferBytesAvailable( StreamBufferHandle_t xStreamBuffer ) /* FREERTOS_SYSTEM_CALL */
    {
        size_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferBytesAvailable( xStreamBuffer );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferBytesAvailable( xStreamBuffer );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    BaseType_t MPU_xStreamBufferSetTriggerLevel( StreamBufferHandle_t xStreamBuffer,
                                                 size_t xTriggerLevel ) /* FREERTOS_SYSTEM_CALL */
    {
        BaseType_t xReturn;

        if( portIS_PRIVILEGED() == pdFALSE )
        {
            portRAISE_PRIVILEGE();
            portMEMORY_BARRIER();

            xReturn = xStreamBufferSetTriggerLevel( xStreamBuffer, xTriggerLevel );
            portMEMORY_BARRIER();

            portRESET_PRIVILEGE();
            portMEMORY_BARRIER();
        }
        else
        {
            xReturn = xStreamBufferSetTriggerLevel( xStreamBuffer, xTriggerLevel );
        }

        return xReturn;
    }
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_DYNAMIC_ALLOCATION == 1 )
        StreamBufferHandle_t MPU_xStreamBufferGenericCreate( size_t xBufferSizeBytes,
                                                             size_t xTriggerLevelBytes,
                                                             BaseType_t xIsMessageBuffer,
                                                             StreamBufferCallbackFunction_t pxSendCompletedCallback,
                                                             StreamBufferCallbackFunction_t pxReceiveCompletedCallback ) /* FREERTOS_SYSTEM_CALL */
        {
            StreamBufferHandle_t xReturn;

            /**
             * Streambuffer application level callback functionality is disabled for MPU
             * enabled ports.
             */
            configASSERT( ( pxSendCompletedCallback == NULL ) &&
                          ( pxReceiveCompletedCallback == NULL ) );

            if( ( pxSendCompletedCallback == NULL ) &&
                ( pxReceiveCompletedCallback == NULL ) )
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xStreamBufferGenericCreate( xBufferSizeBytes,
                                                          xTriggerLevelBytes,
                                                          xIsMessageBuffer,
                                                          NULL,
                                                          NULL );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xStreamBufferGenericCreate( xBufferSizeBytes,
                                                          xTriggerLevelBytes,
                                                          xIsMessageBuffer,
                                                          NULL,
                                                          NULL );
                }
            }
            else
            {
                traceSTREAM_BUFFER_CREATE_FAILED( xIsMessageBuffer );
                xReturn = NULL;
            }

            return xReturn;
        }
    #endif /* configSUPPORT_DYNAMIC_ALLOCATION */
/*-----------------------------------------------------------*/

    #if ( configSUPPORT_STATIC_ALLOCATION == 1 )
        StreamBufferHandle_t MPU_xStreamBufferGenericCreateStatic( size_t xBufferSizeBytes,
                                                                   size_t xTriggerLevelBytes,
                                                                   BaseType_t xIsMessageBuffer,
                                                                   uint8_t * const pucStreamBufferStorageArea,
                                                                   StaticStreamBuffer_t * const pxStaticStreamBuffer,
                                                                   StreamBufferCallbackFunction_t pxSendCompletedCallback,
                                                                   StreamBufferCallbackFunction_t pxReceiveCompletedCallback ) /* FREERTOS_SYSTEM_CALL */
        {
            StreamBufferHandle_t xReturn;

            /**
             * Streambuffer application level callback functionality is disabled for MPU
             * enabled ports.
             */
            configASSERT( ( pxSendCompletedCallback == NULL ) &&
                          ( pxReceiveCompletedCallback == NULL ) );

            if( ( pxSendCompletedCallback == NULL ) &&
                ( pxReceiveCompletedCallback == NULL ) )
            {
                if( portIS_PRIVILEGED() == pdFALSE )
                {
                    portRAISE_PRIVILEGE();
                    portMEMORY_BARRIER();

                    xReturn = xStreamBufferGenericCreateStatic( xBufferSizeBytes,
                                                                xTriggerLevelBytes,
                                                                xIsMessageBuffer,
                                                                pucStreamBufferStorageArea,
                                                                pxStaticStreamBuffer,
                                                                NULL,
                                                                NULL );
                    portMEMORY_BARRIER();

                    portRESET_PRIVILEGE();
                    portMEMORY_BARRIER();
                }
                else
                {
                    xReturn = xStreamBufferGenericCreateStatic( xBufferSizeBytes,
                                                                xTriggerLevelBytes,
                                                                xIsMessageBuffer,
                                                                pucStreamBufferStorageArea,
                                                                pxStaticStreamBuffer,
                                                                NULL,
                                                                NULL );
                }
            }
            else
            {
                traceSTREAM_BUFFER_CREATE_STATIC_FAILED( xReturn, xIsMessageBuffer );
                xReturn = NULL;
            }

            return xReturn;
        }
    #endif /* configSUPPORT_STATIC_ALLOCATION */
/*-----------------------------------------------------------*/


/* Functions that the application writer wants to execute in privileged mode
 * can be defined in application_defined_privileged_functions.h.  The functions
 * must take the same format as those above whereby the privilege state on exit
 * equals the privilege state on entry.  For example:
 *
 * void MPU_FunctionName( [parameters ] ) FREERTOS_SYSTEM_CALL;
 * void MPU_FunctionName( [parameters ] )
 * {
 *      if( portIS_PRIVILEGED() == pdFALSE )
 *      {
 *          portRAISE_PRIVILEGE();
 *          portMEMORY_BARRIER();
 *
 *          FunctionName( [parameters ] );
 *          portMEMORY_BARRIER();
 *
 *          portRESET_PRIVILEGE();
 *          portMEMORY_BARRIER();
 *      }
 *      else
 *      {
 *          FunctionName( [parameters ] );
 *      }
 * }
 */

    #if configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS == 1
        #include "application_defined_privileged_functions.h"
    #endif
/*-----------------------------------------------------------*/

#endif /* portUSING_MPU_WRAPPERS == 1 */
/*-----------------------------------------------------------*/
