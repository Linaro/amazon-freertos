/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (c) 2019-2020 Arm Limited. All rights reserved.
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

/*
 * This file is derivative of FreeRTOS V10.2.1
 * demo\CORTEX_MPU_M33F_Simulator_Keil_GCC\NonSecure\main_ns.c
 */

#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Log print include */
#include "print_log.h"
#include "iot_logging_task.h"

/* Hardware setup include */
#include "hardware_setup.h"

/* Test includes */
#include "aws_test_runner.h"

#define MUSCA_B_NS_PERIPHERAL_REGION_START_ADDRESS    ( 0x40000000 )
#define MUSCA_B_NS_PERIPHERAL_REGION_SIZE             ( 0x10000000 )

#define mainLOGGING_TASK_STACK_SIZE                   ( configMINIMAL_STACK_SIZE * 8 )
#define mainLOGGING_MESSAGE_QUEUE_LENGTH              ( 20 )

extern uint32_t Image$$ER_IROM_NS_PRIVILEGED$$Base;
extern uint32_t Image$$ER_IROM_NS_FREERTOS_SYSTEM_CALLS$$Base;
extern uint32_t Image$$ER_IROM_NS_UNPRIVILEGED$$Base;
extern uint32_t Image$$ER_IRAM_NS_PRIVILEGED$$Base;
extern uint32_t Image$$ER_IRAM_NS_TASKS_SHARE$$Base;

extern uint32_t Image$$IROM_NS_PRIVILEGED_ALIGN$$Limit;
extern uint32_t Image$$IROM_NS_FREERTOS_ALIGN$$Limit;
extern uint32_t Image$$IROM_NS_UNPRIVILEGED_ALIGN$$Limit;
extern uint32_t Image$$IRAM_NS_PRIVILEGED_ALIGN$$Limit;
extern uint32_t Image$$IRAM_NS_TASKS_SHARE_ALIGN$$Limit;

/* Externs needed by the MPU setup code. These must match the memory map as
 * specified in Scatter-Loading description file (musca_ns.sct). */
/* Privileged flash. */
const uint32_t * __privileged_functions_start__  = ( uint32_t * ) &( Image$$ER_IROM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_functions_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in privileged Flash region. */

/* Flash containing system calls. */
const uint32_t * __syscalls_flash_start__    = ( uint32_t * ) &( Image$$ER_IROM_NS_FREERTOS_SYSTEM_CALLS$$Base );
const uint32_t * __syscalls_flash_end__      = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_FREERTOS_ALIGN$$Limit ) - 0x1 );  /* Last address in Flash region containing system calls. */

/* Unprivileged flash. Note that the section containing
 * system calls is unprivilged so that unprivleged tasks
 * can make system calls. */
const uint32_t * __unprivileged_flash_start__  = ( uint32_t * ) &( Image$$ER_IROM_NS_UNPRIVILEGED$$Base );
const uint32_t * __unprivileged_flash_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_UNPRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in un-privileged Flash region. */

/* Priviledge ram. This contains kernel data. */
const uint32_t * __privileged_sram_start__  = ( uint32_t * ) &( Image$$ER_IRAM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_sram_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IRAM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 ); /* Last address in privileged RAM. */

/*-----------------------------------------------------------*/

/**
 * @brief Creates all the tasks for this demo.
 */
static void prvCreateTasks( void );

/**
 * @brief Implements the task which run the PKCS#11 tests.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvPCKS11TestTask( void * pvParameters );

/**
 * @brief Called from the kernel if a stack overflow is detected.
 *
 * @param xTask[in] Task handle of the task for which stack overflow
 *                  is detected.
 * @param pcTaskName[in] The name of the task for which stack overflow
 *                       is detected.
 */
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );

/* Non-secure main() */
int main( void )
{
BaseType_t xRet;
    xRet = xHardwareSetup();
    configASSERT( xRet == pdPASS );

    /* Initialise the UART lock. */
    vUARTLockInit();

    /* Create tasks. */
    prvCreateTasks();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not reach here as the scheduler is
    already started. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

static void prvCreateTasks( void )
{
static StackType_t xPKCS11TestTaskStack[ 5 * configMINIMAL_STACK_SIZE ] __attribute__(( aligned( 32 ) ));

/* Shared ram. This contains shared data. */
const uint32_t * __tasks_share_mem_start__    = ( uint32_t * ) &( Image$$ER_IRAM_NS_TASKS_SHARE$$Base );
const uint32_t * __tasks_share_mem_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IRAM_NS_TASKS_SHARE_ALIGN$$Limit ) - 0x1 );
const uint32_t ulSharedMemSize = ( uint32_t )__tasks_share_mem_end__ - ( uint32_t )__tasks_share_mem_start__;

TaskParameters_t xPKCS11TestTaskParameters =
{
    .pvTaskCode        = prvPCKS11TestTask,
    .pcName            = "PKCS#11 Test",
    .usStackDepth      = 5 * configMINIMAL_STACK_SIZE,
    .pvParameters      = NULL,
    .uxPriority        = tskIDLE_PRIORITY | portPRIVILEGE_BIT,
    .puxStackBuffer    = xPKCS11TestTaskStack,
    .xRegions          = {
                            {
                                ( void* ) __tasks_share_mem_start__,        \
                                ulSharedMemSize,                            \
                                tskMPU_REGION_READ_WRITE | tskMPU_REGION_EXECUTE_NEVER
                            },
                            {
                                ( void* ) MUSCA_B_NS_PERIPHERAL_REGION_START_ADDRESS,        \
                                ( uint32_t ) MUSCA_B_NS_PERIPHERAL_REGION_SIZE,             \
                                tskMPU_REGION_READ_WRITE | tskMPU_REGION_EXECUTE_NEVER |    \
                                tskMPU_REGION_DEVICE_MEMORY
                            },
                            { 0, 0, 0 },
                         }
};

    xTaskCreateRestricted( &( xPKCS11TestTaskParameters ), NULL );
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            tskIDLE_PRIORITY | portPRIVILEGE_BIT,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );
}
/*-----------------------------------------------------------*/

static void prvPCKS11TestTask( void * pvParameters )
{
    /* This task calls secure side functions. So allocate a
    secure context for it. */
    portALLOCATE_SECURE_CONTEXT( 0 );

    for( ; ; )
    {
        /* Run the test cases. */
        TEST_RUNNER_RunTests_task( NULL );
    }
}
/*-----------------------------------------------------------*/

void prvMemManageDebug( void )
{
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

/**
 * @brief The fault handler implementation calls a function called
 * prvGetRegistersFromStack().
 */
void MemManage_Handler( void )
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, handler2_address_const                            \n"
        " bx r1                                                     \n"
        "                                                           \n"
        " .align 2                                                  \n"
        " handler2_address_const: .word prvMemManageDebug           \n"
    );
}
/*-----------------------------------------------------------*/
