/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (c) 2019 Arm Limited. All rights reserved.
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

/* PSA protected storage include */
#include "psa/protected_storage.h"

/* Log print include */
#include "print_log.h"

/* Hardware setup include */
#include "hardware_setup.h"

#define TEST_UID_A		2U
#define TEST_UID_B		4U
#define ASSET_A			"THEQUICKBROWNFOXJUMPSOVERALAZYDOG"
#define ASSET_A_SIZE	( sizeof( ASSET_A ) - 1 )
#define ASSET_B			"THISISAWRITEONCEDATA"
#define ASSET_B_SIZE	( sizeof( ASSET_B ) - 1 )
#define RESETDATA		"THISIS"
#define RESETDATA_SIZE	( sizeof( RESETDATA ) - 1 )
#define READ_LENGTH		( ASSET_A_SIZE > RESETDATA_SIZE ? \
							ASSET_A_SIZE : RESETDATA_SIZE )
#define MUSCA_B_NS_PERIPHERAL_REGION_START_ADDRESS		( 0x40000000 )
#define MUSCA_B_NS_PERIPHERAL_REGION_SIZE				( 0x10000000 )

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
const uint32_t * __privileged_functions_start__	= ( uint32_t * ) &( Image$$ER_IROM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_functions_end__	= ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in privileged Flash region. */

/* Flash containing system calls. */
const uint32_t * __syscalls_flash_start__	= ( uint32_t * ) &( Image$$ER_IROM_NS_FREERTOS_SYSTEM_CALLS$$Base );
const uint32_t * __syscalls_flash_end__		= ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_FREERTOS_ALIGN$$Limit ) - 0x1 );  /* Last address in Flash region containing system calls. */

/* Unprivileged flash. Note that the section containing
 * system calls is unprivilged so that unprivleged tasks
 * can make system calls. */
const uint32_t * __unprivileged_flash_start__	= ( uint32_t * ) &( Image$$ER_IROM_NS_UNPRIVILEGED$$Base );
const uint32_t * __unprivileged_flash_end__	= ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_UNPRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in un-privileged Flash region. */

/* Priviledge ram. This contains kernel data. */
const uint32_t * __privileged_sram_start__	= ( uint32_t * ) &( Image$$ER_IRAM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_sram_end__	= ( uint32_t * ) (( uint32_t )&( Image$$IRAM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 ); /* Last address in privileged RAM. */

/*-----------------------------------------------------------*/

/**
 * @brief Creates all the tasks for this demo.
 */
static void prvCreateTasks( void );

/**
 * @brief Implements the task which sets, resets, gets and removes asset A.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvProtectedStorageAssetATask( void * pvParameters );

/**
 * @brief Implements the task which sets, gets and removes asset B.
 *
 * @param pvParameters[in] Parameters as passed during task creation.
 */
static void prvProtectedStorageAssetBTask( void * pvParameters );

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
static StackType_t xSecureSSTAssetATaskStack[ configMINIMAL_STACK_SIZE ] __attribute__(( aligned( 32 ) ));
static StackType_t xSecureSSTAssetBTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__(( aligned( 32 ) ));

/* Shared ram. This contains shared data. */
const uint32_t * __tasks_share_mem_start__	= ( uint32_t * ) &( Image$$ER_IRAM_NS_TASKS_SHARE$$Base );
const uint32_t * __tasks_share_mem_end__	= ( uint32_t * ) (( uint32_t )&( Image$$IRAM_NS_TASKS_SHARE_ALIGN$$Limit ) - 0x1 );
const uint32_t ulSharedMemSize = ( uint32_t )__tasks_share_mem_end__ - ( uint32_t )__tasks_share_mem_start__;

TaskParameters_t xAssetATaskParameters =
{
	.pvTaskCode		= prvProtectedStorageAssetATask,
	.pcName			= "PSSAssetA",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xSecureSSTAssetATaskStack,
	.xRegions		= {
						{
							( void* ) __tasks_share_mem_start__,		\
							ulSharedMemSize,							\
							tskMPU_REGION_READ_ONLY | tskMPU_REGION_EXECUTE_NEVER
						},
						{
							( void* ) MUSCA_B_NS_PERIPHERAL_REGION_START_ADDRESS,		\
							( uint32_t ) MUSCA_B_NS_PERIPHERAL_REGION_SIZE, 			\
							tskMPU_REGION_READ_WRITE | tskMPU_REGION_EXECUTE_NEVER |	\
							tskMPU_REGION_DEVICE_MEMORY
						},
						{ 0, 0, 0 },
					  }
};
TaskParameters_t xAssetBTaskParameters =
{
	.pvTaskCode		= prvProtectedStorageAssetBTask,
	.pcName			= "PSSAssetB",
	.usStackDepth	= configMINIMAL_STACK_SIZE,
	.pvParameters	= NULL,
	.uxPriority		= tskIDLE_PRIORITY,
	.puxStackBuffer	= xSecureSSTAssetBTaskStack,
	.xRegions		= {
						{
							( void* ) __tasks_share_mem_start__,			\
							ulSharedMemSize,								\
							tskMPU_REGION_READ_ONLY | tskMPU_REGION_EXECUTE_NEVER
						},
						{
							( void* ) MUSCA_B_NS_PERIPHERAL_REGION_START_ADDRESS,		\
							( uint32_t ) MUSCA_B_NS_PERIPHERAL_REGION_SIZE, 			\
							tskMPU_REGION_READ_WRITE | tskMPU_REGION_EXECUTE_NEVER |	\
							tskMPU_REGION_DEVICE_MEMORY
						},
						{ 0, 0, 0},
					}
};

	xTaskCreateRestricted( &( xAssetATaskParameters ), NULL );
	xTaskCreateRestricted( &( xAssetBTaskParameters ), NULL );
}
/*-----------------------------------------------------------*/

static void prvProtectedStorageAssetATask( void * pvParameters )
{
psa_ps_status_t uxStatus;
const psa_ps_uid_t uxUID = TEST_UID_A;
const psa_ps_create_flags_t uxFlags = PSA_PS_FLAG_NONE;
uint8_t ucWriteData[] = ASSET_A;
const uint32_t ulDataLength = ASSET_A_SIZE;
uint8_t ucReWriteData[] = RESETDATA;
const uint32_t ulResetDataLength = RESETDATA_SIZE;
uint8_t ucGetData[ READ_LENGTH ];
uint32_t ulCounter = 0;

	/* This task calls secure side functions. So allocate a
	secure context for it. */
	portALLOCATE_SECURE_CONTEXT( 0 );

	for( ; ; )
	{
		/* Call TF-M protected storage service and set the asset. */
		uxStatus = psa_ps_set( uxUID, ulDataLength, ucWriteData, uxFlags );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset A Set Round %ld :] Fail\r\n", ulCounter );
			for( ;; );
		}

		INFO_LOG( "[Protected Storage Asset A Set Round %ld :] Success\r\n", ulCounter );

		/* Read the asset. */
		uxStatus = psa_ps_get( uxUID, 0, ulDataLength, ucGetData );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Fail\r\n", ulCounter );
			for( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Success\r\n", ulCounter );

		/* Check the read data. */
		if( memcmp( ucWriteData, ucGetData, sizeof( ucWriteData ) - 1 ) != 0 )
		{
			INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Get the wrong data\r\n", ulCounter );
			for( ; ; );
		}

		/* Change the asset. */
		uxStatus = psa_ps_set( uxUID, ulResetDataLength, ucReWriteData, uxFlags );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset A Reset Round %ld :] Fail\r\n", ulCounter );
		}

		INFO_LOG( "[Protected Storage Asset A Reset Round %ld :] Success\r\n", ulCounter );

		/* Read the asset. */
		uxStatus = psa_ps_get( uxUID, 0, ulResetDataLength, ucGetData );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Fail\r\n", ulCounter );
			for( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Success\r\n", ulCounter );

		/* Check the read data. */
		if( memcmp( ucReWriteData, ucGetData, sizeof( ucReWriteData ) - 1 ) != 0 )
		{
			INFO_LOG( "[Protected Storage Asset A Get Round %ld :] Get the wrong data\r\n", ulCounter );
			for( ; ; );
		}

		/* Remove the asset. */
		uxStatus = psa_ps_remove( uxUID );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset A Remove Round %ld :] Fail\r\n", ulCounter );
			for( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset A Remove Round %ld :] Success\r\n\n", ulCounter );

		/* Wait for a second. */
		vTaskDelay( pdMS_TO_TICKS( 1000 ) );
		ulCounter++;
	}
}
/*-----------------------------------------------------------*/

static void prvProtectedStorageAssetBTask( void * pvParameters )
{
psa_ps_status_t uxStatus;
const psa_ps_uid_t uxUID = TEST_UID_B;
const psa_ps_create_flags_t uxFlags = PSA_PS_FLAG_NONE;
uint8_t ucWriteData[] = ASSET_B;
const uint32_t ulDataLength = ASSET_B_SIZE;
uint8_t ucGetDataLength = sizeof( ucWriteData );
uint8_t ucGetData[ ucGetDataLength ];
uint32_t ulCounter = 0;

	/* This task calls secure side functions. So allocate a
	secure context for it. */
	portALLOCATE_SECURE_CONTEXT( 0 );

	for( ; ; )
	{
		/* Call TF-M protected storage service and set the asset. */
		uxStatus = psa_ps_set( uxUID, ulDataLength, ucWriteData, uxFlags );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset B Set Round %ld :] Fail\r\n", ulCounter );
			for ( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset B Set Round %ld :] Success\r\n", ulCounter );

		/* Read the asset. */
		uxStatus = psa_ps_get( uxUID, 0, ulDataLength, ucGetData );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset B Get Round %ld :] Fail\r\n", ulCounter );
			for( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset B Get Round %ld :] Success\r\n", ulCounter );

		/* Check the read data. */
		if( memcmp( ucWriteData, ucGetData, sizeof( ucWriteData ) - 1 ) != 0 )
		{
			INFO_LOG( "[Protected Storage Asset B Get Round %ld :] Get the wrong data.\r\n" );
			for( ; ; );
		}

		/* Remove the asset. */
		uxStatus = psa_ps_remove( uxUID );
		if( uxStatus != PSA_PS_SUCCESS )
		{
			INFO_LOG( "[Protected Storage Asset B Remove Round %ld :] Fail\r\n", ulCounter );
			for ( ; ; );
		}

		INFO_LOG( "[Protected Storage Asset B Remove Round %ld :] Success\r\n", ulCounter );

		/* Wait for a second. */
		vTaskDelay( pdMS_TO_TICKS( 1000 ) );
		ulCounter++;
	}
}
/*-----------------------------------------------------------*/

/* Stack overflow hook. */
void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	/* Force an assert. */
	configASSERT( pcTaskName == 0 );
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that
 * is used by the Idle task. */
void vApplicationGetIdleTaskMemory(	StaticTask_t ** ppxIdleTaskTCBBuffer,
									StackType_t ** ppxIdleTaskStackBuffer,
									uint32_t * pulIdleTaskStackSize )
{
	/* If the buffers to be provided to the Idle task are declared inside this
	 * function then they must be declared static - otherwise they will be
	 * allocated on the stack and so not exists after this function exits. */
	static StaticTask_t xIdleTaskTCB;
	static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__((aligned(32)));

	/* Pass out a pointer to the StaticTask_t structure in which the Idle
	 * task's state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	 * Note that, as the array is necessarily of type StackType_t,
	 * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
									 StackType_t ** ppxTimerTaskStackBuffer,
									 uint32_t * pulTimerTaskStackSize )
{
	/* If the buffers to be provided to the Timer task are declared inside this
	 * function then they must be declared static - otherwise they will be
	 * allocated on the stack and so not exists after this function exits. */
	static StaticTask_t xTimerTaskTCB;
	static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ] __attribute__((aligned(32)));

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	 * task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	 * Note that, as the array is necessarily of type StackType_t,
	 * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/

void prvMemManageDebug( void )
{
	for( ; ; )
	{
	}
}
/*-----------------------------------------------------------*/

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
