/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (c) 2019-2021 Arm Limited. All rights reserved.
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

/* Arm Musca_b1 platform log print. */
#include "print_log.h"

/* Logging task. */
#include "iot_logging_task.h"

/* Arm Musca_b1 platform hardware setup. */
#include "hardware_setup.h"

/* TF-M platform service. */
#include "musca_b1_scc_drv.h"
#include "tfm_ioctl_api.h"

/* Wifi and secure socket libray. */
#include "iot_wifi.h"
#include "esp/esp_private.h"
#include "iot_secure_sockets.h"

/* Credentials and their provisioning. */
#include "aws_clientcredential.h"
#include "aws_dev_mode_key_provisioning.h"

/* TF-M Firmware Update service. */
#include "psa/update.h"

/* Application version. */
#include "application_version.h"
#include "aws_demo.h"

/* TF-M Crypto service. */
#include "psa/crypto.h"

/* Code verify key provisioning. */
#include "ota_provision.h"
#include "tfm_ns_interface.h"

#define mainLOGGING_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE * 8 )
#define mainLOGGING_MESSAGE_QUEUE_LENGTH ( 20 )
#define wifiCONNECTION_DELAY    pdMS_TO_TICKS( 1000 )

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
const uint32_t * __privileged_functions_start__    = ( uint32_t * ) &( Image$$ER_IROM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_functions_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in privileged Flash region. */

/* Flash containing system calls. */
const uint32_t * __syscalls_flash_start__    = ( uint32_t * ) &( Image$$ER_IROM_NS_FREERTOS_SYSTEM_CALLS$$Base );
const uint32_t * __syscalls_flash_end__        = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_FREERTOS_ALIGN$$Limit ) - 0x1 );  /* Last address in Flash region containing system calls. */

/* Unprivileged flash. Note that the section containing
 * system calls is unprivilged so that unprivleged tasks
 * can make system calls. */
const uint32_t * __unprivileged_flash_start__    = ( uint32_t * ) &( Image$$ER_IROM_NS_UNPRIVILEGED$$Base );
const uint32_t * __unprivileged_flash_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IROM_NS_UNPRIVILEGED_ALIGN$$Limit ) - 0x1 );  /* Last address in un-privileged Flash region. */

/* Priviledge ram. This contains kernel data. */
const uint32_t * __privileged_sram_start__    = ( uint32_t * ) &( Image$$ER_IRAM_NS_PRIVILEGED$$Base );
const uint32_t * __privileged_sram_end__    = ( uint32_t * ) (( uint32_t )&( Image$$IRAM_NS_PRIVILEGED_ALIGN$$Limit ) - 0x1 ); /* Last address in privileged RAM. */

/* The key for OTA code verification. */
psa_key_handle_t ota_code_verify_key_handle = NULL;

/*-----------------------------------------------------------*/

/**
 * @brief Creates all the tasks for this demo.
 */
static void prvCreateTasks( void );

/* Non-secure main() */
int main( void )
{
BaseType_t xRet;
    xRet = xHardwareSetup();
    configASSERT( xRet == pdPASS );

    /* Initialise the UART lock. */
    vUARTLockInit();

    /* Initialize the secure interface lock */
    tfm_ns_interface_init();

    /* Create tasks. */
    prvCreateTasks();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Should not reach here as the scheduler is already started. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

static void prvCreateTasks( void )
{
    xLoggingTaskInitialize( mainLOGGING_TASK_STACK_SIZE,
                            (tskIDLE_PRIORITY + 2) | portPRIVILEGE_BIT,
                            mainLOGGING_MESSAGE_QUEUE_LENGTH );
}
/*-----------------------------------------------------------*/

void vApplicationDaemonTaskStartupHook( void )
{
enum tfm_platform_err_t xErr;
uint32_t ulReqResult;
WIFIReturnCode_t xWifiStatus;
WIFINetworkParams_t xNetworkParams;
BaseType_t xResult = pdPASS;

xNetworkParams.xSecurity = clientcredentialWIFI_SECURITY;

xNetworkParams.ucSSIDLength = strlen( clientcredentialWIFI_SSID );
memcpy( xNetworkParams.ucSSID, clientcredentialWIFI_SSID, xNetworkParams.ucSSIDLength );

xNetworkParams.xPassword.xWPA.ucLength = strlen( clientcredentialWIFI_PASSWORD );
memcpy( xNetworkParams.xPassword.xWPA.cPassphrase, clientcredentialWIFI_PASSWORD, xNetworkParams.xPassword.xWPA.ucLength );

    /* This task calls secure side functions. So allocate a
     * secure context for it. */
    portALLOCATE_SECURE_CONTEXT( 0 );

    /* This demo is to update the Non-Secure image. */
    configASSERT( GetImageVersionPSA(FWU_IMAGE_TYPE_NONSECURE) == 0 )

    /* TODO: currently the ns_lock mutex only support calling TF-M
     * services in the task. Move the GPIO set to the hardwaresetup
     * after this restriction is removed.
     */
    /* set GPIO 0 & 1 as UART */
    xErr = tfm_platform_set_pin_alt_func( 1, 0b11, &ulReqResult );
    if( xErr != TFM_PLATFORM_ERR_SUCCESS )
    {
        xResult = pdFAIL;
    }
    configASSERT( xResult == pdPASS );

    /* Provision the credentials for TLS connection. */
    vDevModeKeyProvisioning();

    /* Provision the key which is used to verify the image in OTA. */
    xResult = ota_provision_code_verify_key( &ota_code_verify_key_handle );
    configASSERT( xResult == 0 );

    /* Turn on wifi and connect to the access point as a station. */
    xWifiStatus = WIFI_On();
    configASSERT( xWifiStatus == eWiFiSuccess );
    xWifiStatus = WIFI_ConnectAP( &xNetworkParams );
    if( xWifiStatus != eWiFiSuccess )
    {
        print_log( "WIFI_ConnectAP failed" );
    }

    /* The AT command is sent by the uart task which has a low priority
     * than this task, so delay some time to yield.
      */
    vTaskDelay( wifiCONNECTION_DELAY );

    /* Disable sleep mode so that the wifi connect can keep alive
     * during the TLS connect process. This is to avoid the bug in
     * ESP8266.
     */
    ESP_AT_PORT_SEND_BEGIN();
    ESP_AT_PORT_SEND_STR( "+SLEEP=0" );
    ESP_AT_PORT_SEND_END();
    vTaskDelay( wifiCONNECTION_DELAY );

    SOCKETS_Init();

    /* Run the demo. Only the OTA demo is enabled in the configuration. */
    DEMO_RUNNER_RunDemos();
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
        " .align 2                                                    \n"
        " handler2_address_const: .word prvMemManageDebug           \n"
    );
}
/*-----------------------------------------------------------*/
