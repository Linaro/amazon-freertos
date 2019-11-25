/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Copyright (c) 2019 Arm Limited. All Rights Reserved.
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
 * This file is derivative of FreeRTOS 10.2.1
 * FreeRTOS\Source\portable\GCC\ARM_CM33\secure\secure_context.c
 */

/* Trustzone context management includes. */
#include "tz_context_mgmt.h"

/*-----------------------------------------------------------*/

void SecureContext_Init( void )
{
    /* Require Secure state to initialize secure context system */
    TZ_InitContextSystem_S();
}
/*-----------------------------------------------------------*/

SecureContext_t SecureContext_AllocateContext( TZ_ModuleId_t xModule )
{
    SecureContext_t xSecureCtx = INVALID_SECURE_CONTEXT_VAL;

    /* Require Secure state to allocate the secure context. */
    xSecureCtx = TZ_AllocModuleContext_S( xModule );

    return xSecureCtx;
}
/*-----------------------------------------------------------*/

void SecureContext_FreeContext( SecureContext_t xSecureCtx )
{
    /* Ensure that valid parameters are passed. */
    configASSERT( xSecureCtx != INVALID_SECURE_CONTEXT_VAL );

    /* Require Secure state to free the secure context */
    TZ_FreeModuleContext_S( xSecureCtx );
}
/*-----------------------------------------------------------*/

void SecureContext_LoadContext( SecureContext_t xSecureCtx )
{
    /* Ensure that valid parameters are passed. */
    configASSERT( xSecureCtx != INVALID_SECURE_CONTEXT_VAL );

    /* Indicate Secure state to load the next secure context */
    TZ_LoadContext_S( xSecureCtx );
}
/*-----------------------------------------------------------*/

void SecureContext_SaveContext( SecureContext_t xSecureCtx )
{
    /* Ensure that valid parameters are passed. */
    configASSERT( xSecureCtx != INVALID_SECURE_CONTEXT_VAL );

    /* Indicate Secure state to save the next secure context */
    TZ_StoreContext_S ( xSecureCtx );
}
/*-----------------------------------------------------------*/
