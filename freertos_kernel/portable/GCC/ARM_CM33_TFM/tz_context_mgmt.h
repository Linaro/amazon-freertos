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
 * FreeRTOS\Source\portable\GCC\ARM_CM33\secure\secure_context.h
 */

#ifndef __TZ_CONTEXT_MGMT_H__
#define __TZ_CONTEXT_MGMT_H__

/* Standard includes. */
#include <stdint.h>

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE
#include "FreeRTOS.h"
#include "task.h"
#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Include CMSIS Context Management for TrustZone on Armv8-M */
#include "tz_context.h"

/**
 * @brief the invalid SecureContext_t value.
 */
#define INVALID_SECURE_CONTEXT_VAL       0x0

/**
 * Opaque handle.
 */
typedef TZ_MemoryId_t SecureContext_t;

/*-----------------------------------------------------------*/

/**
 * @brief Initialize the secure context management system.
 *
 * Must be called in Handler mode.
 * The specific initialization is implemented in Secure state.
 */
void SecureContext_Init( void ) PRIVILEGED_FUNCTION;

/**
 * @brief Allocates a context on the secure side.
 *
 * Must be called in Handler mode.
 *
 * @param[in] xModule describes the set of secure services that are clled by
 *            the non-secure side.
 *            Set the module to zero if no secure calls are used. This leads
 *            to secure context allocation only without further secure service
 *            execution, which should be used at the beginning of
 *            a non-secure thread.
 *
 * @return Opaque context ID if context is successfully allocated,
 *         NULL otherwise.
 */
SecureContext_t SecureContext_AllocateContext( TZ_ModuleId_t xModule );

/**
 * @brief Frees the given context.
 *
 * Must be called in Handler mode.
 *
 * @param[in] xSecureCtx Context ID corresponding to the
 *            context to be freed.
 */
void SecureContext_FreeContext( SecureContext_t xSecureCtx );

/**
 * @brief Loads the given context.
 *
 * Must be called in the Handler mode.
 *
 * @param[in] xSecureCtx Context ID corresponding to the
 *            context to be loaded.
 */
void SecureContext_LoadContext( SecureContext_t xSecureCtx ) PRIVILEGED_FUNCTION;

/**
 * @brief Saves the given context.
 *
 * Must be called in the Handler mode.
 *
 * @param[in] xSecureCtx Context ID corresponding to the
 *            context to be saved.
 */
void SecureContext_SaveContext( SecureContext_t xSecureCtx ) PRIVILEGED_FUNCTION;

#endif /* __TZ_CONTEXT_MGMT_H__ */
