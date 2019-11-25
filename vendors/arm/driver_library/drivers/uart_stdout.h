/*
 * Copyright (c) 2018-2019 Arm Limited. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This file is derivative of TF-M platform/ext/common/uart_stdout.h
 * (TF-M commit 788bbc17465f7b97f11617b09d5fadecf8796b2a)
 */

#ifndef _UART_STDOUT_H_
#define _UART_STDOUT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Include the platform specific UART definitions */
#include "uart_musca_b.h"

/*
 * \brief Initialize stdout device.
 * \details Configure and enable UART device.
 * \return	0 initialization succeeded.
 * \return	1 initialization failed.
 */
int stdout_init( void );

#ifdef __cplusplus
}
#endif

#endif /* _UART_STDOUT_H_ */
