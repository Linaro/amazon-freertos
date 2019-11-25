/*
 * Copyright (c) 2018-2019 Arm Limited
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
 * This file is derivative of TF-M platform/ext/target/musca_a/CMSIS_Driver/Config/cmsis_driver_config.h
 * (TF-M commit 788bbc17465f7b97f11617b09d5fadecf8796b2a)
 */

#ifndef __CMSIS_DRIVER_CONFIG_H__
#define __CMSIS_DRIVER_CONFIG_H__

#include "device_cfg.h"
#include "platform_retarget_pins.h"
#include "platform_retarget_dev.h"

#define UART1_DEV           UART1_PL011_DEV_NS

#define MUSCA_B1_SCC_DEV    MUSCA_B1_SCC_DEV_NS

#endif  /* __CMSIS_DRIVER_CONFIG_H__ */
