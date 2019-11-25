/*
 * Copyright (c) 2017-2019 Arm Limited
 *
 * Licensed under the Apache License Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing software
 * distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * This file is derivative of TF-M platform/ext/target/musca_a/Device/Config/device_cfg.h
 * (TF-M commit 788bbc17465f7b97f11617b09d5fadecf8796b2a)
 */

#ifndef __ARM_LTD_DEVICE_CFG_H__
#define __ARM_LTD_DEVICE_CFG_H__

/**
 * \file device_cfg.h
 * \brief Configuration file native driver re-targeting
 *
 * \details This file can be used to add native driver specific macro
 *          definitions to select which peripherals are available in the build.
 *
 * This is a default device configuration file with all peripherals enabled.
 */

/*ARM UART Controller PL011*/
#define UART1_PL011_NS

#endif  /* __ARM_LTD_DEVICE_CFG_H__ */
