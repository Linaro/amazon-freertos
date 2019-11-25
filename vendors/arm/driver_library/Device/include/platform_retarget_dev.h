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

/**
 * \file platform_retarget_dev.h
 * \brief The structure definitions in this file are exported based on the peripheral
 * definitions from device_cfg.h.
 * This retarget file is meant to be used as a helper for baremetal
 * applications and/or as an example of how to configure the generic
 * driver structures.
 */

/*
 * This file is derivative of TF-M platform/ext/target/musca_a/Device/Include/platform_retarget_dev.h
 * (TF-M commit 788bbc17465f7b97f11617b09d5fadecf8796b2a)
 */

#ifndef __ARM_LTD_MUSCA_B1_RETARGET_DEV_H__
#define __ARM_LTD_MUSCA_B1_RETARGET_DEV_H__

#include "device_cfg.h"

/* ======= Peripheral configuration structure declarations ======= */

/* ARM SCC driver structures */
#ifdef MUSCA_B1_SCC_NS
#include "musca_b1_scc_drv.h"
extern struct musca_b1_scc_dev_t MUSCA_B1_SCC_DEV_NS;
#endif

#ifdef MPC_CODE_SRAM_NS
#include "mpc_sie200_drv.h"
extern struct mpc_sie200_dev_t MPC_CODE_SRAM_DEV_NS;
#endif
#ifdef MPC_QSPI_NS
#include "mpc_sie200_drv.h"
extern struct mpc_sie200_dev_t MPC_QSPI_DEV_NS;
#endif

/* ARM UART PL011 driver structures */
#ifdef UART0_PL011_NS
#include "uart_pl011_drv.h"
extern struct uart_pl011_dev_t UART0_PL011_DEV_NS;
#endif
#ifdef UART1_PL011_NS
#include "uart_pl011_drv.h"
extern struct uart_pl011_dev_t UART1_PL011_DEV_NS;
#endif

/* CMSDK Timer driver structures */
#ifdef CMSDK_TIMER0_NS
#include "timer_cmsdk_drv.h"
extern struct timer_cmsdk_dev_t CMSDK_TIMER0_DEV_NS;
#endif

#ifdef CMSDK_TIMER1_NS
#include "timer_cmsdk_drv.h"
extern struct timer_cmsdk_dev_t CMSDK_TIMER1_DEV_NS;
#endif

/* QSPI Flash Controller driver structures */
#ifdef QSPI_IP6514E_NS
#include "qspi_ip6514e_drv.h"
extern struct qspi_ip6514e_dev_t QSPI_DEV_NS;
#endif

/* ======= External peripheral configuration structure declarations ======= */

/* MT25QL Flash memory library structures */
#if (defined(MT25QL_NS) && defined(QSPI_IP6514E_NS))
#include "Libraries/mt25ql_flash_lib.h"
extern struct mt25ql_dev_t MT25QL_DEV_NS;
#endif

#endif  /* __ARM_LTD_MUSCA_B1_RETARGET_DEV_H__ */
