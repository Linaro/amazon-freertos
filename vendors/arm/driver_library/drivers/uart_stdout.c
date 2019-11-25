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
 * This file is derivative of TF-M platform/ext/common/uart_stdout.c
 * (TF-M commit 788bbc17465f7b97f11617b09d5fadecf8796b2a)
 */

#include <stdio.h>
#include <stddef.h>

#include "Driver_USART.h"

#include "uart_stdout.h"

/* Imports NS_DRIVER_STDOUT from platfrom specific configuration */
extern ARM_DRIVER_USART NS_DRIVER_STDOUT;
#define pUSART			( &NS_DRIVER_STDOUT )

FILE __stdout;

static int32_t prvUART_putc( unsigned char ch )
{
	return pUSART->Send( &ch, 1 );
}

#if defined ( __ARMCC_VERSION )
/* redirect in ARM CC. However, only Arm compiler version 6 is supported here.
 * Not sure about how Arm CC version 4/5 requires.
 */
int fputc( int ch, FILE *f )
{
int32_t lRet;

	lRet = prvUART_putc( ch );

	if( lRet != ARM_DRIVER_OK )
	{
		return EOF;
	}

	/* Return the ch itself */
	return ch;
}

#elif defined ( __GNUC__ )
/* redirect in ARM GCC */
int _write( int fd, const void *buf, size_t count )
{
size_t i = 0;
int32_t lRet;
const char *str = ( const char * )buf;

	if( !buf )
	{
		return 0;
	}

	for( i = 0; i < count; i++ )
	{
		/* Send a single char each time */
		lRet = prvUART_putc( str[ i ] );
		if( lRet != ARM_DRIVER_OK )
		{
			break;
		}
	}

	/* Return the actual number of char have been written */
	return i;
}

/*
 * FIXME
 * This _write_r() actually cannot fully support reentrancy.
 * It requires to enchance the full UART stack to support reentrancy.
 */
int _write_r( void *reent, int fd, const void *buf, size_t count )
{
size_t i = 0;
int32_t lRet;
const char *str = ( const char * )buf;

	if( !buf )
	{
		return 0;
	}

	for( i = 0; i < count; i++ )
	{
		/* Send a single char each time */
		lRet = prvUART_putc( str[ i ] );
		if( lRet != ARM_DRIVER_OK )
		{
			break;
		}
	}

	/* Return the actual number of char have been written */
	return i;
}
#endif

int stdout_init( void )
{
int32_t lRet;

	lRet = pUSART->Initialize( NULL );
	if( lRet != ARM_DRIVER_OK )
	{
		return -1;
	}

	lRet = pUSART->Control( USART_OUT_MODE, USART_OUT_BAUDRATE );
	if( lRet != ARM_DRIVER_OK )
	{
		return -1;
	}

	return 0;
}
