/*
 * Copyright (C) 2022 Rajeev Piyare <rajeev@conexiotech.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup nrf
 * @{
 *
 * \addtogroup nrf-dev Device drivers
 * @{
 *
 * \addtogroup nrf-twim I2C driver
 * @{
 *
 * \file
 *         I2C implementation for the nRF.
 * \author
 *         Rajeev Piyare <rajeev@conexiotech.com>
 *
 */

/* 
 * Note: You can't use two components at the same time that share the same 
 * base address (UART and UARTE do, as well as TWI0, UARTE0 and SPI0 and some others as well). 
 * That should be explicitly outlined somewhere.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
/*---------------------------------------------------------------------------*/
#if PLATFORM_HAS_I2C 
/*---------------------------------------------------------------------------*/
#include "nrfx_config.h"
#include "nrfx_twim.h"
#include "hal/nrf_gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define NRF_I2C0_SDA NRF_GPIO_PIN_MAP(NRF_I2C0_SDA_PORT, NRF_I2C0_SDA_PIN)
#define NRF_I2C0_SCL NRF_GPIO_PIN_MAP(NRF_I2C0_SCL_PORT, NRF_I2C0_SCL_PIN)
/*---------------------------------------------------------------------------*/
static nrfx_twim_t i2c_instance = NRFX_TWIM_INSTANCE(1);
const nrfx_twim_config_t i2c_config =
{
	.scl 				=	NRF_I2C0_SCL,
	.sda				=	NRF_I2C0_SDA,
	.frequency			=	NRF_TWIM_FREQ_100K,
	.interrupt_priority	=	NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY,
	.hold_bus_uninit	=	false
};
volatile nrfx_twim_xfer_desc_t i2c_tx_desc;
volatile bool nrf_i2c_transfer_done = false;
volatile bool nrf_i2c_transfer_err = false;
/*---------------------------------------------------------------------------*/
void 
i2c_event_handler(nrfx_twim_evt_t const * p_event, void * p_context)
{
	switch(p_event->type)
		{
		case NRFX_TWIM_EVT_DONE: 
			nrf_i2c_transfer_done = true;
			//printf("NRFX_TWIM_EVT_DONE \n");
			break;
		case NRFX_TWIM_EVT_ADDRESS_NACK: 
			nrf_i2c_transfer_err = true;
			//printf("NRFX_TWIM_EVT_ADDRESS_NACK \n");
			break;
		case NRFX_TWIM_EVT_DATA_NACK: 
			nrf_i2c_transfer_err = true;
			//printf("NRFX_TWIM_EVT_DATA_NACK \n");
			break;
		case NRFX_TWIM_EVT_OVERRUN: 
			nrf_i2c_transfer_err = true;
			//printf("NRFX_TWIM_EVT_OVERRUN \n");
			break;
		case NRFX_TWIM_EVT_BUS_ERROR: 
			nrf_i2c_transfer_err = true;
			//printf("NRFX_TWIM_EVT_BUS_ERROR \n");
			break;
		default:
			//printf("EVNT HANDLER NONE OF THE ABOVE \n");
			break;
		}
}
/*---------------------------------------------------------------------------*/
static void 
i2c_wait(void)
{
	while(nrf_i2c_transfer_done == false){
		if (nrf_i2c_transfer_err == true)
		{
			// printf("I2C TRANSFER ERROR \n");
			nrf_i2c_transfer_err = false;
			break;
		}
	}
	nrf_i2c_transfer_done = false;
}
/*---------------------------------------------------------------------------*/
void 
i2c_write(uint8_t address,uint8_t reg_address,uint8_t *data,uint8_t len)
{
	nrfx_err_t err_code;
	uint8_t buffer[len + 1];

	buffer[0] = reg_address;
	memcpy(&buffer[1], data, len);

	nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(address,buffer,len+1);
	err_code = nrfx_twim_xfer(&i2c_instance, &write_desc, 0);

	if(err_code != NRFX_SUCCESS) {
		return;
	}
	i2c_wait();
}
/*---------------------------------------------------------------------------*/
void 
i2c_read(uint8_t address,uint8_t reg_address,uint8_t *data,uint8_t len)
{
	nrfx_err_t err_code;

	nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(address,&reg_address,1,data,len);
	err_code = nrfx_twim_xfer(&i2c_instance, &read_desc, 0);

	if(err_code != NRFX_SUCCESS) {
		return;
	}
	i2c_wait();
}
/*---------------------------------------------------------------------------*/
void 
i2c_init(void)
{
	nrfx_err_t err_code;

	nrf_gpio_cfg(i2c_config.sda, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
					NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

	nrf_gpio_cfg(i2c_config.scl, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,
					NRF_GPIO_PIN_PULLUP,NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

	err_code = nrfx_twim_init(&i2c_instance, &i2c_config, i2c_event_handler, NULL);

	if (err_code == NRFX_SUCCESS)
	{
		nrfx_twim_enable(&i2c_instance);
	}
}
/*---------------------------------------------------------------------------*/
#endif /* PLATFORM_HAS_I2C */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */