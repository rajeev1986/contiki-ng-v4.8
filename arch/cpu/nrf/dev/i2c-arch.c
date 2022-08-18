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
 * \addtogroup nrf-twim (two-wire interface master with EasyDMA) I2C driver
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

#include <stdbool.h>
#include <string.h> /* For memcpy() */
/*---------------------------------------------------------------------------*/
#define NRF_I2C1_SDA NRF_GPIO_PIN_MAP(NRF_I2C1_SDA_PORT, NRF_I2C1_SDA_PIN)
#define NRF_I2C1_SCL NRF_GPIO_PIN_MAP(NRF_I2C1_SCL_PORT, NRF_I2C1_SCL_PIN)
/*---------------------------------------------------------------------------*/
static nrfx_twim_t i2c_instance = NRFX_TWIM_INSTANCE(1);
static nrfx_twim_config_t i2c_config =  NRFX_TWIM_DEFAULT_CONFIG(NRF_I2C1_SCL, NRF_I2C1_SDA);
/*---------------------------------------------------------------------------*/
volatile nrfx_twim_xfer_desc_t i2c_tx_desc;
volatile bool nrf_i2c_transfer_done = false;
volatile bool nrf_i2c_transfer_err = false;
/*---------------------------------------------------------------------------*/
static void 
i2c_event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
	switch(p_event->type) {
		case NRFX_TWIM_EVT_DONE: 
			nrf_i2c_transfer_done = true;
			break;
		case NRFX_TWIM_EVT_ADDRESS_NACK: 
			nrf_i2c_transfer_err = true;
			break;
		case NRFX_TWIM_EVT_DATA_NACK: 
			nrf_i2c_transfer_err = true;
			break;
		case NRFX_TWIM_EVT_OVERRUN: 
			nrf_i2c_transfer_err = true;
			break;
		case NRFX_TWIM_EVT_BUS_ERROR: 
			nrf_i2c_transfer_err = true;
			break;
		default:
			break;
	}
}
/*---------------------------------------------------------------------------*/
static void 
i2c_wait(void)
{
	while(nrf_i2c_transfer_done == false) {
		if (nrf_i2c_transfer_err == true)
		{
			nrf_i2c_transfer_err = false;
			break;
		}
	}
	nrf_i2c_transfer_done = false;
}
/*---------------------------------------------------------------------------*/
void 
i2c_write(uint8_t slave_addr, uint8_t reg_addr,uint8_t *wdata, uint8_t wlen)
{
	nrfx_err_t err_code;
	uint8_t buffer[wlen + 1];

	buffer[0] = reg_addr;
	memcpy(&buffer[1], wdata, wlen);

	nrfx_twim_xfer_desc_t write_desc = NRFX_TWIM_XFER_DESC_TX(slave_addr, buffer, wlen + 1);
	err_code = nrfx_twim_xfer(&i2c_instance, &write_desc, 0);

	if(err_code != NRFX_SUCCESS) {
		return;
	}
	i2c_wait();
}
/*---------------------------------------------------------------------------*/
void 
i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *rdata, uint8_t rlen)
{
	nrfx_err_t err_code;

	nrfx_twim_xfer_desc_t read_desc = NRFX_TWIM_XFER_DESC_TXRX(slave_addr, &reg_addr, 1, rdata, rlen);
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
    
	err_code = nrfx_twim_init(&i2c_instance, &i2c_config, i2c_event_handler, NULL);

	if(err_code != NRFX_SUCCESS) {
		return;
	}

	nrfx_twim_enable(&i2c_instance);
}
/*---------------------------------------------------------------------------*/
#endif /* PLATFORM_HAS_I2C */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */