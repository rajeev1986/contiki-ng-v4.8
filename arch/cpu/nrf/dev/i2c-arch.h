/*
 * Copyright (C)2022 Rajeev Piyare <rajeev@conexiotech.com>
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
 *         I2C header file for the nRF.
 * \author
 *         Rajeev Piyare <rajeev@conexiotech.com>
 *
 */
/*---------------------------------------------------------------------------*/
#ifndef I2C_ARCH_H
#define I2C_ARCH_H
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "nrfx_twim.h"
/*---------------------------------------------------------------------------*/
/**
 * \brief             One-time initialisation of the nRF I2C Driver
 * \retval nrfx_err_t nrfx_error_codes
 *
 * This function must be called before any other I2C driver calls.
 */
nrfx_err_t i2c_init(void);
/*---------------------------------------------------------------------------*/
/**
 * \brief             Perform a write-only I2C transaction.
 * \param  slave_addr The address of the slave device on the I2C bus
 * \param  wdata      Write data during the I2C transaction.
 * \param  wlen       Length of data to be written
 * \retval nrfx_err_t nrfx_error_codes
 */
nrfx_err_t i2c_write(uint8_t slave_addr, uint8_t *wdata, uint8_t wlen);
/*---------------------------------------------------------------------------*/
/**
 * \brief             Perform a read-only I2C transaction.
 * \param  slave_addr The address of the slave device on the I2C bus
 * \param  rdata      Read data during the I2C transaction.
 * \param  rlen       Length of data to be read
 * \retval nrfx_err_t nrfx_error_codes
 */
nrfx_err_t i2c_read(uint8_t slave_addr, uint8_t *rdata, uint8_t rlen);
/*---------------------------------------------------------------------------*/
/**
 * \brief             Write data to a specific register address
 * \param  slave_addr The address of the slave device on the I2C bus
 * \param  reg_addr   The address of the register to write to
 * \param  wdata      Write data during the I2C transaction.
 * \param  wlen       Length of data to be written
 * \retval nrfx_err_t nrfx_error_codes
 */
nrfx_err_t i2c_write_reg(uint8_t slave_addr, uint8_t reg_addr, uint8_t *wdata, uint8_t wlen);
/*---------------------------------------------------------------------------*/
/**
 * \brief             Read data from a specific register address
 * \param  slave_addr The address of the slave device on the I2C bus
 * \param  reg_addr   The address of the register to read from
 * \param  rdata      Read data during the I2C transaction.
 * \param  rlen       Length of data to be read
 * \retval nrfx_err_t nrfx_error_codes
 */
nrfx_err_t i2c_read_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t *rdata, uint8_t rlen);
/*---------------------------------------------------------------------------*/
#endif /* I2C_ARCH_H */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
