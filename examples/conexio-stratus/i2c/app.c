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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         	A very simple Contiki application showing how to use printf() 
 * 			and blink LEDs on the Conexio Stratus dev kit.
 * \author
 *         	Rajeev Piyare <rajeev@conexiotech.com>
 */

#include "contiki.h"
#include <sys/clock.h>
#include "sys/etimer.h"
#include "dev/leds.h"

#include "i2c-arch.h"
#include "gpio-hal-arch.h"

#include "sys/log.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
#define LOG_MODULE "app"
#define LOG_LEVEL LOG_LEVEL_DBG
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "app process");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
#define SENSOR_POWER_PIN    11 /* ENABLE_3V3_SENSOR --> i2c sensors  */
#define BLUE_LED_PIN        3
#define LIS2DH_ADD          0x18
#define LIS2DH_REG_WAI	    0x0f
#define LIS2DH_CHIP_ID	    0x33
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data)
{
  static struct etimer timer;
  uint8_t id;

  PROCESS_BEGIN();

  LOG_INFO("testing I2C\n");

  /* Turn on the power to the sensors */
  gpio_hal_arch_write_pin(0, SENSOR_POWER_PIN, 0);

  while(1) {
    /* Setup a periodic timer that expires after 2 seconds. */
    etimer_set(&timer, CLOCK_SECOND * 2);
    /* Wait for the periodic timer to expire. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    i2c_read(LIS2DH_ADD, LIS2DH_REG_WAI, &id, 1);

    if (id != LIS2DH_CHIP_ID) {
		LOG_ERR("Invalid chip ID: %02x\n", id);
    } else {
      LOG_INFO("chip ID: %02x\n", id);
    }

    gpio_hal_arch_write_pin(0, BLUE_LED_PIN, 0);

    /* Setup a periodic timer that expires after a second. */
    etimer_set(&timer, CLOCK_SECOND);
    /* Wait for the periodic timer to expire. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    gpio_hal_arch_write_pin(0, BLUE_LED_PIN, 1);
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
