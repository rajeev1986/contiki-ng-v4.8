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
#include "sys/log.h"
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
#define LOG_MODULE "Stratus"
#define LOG_LEVEL LOG_LEVEL_DBG
/*---------------------------------------------------------------------------*/
/* 1 if STRATUS_SHIELD_CONNECTED else 0 */
#define STRATUS_SHIELD_CONNECTED    0
/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  while(1) {
    LOG_INFO("Hello from Conexio Stratus\n");

    /* Setup a periodic timer that expires after 2 seconds. */
    etimer_set(&timer, CLOCK_SECOND * 2);

    /* Wait for the periodic timer to expire. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    /* Turn ON BLUE LED on Stratus DK. */
    leds_on(LEDS_BLUE);

#if STRATUS_SHIELD_CONNECTED
    /* Turn ON Orange LED on Stratus Shield. */
    leds_on(LEDS_ORANGE);
#endif
    
    /* Setup a periodic timer that expires after a second. */
    etimer_set(&timer, CLOCK_SECOND);
    /* Wait for the periodic timer to expire. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    
    /* Turn OFF BLUE LED on Stratus DK. */
    leds_off(LEDS_BLUE);

#if STRATUS_SHIELD_CONNECTED
    /* Turn OFF Orange LED on Stratus Shield. */
    leds_off(LEDS_ORANGE);
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
