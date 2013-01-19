
/**
 * @file   counter.c
 * @author Rohit Grover, Dyuman Mezzetti, Chandran Goodchild
 * @date   Mon Nov 28 2011
 *
 * @brief This program drives the Financial Service Token System.
 *
 * Copyright 2011-2020 Rohit Grover, Dyuman Mezzetti, Chandran
 * Goodchild. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY Rohit, Chandran and Dyuman ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rohit, Chandran or Dyuman BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE
 */

#include <util/delay.h>
#include "USI_TWI_Master.h"
#include "USI_TWI_Slave.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <stdbool.h>


/* All definitions */
#define F_CPU 1000000         /* Define the CPU clockrate--this plays a role */
                              /* when using the delay loop.  */

#define BAUD  9600
#include <util/setbaud.h>

#define TIMER_SNAPSHOT_MS()   ((ticks << 8) | TCNT0)
#define DEBOUNCE_DELAY        1500
#define CUR_NUM_CMD           0xF2
#define CUR_NUM_CMD_SIZE      3
#define UPPER_RANGE_CMD       0xF3
#define UPPER_RANGE_CMD_SIZE  3
#define RETRANSMIT_COUNT      25
#define TWI_BROADCAST_ADDRESS (0x0)

volatile uint32_t ticks;


ISR(TIMER0_OVF_vect)
{
  ticks++;
}


void
timer_init(void)
{
  TCCR0B = _BV(CS02) | _BV(CS00); /* set prescaler to use clock/1024  */
  TIMSK  = _BV(TOIE0);   /* enable the overflow interrupt */

  ticks = 0;
}

/* This function transmits the data in the byte argument. */
void
sendUart(uint8_t byte)
{
  while (!(UCSRA & (1 << UDRE)))
    ;               /* spin while UDRE is not set */
  UDR = byte;
}


#if THE_FOLLOWING_ARE_PLACEHOLDERS_BECOME_NECESSARY
/* Inform the other modules that we are now serving currentNumber */
void
sendUpdate(uint8_t currentNumber)
{
    USI_TWI_Master_Initialise();

    uint8_t buffer[CUR_NUM_CMD_SIZE];
    buffer[0] =
        (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
        (FALSE << TWI_READ_BIT); /* write operation */
    buffer[1] = CUR_NUM_CMD;   /* we are transmitting the current number */
    buffer[2] = currentNumber;

    /* Remember that you are only transmitting it 25 times, and that you can
     * transmit multiple times if it makes trouble... */
    for (int i = 0; i < RETRANSMIT_COUNT; i++) {
        USI_TWI_Start_Transceiver_With_Data(buffer, sizeof(buffer));
    }

    /* reinitialize ourself as a slave */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
}

/* inform the other modules that we dispensed currentNumber */
void sendUpdateUpperRange(uint8_t currentNumber)
{

    USI_TWI_Master_Initialise();

    uint8_t buffer[UPPER_RANGE_CMD_SIZE];
    buffer[0] =
        (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
        (FALSE << TWI_READ_BIT); /* write operation */
    buffer[1] = UPPER_RANGE_CMD; /* we are transmitting the dispensed number */
    buffer[2] = currentNumber;

    /* Remember that you are only transmitting it 10 times, and that you can
     * transmit multiple times if it makes trouble... */
    for (int i = 0; i < RETRANSMIT_COUNT; i++) {
        USI_TWI_Start_Transceiver_With_Data(buffer, sizeof(buffer));
    }

    /* reinitialize ourself as a slave */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
}
#endif /* #if THE_FOLLOWING_ARE_PLACEHOLDERS_BECOME_NECESSARY */


/* Initialize I/O pins and find out which mode we operate as */
void
ioinit()
{
    /* setup the outputs for the two digits and the communication LED */
    DDRA |= _BV(1);
    DDRB |= _BV(0) | _BV(1) | _BV(2) | _BV(3);
    DDRD |= _BV(0) | _BV(1) | _BV(2) | _BV(4);
    DDRD |= _BV(5);

    /* set the baud rate */
    UBRRH = UBRRH_VALUE;
    UBRRL = UBRRL_VALUE;
#if USE_2X
    UCSRA |= (1 << U2X);
#else
    UCSRA &= ~(1 << U2X);
#endif

    /* Set frame format: 8data, no parity, 1stop bit */
    UCSRC = _BV(UCSZ1) | _BV(UCSZ0);

    /* Enable the transmitter */
    UCSRB =
        /* _BV(RXEN)  | */
        /* _BV(RXCIE) | */
        _BV(TXEN);

    sei();                      // turn on interrupts!
}

int
main(void)
{
    ioinit();
    timer_init();

    /* all devices initialize themselves as slaves */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);

    /* This is for testing the UART */
    uint8_t counter = 0;
    while (true) {
        ++counter;
        sendUart('a' + (counter % 26));
        _delay_ms(1000);
    }

    /* /\* loop forever *\/ */
    /* for (;; ) { */
    /*     uint8_t byte = USI_TWI_Receive_Byte(); */
    /*     sendUart(byte); */
    /* } */

    /* we will never come here. */
    return (0);
}
