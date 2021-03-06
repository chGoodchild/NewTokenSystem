
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
 * THIS SOFTWARE IS PROVIDED BY Rohit, Chandran and Dyuman ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rohit,
 * Chandran or Dyuman BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE
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
#define TIMER_SNAPSHOT_MS()   ((ticks << 8) | TCNT0)
#define DEBOUNCE_DELAY        1500

typedef enum {
    CUR_NUM_CMD     = (uint8_t)0xF2,
    UPPER_RANGE_CMD = (uint8_t)0xF3
} CommandType;
#define CMD_SIZE             4
#define CMD_RETRANSMIT_COUNT 2

#define TWI_BROADCAST_ADDRESS (0x0)

uint8_t  lowerRange;     /* The latest number that has been served. */
uint8_t  upperRange;     /* The last number that was dispensed. */
uint8_t  mode;
uint8_t  mode2;                 /* mode variables */
volatile bool     buttonPressed = false;
volatile uint32_t ticks;
volatile uint32_t buttonPressSnapshot;
uint8_t  lastToken;    /* the number that we displayed last, (previous
                        * number for the PWM). */


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


/* Starts the whole button press action */
ISR(INT1_vect)
{
    cli();
    if ((TIMER_SNAPSHOT_MS() - buttonPressSnapshot) >= DEBOUNCE_DELAY) {
        buttonPressSnapshot = TIMER_SNAPSHOT_MS();
        buttonPressed       = true;
    }
    sei();
}

/**
 * Broadcast a command to all peers on the I2C bus.
 *
 * @param[in] cmd
 * @param[in] prevNumber
 *                The previous value of the token. This is sent out
 *                only to cause messages to be unique; it helps
 *                generate collisions.
 * @param[in] newNumber
 *                The actual new value of the token based on the command.
 *
 * @return
 *     'true' if at least one of the broadcasts was successful. The
 *     typical cause for failure would be a multi-master collision in
 *     the case where multiple counter modules decide to send their
 *     counter updates at the same time; i.e. if their buttons are
 *     pressed at roughly the same time.
 */
bool
sendUpdate(CommandType cmd, uint8_t prevNumber, uint8_t newNumber)
{
    USI_TWI_Master_Initialise();

    uint8_t buffer[CMD_SIZE];
    buffer[0] =
        (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
        (FALSE << TWI_READ_BIT); /* write operation */
    buffer[1] = (uint8_t)cmd;
    buffer[2] = prevNumber;     /* We send the previous number along
             * with the command to make every broadcast unique--if we
             * did not do so then there would be no difference in the
             * transmissions between two counter moudules wanting to
             * update the current-token at nearly the same time.
             *
             * Sending the previous number is like sending the
             * identity of the transmitter; and it helps generate
             * collisions when multiple counter-modules choose to send
             * out the CUR_NUM command at the same time. Collisions
             * can be detected by all but one of the transmitters; and
             * hence all but one of them back off and retry later. */
    buffer[3] = newNumber;

    bool transmitted = false;
    for (int i = 0; i < CMD_RETRANSMIT_COUNT; i++) {
        if (USI_TWI_Start_Transceiver_With_Data(buffer, sizeof(buffer))) {
            transmitted = true; /* transmitted at least once */
        } else {
            if (COLLISION_DETECTED()) {
                return (false); /* we've noticed a transmit collision;
                     * this arises when two distinct masters attempt
                     * to send on the I2C at the same time. All but
                     * one of the colliding transmitters will notice
                     * the collision and return false from this
                     * function. */
            }
        }
    }

    /* reinitialize ourself as a slave */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
    return (transmitted);
}

/* This function writes to the display decoders. */
void
writeDecoder(uint8_t tens, uint8_t unit)
{
    // set the tens digit
    PORTD &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(4)); /* begin with
                                                    * zeroing out all
                                                    * pins attached to
                                                    * the tens decoder */
    PORTD |= (tens & 0x07);
    /* We are manually setting the fourth bit because the bits in
     * PORTD aren't arranged consecutively */
    if ((tens == 8) || (tens == 9)) {
        PORTD |= _BV(4);
    }

    // set the units digit
    PORTB &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3));
    PORTB |= (unit & 0x0f);
}

/* Initialize I/O pins and find out which mode we operate as */
void
ioinit()
{
    /* setup the outputs for the two digits and the communication LED */
    DDRA |= _BV(1);
    DDRB |= _BV(0) | _BV(1) | _BV(2) | _BV(3);
    DDRD |= _BV(0) | _BV(1) | _BV(2) | _BV(4);
    DDRD |= _BV(5);

    /* set the BL pin to ground */
    PORTD &= ~_BV(5);

    PORTB |= _BV(4); /* turn on the internal pull-up for DIP-switch */
    PORTB |= _BV(6);

    /* find out the required mode of operation */
    mode = 0;
    mode = (PINB & _BV(4));
    mode2 = 0;
    mode2 = (PINB & _BV(6));

    /* If both DIP-switches are on, stop execution of the code and
     * flash 88 on the display to indicate an error condition. */
    if (mode && mode2) {
        while(1) {
            writeDecoder(8, 8);
            PORTD |= _BV(5);
            _delay_ms(750);
            PORTD &= ~_BV(5);
            _delay_ms(750);
        }
    }

    if (mode || mode2) {             /* this is a module which requires
                                      * input from the button. */

        PORTD |= _BV(3); // enable pull-up for button pin

        /* enable interrupt for the button */
        MCUCR |= _BV(ISC11); /* only the falling edge generates an interrupt */
        GIMSK |= _BV(INT1);
    }

    writeDecoder(0, 0);    /* make sure we always start by displaying 0 */

    sei();                  // turn on interrupts!
}

/* This function displays currentNumber on the display */
void
displayNumber(uint8_t currentNumber, uint8_t lastNumber)
{
    uint8_t tens;
    uint8_t unit;
    uint8_t tens1;
    uint8_t unit1;
    int     other = 0;

    // divide the number to be displayed into tens and units
    tens = currentNumber / 10;
    unit = currentNumber % 10;

    // we cannot display greater than 99
    if (tens > 9) {
        tens = tens % 10;
    }

    // divide the previously displayed number into tens and units
    tens1 = lastNumber / 10;
    unit1 = lastNumber % 10;

    // we cannot display greater than 99
    if (tens1 > 9) {
        tens1 = tens1 % 10;
    }

    /* This loop will keep on writing to the decoders, alternating between
     * the new number to be displayed and the old one, this gives a fading
     * effect on the display. */
    for (int i = 100; i>0; i--) {
        // write the new number
        writeDecoder(tens, unit);

        // wait a bit
        for (int o = 0; o<other / 20; o++) {
            _delay_ms(1);
        }

        // and show the old number again
        writeDecoder(tens1, unit1);

        // wait a bit again
        for (int o = 0; o<i / 20; o++) {
            _delay_ms(1);
        }

        other++;
    }

    // finally, write the new number once and for all
    writeDecoder(tens, unit);
}

int
main(void)
{
    lowerRange = 0;
    upperRange = 0;

    ioinit();
    timer_init();

    /* all devices initialize themselves as slaves */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);

    /* loop forever */
    for (;; ) {
        sendUpdate(UPPER_RANGE_CMD, 0, 0);
        _delay_ms(1000);

        /* if (buttonPressed) { */
        /*     buttonPressed = false; */

        /*     if (mode2) {    /\* if we have to run as token dispensor *\/ */
        /*         lastToken = upperRange; /\* save the current number for */
        /*                                  * use with PWM *\/ */

        /*         ++upperRange; */
        /*         if (upperRange == 100) { */
        /*             upperRange = 0; */
        /*         } */

        /*         if (sendUpdate(UPPER_RANGE_CMD, lastToken, upperRange)) { /\* we */
        /*                                 * were able to send an update */
        /*                                 * successfully over I2C; we */
        /*                                 * can update the display. *\/ */
        /*             displayNumber(upperRange, lastToken); */
        /*         } else {        /\* There was a problem with sending */
        /*                          * the update; we need to rollback the */
        /*                          * value of upperRange and handle */
        /*                          * button press event again in a */
        /*                          * subsequent iteration. *\/ */
        /*             upperRange    = lastToken; */
        /*             buttonPressed = true; */
        /*         } */
        /*     } else if (mode) { /\* else, we are running as a counter module *\/ */
        /*         if (upperRange != lowerRange) { */
        /*             lastToken = lowerRange; /\* save the current number */
        /*                                      * for use with PWM *\/ */

        /*             ++lowerRange; */
        /*             if (lowerRange == 100) { */
        /*                 lowerRange = 0; */
        /*             } */

        /*             if (sendUpdate(CUR_NUM_CMD, lastToken, lowerRange)) { /\* we */
        /*                                  * were able to send an update */
        /*                                  * successfully over I2C; we */
        /*                                  * can update the display. *\/ */
        /*                 displayNumber(lowerRange, lastToken); */
        /*             } else {    /\* There was a problem with sending */
        /*                          * the update; we need to rollback the */
        /*                          * value of lowerRange and handle */
        /*                          * button press event again in a */
        /*                          * subsequent iteration. *\/ */
        /*                 lowerRange    = lastToken; */
        /*                 buttonPressed = true; */
        /*             } */
        /*         } */
        /*     } */
        /* } */

        /* if (USI_TWI_Data_In_Receive_Buffer()) { /\* have */
        /*                                          * we received anything */
        /*                                          * over TWI? *\/ */

        /*     uint8_t command = USI_TWI_Receive_Byte(); */
        /*     switch (command) { */

        /*         // if we received a last served number update */
        /*     case CUR_NUM_CMD: */
        /*         USI_TWI_Receive_Byte(); /\* read the previous value; */
        /*                                  * but this is of no use. *\/ */
        /*         lowerRange = USI_TWI_Receive_Byte(); */

        /*         /\* If general status monitor mode *\/ */
        /*         if (!mode && !mode2) { */
        /*             displayNumber(lowerRange, lastToken); */
        /*             lastToken = lowerRange; */
        /*         } */
        /*         break; */

        /*         // if we received a last dispensed number update */
        /*     case UPPER_RANGE_CMD: */
        /*         USI_TWI_Receive_Byte(); /\* read the previous value; */
        /*                                  * but this is of no use. *\/ */
        /*         upperRange = USI_TWI_Receive_Byte(); */
        /*         break; */

        /*     default: */
        /*         /\* Unexpected communication. consume everything we've */
        /*          * received *\/ */
        /*         while (USI_TWI_Data_In_Receive_Buffer()) { */
        /*             USI_TWI_Receive_Byte(); */
        /*         } */
        /*         break; */
        /*     } */
        /* } */
    }

    /* we will never come here. */
    return (0);
}
