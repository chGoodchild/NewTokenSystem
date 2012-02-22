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
 * THIS SOFTWARE IS PROVIDED BY Rohit and Dyuman ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rohit or Dyuman BE LIABLE
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
#define TIMER_SNAPSHOT_MS() ((ticks << 8) | TCNT0)
#define DEBOUNCE_DELAY   1500
#define LED_PIN          PB1
#define BUTTON_PIN       PB0
#define CUR_NUM_CMD      0xF2
#define CUR_NUM_CMD_SIZE 3
#define UPPER_RANGE_CMD      0xF3
#define UPPER_RANGE_CMD_SIZE 3
#define RETRANSMIT_COUNT 25
#define TWI_BROADCAST_ADDRESS (0x0)



/* ---------------------------------------------------------------------------------------------------------- */


/* volatile uint8_t big_ticks_since_the_beginning;   */
/* volatile uint8_t temp_ticks; */
/* volatile uint8_t number_of_members_since_the_beginning; */
/* volatile uint8_t average_time_per_token; /\* this is measuredy in big_ticks/token */ 


/* ---------------------------------------------------------------------------------------------------------- */

volatile uint8_t token; /* the current number that is being displayed */
volatile uint8_t lower_range;	/* the current number that has been recieved over twi */
volatile uint8_t upper_range;
volatile uint8_t mode;
volatile uint8_t mode2;
volatile bool     buttonPressed = false;
volatile uint32_t ticks;
volatile uint32_t buttonPressSnapshot;
volatile uint8_t  lastToken;           /* the number that we displayed
					  last, (previous number for
					  the PWM) */

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
  /* big_ticks_since_the_beginning = 0; */
  /* temp_ticks = 0; */
}

ISR(INT1_vect)
{
  cli();
  if ((TIMER_SNAPSHOT_MS() - buttonPressSnapshot) >= DEBOUNCE_DELAY) {
    buttonPressSnapshot = TIMER_SNAPSHOT_MS();
    buttonPressed       = true;
  }
}

void
sendUpdate(uint8_t currentNumber)
{
  USI_TWI_Master_Initialise();

  uint8_t buffer[CUR_NUM_CMD_SIZE];
  buffer[0] =
    (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
    (FALSE << TWI_READ_BIT); /* write operation */
  buffer[1] = CUR_NUM_CMD;
  buffer[2] = currentNumber;

  /* Remember that you are only transmitting it 10 times, and that you can
   * transmit multiple times if it makes trouble... */
  for (int i = 0; i < RETRANSMIT_COUNT; i++) {
    _delay_ms(10); /* Is this necessary? */
    USI_TWI_Start_Transceiver_With_Data(buffer, sizeof(buffer));
    /* keep retransmitting until it is successful */


  }
  _delay_ms(10);
 
  /* reinitialize ourself as a slave */
  USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
}

void sendUpdateUpperRange(uint8_t currentNumber)
{
  USI_TWI_Master_Initialise();

  uint8_t buffer[UPPER_RANGE_CMD_SIZE];
  buffer[0] =
    (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
    (FALSE << TWI_READ_BIT); /* write operation */
  buffer[1] = UPPER_RANGE_CMD;
  buffer[2] = currentNumber;

  /* Remember that you are only transmitting it 10 times, and that you can
   * transmit multiple times if it makes trouble... */
  for (int i = 0; i < RETRANSMIT_COUNT; i++) {
    _delay_ms(10); /* Is this necessary? */
    USI_TWI_Start_Transceiver_With_Data(buffer, sizeof(buffer));
    /* keep retransmitting until it is successful */


  }
  _delay_ms(10);
 
  /* reinitialize ourself as a slave */
  USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
}


/* Initialize I/O pins and other stuff */
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

  /* find out the required mode of operation */
  mode = 0;
  mode = (PINB & _BV(4));
  if (mode) {             /* this is a service-module */
    PORTD |= _BV(3); // enable pull-up for button pin

    /* enable interrupt for the button */
    MCUCR |= _BV(ISC11); /* only the falling edge generates an interrupt */
    GIMSK |= _BV(INT1);
  }

  mode2 = 0;
  mode2 = (PINB & _BV(6));
  if(mode2) {
    PORTD |= _BV(3); // enable pull-up for button pin

    /* enable interrupt for the button */
    MCUCR |= _BV(ISC11); /* only the falling edge generates an interrupt */
    GIMSK |= _BV(INT1);
  }


  sei();                  // turn on interrupts!
}

/* This function does nothing but display the numbers that it is given. */
void
writeDecoder(uint8_t tens, uint8_t unit)
{
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
  /* else { */
  /*     PORTD &= ~_BV(4); */
  /* } */

  // set the units digit
  PORTB &= ~(_BV(0) | _BV(1) | _BV(2) | _BV(3));
  PORTB |= (unit & 0x0f);
}

void
displayNumber(uint8_t currentNumber)
{
  uint8_t tens;
  uint8_t unit;
  uint8_t tens1;
  uint8_t unit1;
  int     other = 0;

  tens = currentNumber / 10;
  unit = currentNumber % 10;

  tens1 = lastToken / 10;
  unit1 = lastToken % 10;

  for (int i = 100; i>0; i--) {
    /* set the tens digit */
    writeDecoder(tens, unit);
    for (int o = 0; o<other / 20; o++) {
      _delay_ms(1);
    }
    /* _delay_ms(other); */

    writeDecoder(tens1, unit1);
    for (int o = 0; o<i / 20; o++) {
      _delay_ms(1);
    }
    /* _delay_ms(100-other); */
    other++;
  }

  writeDecoder(tens, unit);
}

/* void time(){ */
/*   if(temp_ticks + 1000 <= ticks){ */
/*     big_ticks_since_the_beginning++; */
/*   } */
/*   average_time_per_token = big_ticks_since_the_beginning/number_of_members_since_the_beginning; */
/* } */

int
main(void){
  token =0;
  lower_range =0;

  ioinit();
  timer_init();


  /* all devices initialize themselves as slaves */
  USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);

  /* loop forever */
  for (;; ) {
    /* Block interrupts while we're probing the value of
     * buttonPressed--the interrupt handler may change this value
     * asynchronously */

    /* _delay_ms(15); */

    if (buttonPressed) {
  
      /* now that we've determined that there has been a button
       * press event, we can re-enable interrupts and proceed to
       * act upon it. */

      buttonPressed = false;
      sei();

      if(mode2){
	upper_range++;

	if(upper_range > 99 && token > 99 && lower_range > 99){
	  upper_range = upper_range - 100;
	  token = lower_range = 0;

	} /* end of if mode2 */
      }	  /* end of if button pressed */

      sendUpdateUpperRange(upper_range);
      sendUpdateUpperRange(lower_range); /* Even though we don't have
					    to send both lets do this
					    every where in order to
					    ensure the same number
					    every where... */

      displayNumber(upper_range);
      lastToken = upper_range;

      /* _delay_ms(5000); */

      /* displayNumber((upper_range - token) *  */
      /* 		    average_time_per_token);  */
      /* We must point out to the users of this system that this is just to
	 give them an idea of how many people are waiting and how long it
	 could take (we should make it very clear that our internal clock is
	 not accurate and that this is an average). This would display the
	 time in big_ticks that this person will have to wait. What I would
	 like to do later on is make sure that the big ticks is time in
	 minutes by changing it's value from 1000 to something else
	 depending on the clock speed of the microcontroller, (this is what
	 we need to consult Rohit for).... */

    }else{

      lastToken = token;
      /* number_of_members_since_the_beginning++; */
      lower_range++;
      token = lower_range;	/* This is the same as before, just
				   clearer */

    
      /* If you are questioning this, think of what would happen
	 when we just set them to zero individually when they reach
	 100... */
      if(upper_range < 99 && token < 99 && lower_range < 99){
	upper_range = upper_range - 100;
	token = lower_range = 0;

	sendUpdateUpperRange(upper_range);
	sendUpdateUpperRange(lower_range); /* Even though we don't have
					      to send both lets do this
					      every where in order to
					      ensure the same number
					      every where... */

      }	/* end of if reach 100  */


      /* if (SearchForOthers() || RThereOthers) { */
      sendUpdate(token); /* Also update every other
			  * module on the bus*/

      displayNumber(token); /* update the display */

    } /* end of mode2/mode1 */
  }   /* end of button Press if */

  if (USI_TWI_Data_In_Receive_Buffer()) { 
    /* have we received anything over TWI? */

    uint8_t command = USI_TWI_Receive_Byte();
    switch (command) {
    case CUR_NUM_CMD:
      lower_range = USI_TWI_Receive_Byte();
      if (!mode && !mode2) {
	displayNumber(lower_range);
	lastToken = lower_range;
      }
      break;
    case UPPER_RANGE_CMD:
      upper_range = USI_TWI_Receive_Byte();
      break;
    default:
      /* Unexpected communication. consume everything we've
       * received */
      while (USI_TWI_Data_In_Receive_Buffer()) {
	USI_TWI_Receive_Byte();
      }
      break;
    } /* end of recieve */
  } /* end of endless loop */

  /* we will never come here. */
  return (0);
} /* end of main */
