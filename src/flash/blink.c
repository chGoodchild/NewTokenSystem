/**
 * @file   blink.c
 * @author Rohit Grover <rgrover@thinktop>
 * @date   Sat Oct  2 12:18:42 2010
 *
 * @brief This is a simple LED blinker--toggling PORTB0 using a
 * timer. It often serves as a first-program for a
 * micro-controller. By the time an LED blinks at a pre-determined
 * rate, one has learned a lot about the micro.
 *
 * Connect PORTB0 to an LED, and it should blink.
 */

/// Define the CPU clockrate--this plays a role when using the delay loop.
#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void
setupIOPins(void)
{
  DDRB  = _BV(1) | _BV(0) | _BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6) | _BV(7);          // Setup PORTB0 for output
    //PORTB = _BV(1);          // Turn off the LED connected through PB0
    //DDRB  = _BV(1);
    //PORTB = _BV(1);
}


int
main(void)
{
    setupIOPins();

    while (1) {
      _delay_ms(500);
      PORTB = 0;
      _delay_ms(500);
      PORTB = 0xff;
    }
}

