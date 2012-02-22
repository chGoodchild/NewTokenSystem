/**p
 * @file   blink.c
 * @author Rohit Grover <rgrover@thinktop>
 * @date   Sat Oct  2 12:18:42 2010
 *
 * @brief This is a simple LED blinker--
 */

/// Define the CPU clockrate--this plays a role when using the delay loop.
#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI_TWI_Master.h"
#include "USI_TWI_Slave.h"

volatile bool BP = false;

#define LED_PIN    PB1
#define BUTTON_PIN PB0

#define DEFAULT_COMMAND 0xF2

#define TWI_BROADCAST_ADDRESS (0x0)


/**
 * \brief Interrupt handler for timer-overflow.
 */
ISR(TIMER0_OVF_vect)
{
}

/**
 * Interrupt handler for pin-change interrupt on PB0. This is the
 * input from the motion sensor. It is pulled low when motion is
 * sensed (while normally it remains high)
 */
ISR(PCINT_vect)
{
    if ((PINB & _BV(BUTTON_PIN)) == 0) { /* button has been pressed */
        buttonPressed = true;
    }
}

void
timer_init(void)
{
     TCCR0B  = _BV(CS02) | _BV(CS00);   /* set prescaler to use clock/1024  */
     TIMSK  = _BV(TOIE0);               /* enable the overflow interrupt */
}

void
setupIOPins(void)
{
    /* Enable pin-change interrupt on PB0; which is PCINT0 */
    GIMSK |= _BV(PCIE);
    PCMSK |= _BV(PCINT0);

    DDRB  |= _BV(LED_PIN);      /* set LED_PIN as an output */
    PORTB |= _BV(LED_PIN);      /* set 5V on LED_PIN; this will turn
                                 * off the LED. */
    PINB |= _BV(BUTTON_PIN); /* turn on internal pull-up for button pin */

    /* enable interrupts */
    sei();
}

int
main(void)
{
    setupIOPins();
    timer_init();

    /* all devices initialize themselves as slaves */
    USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);

    /* main loop */
    while(1) {
        if (buttonPressed) {
            USI_TWI_Master_Initialise();

            uint8_t buffer[2];
            buffer[0] =
                (TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
                (FALSE << TWI_READ_BIT);                  /* write operation */
            buffer[1] = DEFAULT_COMMAND;

            uint8_t status;
            for (int i = 0; i < 500; i++) {
	      _delay_ms(100);
	      status = USI_TWI_Start_Transceiver_With_Data(buffer, 2);
	      if (status == FALSE) {
                /* TODO: do something to handle error; we should not leave
                 * this empty here. */
	      }
	    }

            _delay_ms(10);
            buttonPressed = false; /* reset buttonPressed for later */

            /* reinitialize ourself as a slave */
            USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
        } else if (USI_TWI_Data_In_Receive_Buffer()) { /* have we
                                                        * received
                                                        * anything
                                                        * over TWI? */
            uint8_t command = USI_TWI_Receive_Byte();
            switch (command) {
            case DEFAULT_COMMAND:
                /* received our special command; toggle the LED */
                PINB |= _BV(LED_PIN);
                break;

            default:
                /* Unexpected communication. consume everything we've
                 * received */
                while (USI_TWI_Data_In_Receive_Buffer()) {
                    USI_TWI_Receive_Byte();
                }
                break;
            }
        } else {
            /* TODO: sleep */
        }
    }
}
