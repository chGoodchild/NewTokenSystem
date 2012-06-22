/**
 * @file   uartTWI.c
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

#define TWI_BROADCAST_ADDRESS (0x0)

#define BAUD  9600
#include <util/setbaud.h>

volatile uint8_t uart_data;
volatile bool    uart_dataReceived = false;


/* here we receive data */
ISR(USART_RX_vect)
{
    uart_dataReceived = true;
    uart_data = UDR;
}

void
avr_init(void)
{
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
    _BV(RXEN)  |
    _BV(RXCIE) |
    _BV(TXEN);
  
  /* Enable internal pull-ups for TWI pins */
  PORTB &= _BV(5);
  PORTB &= _BV(7);
  
  /* enable interrupts */
    sei();
}

/* Send the data in byte argument */
void
sendUart(uint8_t byte)
{
    while (!(UCSRA & (1 << UDRE)))
        ;               /* spin while UDRE is not set */
    UDR = byte;
}

int
main(void)
{
  avr_init();
  
  /* all devices initialize themselves as slaves in TWI*/
  USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
  
  /* main loop */
  while(1) {
    if (uart_dataReceived) { // if we have received anything over UART
      USI_TWI_Master_Initialise(); // Initialize ourselves as TWI master
      
      uint8_t buffer[2];
      buffer[0] =
	(TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
	(FALSE << TWI_READ_BIT);                  /* write operation */
      buffer[1] = uart_data; /* The data we received over UART has to be sent
			      * over TWI. */
      sendUart(uart_data);
      
      while (!USI_TWI_Start_Transceiver_With_Data(buffer, 2)); /* Transmit, and
								* re-transmit 
								* till
								* successful. */
      sendUart(uart_data);
      uart_dataReceived = false; /* we've consumed the data from UART.. */
      
      /* reinitialize ourself as a slave in TWI*/
      USI_TWI_Slave_Initialise(TWI_BROADCAST_ADDRESS);
    } else if (USI_TWI_Data_In_Receive_Buffer()) { /* have we
						    * received
						    * anything
						    * over TWI? */

      // temporary:

      USI_TWI_Master_Initialise(); // Initialize ourselves as TWI master
      
      uint8_t buffer[2];
      buffer[0] =
	(TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
	(FALSE << TWI_READ_BIT);                  /* write operation */
      buffer[1] = 0xf3; /* The data we received over UART has to be sent
			      * over TWI. */
      while (!USI_TWI_Start_Transceiver_With_Data(buffer, 2)); /* Transmit, and
								* re-transmit 
								* till
								* successful. */
      buffer[0] =
	(TWI_BROADCAST_ADDRESS << TWI_ADR_BITS) | /* address */
	(FALSE << TWI_READ_BIT);                  /* write operation */
      buffer[1] = 0x08; /* The data we received over UART has to be sent
			      * over TWI. */
      while (!USI_TWI_Start_Transceiver_With_Data(buffer, 2)); /* Transmit, and
								* re-transmit 
								* till
								* successful. */
      // temporary till here      


      uint8_t TWI_data = USI_TWI_Receive_Byte(); // Receive that data
      sendUart(TWI_data); // Transmit it over UART
    }
  }
}
