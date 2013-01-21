/*****************************************************************************
*
* Atmel Corporation
*
* File              : USI_TWI_Master.h
* Compiler          : IAR EWAAVR 2.28a/3.10a
* Revision          : $Revision: 1.11 $
* Date              : $Date: Tuesday, September 13, 2005 09:09:36 UTC $
* Updated by        : $Author: jtyssoe $
*
* Support mail      : avr@atmel.com
*
* Supported devices : All device with USI module can be used.  The
*                     example is written for the ATmega169, ATtiny26
*                     and ATtiny2313
*
* AppNote           : AVR310 - Using the USI module as a TWI Master
*
* Description : This is an implementation of an TWI master using the
*                     USI module as basis. The implementation assumes
*                     the AVR to be the only TWI master in the system
*                     and can therefore not be used in a multi-master
*                     system.
* Usage : Initialize the USI module by calling the
*                     USI_TWI_Master_Initialise() function. Hence
*                     messages/data are transceived on the bus using
*                     the USI_TWI_Start_Transceiver_With_Data()
*                     function. If the transceiver returns with a
*                     fail, then use USI_TWI_Get_Status_Info to
*                     evaluate the cause of the failure.
*
****************************************************************************/

#include <util/delay.h>

//********** Defines **********//

// Defines controlling timing limits
/* #define TWI_FAST_MODE */

#define SYS_CLK   1000.0  // [kHz]

#ifdef TWI_FAST_MODE               // TWI FAST mode timing limits. SCL = 100-400kHz
  #define T2_TWI    ((SYS_CLK *1300) /1000000) +1 // >1,3us
  #define T4_TWI    ((SYS_CLK * 600) /1000000) +1 // >0,6us

#else                              // TWI STANDARD mode timing limits. SCL <= 100kHz
  #define T2_TWI    ((SYS_CLK *4700) /1000000) +1 // >4,7us
  #define T4_TWI    ((SYS_CLK *4000) /1000000) +1 // >4,0us
#endif

// Defines controling code generating
//#define PARAM_VERIFICATION
#define NOISE_TESTING 1         /* Enable detection of I2C
                                 * collisions. This may happen if two
                                 * distinct modules decide to become
                                 * masters at nearly the same time and
                                 * transmit conflicting messages. */
//#define SIGNAL_VERIFY

//USI_TWI messages and flags and bit masks
//#define SUCCESS   7
//#define MSG       0
/****************************************************************************
  Bit and byte definitions
****************************************************************************/
#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT  0       // Bit position for (N)ACK bit.

#define USI_TWI_NO_DATA             0x00  // Transmission buffer is empty
#define USI_TWI_DATA_OUT_OF_BOUND   0x01  // Transmission buffer is outside SRAM space
#define USI_TWI_UE_START_CON        0x02  // Unexpected Start Condition
#define USI_TWI_UE_STOP_CON         0x03  // Unexpected Stop Condition
#define USI_TWI_UE_DATA_COL         0x04  // Unexpected Data Collision (arbitration)
#define USI_TWI_NO_ACK_ON_DATA      0x05  // The slave did not acknowledge  all data
#define USI_TWI_NO_ACK_ON_ADDRESS   0x06  // The slave did not acknowledge  the address
#define USI_TWI_MISSING_START_CON   0x07  // Generated Start Condition not detected on bus
#define USI_TWI_MISSING_STOP_CON    0x08  // Generated Stop Condition not detected on bus

// Device dependant defines

#if defined(__AT90Mega169__) | defined(__ATmega169__) | \
    defined(__AT90Mega165__) | defined(__ATmega165__) | \
    defined(__ATmega325__) | defined(__ATmega3250__) | \
    defined(__ATmega645__) | defined(__ATmega6450__) | \
    defined(__ATmega329__) | defined(__ATmega3290__) | \
    defined(__ATmega649__) | defined(__ATmega6490__)
    #define DDR_USI             DDRE
    #define PORT_USI            PORTE
    #define PIN_USI             PINE
    #define PORT_USI_SDA        PORTE5
    #define PORT_USI_SCL        PORTE4
    #define PIN_USI_SDA         PINE5
    #define PIN_USI_SCL         PINE4
#endif

#if defined(__ATtiny25__) | defined(__ATtiny45__) | defined(__ATtiny85__) | \
    defined(__AT90Tiny26__) | defined(__ATtiny26__)
    #define DDR_USI             DDRB
    #define PORT_USI            PORTB
    #define PIN_USI             PINB
    #define PORT_USI_SDA        PORTB0
    #define PORT_USI_SCL        PORTB2
    #define PIN_USI_SDA         PINB0
    #define PIN_USI_SCL         PINB2
#endif

#if defined(__AT90Tiny2313__) | defined(__ATtiny2313__)
    #define DDR_USI             DDRB
    #define PORT_USI            PORTB
    #define PIN_USI             PINB
    #define PORT_USI_SDA        PORTB5
    #define PORT_USI_SCL        PORTB7
    #define PIN_USI_SDA         PINB5
    #define PIN_USI_SCL         PINB7
#endif

// General defines
#define TRUE  1
#define FALSE 0

//********** Prototypes **********//

void          USI_TWI_Master_Initialise( void );
unsigned char USI_TWI_Start_Transceiver_With_Data( unsigned char * , unsigned char );
unsigned char USI_TWI_Get_State_Info( void );

#define COLLISION_DETECTED() (USI_TWI_Get_State_Info() == USI_TWI_UE_DATA_COL)
