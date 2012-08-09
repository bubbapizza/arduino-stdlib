/*
  pins_arduino.h - Pin definition functions for Arduino

  I have no idea what the copyright for this file should be but I 
  lifted it from

      https://code.google.com/p/ardurct/wiki/PinMapping

  Below are the notes from that page.

  Shawn Wilson
  Aug 8, 2012

***************************************
Objectives
----------

While using an Atmegaxx4 processor (Atmega164P, Atmega324P, Atmega644P, 
Atmega1284P):

Preserve as much as possible Arduino functionalities on the same pins 
as the Uno (up to R3) board.

Create an 8 bit bus with 3 control lines to manage external parallel 
devices such as memories and graphic screens, without impacting Arduino 
pins. 

Overview
--------

Port A is the analogic input port and port D is the Serial port and PWM 
port, so can not be used for the 8 bit bus.

If we take port C as the bus and 3 pins for the control lines from port B, 
we only have 2 pins for D4, D7 and D8, as we have to take D11 to D13 from 
port B. On top of that, we can't use the bus during a Wire transfer, 
which can slow things down.

If we take port B as the bus and 3 pins for the control lines from port C, 
we have enough pins left on port C for D4, D7 and D8. The benefit of this 
mapping is that we can use Wire at slow speed. The only constrain is we 
have to use SPI in master mode and have to wait for SPI to complete a 
transfer before using the bus, which is quite easy to do. 

Solution
--------

Arduino Uno mapping has been followed as much as possible, with the 
exception of:

I2C pins which have been mapped to D19=SCL and D20=SDA, as they can not be 
mapped to A4 and A5.

Only 4 PWM pins are mapped : D5, D6, D9 and D10. The 2 last PWM pins are 
on the 8 bits bus and are mapped to D17 and D18, so they can only be used 
if the 8 bits port is not used. But in most cases 4 PWM pins are enough.

SS pin has been mapped to D18 to allow the use of SPI without having D10 
as an output. With this mapping, D10 can also be a PWM. 

PortB can be used as an 8 bits port without affecting Arduino pins (other 
than the SPI pins) when it changes. Some care has to be taken when SPI is 
used: saving and restoring the state of the pins is required if the 8 bis 
port is used.

PortC and PortD provides pins for the Arduino compatible footprint.
PortA provides the analog part. 
*/


#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)
#define digitalPinHasPWM(p)         ((p) == 5 || (p) == 6 || (p) == 9 || (p) == 10 || (p) == 17 || (p) == 18)

const static uint8_t SS   = 18;
const static uint8_t MOSI = 11;
const static uint8_t MISO = 12;
const static uint8_t SCK  = 13;

const static uint8_t SDA = 20;
const static uint8_t SCL = 19;
const static uint8_t LED_BUILTIN = 13;

const static uint8_t A0 = 24;
const static uint8_t A1 = 25;
const static uint8_t A2 = 26;
const static uint8_t A3 = 27;
const static uint8_t A4 = 28;
const static uint8_t A5 = 29;
const static uint8_t A6 = 30;
const static uint8_t A7 = 31;

// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// I've deliberately left out pin mapping to the Hardware USARTs
// and also the analog pins: this already makes for 20 RECEIVE pins !

#define digitalPinToPCICR(p)    (((p) >= 4 && (p) <= 23) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) ( ((p) == 5 || (p) == 6 || (p) == 9 || (p) == 10) ? 3 : \
                                ( ((p) >= 11 && (p) <= 18) ? 1 : 2 ))
#define digitalPinToPCMSK(p)    ( ((p) == 5 || (p) == 6 || (p) == 9 || (p) == 10) ? (&PCMSK3) : \
                                ( ((p) >= 11 && (p) <= 18) ? (&PCMSK1) : \
                                ( ((p) >= 4 && (p) <= 23) ? (&PCMSK2) : ((uint8_t *)0) )))
#define digitalPinToPCMSKbit(p) ( ((p) == 5 || (p) == 6) ? (p)-1 : \
                                ( ((p) == 9 || (p) == 10) ? (p)-3 : \
                                ( ((p) == 4) ? (p)-2 : \
                                ( ((p) == 7 || (p) == 8) ? (p)-4 : \
                                ( ((p) >= 11 || (p) <= 13) ? (p)-6 : \
                                ( ((p) >= 14 || (p) <= 18) ? (p)-14 : \
                                ( ((p) == 19 || (p) == 20) ? (p)-19 : \
                                ( ((p) >= 21 || (p) <= 23) ? (p)-16 : 0) )))))))

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &DDRA,
        (uint16_t) &DDRB,
        (uint16_t) &DDRC,
        (uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PORTA,
        (uint16_t) &PORTB,
        (uint16_t) &PORTC,
        (uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
        NOT_A_PORT,
        (uint16_t) &PINA,
        (uint16_t) &PINB,
        (uint16_t) &PINC,
        (uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
        PD,             // PD0 -  D0 / RX
        PD,             // PD1 -  D1 / TX
        PD,             // PD2 -  D2 / RX2
        PD,             // PD3 -  D3 / TX2
        PC,             // PC2 -  D4
        PD,             // PD4 -  D5 / PWM1
        PD,             // PD5 -  D6 / PWM2
        PC,             // PC3 -  D7
        PC,             // PC4 -  D8
        PD,             // PD6 -  D9 / PWM3
        PD,             // PD7 - D10 / PWM4
        PB,             // PB5 - D11 / MOSI
        PB,             // PB6 - D12 / MISO
        PB,             // PB7 - D13 / SCK / LED_BUILTIN
        PB,             // PB0 - D14
        PB,             // PB1 - D15
        PB,             // PB2 - D16
        PB,             // PB3 - D17 / PWM5
        PB,             // PB4 - D18 / PWM6 / SS
        PC,             // PC0 - D19 / SCL
        PC,             // PC1 - D20 / SDA
        PC,             // PC5 - D21
        PC,             // PC6 - D22
        PC,             // PC7 - D23
        PA,             // PA0 - D24 / A0
        PA,             // PA1 - D25 / A1
        PA,             // PA2 - D26 / A2
        PA,             // PA3 - D27 / A3
        PA,             // PA4 - D28 / A4
        PA,             // PA5 - D29 / A5
        PA,             // PA6 - D30 / A6
        PA,             // PA7 - D31 / A7
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
        _BV(0),         // PD0 -  D0 / RX
        _BV(1),         // PD1 -  D1 / TX
        _BV(2),         // PD2 -  D2 / RX2
        _BV(3),         // PD3 -  D3 / TX2
        _BV(2),         // PC2 -  D4
        _BV(4),         // PD4 -  D5 / PWM1
        _BV(5),         // PD5 -  D6 / PWM2
        _BV(3),         // PC3 -  D7
        _BV(4),         // PC4 -  D8
        _BV(6),         // PD6 -  D9 / PWM3
        _BV(7),         // PD7 - D10 / PWM4
        _BV(5),         // PB5 - D11 / MOSI
        _BV(6),         // PB6 - D12 / MISO
        _BV(7),         // PB7 - D13 / SCK / LED_BUILTIN
        _BV(0),         // PB0 - D14
        _BV(1),         // PB1 - D15
        _BV(2),         // PB2 - D16
        _BV(3),         // PB3 - D17 / PWM5
        _BV(4),         // PB4 - D18 / PWM6 / SS
        _BV(0),         // PC0 - D19 / SCL
        _BV(1),         // PC1 - D20 / SDA
        _BV(5),         // PC5 - D21
        _BV(6),         // PC6 - D22
        _BV(7),         // PC7 - D23
        _BV(0),         // PA0 - D24 / A0
        _BV(1),         // PA1 - D25 / A1
        _BV(2),         // PA2 - D26 / A2
        _BV(3),         // PA3 - D27 / A3
        _BV(4),         // PA4 - D28 / A4
        _BV(5),         // PA5 - D29 / A5
        _BV(6),         // PA6 - D30 / A6
        _BV(7),         // PA7 - D31 / A7
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER1B,        // PD4 -  D5 / PWM1
        TIMER1A,        // PD5 -  D6 / PWM2
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER2B,        // PD6 -  D9 / PWM3
        TIMER2A,        // PD7 - D10 / PWM4
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        TIMER0A,        // PB3 - D17 / PWM5
        TIMER0B,        // PB4 - D18 / PWM6 / SS
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
        NOT_ON_TIMER,
};

#endif

#endif
