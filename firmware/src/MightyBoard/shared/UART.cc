/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include "UART.hh"
#include "EepromMap.hh"
#include "Eeprom.hh"
#include "Pin.hh"
#include <stdint.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/io.h>

// TODO: There should be a better way to enable this flag?
#if ASSERT_LINE_FIX
#include "ExtruderBoard.hh"
#endif

#if HAS_SLAVE_UART
// Avoid repeatedly creating temp objects
const Pin TX_Enable = TX_ENABLE_PIN;
const Pin RX_Enable = RX_ENABLE_PIN;
#endif

// We have to track the number of bytes that have been sent, so that we can
// filter
// them from our receive buffer later.This is only used for RS485 mode.
volatile uint8_t loopback_bytes = 0;

#if defined(ALTERNATE_UART) && HAS_SLAVE_UART != 0
#error Cannot use 2nd UART for both HAS_SLAVE_UART and ALTERNATE_UART
#endif

// MOD Trax BEGIN
#if defined(UART_DEBUG) && (defined(ALTERNATE_UART) || HAS_SLAVE_UART != 0)
#error Cannot use 2nd UART for both UART_DEBUG and ALTERNATE_UART or HAS_SLAVE_UART
#endif
// MOD END

// We support three platforms: Atmega168 (1 UART), Atmega644, and
// Atmega1280/2560
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) ||                \
    defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1280__) ||              \
    defined(__AVR_ATmega2560__)
#else
#error UART not implemented on this processor type!
#endif

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__)

#define UBRR_VALUE 25
#define UCSR0A_VALUE 0

#define INIT_SERIAL(uart_)                                                     \
  {                                                                            \
    UBRR0H = UBRR_VALUE >> 8;                                                  \
    UBRR0L = UBRR_VALUE & 0xff;                                                \
                                                                               \
    /* set config for uart, explicitly clear TX interrupt flag */              \
    UCSR0A = UCSR0A_VALUE | _BV(TXC0);                                         \
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);                                          \
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);                                        \
  }

#elif defined(__AVR_ATmega644P__)

#define UBRR_VALUE 25
#define UBRRA_VALUE 0

// Adapted from ancient arduino/wiring rabbit hole
#define INIT_SERIAL(uart_)                                                     \
  {                                                                            \
    UBRR##uart_##H = UBRR_VALUE >> 8;                                          \
    UBRR##uart_##L = UBRR_VALUE & 0xff;                                        \
                                                                               \
    /* set config for uart_ */                                                 \
    UCSR##uart_##A = UBRRA_VALUE;                                              \
    UCSR##uart_##B = _BV(RXEN##uart_) | _BV(TXEN##uart_);                      \
    UCSR##uart_##C = _BV(UCSZ##uart_##1) | _BV(UCSZ##uart_##0);                \
  }

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// Use double-speed mode for more accurate baud rate?
#define UBRR0_VALUE 16 // 115200 baud
//#ifdef ALTERNATE_UART
#if defined(ALTERNATE_UART) || defined(UART_DEBUG) // MOD Trax
// Alternate UART uses 115200 on both UARTS
#define UBRR1_VALUE 16 // 115200 baud
#else
// SLAVE_UART uses 38400 for the RS485 bus
#define UBRR1_VALUE 51 // 38400 baud
#endif
#define UCSRA_VALUE(uart_) _BV(U2X##uart_)

// Adapted from ancient arduino/wiring rabbit hole
#define INIT_SERIAL(uart_)                                                     \
  {                                                                            \
    UBRR##uart_##H = UBRR##uart_##_VALUE >> 8;                                 \
    UBRR##uart_##L = UBRR##uart_##_VALUE & 0xff;                               \
                                                                               \
    /* set config for uart_ */                                                 \
    UCSR##uart_##A = UCSRA_VALUE(uart_);                                       \
    UCSR##uart_##B = _BV(RXEN##uart_) | _BV(TXEN##uart_);                      \
    UCSR##uart_##C = _BV(UCSZ##uart_##1) | _BV(UCSZ##uart_##0);                \
  }
#endif

#define ENABLE_SERIAL_INTERRUPTS(uart_)                                        \
  { UCSR##uart_##B |= _BV(RXCIE##uart_) | _BV(TXCIE##uart_); }

#define DISABLE_SERIAL_INTERRUPTS(uart_)                                       \
  { UCSR##uart_##B &= ~(_BV(RXCIE##uart_) | _BV(TXCIE##uart_)); }

// TODO: Move these definitions to the board files, where they belong.
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__)

#ifdef ALTERNATE_UART
#error Alternate UART is not available on the ATmega328.
#endif

UART UART::hostUART(0, RS485);

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1280__) ||            \
    defined(__AVR_ATmega2560__)

UART UART::hostUART(0, RS232);

#if HAS_SLAVE_UART
UART UART::slaveUART(1, RS485);
#endif

#endif

void UART::init_serial() {
#ifdef ALTERNATE_UART
  // when using the ALTERNATE_UART option, both UARTS need to be initialized
  INIT_SERIAL(0);
  INIT_SERIAL(1);
#else
  if (index_ == 0) {
    INIT_SERIAL(0);
  }
#endif
#if HAS_SLAVE_UART	 
	else {
    INIT_SERIAL(1);
  }
#endif

// MOD Trax BEGIN
#ifdef UART_DEBUG
	INIT_SERIAL(1);
#endif
// MOD Trax END
}

void UART::send_byte(char data) {
  if (index_ == 0) {
    UDR0 = data;
  } 
#if HAS_SLAVE_UART || defined(ALTERNATE_UART)
  else {
    UDR1 = data;
  }
#endif
}

// MOD Trax BEGIN
#ifdef UART_DEBUG
void UART::print_byte(char data) {
  UDR1 = data;
}
#endif
// MOD Trax END

#if HAS_SLAVE_UART
// Transition to a non-transmitting state. This is only used for RS485 mode.
inline void listen() {
  //        TX_Enable.setValue(false);
  TX_Enable.setValue(false);
}

// Transition to a transmitting state
inline void speak() { TX_Enable.setValue(true); }
#endif

UART::UART(uint8_t index, communication_mode mode)
    : index_(index), mode_(mode), enabled_(false) {
  init_serial();
#ifdef ALTERNATE_UART
  // Value in EEPROM is the UART index: 0 for UART0 (USB), 1 for UART1
  // any other value is ignored.
  setHardwareUART(eeprom::getEeprom8(eeprom_offsets::ENABLE_ALTERNATE_UART, 0));
#endif
}

#ifdef ALTERNATE_UART
void UART::setHardwareUART(uint8_t index) {

  // Do nothing if there is no change in UART index.
  if (index == index_ || index > 1)
    return;

  // Save the new UART index
  index_ = index;

  // If we don't have an enabled UART, the do nothing.
  // the ::enable() function will enable the proper UART interrupts
  if (!enabled_)
    return;

  // Enable/Disable the proper ISR routines
  if (index == 0) {
    DISABLE_SERIAL_INTERRUPTS(1);
    ENABLE_SERIAL_INTERRUPTS(0);
  } else {
    DISABLE_SERIAL_INTERRUPTS(0);
    ENABLE_SERIAL_INTERRUPTS(1);
  }
}
#endif

// Subsequent bytes will be triggered by the tx complete interrupt.
void UART::beginSend() {
  if (!enabled_) {
    return;
  }

#if HAS_SLAVE_UART
  if (mode_ == RS485) {
    speak();
    _delay_us(10);
    loopback_bytes = 1;
  }
#endif
  send_byte(out.getNextByteToSend());
}

// MOD Trax BEGIN
#ifdef UART_DEBUG

#define DBG_STOR_STARE \
	cli(); \
	bool begin = !dbg_out.isSending();

#define DBG_START_SEND \
	if(begin) \
		beginPrint(); \
	sei();

void UART::printPStr(const prog_uchar pstr[])
{
	DBG_STOR_STARE
	dbg_out.appendPStr(pstr);
	DBG_START_SEND
}

void UART::printString(const char* str)
{
	DBG_STOR_STARE
	dbg_out.appendStr(str);
	DBG_START_SEND
}

#define MAX_NUM_STR_LEN 20

void UART::printInt(uint16_t value, uint8_t digits) {

    if(digits > 5)
        digits = 5;
    printInt32(value, digits);
}

void UART::printInt32(uint32_t value, uint8_t digits) {

     uint32_t currentDigit = 1;
     uint32_t nextDigit;
     bool nonzero_seen = false;

	uint8_t p = 0;
	char str[MAX_NUM_STR_LEN + 1];

     if ( digits > 9 )
	  digits = 9;

     for (uint8_t i = digits; i; i--)
	  currentDigit *= 10;

     for (uint8_t i = digits; i; i--) {
	  nextDigit = currentDigit / 10;
	  char c;
	  int8_t d = (value % currentDigit) / nextDigit;
	  if ( nonzero_seen || d != 0 || i == 1) {
	       c = d + '0';
	       nonzero_seen = true;
	  }
	  else
	       c = ' ';
	  str[p++] = c;
	  currentDigit = nextDigit;
     }
     
	str[p] = '\0';

	printString(str);
}

//From: http://www.arduino.cc/playground/Code/PrintFloats
//tim [at] growdown [dot] com   Ammended to write a float to lcd
//If rightJusityToCol = 0, the number is left justified, i.e. printed from the
//current cursor position.  If it's non-zero, it's right justified to end at rightJustifyToCol column.

void UART::printFloat(float value, uint8_t decimalPlaces) {
        // this is used to cast digits
        int digit;
        float tens = 0.1;
        int tenscount = 0;
        int i;
        float tempfloat = value;
	uint8_t p = 0;
	char str[MAX_NUM_STR_LEN + 1];

        // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
        // if this rounding step isn't here, the value  54.321 prints as 54.3209

        // calculate rounding term d:   0.5/pow(10,decimalPlaces)
        float d = 0.5;
        if (value < 0) d *= -1.0;

        // divide by ten for each decimal place
        for (i = decimalPlaces; i; i--) d/= 10.0;

        // this small addition, combined with truncation will round our values properly
        tempfloat +=  d;

        // first get value tens to be the large power of ten less than value
        // tenscount isn't necessary but it would be useful if you wanted to know after this how many chars the number will take

        tempfloat = fabsf(tempfloat);
        while ((tens * 10.0) <= tempfloat) {
                tens *= 10.0;
                tenscount += 1;
        }

        // write out the negative if needed
        if (value < 0) str[p++] = '-';

        if (tenscount == 0) str[p++] = '0';

        for (i = tenscount; i; i--) {
                digit = (int) (tempfloat/tens);
                str[p++] = digit + '0';
                tempfloat = tempfloat - ((float)digit * tens);
                tens /= 10.0;
        }

        // if no decimalPlaces after decimal, stop now and return
        if (decimalPlaces > 0) {
		// otherwise, write the point and continue on
		str[p++] = '.';

		// now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
		for (i = decimalPlaces; i; i--) {
			tempfloat *= 10.0;
			digit = (int) tempfloat;
			str[p++] = digit+'0';
			// once written, subtract off that digit
			tempfloat = tempfloat - (float) digit;
		}
	}

	str[p] = '\0';

	printString(str);
}

void UART::beginPrint() {
  if (!enabled_) {
    return;
  }

  print_byte(dbg_out.getNextByteToSend());
}
#endif
// MOD Trax END

void UART::enable(bool enabled) {
  enabled_ = enabled;
  if (index_ == 0) {
    if (enabled) {
      ENABLE_SERIAL_INTERRUPTS(0);
    } else {
      DISABLE_SERIAL_INTERRUPTS(0);
    }
  }
#if HAS_SLAVE_UART || defined(ALTERNATE_UART)
  else if (index_ == 1) {
    if (enabled) {
      ENABLE_SERIAL_INTERRUPTS(1);
    } else {
      DISABLE_SERIAL_INTERRUPTS(1);
    }
  }
#endif

// MOD Trax BEGIN
#ifdef UART_DEBUG
  if (enabled) {
    ENABLE_SERIAL_INTERRUPTS(1);
  } else {
    DISABLE_SERIAL_INTERRUPTS(1);
  }
#endif
// MOD Trax END

#if HAS_SLAVE_UART
  if (mode_ == RS485) {
    // If this is an RS485 pin, set up the RX and TX enable control lines.
    TX_Enable.setDirection(true);
    RX_Enable.setDirection(true);
    RX_Enable.setValue(false); // Active low
    listen();

    loopback_bytes = 0;
  }
#endif
}

#if HAS_SLAVE_UART
// Reset the UART to a listening state.  This is important for
// RS485-based comms.
void UART::reset() {
  if (mode_ == RS485) {
    loopback_bytes = 0;
    listen();
  }
}
#endif

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__)

// Send and receive interrupts
ISR(USART_RX_vect) {
  static uint8_t byte_in;

  byte_in = UDR0;
  if (loopback_bytes > 0) {
    loopback_bytes--;
  } else {
    UART::getHostUART().in.processByte(byte_in);

// Workaround for buggy hardware: have slave hold line high.
#if ASSERT_LINE_FIX
    if (UART::getHostUART().in.isFinished() &&
        (UART::getHostUART().in.read8(0) ==
         ExtruderBoard::getBoard().getSlaveID())) {
      speak();
    }
#endif
  }
}

ISR(USART_TX_vect) {
  if (UART::getHostUART().out.isSending()) {
    loopback_bytes++;
    UDR0 = UART::getHostUART().out.getNextByteToSend();
  } else {
    listen();
  }
}

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1280__) ||            \
    defined(__AVR_ATmega2560__)

// Send and receive interrupts
ISR(USART0_RX_vect) { UART::getHostUART().in.processByte(UDR0); }

ISR(USART0_TX_vect) {
  if (UART::getHostUART().out.isSending()) {
    UDR0 = UART::getHostUART().out.getNextByteToSend();
  }
}

#ifdef ALTERNATE_UART
ISR(USART1_RX_vect) { UART::getHostUART().in.processByte(UDR1); }

ISR(USART1_TX_vect) {
  if (UART::getHostUART().out.isSending()) {
    UDR1 = UART::getHostUART().out.getNextByteToSend();
  }
}
#endif

// MOD Trax BEGIN
#ifdef UART_DEBUG
ISR(USART1_RX_vect) { UART::getHostUART().dbg_in.processByte(UDR1); }

ISR(USART1_TX_vect) {
  if (UART::getHostUART().dbg_out.isSending()) {
    UDR1 = UART::getHostUART().dbg_out.getNextByteToSend();
  }
}
#endif
// MOD Trax END

#if HAS_SLAVE_UART
ISR(USART1_RX_vect) {
  static uint8_t byte_in;

  byte_in = UDR1;
  if (loopback_bytes > 0) {
    loopback_bytes--;
  } else {
    UART::getSlaveUART().in.processByte(byte_in);
  }
}

ISR(USART1_TX_vect) {
  if (UART::getSlaveUART().out.isSending()) {
    loopback_bytes++;
    UDR1 = UART::getSlaveUART().out.getNextByteToSend();
  } else {
    _delay_us(10);
    listen();
  }
}
#endif

#endif
