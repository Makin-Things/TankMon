//==============================================================================================================================
// P R E S S U R E   S E N S I N G   T A N K   M O N I T O R
//
// Copyright	: 2011 ProAtomic Software Development Pty Ltd
// File Name	: "TankMon.c"
// Title 			: Tank Monitor
// Date 			: 14 October 2011
// Version 		: 110
// Target MCU : ATMEGA168PA
// Author			: Simon Ratcliffe
//
//==============================================================================================================================
//
// Description of port usage
//
// Port/Bit	Direction	Usage
// --------	---------	----------------------------------------------------------
// C.0			Input			Pressure Sensor Input (ADC0)
// C.1			Input			Battery voltage (ADC1)
// C.2			Input			Power Good (PG) from BQ24210
// C.3			Input			Charge (CHG) from BQ24210
//
// D.0			Input			Serial input
// D.1			Output		Serial output
// D.2			Output		RF enable
// D.3			In/Out		1 Wire Temperature Sensor
// D.4			Output		Analog Power Enable
// D.5			Input			Float switch
// D.6			Output		Dorji RF Module Set pin
//
//==============================================================================================================================
//
// Packet format (type 1 device)
//
// Address	Function		Comments
// -------	----------	--------------------------------------------------------
// 00:06		Sensor id		Unique code for each sensor (from DS18B20)
// 07:07		Device type	Indicates the type of device (2 = Watermark)
// 08:08		Version Major Contains the major version of the firmware
// 09:09    Version Minor	Contains the minor version of the firmware
// 10:11		Packet #			Holds the packet sequence number
// 12:12		Type					Used to identify the type of packet (see below)
// 13:xx		Data					Variable data (dependent on Type)
// yy:yy		CRC						Crc added to end of packet
//
// Type			Function		Comments
// -------	----------	--------------------------------------------------------
// 00				Init				Sent when first powered up to pair sensor
// 01				Error				Data contains an error code
// 02				Data				Data contains the pressure,full,battv,chargev,temp
//
// Data Payloads
//
// Packet type 0
//
// Address	Function			Comments
// -------	-------------	--------------------------------------------------------
// No additional data for this type of packet
//
// Packet type 1
//
// Address	Function			Comments
// -------	-------------	--------------------------------------------------------
// 13:13		Error code		Contains the error code (see below)
//
// Packet type 2
//
// Address	Function			Comments
// -------	-------------	--------------------------------------------------------
// 13:14		Pressure			Contains the pressure reading
// 15:15		Full					The status of the full switch
// 16:17		Voltage				The battery voltage (1024 = 5v)
// 18:18		PG						The output from the power good pin
// 19:19		CHG						The output from the charging pin
// 20:21		Temp					Temperature from DS18B20 in 1/16 steps
//
// Error		Description
// -------	--------------------------------------------------------------------
// 01				DS18B20 not responding
// 02				DS18B20 ROM CRC error
// 03				DS18B20 scratchpad CRC error
//
//==============================================================================================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "Version.h"
#include "TankMon.h"

//==============================================================================================================================
//
// Constant defines
//
//==============================================================================================================================

#define DEBUG					1

#define dev_type			1

#define	TY_Init				0x00
#define	TY_Error			0x01
#define	TY_Data				0x02

// Error codes
#define e_ds18b20_no_response			1	// DS18B20 not responding
#define e_ds18b20_rom_crc_err			2 // DS18B20 ROM CRC error
#define e_ds18b20_scratch_crc_err	3	// DS18B20 scratch pad CRC error

// DS1820 Command bytes
#define ReadROM      	0x33
#define ConvertTemp  	0x44
#define RdScratch   	0xbe

//==============================================================================================================================
//
// Pin name defines
//
//==============================================================================================================================

// PORTC
#define PRES					0 // Pressure input
#define BATT					1 // Battery voltage input
#define PG						2 // Power good from BQ24210
#define CHG						3 // Charge from BQ24210

// PORTD
#define DRX						0 // Data receive
#define DTX						1 // Data transmit
#define RFEN					2 // RF enable; 0 = off, 1 = on
#define DSBUS   			3 // 1-wire bus
#define SWVCC					4 // Power to the external circuitry; 0 = on, 1 = off
#define FULL					5 // Tank full switch
#define RFSET					6 // Set pin of Dorji RF module

//==============================================================================================================================
//
// Typedefs
//
//==============================================================================================================================

typedef void (*f_ptr_t)(void);

//==============================================================================================================================
//
// Global Variables
//
//==============================================================================================================================

static FILE mystdout = FDEV_SETUP_STREAM(Uart_PutChar, NULL,_FDEV_SETUP_WRITE);

uint16_t packet = 0;
char strbuf[22];
char outbuf[100];
char* outbufptr;
uint8_t crc = 0;
uint8_t crc_start = 0;
uint8_t romdta[8];
uint8_t ScratchPad[9];
volatile uint8_t wdt_count = 0;
volatile uint32_t accumulator;
volatile uint16_t samples;
volatile uint16_t reading[256];
uint16_t voltage;
volatile char rxbuf[32];
volatile uint8_t rxptr = 0;

//==============================================================================================================================
//
// Initialisation routines
//
//==============================================================================================================================

void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));

void get_mcusr(void)  // protect against initial watchdog timeouts
{
	MCUSR = 0;
	wdt_disable();
}

//==============================================================================================================================
//
// Interrupt handlers
//
//==============================================================================================================================

ISR(WDT_vect)
{
	wdt_count++;
}

//==============================================================================================================================

ISR(ADC_vect)
{
	accumulator += ADC;
	reading[samples] = ADC;
}

//==============================================================================================================================

ISR(USART_RX_vect)
{
	rxbuf[rxptr] = UDR0;
	rxptr++;
	if (rxptr >= sizeof(rxbuf))
	{
		rxptr = 0;
	}
}

//==============================================================================================================================
//
// Functions
//
//==============================================================================================================================

static int Uart_PutChar(char c, FILE *stream)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

//==============================================================================================================================

static void Uart_Init()
{
	#define BAUD 9600
	#include <util/setbaud.h>
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif
	UCSR0B = _BV(TXEN0) | _BV(RXEN0) | _BV(RXCIE0);
	UCSR0C = _BV(UCSZ00) | _BV(UCSZ01);
}

//==============================================================================================================================

uint8_t DSReset ()
{
	uint8_t res;
	
	cli (); // Disable interrupts
	DDRD |= _BV(DSBUS); // DDRB bit 2 = 1 = output
	PORTD &= ~_BV(DSBUS); // set 1-wire bus low
	_delay_us (500); // Wait 500us
	DDRD &= ~_BV(DSBUS); // Switch DS bus bit to input (pulled up high)
	PORTD |= _BV(DSBUS); // turn on pullup
	_delay_us (100); // Wait 100 us for the DS1820 to respond
	res = PIND & _BV(DSBUS); // bit was clear (=0) so return true
	_delay_us (500); // Wait 500 us
	sei (); // Enable interrupts
	return res;
}

//==============================================================================================================================

uint8_t DSReadRom ()
{
	uint8_t i;

	crc = 0;
	
	cli (); // Disable interrupts
	PutByte (ReadROM); // Send the readrom command
	
	for (i = 0; i < 8; i++)
	{
		romdta[i] = GetByte (); // Get a byte
	}
	
	sei ();
	
	// check crc
	for (i = 0; i < sizeof romdta / sizeof romdta[0]; i++)
	{
		crc = _crc_ibutton_update (crc, romdta[i]);
	}
	
	if (crc)
	{
		SendError (e_ds18b20_rom_crc_err);
	}
	
	return (crc);
}

//==============================================================================================================================

uint8_t DSReadScratchPad ()
{
	uint8_t i;
	
	if (DSReset ())
	{
		return 1;
	}

	if (DSReadRom ())
	{
		return 2;
	}
	
	PutByte (ConvertTemp);
	while (!GetBit ())
	{
		_delay_us (500);
	}
	
	if (DSReset ())
	{
		return 1;
	}

	if (DSReadRom ())
	{
		//error 2
		return 2;
	}
	
	PutByte (RdScratch);
	
	crc = 0;

	for (i = 0; i < 9; i++)
	{
		ScratchPad[i] = GetByte ();
		crc = _crc_ibutton_update (crc, ScratchPad[i]);
	}
	
	if (crc)
	{
		// error 3
		return 3;
	}
	
	return 0;
}

//==============================================================================================================================

void PutByte (uint8_t c)
{
	uint8_t i;
	
	for (i = 0; i < 8; i++)
	{
		PutBit (c & 0x01);
		c >>= 1;
	}
}

//==============================================================================================================================

void PutBit (uint8_t b)
{
	DDRD |= _BV(DSBUS); // DDRD bit 2 = 1 = output
	PORTD &= ~_BV(DSBUS); // Pull Port D, bit 4 low
	_delay_us (10); // Wait 10us
	if (b)
	{
		DDRD &= ~_BV(DSBUS); // send a 1
		PORTD |= _BV(DSBUS); // turn on pullup
	}
	_delay_us (50); // and wait 50uS
	DDRD &= ~_BV(DSBUS); // this signals the end of a write 0
	PORTD |= _BV(DSBUS); // turn on pullup
	_delay_us (2); // and recover for 2uS
}

//==============================================================================================================================

uint8_t GetByte ()
{
	uint8_t res = 0;
	uint8_t i;
	
	for (i = 0; i < 8; i++)
	{
		res >>= 1;
		res |= GetBit ();
	}
	return res;
}

//==============================================================================================================================

uint8_t GetBit ()
{
	uint8_t res;
	
	DDRD |= _BV(DSBUS); // DDRD bit 2 = 1 = output
	PORTD &= ~_BV(DSBUS); // set 1-wire bus low
	_delay_us (2); // wait 2uS
	DDRD &= ~_BV(DSBUS); // DDRD bit 2 = 0 = input
	PORTD |= _BV(DSBUS); // turn on pullup
	_delay_us (10); // wait 10uS for the 1820 data
	res = (PIND & _BV(DSBUS)) << (7-DSBUS); // Read the value from the pin
	_delay_us (52);	// wait 52us for cycle to end and recovery time
	return res;
}

//==============================================================================================================================

char* CopyDataStr (char* dst, char* src, uint8_t len)
{
	uint8_t i;
	
	for (i = 0; i < len; i++)
	{
		crc = _crc_ibutton_update (crc, src[i]);
		
		*dst++ = src[i];
	}
	
	*dst++ = crc;
	
	return dst;
}

//==============================================================================================================================

uint16_t GetVoltage ()
{
	uint16_t retval;

	while(1)
	{
		// Get the battery voltage reading
		ADCSRA |= _BV(ADEN); // Turn on the ADC
		ADMUX = _BV(REFS0) | _BV(MUX0); // Select channel 1 and vcc ref
		ADCSRA |= _BV(ADSC); // Start the conversion
		while (!(ADCSRA & _BV(ADIF))); // Wait for conversion to complete
		retval = ADC; // Store analog value
		ADCSRA |= _BV(ADIF); // Clear the complete flag
		ADCSRA &= ~_BV(ADEN); // Turn off the ADC

		if (retval < 614) // 3.0v
		{
			while (wdt_count < 73) // about 5 minutes
			{
				sleep_mode ();
				WDTCSR |= _BV(WDIE);
			}

			wdt_count = 0;
		}
		else
		{
			return retval;
		}
	}
}

//==============================================================================================================================

uint8_t SendError (uint8_t ecode)
{
	if (ecode)
	{
		strbuf[10] = packet >> 8; // High byte of packet number
		strbuf[11] = packet & 0xFF; // Low byte of packet number
		strbuf[12] = TY_Error; // About to send an error code
		strbuf[13] = ecode; // Add the error code

		SendPacket (strbuf, 14); // Transmit the packet
	}

	return ecode;
}

//==============================================================================================================================

void SendPacket (char* szPacket, uint8_t len)
{
	uint8_t i, j, k;

	for (k = 0; k < 3; k++)
	{
		crc = crc_start; // Start with the crc of the SensorID

		putchar(len+2); // Send the total length of the packet (including the len and crc bytes)

		for (i = 0; i < len; i++)
		{
			if (i >= 10)
			{
				crc = _crc_ibutton_update (crc, szPacket[i]);
			}

			putchar(szPacket[i]); // Send the data
		}
		putchar(crc); // Send the crc byte
		
		rxptr = 0; // Empty the receive buffer

		_delay_ms(500); // Wait for transmission to end and give time for response to arrive
		crc = 0;
		for (j = 1; j < rxptr; j++)
		{
			crc = _crc_ibutton_update(crc, rxbuf[j]);
		}
		if ((rxptr == 10) && (rxbuf[0] == 10) && (!crc))
		{
			if (memcmp((void*)(rxbuf+1), strbuf, 7) == 0)
			{
				if (rxbuf[8] == 'a')
				{
					rxptr = 0;
					break;
				}
				if (rxbuf[8] == 'u')
				{
					cli();
					wdt_disable();
					asm volatile(  "jmp 0x3802\n"  );  // jump to boot-start
				}
			}
		}
		wdt_reset (); // Reset watchdog
		_delay_ms(750);
		rxptr = 0;
	}
	packet++; // Increment the packet number
}

//==============================================================================================================================

void SendReadings ()
{
	uint16_t i;
	putchar('$');
	for (i = 0; i < 256; i++)
	{
		if (i)
		{
			putchar(',');
		}
		printf ("%u", reading[i]);
	}
	printf ("\n");
}

//==============================================================================================================================
//
// Main program
//
//==============================================================================================================================

int main (void)
{
	uint16_t i;
	uint8_t j;
	uint16_t min;
	uint16_t max;
	
	// Setup I/O ports
	PORTC = _BV(PG) | _BV(CHG); // Enable pullups on PG and CHG
	DDRD = _BV(DTX) | _BV(RFEN) | _BV(DSBUS) | _BV(SWVCC) | _BV(RFSET); // Setup input/output pins on PORTD. 0=in 1=out.
	PORTD = _BV(FULL) | _BV(SWVCC) | _BV(RFSET) | _BV(RFEN); // Enable pullups on float switch and turn on analog power
	DIDR0 = _BV(ADC1D) | _BV(ADC0D); // Disable digital input buffers for ADC pins being used

	// Setup serial port
	Uart_Init();
	stdout = &mystdout;
	
	// Initialise Dorji module
	_delay_ms(100);
	PORTD &= ~_BV(RFSET);
	_delay_ms(10);
	printf("WR 433920 3 9 3 0\r\n");
	_delay_ms(500);
	PORTD |= _BV(RFSET);

	// Setup ADC
	ADMUX = _BV(REFS0); // Select channel 0 and vcc ref
	ADCSRA = _BV(ADPS1) | _BV(ADPS0); // Select div 8 prescaler

	// Setup Watchdog
	MCUSR &= ~_BV(WDRF);
	WDTCSR = _BV(WDCE) | _BV(WDE); // Allow the watchdog settings to be changed by the next line
	WDTCSR = _BV(WDE) | _BV(WDIE) | _BV(WDP3); // 4S with interrupt and system reset

	sei ();

	// Don't continue until the battery voltage is above 3.0v
	set_sleep_mode (SLEEP_MODE_PWR_DOWN);
	GetVoltage(); // Get the voltage, wont return until above 3.0v
	wdt_count = 0;

	PORTD |= _BV(RFEN); // Enable the RF module
	
	_delay_ms(500); //Wait for everything to power up properly
	
	do
	{
		while (DSReset ())
		{
			SendError (e_ds18b20_no_response);
		}
	}
	while (DSReadRom ());

	for (i = 0; i < (sizeof romdta / sizeof romdta[0])-1; i++)
	{
		crc_start = _crc_ibutton_update (crc_start, romdta[i]);
	}

	// build the packet header
	crc = 0;
	memcpy (strbuf, romdta, 7); // copy the sensor id
	strbuf[7] = dev_type; // Add the device type
	crc_start = _crc_ibutton_update (crc_start, dev_type);
	strbuf[8] = build >> 8; // Add the major version
	crc_start = _crc_ibutton_update (crc_start, build >> 8);
	strbuf[9] = build & 0xFF; // Add the minor version
	crc_start = _crc_ibutton_update (crc_start, build & 0xFF);
	strbuf[10] = 0; // add the packet #
	strbuf[11] = 0; // add the packet #
	strbuf[12] = TY_Init; // add the init byte

	SendPacket (strbuf, 13); // Transmit the packet
	UCSR0B &= ~_BV(RXEN0); // Disable USART receiver
	PORTD &= ~_BV(RFEN); // Disable RF module

	wdt_reset (); // Reset watchdog
	
	while (1)
	{
		voltage = GetVoltage(); // Wont return unless 3.0v

		if (!SendError (DSReadScratchPad ()))
		{
			PORTD |= _BV(SWVCC); // turn analog power on
			
			ADCSRA |= _BV(ADEN); // Turn on the ADC
			
			strbuf[10] = packet >> 8; // High byte of packet number
			strbuf[11] = packet & 0xFF; // Low byte of packet number
			strbuf[12] = TY_Data; // About to send data

			// Get the pressure reading
			_delay_ms (100); // Make sure the pressure sensor is warmed up
			accumulator = 0;
			min = 65535;
			max = 0;
			ADMUX = _BV(REFS0); // Select channel 0 and vcc ref
			ADCSRA |= _BV(ADIE); // Enable the ADC interrupt
			set_sleep_mode (SLEEP_MODE_ADC);

			for (j = 0; j < 4; j++)
			{
				for (samples = 0; samples < 256; samples++)
				{
					sleep_mode ();
				}
			
				for (i = 0; i < 256; i++)
				{
					if (reading[i] < min)
					{
						min = reading[i];
					}
					if (reading[i] > max)
					{
						max = reading[i];
					}
				}
			}
			
			PORTD &= ~_BV(SWVCC); // turn analog power off
			ADCSRA &= ~_BV(ADIE); // Disable ADC interrupt

			if (max-min >= 10) // bad reading so store 0xFFFF
			{
				strbuf[13] = 0xFF;// Store the high byte
				strbuf[14] = 0xFF; // Store the low byte
			}
			else
			{
				if (accumulator & 32)
				{
					accumulator >>= 6;
					accumulator++;
				}
				else
				{
					accumulator >>= 6;
				}
				strbuf[13] = accumulator >> 8;// Store the high byte
				strbuf[14] = accumulator & 0xFF; // Store the low byte
			}

			// Get the full switch reading
			strbuf[15] = (PIND & _BV(FULL)) ? 0 : 1; // Add the fullness byte

			// Get the battery voltage reading
			ADMUX = _BV(REFS0) | _BV(MUX0); // Select channel 1 and vcc ref
			ADCSRA |= _BV(ADSC); // Start the conversion
			while (!(ADCSRA & _BV(ADIF))); // Wait for conversion to complete
			strbuf[17] = ADCL; // Store the low byte (this must be read first)
			strbuf[16] = ADCH;// Store the high byte
			ADCSRA |= _BV(ADIF); // Clear the complete flag
			ADCSRA &= ~_BV(ADEN); // Turn off the ADC

			// Get the power good reading
			strbuf[18] = (PINC & _BV(PG)) ? 0 : 1; // Add the PG byte

			// Get the charging reading
			strbuf[19] = (PINC & _BV(CHG)) ? 0 : 1; // Add the charging byte

			// Store the temp that was retrieved by DSReadScratchPad
			strbuf[20] = ScratchPad[1]; // Add the Temperature MSB
			strbuf[21] = ScratchPad[0]; // Add the Temperature LSB
			
			UCSR0B |= _BV(RXEN0);  // Enable USART receiver
			PORTD |= _BV(RFEN); // Enable the RF module
			_delay_ms (50);  // wait 50ms so rf module can wake up

			SendPacket (strbuf, 22); // Transmit the packet
			
			_delay_ms (200); // wait a bit to give the usart time to finish sending

			printf("{min=%u max=%u", min, max);
				_delay_ms(100);

				UCSR0B &= ~_BV(RXEN0); // Disable USART receiver
				PORTD &= ~_BV(RFEN); // Disable RF module
				
				set_sleep_mode (SLEEP_MODE_PWR_DOWN);
			}

			while (wdt_count < 13) // 15 = 60s
			{
				sleep_mode ();
				WDTCSR |= _BV(WDIE);
			}
			
			wdt_count = 0;
		}
		return 0;
	}

	//==============================================================================================================================
