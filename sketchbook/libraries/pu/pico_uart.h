#ifndef PICO_UART_H_
#define PICO_UART_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// PICO UART
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t	pu_open(uint32_t baudrate);
void		pu_close(void);
uint8_t	pu_push(uint8_t byte);
uint8_t	pu_send(void);
uint8_t	pu_pull(void);
uint8_t	pu_peek(void);
void		pu_clear(void);
uint8_t	pu_poll(void);
uint8_t	pu_print(uint8_t num);

#define PICO_UART_BOOT_ADDRESS			0
uint8_t	pu_boot(void);

#define PICO_UART_BUFFER_LENGTH			16

volatile uint8_t pu_uRWrite = 0, pu_uRRead = 0;
volatile uint8_t pu_bRead[PICO_UART_BUFFER_LENGTH];

volatile uint8_t pu_uWWrite = 0, pu_uWRead = 0;
volatile uint8_t pu_bWrite[PICO_UART_BUFFER_LENGTH];

volatile uint8_t pu_uBoot = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_open(uint32_t baudrate)
{
	eeprom_write_byte((unsigned char*)PICO_UART_BOOT_ADDRESS, 0);
	pu_close();

	// Double USART Transmission Speed
	// RXD Interupt enable, RXD enable, TXD enable
	// Asynchronous USART, No Parity, One Stop Bit, Eight Data Bits, Rising Edge
	// Baud rate setting
	UCSR0A = (0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (1<<U2X0) | (0<<MPCM0);
	UCSR0B = (1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C = (0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H = ((F_CPU/(baudrate<<3))-1)>>8;
	UBRR0L = ((F_CPU/(baudrate<<3))-1);

	pu_clear();

	return 1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pu_close(void)
{
	// RXD Interupt disable, RXD disable, TXD disable
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_push(uint8_t byte)
{
	if (pu_uWWrite<(PICO_UART_BUFFER_LENGTH-1))
	{
		pu_bWrite[pu_uWWrite++] = byte;
		return (PICO_UART_BUFFER_LENGTH-pu_uWWrite);
	}
	else
		return 0xFF;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_send(void)
{
//	if (USCR0A&(1<<UDRE0))
//	while ( !(UCSR0A&(1<<UDRE0)) );
//	UDR0 = byte;

	if (pu_uWRead==pu_uWWrite)
		return 0;

	UDR0 = pu_bWrite[pu_uWRead++];

	if (pu_uWRead>(PICO_UART_BUFFER_LENGTH-1))
		pu_uWRead = 0;

	UCSR0B |= (uint8_t)(1<<UDRIE0);

	return 1;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_pull(void)
{
	if (pu_uRWrite==pu_uRRead)
		return 0xFF;

	uint8_t data = pu_bRead[pu_uRRead++];

	if (pu_uRRead>(PICO_UART_BUFFER_LENGTH-1))
		pu_uRRead = 0;

	return data;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_peek(void)
{
	if (pu_uRWrite==pu_uRRead)
		return 0xFF;

	uint8_t data = pu_bRead[pu_uRRead];

	return data;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void pu_clear(void)
{
	pu_uRWrite = 0;
	pu_uRRead = 0;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_poll(void)
{
	if (pu_uRRead<=pu_uRWrite)
		return (uint8_t) (pu_uRWrite-pu_uRRead);
	else
		return (uint8_t) (PICO_UART_BUFFER_LENGTH-(pu_uRRead-pu_uRWrite));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_print(uint8_t str)
{
	uint8_t sreg = SREG;
	cli();

	if (str == 1)
	{
		char stst[] = "\nDebug: booted.\n";
		uint8_t len = 16, lim = 0;

		while(lim<len)
		{
			while ( !(UCSR0A&(1<<UDRE0)) );
			UDR0 = stst[lim++];
		}
	}
	else if (str == 2)
	{
		char stst[] = "\nDebug: had Watchdog reboot.\n";
		uint8_t len = 29, lim = 0;

		while(lim<len)
		{
			while ( !(UCSR0A&(1<<UDRE0)) );
			UDR0 = stst[lim++];
		}
	}

	SREG=sreg;
	return str;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
uint8_t pu_boot(void)
{
	return eeprom_read_byte((unsigned char *) PICO_UART_BOOT_ADDRESS);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ISR(USART_RX_vect)
{
	uint8_t sreg = SREG;
	cli();

	uint8_t temp;
	temp = UDR0;

	if (temp=='#')
	{
		pu_uBoot++;
		if (pu_uBoot>15)
		{
			eeprom_write_byte((unsigned char*) PICO_UART_BOOT_ADDRESS, 1);
			wdt_enable(WDTO_15MS);
			while(1);
		}
	}
	else
	{
		pu_uBoot = 0;

		pu_bRead[pu_uRWrite++] = temp;

		if (pu_uRRead==pu_uRWrite)
			pu_uRRead++;
		if (pu_uRWrite>=(PICO_UART_BUFFER_LENGTH-1))
			pu_uRWrite = 0;
		if (pu_uRRead>=(PICO_UART_BUFFER_LENGTH-1))
			pu_uRRead = 0;
	}
	SREG=sreg;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ISR(USART_UDRE_vect)
{
	uint8_t sreg = SREG;
	cli();

	if (pu_uWRead==pu_uWWrite)
	{
		UCSR0B &= (uint8_t)~(1<<UDRIE0);
		pu_uWRead=0;
		pu_uWWrite=0;
	}
	else
	{
		UDR0 = pu_bWrite[pu_uWRead++];

//		if (pu_uWRead>(PICO_UART_BUFFER_LENGTH-1))
//			pu_uWRead = 0;
	}
	SREG=sreg;
}



#endif

