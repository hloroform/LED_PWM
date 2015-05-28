/*
 * ISR.c
 *
 * Created: 12.02.2014 19:10:07
 *  Author: vanya
 */ 

#include "Uart.h"
bool Green_LED;


uint16_t uart_analizePacket()
{
	uint16_t j = 0;
	if(uart_buffer_index >= 0x4)
	{
		if(uart_buffer[uart_buffer_index-1] == 0x7E)
		{
			for (uint16_t i = 0; i < (uart_buffer_index-1); i++)
			    {
					if (uart_buffer[i] == 0x7d)
					{
						uart_buffer[j] = (uart_buffer[(i + 1)] ^ 0x20);
						i++;
					}
					else
					{
						uart_buffer[j] = uart_buffer[i];
					}
						j++;
				}
				
			uint16_t crc = crc16_image(uart_buffer, j - 2);
			if ((crc >> 8) == uart_buffer[j - 2])
			{
				if ((crc & 0x00ff) == uart_buffer[j - 1])
				{
					return j - 2;
				}
				else
					return 0;
			}
			else
				return 0;
		}
		else
			return 0;
	}
	else
		return 0;
}

void uart_transmit( uint8_t data )
{
	/* Wait for data to be transmitted */
	while ( !(UCSR0A & (1<<UDRE0)) );
	UDR0 = data;
}



void uart_init()
{
	
	uart_buffer_index = 0;
	Prescaller = RUN_TIMER;
	ReloadValue = RELOAD_TIMER_VALUE;

	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: Off
	// USART Mode: Asynchronous
	// USART Baud Rate: 3840
	UCSR0A=(0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (DoubleSpeed<<U2X0) | (0<<MPCM0);
	UCSR0B=(1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
	UCSR0C=(1<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
	UBRR0H = UBRH;
	UBRR0L = UBRL;

	// Timer(s)/Counter(s) Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (1<<TOIE0);
	StopTimerT0;
}

void uart_flush()
{
	for (uart_buffer_index = 0; uart_buffer_index < UART_BUFFER_SIZE; uart_buffer_index++)
		uart_buffer[uart_buffer_index] = 0;
	uart_buffer_index = 0;
	BytesRead = 0;
	EnableUARTReciver;
}

void uart_timer_isr()
{
	DisableUARTReciver;
	StopTimerT0;
	BytesRead = uart_analizePacket();
	if( BytesRead == 0)
	uart_flush();
}

void uart_rxc_isr()
{
	ReloadTimerT0;// Reload timer for next byte timeframe ~ 30ms
	uart_buffer[uart_buffer_index] = UDR0;
	uart_buffer_index++;
	if (uart_buffer_index >= UART_BUFFER_SIZE) uart_buffer_index = 0;
	RunTimerT0;	
	
}

void uart_activity()
{
	if(!Green_LED)
	{
		ClearBit(PORTD,7);
		Green_LED = true;
	}
	else
	{
		SetBit(PORTD,7);
		Green_LED = false;
	}

	
}

void send_buff(uint8_t *buffer, uint16_t length)
{
	uart_activity();
	uint16_t crc = crc16_image(buffer, length);
	
	for (uint16_t i = 0; i < length; i++)
	{
		if((buffer[i] == 0x7e)|(buffer[i] == 0x7d))
		{
			uart_transmit(0x7d);
			uart_transmit((buffer[i] ^ 0x20));
		}
		else
		{
		uart_transmit(buffer[i]);
		}
	}

	if (((crc >> 8) == 0x7e)|((crc >> 8) == 0x7d))
	{
		uart_transmit(0x7d);
		uart_transmit((crc >> 8) ^ 0x20);
	}
	else
	{
		uart_transmit((crc >> 8));
	}
	
	if (((crc & 0xff) == 0x7e)|((crc & 0xff) == 0x7d))
	{
		uart_transmit(0x7d);
		uart_transmit((crc & 0xff) ^ 0x20);
	}
	else
	{
		uart_transmit((crc & 0xff));
	}
	uart_transmit(0x7e);
	uart_activity();
}

void send_string(const char *_String)
{
	char str[16];
	sprintf(str,_String);
	uart_activity();
//	for (uint8_t i = 0; i < 16; i++)
//	{
//		uart_transmit(str[i]);
//	}
	send_buff(str,16);
	uart_activity();
}

uint16_t crc16_image(uint8_t  *a, uint16_t Length)
        {
	        uint16_t  j, crc;
	        uint8_t i;
	        crc = 0xffff;
	        for (j = 0; j < Length; j++)
	        {
		        crc ^= a[j];
		        for (i = 0; i < 8; ++i)
		        {
			        if ((crc & 1) == 1)
			        crc = (crc >> 1) ^ 0x8408;
			        else
			        crc = (crc >> 1);
		        }
	        }

	        crc = (~(((crc & 0xff) << 8) | (crc >> 8)));
	        return crc;
        }
		
void uart_crc_transmit(uint8_t data)
{
	uint16_t crc;
	uint8_t i;
	crc = 0xffff;
	
	uart_activity();
	
	crc ^= data;
		for (i = 0; i < 8; ++i)
		{
		if ((crc & 1) == 1)
			crc = (crc >> 1) ^ 0x8408;
		else
			crc = (crc >> 1);
		}
    crc = (~(((crc & 0xff) << 8) | (crc >> 8)));
	if ((data == 0x7e)|(data == 0x7d))
	{
		uart_transmit(0x7d);
		uart_transmit(data ^ 0x20);
	}
	else
	{
		uart_transmit(data);
	}

	if (((crc >> 8) == 0x7e)|((crc >> 8) == 0x7d))
	{
		uart_transmit(0x7d);
		uart_transmit((crc >> 8) ^ 0x20);
	}
	else
	{
		uart_transmit((crc >> 8));
	}
	
	if (((crc & 0xff) == 0x7e)|((crc & 0xff) == 0x7d))
	{
		uart_transmit(0x7d);
		uart_transmit((crc & 0xff) ^ 0x20);
	}
	else
	{
		uart_transmit((crc & 0xff));
	}
	uart_transmit(0x7e);
	uart_activity();
}
