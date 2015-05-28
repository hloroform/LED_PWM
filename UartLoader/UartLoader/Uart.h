
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include <avr/io.h>

#include "Defines.h"
																	
#define UART_BUFFER_SIZE 548									// 1 - no prescaller   2 - 8 prescaller   
#define STOP_TIMER	0											// 3 - 64 prescaller  4 - 256 prescaller
#define RUN_TIMER	4											// 5 - 1024 prescaller
#define RELOAD_TIMER_VALUE 0x70									//30ms 8Mhz 1024 prescaller   0x16
#define StopTimerT0	TCCR0B = STOP_TIMER							//30ms 7.3728Mhz 1024 prescaller   0x28    500us 7.3728Mhz 64 prescaller  0xc6
#define RunTimerT0	TCCR0B = Prescaller							// 50us 7.3728Mhz 8 prescaller  0xd2		5ms 7.3728Mhz 256 prescaller  0x70
#define ReloadTimerT0	TCNT0 = ReloadValue
#define UBRL 0x02												// 8 Mhz  19200 0x0019 normal mode
#define UBRH 0x00												// 7.3728Mhz 115200  0x0003  normal mode
#define DoubleSpeed 1											// 7.3728Mhz 230400  0x0001 normal mode
																// 11.0592Mhz 460800 0x0002 Double speed mode  

#define DisableUARTReciver		ClearBit(UCSR0B,RXEN0)
#define EnableUARTReciver		SetBit(UCSR0B,RXEN0)

uint8_t Prescaller;
uint8_t ReloadValue;

uint16_t BytesRead;
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint16_t uart_buffer_index;

uint16_t uart_analizePacket(void);
void uart_transmit( uint8_t data );
void uart_init(void);
void uart_timer_isr();
void uart_rxc_isr();
void send_buff(uint8_t *buffer, uint16_t length);
void send_string(const char *_String);
uint16_t crc16_image(uint8_t  *a, uint16_t Length);
void uart_crc_transmit(uint8_t data);
void uart_flush();