/*
 * UartLoader.c
 *
 * Created: 17.03.2014 22:05:26
 *  Author: vanya
 */ 

#include <avr/io.h>
#include "UartLoader.h"

#define BOOTSIZE 2048
#define APP_END (FLASHEND - (BOOTSIZE * 2))

uint8_t gBuffer[SPM_PAGESIZE];

void ExecuteCommand_1();
void ExecuteCommand_2();
void init_avr();
void fill_page();
void unfill_page();
void run_task(uint8_t *Task);

bool TimerEvent;
bool Green_LED;
bool Red_LED;

uint8_t	Task[16];
uint8_t	data_buffer[256];
uint16_t page_address;

ISR(TIMER0_OVF_vect)
{
	uart_timer_isr ();
}


ISR(USART_RX_vect)
{
	uart_rxc_isr();
}


ISR(TIMER1_OVF_vect)
{
	Task[2] = '7';
	TimerEvent = true;
	TCNT1H=0x57;
	TCNT1L=0x40;
}

static inline uint16_t writeFlashPage(uint16_t waddr, uint8_t size)
{
	uint32_t pagestart = (uint32_t)waddr<<1;
	uint32_t baddr = pagestart;
	uint16_t data;
	uint8_t *tmp = gBuffer;

	do
	{
		data = *tmp++;
		data |= *tmp++ << 8;
		boot_page_fill(baddr, data);	// call asm routine.

		baddr += 2;			// Select next word in memory
		size -= 2;			// Reduce number of bytes to write by two
	}
	while (size);				// Loop until all bytes written

	boot_page_write(pagestart);
	boot_spm_busy_wait();
	boot_rww_enable();		// Re-enable the RWW section

	return baddr>>1;
}

static inline void eraseFlash(void)
{
	// erase only main section (bootloader protection)
	uint32_t addr = 0;
	while (APP_END > addr)
	{
		boot_page_erase(addr);		// Perform page erase
		boot_spm_busy_wait();		// Wait until the memory is erased.
		addr += SPM_PAGESIZE;
	}
	boot_rww_enable();
}

static inline uint16_t readFlashPage(uint16_t waddr, uint8_t size)
{
	uint32_t baddr = (uint32_t)waddr<<1;
	uint16_t data;
	uint8_t i;
	i = 0x00;

	do
	{

		#ifndef READ_PROTECT_BOOTLOADER
		#warning "Bootloader not read-protected"

		#if defined(RAMPZ)
		data = pgm_read_word_far(baddr);
		#else
		data = pgm_read_word_near(baddr);
		#endif

		#else
		// don't read bootloader
		if ( baddr < APP_END )
		{
			#if defined(RAMPZ)
			data = pgm_read_word_far(baddr);
			#else
			data = pgm_read_word_near(baddr);
			#endif
		}
		else
		{
			data = 0xFFFF; // fake empty
		}
		#endif
		
		gBuffer[i] = data;			// send LSB
		i++;
		gBuffer[i] =data >> 8;		// send MSB
		i++;

		baddr += 2;			// Select next word in memory
		size -= 2;			// Subtract two bytes from number of bytes to read
	}
	while (size);				// Repeat until block has been read
	
	return baddr>>1;
}


int main(void)
{
	
	init_avr();

	uart_init();

	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (1<<CS12) | (0<<CS11) | (0<<CS10);
	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (1<<TOIE1);

	sei();
	
	Task[0] = 0;
	
	while(1)
	{
		
		if(BytesRead != 0)
		{
			TCCR1B = 0x00; // stop timer1
			memcpy(data_buffer, uart_buffer, BytesRead);
			uart_flush();
			for(int i = 0; i < 16; i++)
			Task[i] = data_buffer[i];
			run_task(Task);
			for(int i = 0; i < 16; i++)
			Task[i] = 0;
			TCCR1B = 0x04; // run timer1
			TimerEvent = false;
		}
		
		if(TimerEvent)
		{
			TimerEvent = false;
			run_task(Task);
			for(int i = 0; i < 16; i++)
			Task[i] = 0;
		}
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();
	}
		
}


void init_avr()
{
	MCUCR |= 1<<IVCE;
	MCUCR = 1<<IVSEL;

	// Input/Output Ports initialization
	PORTB = 0xff;
	DDRB = 0xff;
	PORTC = 0xff;
	DDRC = 0xff;
	PORTD = 0xff;
	DDRD = 0xff;

	// Timer/Counter 0 initialization
	// Clock source: System Clock
	// Clock value: Timer 0 Stopped
	// Mode: Normal top=0xFF
	// OC0A output: Disconnected
	// OC0B output: Disconnected
	TCCR0A=(0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
	TCCR0B=(0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
	TCNT0=0x00;
	OCR0A=0x00;
	OCR0B=0x00;

	// Timer/Counter 1 initialization
	// Clock source: System Clock
	// Clock value: Timer1 Stopped
	// Mode: Normal top=0xFFFF
	// OC1A output: Disconnected
	// OC1B output: Disconnected
	// Noise Canceler: Off
	// Input Capture on Falling Edge
	// Timer1 Overflow Interrupt: Off
	// Input Capture Interrupt: Off
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
	ICR1H=0x00;
	ICR1L=0x00;
	OCR1AH=0x00;
	OCR1AL=0x00;
	OCR1BH=0x00;
	OCR1BL=0x00;

	// Timer/Counter 2 initialization
	// Clock source: System Clock
	// Clock value: Timer2 Stopped
	// Mode: Normal top=0xFF
	// OC2A output: Disconnected
	// OC2B output: Disconnected
	ASSR=(0<<EXCLK) | (0<<AS2);
	TCCR2A=(0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (0<<WGM21) | (0<<WGM20);
	TCCR2B=(0<<WGM22) | (0<<CS22) | (0<<CS21) | (0<<CS20);
	TCNT2=0x00;
	OCR2A=0x00;
	OCR2B=0x00;

	// Timer/Counter 0 Interrupt(s) initialization
	TIMSK0=(0<<OCIE0B) | (0<<OCIE0A) | (0<<TOIE0);

	// Timer/Counter 1 Interrupt(s) initialization
	TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (0<<OCIE1A) | (0<<TOIE1);

	// Timer/Counter 2 Interrupt(s) initialization
	TIMSK2=(0<<OCIE2B) | (0<<OCIE2A) | (0<<TOIE2);

	// External Interrupt(s) initialization
	// INT0: Off
	// INT1: Off
	// Interrupt on any change on pins PCINT0-7: Off
	// Interrupt on any change on pins PCINT8-14: Off
	// Interrupt on any change on pins PCINT16-23: Off
	EICRA=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);
	EIMSK=(0<<INT1) | (0<<INT0);
	PCICR=(0<<PCIE2) | (0<<PCIE1) | (0<<PCIE0);

	// USART initialization
	// USART disabled
	UCSR0B=(0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

	// Analog Comparator initialization
	// Analog Comparator: Off
	// The Analog Comparator's positive input is
	// connected to the AIN0 pin
	// The Analog Comparator's negative input is
	// connected to the AIN1 pin
	ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
	ADCSRB=(0<<ACME);
	// Digital input buffer on AIN0: On
	// Digital input buffer on AIN1: On
	DIDR1=(0<<AIN0D) | (0<<AIN1D);

	// ADC initialization
	// ADC disabled
	ADCSRA=(0<<ADEN) | (0<<ADSC) | (0<<ADATE) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);

	// SPI initialization
	// SPI disabled
	SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

	// TWI initialization
	// TWI disabled
	TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);


	TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
	TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
	TCNT1H=0x00;
	TCNT1L=0x00;
}

void fill_page()
{
for (int i = 0; i < SPM_PAGESIZE; i ++)
{
	gBuffer[i] = i;
}
}

void blink_led()
{
	if(!Red_LED)
	{
		ClearBit(PORTD,7);
		Red_LED = true;
	}
	else
	{
		SetBit(PORTD,7);
		Red_LED = false;
	}
}

static void (*jump_to_app)(void) = 0x0000;

void run_task(uint8_t *Task)
{
	switch(Task[2])
	{
		case '0':
		send_string("boot");
		break;
		case '1':
		page_address = Task[9] << 8;
		page_address |= Task[8];
		readFlashPage(page_address, 128);
		send_buff(gBuffer, 128);
		Task[2] = 0;
		break;
		case '2':
		Task[2] = 0;
		break;
		case '3':
		page_address = Task[9] << 8;
		page_address |= Task[8];
		for (uint16_t i = 0; i < SPM_PAGESIZE; i++)
		gBuffer[i] = data_buffer[i + 16];
		writeFlashPage(page_address,SPM_PAGESIZE);
		send_string("page done");
		Task[2] = 0;
		break;
		case '4':
		eraseFlash();
		send_string("erase ok");
		Task[2] = 0;
		break;
		case '5':
		Task[2] = 0;
		break;
		case '6':
		fill_page();
		send_buff(gBuffer, 256);
		Task[2] = 0;
		break;
		case '7':
		blink_led();
		Task[2] = 0;
		break;
		case '8':
		MCUCR |= 1<<IVCE;
		MCUCR = 0<<IVSEL;
		send_string("jump ok");
		jump_to_app();		// Jump to application sector
		Task[2] = 0;
		break;
		default:
		Task[2] = 0;
	}
}