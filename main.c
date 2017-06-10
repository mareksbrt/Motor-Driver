
/*
 * SPI-SLAVE.cpp
 *
 * Created: 2/16/2017 10:42:51 AM
 * Author : kun
*/
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define ACK 0x7E

#define Speed_Left	1
#define Speed_Right 2
#define Forwards	3
#define Backwards	4
#define Left		5
#define Right		6
#define Stop		7

void init_pwm(void);
void spi_slave_init(void);
void set_rover_speed(uint8_t speed_address);
void direction(uint8_t some_address);
void init_motor_pins(void);
uint8_t spi_send(uint8_t value);




int x = 0;



int main(void)
{
	DDRD = 0xff;
	init_pwm();
	spi_slave_init();
	init_motor_pins();

	


 /* Replace with your application code */
 while (1)
	{

spi_send(ACK);
set_rover_speed(SPDR);
direction(SPDR);

	
	//set_rover_direction();

		
	
	
	
	
	
	
//	set_rover_speed(SPEED_ADRESS); // only will work if SPEED_ADRESS is sent
//	set_rover_direction(); //only will work if DIRECTION_ADRESS is sent
	}
}

void init_pwm()
{
	DDRD |= (1 << PIND5);
	DDRD |= (1 << PIND6);
	// PD5 is now an output
	TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
	// set none-inverting mode and fast pwm
	TCCR0B |= (0<<CS02) | (1 << CS01) | (1<<CS00);
	// set prescaler to 64 and starts PWM
	TCNT0 = 0;
	//reset TCNT
	OCR0B = 0;
	OCR0A = 0;
	//Output compare registers A&B
	
}




void spi_slave_init (void)
{
	//Set MISO as output
	DDRB |= (1<<PINB4);
	//Enable SPI
	SPCR |= (1<<SPE);
	
}

void set_rover_speed(uint8_t speed_address)
{
	
	if ( SPDR == Speed_Left)
	{
		spi_send(ACK);
		OCR0A = SPDR;
		
	}
	
	if (SPDR == Speed_Right)
	{
		spi_send(ACK);
		OCR0B = SPDR;
	}
}



void init_motor_pins(void)
{
	DDRD |= (1<<PIND2); //Left motor +
	DDRD |= (1<<PIND3); //Left motor -
	DDRD |= (1<<PIND4); //Right motor +
	DDRD |= (1<<PIND7); //Right motor -
	
	PORTD &= ~(1<<PIND2) | (1<<PIND3) | (1<<PIND4) | (1<<PIND7); //Set all pins initially to 0;
}


uint8_t spi_send(uint8_t value)
{
	//shift the first byte of the value
	SPDR = value;
	//wait for the SPI bus to finish
	while(!(SPSR & (1<<SPIF)));
	
	//get the received data
	return SPDR;
	
}

void direction(uint8_t some_address)
{	
	if (some_address == Forwards)
	{	
		PORTD |= (1<<PIND2);
		PORTD &= ~(1<<PIND3);
		
		PORTD |= (1<<PIND4);
		PORTD &= ~(1<<PIND7);		
	}
	
	if (some_address == Backwards)
	{
		PORTD |= (1<<PIND3);
		PORTD &= ~(1<<PIND2);
		
		PORTD |= (1<<PIND7);
		PORTD &= ~(1<<PIND4);
	}
	
	if (some_address == Left)
	{
		PORTD |= (1<<PIND2);
		PORTD &= ~(1<<PIND3);
		PORTD &= ~(1<<PIND4);
		PORTD &= ~(1<<PIND7);
	}
	
	if (some_address == Right)
	{
		PORTD |= (1<<PIND4);
		PORTD &= ~(1<<PIND7);
		PORTD &= ~(1<<PIND2);
		PORTD &= ~(1<<PIND3);
	}
	
	
	if (some_address == Stop)
	{
		PORTD &= ~(1<<PIND2);
		PORTD &= ~(1<<PIND4);
		PORTD &= ~(1<<PIND3);
		PORTD &= ~(1<<PIND7);
	}
}