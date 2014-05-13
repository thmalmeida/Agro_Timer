// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)		SCK
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12) 		MISO
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM	MOSI
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM	SS
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+
//
#include <Arduino.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <compat/deprecated.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

// 0 - just power off automatically
// 1 - power on and off automatically
const uint8_t  mode = 0;
//const uint8_t debugMode = 1;
//#define debugMode
int timeWorkingSet = 80;			// Min - Timer working
int timeWaitingSet = (24*60-69);	// Min - Time to wait to recycle. 72 because 5% of delay of functions on ISR.


#ifdef debugMode
#include "LiquidCrystal.h"
LiquidCrystal lcd(21, 20, 19, 18, 17, 16);
#endif

enum{
	WAITING,
	TURN_ON,
	DECREASING,
	TURN_OFF
};
uint8_t state = TURN_OFF;

// Volatile because it's changed by the interruption (its for time working)
static volatile uint8_t flag_decreasing = 0;
static volatile uint8_t flag_timeOver1 = 0;
static volatile uint8_t flag_timeOver2 = 0;
static volatile uint8_t flag_status	= 0;
static volatile int minutes = 0;
static volatile uint8_t seconds = 0;
static volatile int minutes2 = 0;
static volatile uint8_t seconds2 = 0;

#define BUTTON_PORT PORTD       /* PORTx - register for button output */
#define BUTTON_PIN PIND         /* PINx - register for button input */
#define BUTTON_BIT PD2          /* bit for button input/output */

#define DEBOUNCE_TIME 25        /* time to wait while "de-bouncing" button */
#define LOCK_INPUT_TIME 250     /* time to wait after a button press */

#define k3_read (~PIND & 0b00000100)
#define buttonRead	bit_is_set(PIND, 2)

int button_is_pressed()
{
	/* the button is pressed when BUTTON_BIT is clear */
	if (bit_is_clear(BUTTON_PIN, BUTTON_BIT))
	{
		_delay_ms(20);
		if (bit_is_clear(BUTTON_PIN, BUTTON_BIT))
			return 1;
	}
	return 0;
}
#ifdef debugMode
void summary()
{
	char buffer[16];

	lcd.setCursor(0,0);
	if(flag_status)
		lcd.print("Ligado   ");
	else
		lcd.print("Desligado");

	// Model
	// |Desligado      2|
	// |  1:03      3:40|

	// Time printing.
	lcd.setCursor(0,1);
	sprintf(buffer,"%3.u:%.2u   %4.u:%.2u",minutes,seconds,minutes2,seconds2);
	lcd.print(buffer);


	lcd.setCursor(11,0);
	sprintf(buffer,"%d",k3_read);
	lcd.print(buffer);
}
#endif
void turn_on()
{
	// LEDs
	PORTB |= (1<<1);
	PORTB &= ~(1<<2);

	flag_status = 1;

	// Triacs
	PORTD |= (1<<3);
	PORTD |= (1<<4);
	_delay_ms(500);
	PORTD &= ~(1<<4);

	_delay_ms(3000);
}
void turn_off()
{
	// LEDs
	PORTB &= ~(1<<1);
	PORTB |= (1<<2);

	flag_status = 0;

	// Triacs
	PORTD &= ~(1<<3);
	PORTD &= ~(1<<4);
	_delay_ms(3000);
//	PORTD |= (1<<3);
}
void led_red_on(){
	PORTB |= (1<<1);
}
void led_red_off(){
	PORTB &= ~(1<<1);
}
void led_green_on(){
	PORTB |= (1<<2);
}
void led_green_off(){
	PORTB &= ~(1<<2);
}
void timer1_enable()
{
	// mode [10] PWM, Phase correct
	// I want a 1Hz clock to create a 1 second.
	// Dividing the 8MHz clock by 256
	// And put the top ICR to 31250/2.

	//	TCCR1A = COM1A1 COM1A0 COM1B1 COM1B0 FOC1A FOC1B WGM11 WGM10
	TCCR1A = 0b00000010;
	//	TCCR1B = ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
	TCCR1B = 0b00010100;

	//	TIMSK = OCIE2 TOIE2 TICIE1 OCIE1A OCIE1B TOIE1 – TOIE0
	TIMSK = 0b00000100;

	ICR1 = 15625;

	OCR1A = 15625;
}
void timer1_disable() {
	TCCR1B = 0;
	TIMSK = 0;

}
void init_all()
{
	// Enable 1Hz counter.
//	timer1_enable();

	// set output for red (PB1) and green (PB2) LED.
	DDRB |= (1<<1) | (1<<2);
	// set output for triac driver.
	DDRD |= (1<<3) | (1<<4);
	// set input for push button.
	DDRD &= ~(1<<2);
	/* turn on internal pull-up resistor for the switch */
	BUTTON_PORT |= _BV(BUTTON_BIT);

	minutes  = timeWorkingSet;
	seconds  = 0;
	minutes2 = timeWaitingSet;
	seconds2 = 0;
}
//ISR(INT0_vect)
//{
//	estado = 1;
//	_delay_ms(200);
//}
ISR(TIMER1_OVF_vect)
{
	if(flag_decreasing)
	{
		if(seconds == 0)
		{
			minutes--;
			seconds = 59;
		}
		else
			seconds--;

		if((!seconds)&(!minutes))
		{
			flag_timeOver1 = 1;
		}
	}

	if(mode)
	{
		if(seconds2 == 0)
		{
			minutes2--;
			seconds2 = 59;
		}
		else
			seconds2--;

		if((!seconds2)&(!minutes2))
		{
			flag_timeOver2 = 1;
		}
	}
}

int main(void) {

	init();

	#ifdef debugMode
	lcd.begin(16,2);
	#endif

	init_all();

	turn_off();
	_delay_ms(500);
	turn_off();
	_delay_ms(500);

    while(1)
	{
		#ifdef debugMode
    	summary();
		#endif



    	if(button_is_pressed())
    	{
    		if(state == DECREASING)
    		{
    			flag_decreasing = 0;
    			flag_timeOver1 = 0;
    			state = TURN_OFF;
    		}
    		else
    		{
    			state = TURN_ON;
				#ifdef debugMode
    			lcd.begin(16,2);
				#endif
			}
    	}

    	switch (state)
    	{
    		case WAITING:
    			if(flag_timeOver2&mode)
    			{
    				flag_timeOver2 = 0;
    				state = TURN_ON;
    			}
    			break;

    		case TURN_ON:
    			timer1_enable();
    			turn_on();
    			minutes  = timeWorkingSet;
    			minutes2 = timeWaitingSet;
    			seconds  = 0;
    			seconds2 = 0;
    			flag_decreasing = 1;

    			state = DECREASING;
    			break;

    		case DECREASING:
    			if(flag_timeOver1)
    			{
    				flag_decreasing = 0;
    				flag_timeOver1 = 0;
    				state = TURN_OFF;
    			}
    			break;

    		case TURN_OFF:
    			timer1_disable();
    			state = WAITING;
    			turn_off();
    			break;
    	}
	}
	return 0;
}




//void ext_int0_enable(){
//
//	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
//	// PD2 (INT0 pin) is now an input
//
//	PORTD |= (1 << PORTD2);    // turn On the Pull-up
//	// PD0 is now an input with pull-up enabled
//
//
//	MCUCR &= ~(1<<ISC01) | ~(1<<ISC00);    // set INT0 to trigger on ANY logic change
//	GICR |= (1 << INT0);      // Turns on INT0
//
//	sei();                    // turn on interrupts
//}
