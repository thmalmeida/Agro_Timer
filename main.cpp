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
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
//#include <avr/eeprom.h>
//#include <compat/deprecated.h>
#include <Arduino.h>


// 0 - just power off automatically
// 1 - power on and off automatically
//const uint8_t  mode = 0;
//const uint8_t debugMode = 1;
//#define debugMode
//int timeWorkingSet = 80;			// Min - Timer working
//int timeWaitingSet = (24*60-69);	// Min - Time to wait to recycle. 72 because 5% of delay of functions on ISR.

enum states01 {
	redTime,
	greenTime
};
enum states01 periodo = redTime;

enum{
	timerState,
	automaticState
};
uint8_t stateMode = automaticState;

enum{
	WAITING,
	TURN_ON,
	DECREASING,
	TURN_OFF
};
uint8_t state = TURN_OFF;


// season times
uint8_t HourOn  = 21;
uint8_t MinOn   = 30;

uint8_t HourOff = 6;
uint8_t MinOff  = 0;

//uint8_t flag_02 = 0;
//uint8_t flag_03 = 0;

const uint8_t timeMinSet = 80;

const uint16_t timeSecSet = 60*timeMinSet;
volatile uint16_t timeCounter = 0;
volatile uint8_t flag_timeOver = 0;

volatile uint8_t timeCounterSync = 0;
volatile uint8_t flag_timeSync = 0;

const uint16_t timeMainSet = 6*60*60;
volatile uint16_t timeMainCounter;


// Sensor variables
uint8_t levelSensorMLL, levelSensorLL;
uint8_t flag_LL = 0, flag_MLL = 0;

// UART Communication
uint8_t j2 =0;
char inChar;
uint8_t flag_frameStart = 0;
char sInstrRx[10], sInstr[10];
uint8_t enableTranslate = 0;
uint8_t rLength = 0;
uint8_t enableDecode = 0;
uint8_t opcode = 0;
char aux[3];


uint8_t motorStatus = 0;

//#define BUTTON_PORT PORTD       /* PORTx - register for button output */
//#define BUTTON_PIN PIND         /* PINx - register for button input */
//#define BUTTON_BIT PD2          /* bit for button input/output */
//
//#define DEBOUNCE_TIME 25        /* time to wait while "de-bouncing" button */
//#define LOCK_INPUT_TIME 250     /* time to wait after a button press */

#define button_read	(~PIND & 0b00000100)
//#define buttonRead	bit_is_set(PIND, 2)

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
//	BUTTON_PORT |= _BV(BUTTON_BIT);
}
void init_ADC()
{
//	ADCSRA ==> ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	// Set 128 division clock
	ADCSRA |= (1<<ADEN); 							// Enable module

//	ADCSRB ==>	–	ACME	–	–	MUX5	ADTS2	ADTS1	ADTS0
//	ADCSRB &= ~(1<<ADTS2);							// Free running mode.
//	ADCSRB &= ~(1<<ADTS1);
//	ADCSRB &= ~(1<<ADTS0);

//	ADCSRB &= ~(1<<MUX5);							// To select ADC0;

//	ADMUX ==> REFS1 REFS0 ADLAR MUX4 MUX3 MUX2 MUX1 MUX0
	ADMUX &= ~(1<<REFS1);							// AVCC is the Vref
	ADMUX |=  (1<<REFS0);

	ADMUX &= ~(1<<ADLAR);							// Left Adjust result. To ADCH register.


//	ADMUX &= ~(1<<MUX4);							// Select ADC0
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	ADMUX |=  (1<<REFS0);							// Internal 2.56V reference
	ADMUX |=  (1<<REFS1);
}
void init_timer1()
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
void stop_timer1() {
	TCCR1B = 0;
	TIMSK = 0;
}

void motor_start()
{
	// LEDs
	PORTB |= (1<<1);
	PORTB &= ~(1<<2);

	// Triacs
	PORTD |= (1<<3);
	PORTD |= (1<<4);
	_delay_ms(500);
	PORTD &= ~(1<<4);

	motorStatus = 1;

	_delay_ms(3000);
}
void motor_stop()
{
	// LEDs
	PORTB &= ~(1<<1);
	PORTB |= (1<<2);

	motorStatus = 0;

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

float calcIrms()
{
	int i, j=0;
	uint8_t high, low;
	int divScale_count = 1;

	float I = 0.0;		// Current statement;
	float k = 2020.0;	// sensor constant;
	float R = 310.0;	// burden resistor [ohms];

	// Select ADC0 channel
	ADMUX &= ~(1<<MUX3);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	// ADC converter
	const float f = 60.0;			// signal frequency [Hz];
	const int numberOfCycles = 16;	// Number of cycles [cycles];
	const int divScale = 4;			// Prescale for real sample rate Fs [un];

	const float F_clk = 8000000.0;								// Crystal system clock;
	const float Fs = F_clk/128.0/13.0;							// Sample rate of signal processed;
	const int nPointsPerCycle = (int) Fs/f;						// Number of points per cycle;
	const int nPoints = (int) nPointsPerCycle*numberOfCycles; 	// Number of signal points.

	const float Fs_div = F_clk/128.0/13.0/divScale;						// Sample rate of signal processed;
	const int nPointsPerCycle_div = (int) Fs_div/f;						// Number of points per cycle;
	const int nPoints_div = (int) nPointsPerCycle_div*numberOfCycles;	// Number of signal points.


//	sprintf(buffer,"---- Signal Captured ----");
//	Serial.println(buffer);
//	Serial.println("");
//
//	Serial.print("Fs:");
//	Serial.println(Fs);
//	Serial.println("");
//
//	sprintf(buffer,", nPointsPerCycle:%d", nPointsPerCycle);
//	Serial.println(buffer);
//	Serial.println("");
//
//	sprintf(buffer,"nPoints:%d", nPoints);
//	Serial.println(buffer);
//	Serial.println("");
//
//
//
//	sprintf(buffer,"---- Signal Processed ----");
//	Serial.println(buffer);
//	Serial.println("");
//
//	Serial.print("Fs:");
//	Serial.println(Fs_div);
//	Serial.println("");
//
//	sprintf(buffer,", nPointsPerCycle:%d", nPointsPerCycle_div);
//	Serial.println(buffer);
//	Serial.println("");
//
//	sprintf(buffer,"nPoints:%d", nPoints_div);
//	Serial.println(buffer);
//	Serial.println("");


	int *adcSamples = NULL;
	adcSamples = (int*)malloc(nPoints_div * sizeof(int));

	// 160.2564 = 16000000/128/13/60.0;
	for(i=0;i<nPoints;i++)
	{
		ADCSRA |= (1<<ADSC);				// Start conversion;
		while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

//		Serial.println((ADCH << 8) | ADCL);

		if(divScale_count == 1)
		{
			low  = ADCL;
			high = ADCH;

			j = (int) i/divScale;
			adcSamples[j] = (high << 8) | low;
			divScale_count = divScale;
		}
		else
		{
			divScale_count--;
		}
	}

//	Serial.println("ENTROU!");
//	for(i=0;i<nPoints_div;i++)
//	{
//		Serial.println(adcSamples[i]);
//	}
//	Serial.println("SAIU!");


	float *vs = NULL;
	vs = (float*)malloc(nPoints_div * sizeof(float));

	for(i=0;i<nPoints_div;i++)
	{
		vs[i] = (adcSamples[i]*5.0)/1024.0;
	}

	free(adcSamples);

	// Offset remove.
	float Vmean = 0.0;
	for(i=0;i<nPoints_div;i++)
		Vmean += vs[i];

	Vmean = Vmean/nPoints_div;

	for(i=0;i<nPoints_div;i++)
		vs[i] = vs[i] - Vmean;

	float *vs2 = NULL;
	vs2 = (float*)malloc(nPoints_div * sizeof(float));

	// Power signal
	for(i=0;i<nPoints_div;i++)
		vs2[i] = vs[i]*vs[i];

	free(vs);

	float sum=0;
	float V2mean;

	// mean finder
	for(i=0;i<nPoints_div;i++)
		sum += vs2[i];
	V2mean = sum/nPoints_div;

	free(vs2);

	// RMS equation
	I = (k*sqrt(V2mean))/R;

	return I;
}

//int button_is_pressed()
//{
//	/* the button is pressed when BUTTON_BIT is clear */
//	if (bit_is_clear(BUTTON_PIN, BUTTON_BIT))
//	{
//		_delay_ms(20);
//		if (bit_is_clear(BUTTON_PIN, BUTTON_BIT))
//			return 1;
//	}
//	return 0;
//}

void timeSyncSendCommand()
{
	Serial.println("$02;");
}
void sensorRead_Level()
{
	uint8_t low, high;
	uint16_t value;
	const uint16_t reference = 300;

	ADMUX &= ~(1<<MUX3);							// Select ADC0
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	low  = ADCL;
	high = ADCH;
	value = (high << 8) | low;
//	levelSensorLL_d = value;

	if(value<reference)
		levelSensorLL = 1;
	else
		levelSensorLL = 0;


	ADMUX &= ~(1<<MUX3);							// Select ADC1
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX1);
	ADMUX |=  (1<<MUX0);

	ADCSRA |= (1<<ADSC);				// Start conversion;
	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;

	low  = ADCL;
	high = ADCH;
	value = (high << 8) | low;
//	levelSensorMLL_d = value;

	if(value<reference)
		levelSensorMLL = 1;
	else
		levelSensorMLL = 0;


//	ADMUX &= ~(1<<MUX3);							// Select ADC2
//	ADMUX &= ~(1<<MUX2);
//	ADMUX |=  (1<<MUX1);
//	ADMUX &= ~(1<<MUX0);
//
//	ADCSRA |= (1<<ADSC);				// Start conversion;
//	while (bit_is_set(ADCSRA, ADSC));	// wait until conversion done;
//
//	low  = ADCL;
//	high = ADCH;
//	value = (high << 8) | low;
//	levelSensorHL_d = value;
//
//	if(value<reference)
//		levelSensorHL = 1;
//	else
//		levelSensorHL = 0;
}
void motorControl_bySensors()
{
	float Irms;
	Irms = calcIrms();

	sensorRead_Level();

	if(levelSensorMLL)
	{
		if(!flag_MLL)
		{
			flag_LL = 0;
			flag_MLL = 1;

			motor_start();
		}
	}

	if(!levelSensorLL)
	{
		if(!flag_LL)
		{
			flag_LL = 1;
			flag_MLL = 0;

			motor_stop();
		}
	}
}
void motorPeriodDecision()
{
	switch (periodo)
	{
	case redTime:
		if(motorStatus)
		{
			motor_stop();

			flag_MLL = 0;
			flag_LL = 0;
		}
		break;

	case greenTime:
		motorControl_bySensors();
		break;
	}
}

void periodVerify2()
{
	if (timeMainCounter < timeMainSet)
	{
		periodo = greenTime;
//		flag_02 = 1;
//		flag_03 = 0;
	}
	else
	{
		periodo = redTime;
//		flag_02 = 0;
//		flag_03 = 1;
	}
}

void refreshVariables()
{
	if(flag_timeSync)
	{
		flag_timeSync = 0;
		timeSyncSendCommand();
	}

	periodVerify2();

	if(button_read)
	{
		if(state == DECREASING)
		{
			state = TURN_OFF;
		}
		else
		{
			stateMode = timerState;
			state = TURN_ON;
		}
	}
}
void processTimer()
{
	switch (state)
	{
		case WAITING:

			break;

		case TURN_ON:
			motor_start();
			timeCounter = 0;

			state = DECREASING;
			break;

		case DECREASING:
			if(flag_timeOver)
			{
				flag_timeOver = 0;
				state = TURN_OFF;
			}
			break;

		case TURN_OFF:
			motor_stop();
			flag_timeOver = 0;
			stateMode = automaticState;
			state = WAITING;

			break;
	}
}
void processExecution()
{
	switch(stateMode)
	{
		case timerState:
			processTimer();
			break;

		case automaticState:
			motorPeriodDecision();
			break;

		default:
			stateMode = automaticState;
			break;
	}
}

void handleMessage()
{
	if(enableDecode)
	{
		enableDecode = 0;

		// Getting the opcode
		aux[0] = '0';
		aux[1] = sInstr[0];
		aux[2] = '\0';
		opcode = (uint8_t) atoi(aux);

		switch (opcode)
		{
// -----------------------------------------------------------------
			case 1:		// Set-up clock
			{
				// Getting the parameters
				uint8_t tmHour, tmMinute;//, tmSecond;
				aux[0] = sInstr[1];
				aux[1] = sInstr[2];
				aux[2] = '\0';
				tmHour = (uint8_t) atoi(aux);

				aux[0] = sInstr[3];
				aux[1] = sInstr[4];
				aux[2] = '\0';
				tmMinute = (uint8_t) atoi(aux);

//				aux[0] = sInstr[5];
//				aux[1] = sInstr[6];
//				aux[2] = '\0';
//				tmSecond = (uint8_t) atoi(aux);

				timeMainCounter = tmHour*tmMinute;
//				timeMainCounter = tmHour*tmMinute*tmSecond;
			}
			break;
		}
		memset(sInstr,0,sizeof(sInstr));
	}
}

void comm_Serial()
{
	// Rx - Always listening
	while((Serial.available()>0))	// Reading from serial
	{
		inChar = Serial.read();

		if(inChar=='$')
		{
			j2 = 0;
			flag_frameStart = 1;
		}

		if(flag_frameStart)
			sInstrRx[j2] = inChar;

		j2++;

		if(j2>=sizeof(sInstrRx))
		{
			memset(sInstrRx,0,sizeof(sInstrRx));
			j2=0;
		}

		if(inChar==';')
		{
			if(flag_frameStart)
			{
				flag_frameStart = 0;
//				rLength = j2;
				j2 = 0;
				enableTranslate = 1;
			}
		}
	}

	if(enableTranslate)
	{
		enableTranslate = 0;

		char *pi0, *pf0;
		pi0 = strchr(sInstrRx,'$');
		pf0 = strchr(sInstrRx,';');

		if(pi0!=NULL)
		{
			uint8_t l0=0;
			l0 = pf0 - pi0;

			int i;
			for(i=1;i<=l0;i++)
			{
				sInstr[i-1] = pi0[i];
			}
			memset(sInstrRx,0,sizeof(sInstrRx));

			enableDecode = 1;
		}
//		else
//		{
//			Serial.println("Error BT!!");
//			Serial.write(pi0[0]);
//			Serial.write(pf0[0]);
//		}
	}
}

ISR(TIMER1_OVF_vect)
{
	if(timeCounterSync == 240)
	{
		flag_timeSync = 1;
		timeCounterSync = 0;
	}
	else
	{
		timeCounterSync++;
	}

	if(state == DECREASING)
	{
		if(timeCounter > timeSecSet)
		{
			flag_timeOver = 1;
		}
		else
		{
			timeCounter++;
		}
	}
	timeMainCounter++;
}


//ISR(INT0_vect)
//{
//	estado = 1;
//	_delay_ms(200);
//}
//ISR(TIMER1_OVF_vect)
//{
//	if(flag_decreasing)
//	{
//		if(seconds == 0)
//		{
//			minutes--;
//			seconds = 59;
//		}
//		else
//			seconds--;
//
//		if((!seconds)&(!minutes))
//		{
//			flag_timeOver1 = 1;
//		}
//	}
//
//	if(mode)
//	{
//		if(seconds2 == 0)
//		{
//			minutes2--;
//			seconds2 = 59;
//		}
//		else
//			seconds2--;
//
//		if((!seconds2)&(!minutes2))
//		{
//			flag_timeOver2 = 1;
//		}
//	}
//}

int main(void) {

	init();
	init_all();
	init_ADC();
	Serial.begin(9600);

	motor_stop();

    while(1)
	{
    	// refresh variables
    	refreshVariables();

    	// process mode
    	processExecution();

    	// UART communication
    	comm_Serial();

    	// handle message
    	handleMessage();
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
