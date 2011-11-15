# define F_CPU 8000000UL 

#include <avr/io.h>
#include <avr/iom2561.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include "serial.h"

#define LED_0_PIN		6
#define LED_1_PIN		7
#define USART1_PIN_RX 	2
#define USART1_PIN_TX 	3
#define ACC_X_PIN	 	3
#define ACC_Y_PIN	 	2
#define ACC_Z_PIN	 	1
#define SPEAKER_PIN 	5 //port G


static volatile long timeMs = 0;



int MotherBoard_getLed(int index) {
	if(index==0) {
		return (~PINB & (1<<LED_0_PIN))!=0;
	}
	else if(index == 1) {
		return (~PINB & (1<<LED_1_PIN))!=0;
	}
	else return 0;
}

void MotherBoard_setLed(int index) {
	if(index==0) {
		PORTB = PORTB & ~(1<<LED_0_PIN);
	}
	else if(index == 1) {
		PORTB = PORTB & ~(1<<LED_1_PIN);
	}
}



void MotherBoard_clearLed(int index) {
	if(index==0) {
		PORTB = PORTB | (1<<LED_0_PIN);
	}
	else if(index == 1) {
		PORTB = PORTB | (1<<LED_1_PIN);
	}	
}

void MotherBoard_toggleLed(int index) {
	if(MotherBoard_getLed(index)==0) {
		MotherBoard_setLed(index);
	}
	else {
		MotherBoard_clearLed(index);
	}
}

void MotherBoard_initLeds() {
	DDRB  = DDRB | (1<<LED_0_PIN) | (1<<LED_1_PIN); //set direction on the two pins
	MotherBoard_clearLed(0);
	MotherBoard_clearLed(1);
}

static void delay(int nCount) {
	for(int i=0;i<nCount;i++) {
		for(int j=0;j<nCount;j++) {
			asm("nop");	
		}
	}
}

ISR(TIMER1_COMPA_vect) {
	timeMs++;
	OCR1A += 124;
}

static void delayMs(int msDelay) {
	long endTime = timeMs + msDelay;
	while(endTime>timeMs);

}

void testTimer() {
	//Init Timer
	TCCR1B = (1<<CS11)|(1<<CS10); //64 prescaler
	OCR1A = 124;
	TIMSK1 = (1<<OCIE1A);
	TCNT1 = 0;
	sei();
}

void testSerial() {
	DDRD  = DDRD & ~(1<<USART1_PIN_RX);  //RX set to output
	DDRD  = DDRD | (1<<USART1_PIN_TX); 	 //TX set to input
	serial_initialize(57600);			// USART Initialize (57600 is a dummy value for now!)
	printf("Serial Initialized\n");
	sei();
}

int getADC(int index) {
	if(index==0) {
		ADMUX = (1<<REFS1) | (1<<REFS0) | 0 ;		// ADC Port 1 Select
	}
	else if(index==1) {
		ADMUX = (1<<REFS1) | (1<<REFS0) | 1;
	}
	else if(index==2) {
		ADMUX = (1<<REFS1) | (1<<REFS0) | 2;
	}
	delayMs(10);					// Short Delay for rising sensor signal
	ADCSRA |= (1 << ADIF);		// AD-Conversion Interrupt Flag Clear
	ADCSRA |= (1 << ADSC);		// AD-Conversion Start
	delayMs(10);
	while( !(ADCSRA & (1 << ADIF)) );	// Wait until AD-Conversion complete
	return ADC;
}

void testAccelerometer() {
	DDRF  = DDRF | (1<<ACC_X_PIN) | (1<<ACC_Y_PIN) | (1<<ACC_Z_PIN); 	 //set to input
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);	// ADC Enable, Clock 1/64div.
}

void testSpeaker() {
	DDRG  = DDRG & ~(1<<SPEAKER_PIN);
	PING  = PING | (1<<SPEAKER_PIN);
	//Init Timer
	//TCCR0A = (1<<COM0A1)|(1<<COM0A0);  //Set OC0A on Compare Match
	//TCCR0B = (1<<FOC0A)|(1<<CS01)|(1<<CS00); //Force Output Compare A and 64 prescaler
	//OCR0A = 249;	//1ms according to KAVRCalc (16Mhz, 1msm, 64 prescaler, 0.0% error)
	//TIMSK0 = 1<<OCIE0A; 	// Enable Timer 0 Output Compare A Match Event Interrupt

}

static int counter=0;
int main() 
{
	MotherBoard_initLeds();
	testSerial();
	testTimer();
	testAccelerometer();
	testSpeaker();
	while(1) {
		MotherBoard_toggleLed(0);
		delayMs(500);
		MotherBoard_toggleLed(1);  	
		printf("hello world %i ---- ADC=(%i, %i, %i)\n", counter, getADC(0), getADC(1),getADC(2) );
		counter++;

		while(serial_get_qstate()>0) {
			std_putchar(serial_get_queue());
		}
	}
	return 0;
}
