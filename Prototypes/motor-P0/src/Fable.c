#define F_CPU 8000000UL 

#include <avr/io.h>
#include <avr/iom2561.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
#include <float.h>
#include "serial.h"
#include "dynamixel.h"
#include "fable.h"

#define LED_0_PIN		4
#define USART1_PIN_RX 	2
#define USART1_PIN_TX 	3
#define ACC_X_PIN	 	3
#define ACC_Y_PIN	 	2
#define ACC_Z_PIN	 	1
#define SPEAKER_PIN 	5 //port G


static volatile long timeMs = 0;



int MotherBoard_getLed(int index) {
	if(index==0) {
		return (~PINA & (1<<LED_0_PIN))!=0;
	}
	else return 0;
}

void MotherBoard_setLed(int index) {
	if(index==0) {
		PORTA = PORTA & ~(1<<LED_0_PIN);
	}
}



void MotherBoard_clearLed(int index) {
	if(index==0) {
		PORTA = PORTA | (1<<LED_0_PIN);
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
	DDRA  = DDRA | (1<<LED_0_PIN);
	MotherBoard_clearLed(0);
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
	serial_initialize(34);			// USART Initialize (57600 is a dummy value for now!)
	sei();
	printf("Serial Initialized\n");
}

int getADC(int index) {
	if(index==0) {
		//ADMUX = (0<<REFS1) | (0<<REFS0) | 0;		// ADC Port 1 Select
		ADMUX = 3;		// ADC Port 1 Select
	}
	else if(index==1) {
		//ADMUX = (0<<REFS1) | (0<<REFS0) | 1;
		ADMUX = 2;
	}
	else if(index==2) {
		//ADMUX = (0<<REFS1) | (0<<REFS0) | 2;
		ADMUX = 1;
	}
//	delayMs(10);	// Short Delay for rising sensor signal
	ADCSRA |= (1 << ADIF);		// AD-Conversion Interrupt Flag Clear
	ADCSRA |= (1 << ADSC);		// AD-Conversion Start
//	delayMs(10);
	while( !(ADCSRA & (1 << ADIF)) );	// Wait until AD-Conversion complete
	return ADC;
}

int getAcc(int index) {
	float acc = (3.3f*getADC(index)/1024.0f-1.65f);
	//float acc = 3.3f*getADC(index)/1 024.0f;
	return (int) 100*acc;
}


void testAccelerometer() {
	DDRF  |= (0<<ACC_X_PIN) | (0<<ACC_Y_PIN) | (0<<ACC_Z_PIN); 	 //set to input
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

#define DIR_TXD 	PORTE &= ~0x08, PORTE |= 0x04
#define DIR_RXD 	PORTE &= ~0x04, PORTE |= 0x08
#define DIR_TXRX 	PORTE |= 0x0C
#define DIR_TXNEW 	PORTE = 0x0C
volatile int ignoreByte = 0;
void tx( unsigned char c ) {
	cli();
	while(!bit_is_set(UCSR0A,5));
	UCSR0A |= 0x40;
	UDR0 = c;
	while( !bit_is_set(UCSR0A,6) );
	sei();
}

/*SIGNAL(USART0_RX_vect)
{
	printf("got: %i\n",UDR0);
}*/

void testDynamixelLoopback() {
	DDRE  |= 0x0E; // rx input, tx, rx enable, tx,  enable output
	PORTE |= 0x01; //enable pull-up resistor on rx pin
	int res = dxl_initialize(0, 34);
	if(res==1) {
		printf("Initialized dxl\n");
		int counter =0;
		
		DIR_TXRX;
		while(1) {
			tx(counter++);
			delayMs(100);
			MotherBoard_toggleLed(0);
		}
	}
	else {
		printf("Unable to connect to dynamixel\n");
	}

}

void testDynamixel() {
	DDRE  |= 0x0E; // rx input, tx, rx enable, tx,  enable output
	PORTE |= 0x01; //enable pull-up resistor on rx pin
	
	int res = dxl_initialize(0, 34);
	if(res==1) {
		printf("Initialized dxl\n");
		int dynaID = 3;
		int counter =0;
		
		while(1) {
			delayMs(250);
			dxl_ping(dynaID);
			printf("Trying to %i\n", (unsigned char) dynaID);
			//dynaID++;
			if(dxl_get_result() == COMM_RXSUCCESS) 	{
				printf("Connected to dynamixel!!!\n");
				int wValue = dxl_read_word(3, P_MODEL_NUMBER_L);
				if (dxl_get_result() == COMM_RXSUCCESS) {
					printf("Model number:%d, ", wValue);
				}
					
				int bValue = dxl_read_byte(3, P_VERSION);
				if (dxl_get_result() == COMM_RXSUCCESS) {
					printf("Version:%d\n", bValue);
				}			
			}
			else {
				printf("Unable to connect to dynamixel %i error = %i\n",dynaID, dxl_get_result());
				//dxl_write_word(dynaID, P_GOAL_POSITION_L, 500 );
			}

		}
	}
	else {
		printf("Unable to connect to dynamixel\n");
	}

}

static int counter=0;
int main() 
{
	MotherBoard_initLeds();
	testSerial();
	testTimer();
	testAccelerometer();
	testGyro();
	//testDynamixel();
	//testDynamixelLoopback();
	//testSpeaker();
	while(1) { 
		MotherBoard_toggleLed(0);
		delayMs(100);
		MotherBoard_toggleLed(1);  	
		delayMs(100);
		printf("ADC=(%i, %i, %i)\n", getAcc(0), getAcc(1), getAcc(2));
		counter++;
		while(serial_get_qstate()>0) {
			std_putchar(serial_get_queue());
		}
	}
	return 0;
}
