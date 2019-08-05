#include "t_usb.h"
#include "teensy_general.h"
int error_after1, error_before1 = 0, sum_error1=0;
int error_after2, error_before2 = 0, sum_error2=0;
int Speed1,Speed2;
double Kp1=3, Ki1=0, Kd1=0.1, Kp2=0.5, Ki2=0, Kd2=0.05;
uint16_t ADCF0,ADCF5,ADCF1,ADCF4; // Assign the ADC

// Set ADC Port
uint16_t readADC(uint8_t channel)
{
  ADMUX = (ADMUX & 0xf0)|channel; 
  set(ADCSRA, ADSC);
  loop_until_bit_is_clear(ADCSRA,ADSC);;
  return (ADC);
}

void joint_1(){
	//set Port
	set(DDRB,6);set(DDRB,5); set(DDRB,4);
	set(PORTB,6);
	//read_ADC
	ADCF0=readADC(0); 
	m_usb_tx_string("F0  ");
	m_usb_tx_int(ADCF0); // ADC of F0 
	ADCF5=readADC(5);
	m_usb_tx_string("  F5  ");
	m_usb_tx_int(ADCF5);  //ADC of F5
	error_after1 = ADCF0 - ADCF5;
	m_usb_tx_string("  Error1: ");
	m_usb_tx_int(error_after1);
	m_usb_tx_string("\n"); //line feed
	if(error_after1 > 20){
		set(PORTB,5);
		clear(PORTB,4);
		OCR1B = Speed1;
	}

	else if(error_after1 < -20){
		set(PORTB,4);
		clear(PORTB,5);
		OCR1B = Speed1;
	}

	else {
		clear(PORTB,4);
		clear(PORTB,5);
	}
	set(TCCR1B,CS12);clear(TCCR1B,CS11);set(TCCR1B,CS10); // set the prescaler as 1024
	set(TCCR1B,WGM12);set(TCCR1A,WGM10); // up to 0xFF, PWM
	set(TCCR1A,COM1B1);set(TCCR1A,COM1B0); // set at OCR1B
	Speed1 = Kp1*abs(error_after1);
	Speed1 += abs(Ki1*sum_error1);
	Speed1 += abs(Kd1*(error_after1-error_before1));
	m_usb_tx_string("  Speed1 ");
	m_usb_tx_int(Speed1);
	sum_error1 += error_after1;
	error_before1 = error_after1;
}

void joint_2(){
	//set Port
	set(DDRB,7);set(DDRD,0); set(DDRC,6);
	set(PORTB,7);
	//read_ADC
	ADCF1=readADC(1); 
	m_usb_tx_string("F1  ");
	m_usb_tx_int(ADCF1); // ADC of F1 
	ADCF4=readADC(4);
	m_usb_tx_string("  F4  ");
	m_usb_tx_int(ADCF4);  //ADC of F4
	error_after2 = ADCF1 - ADCF4;
	m_usb_tx_string("  Error2: ");
	m_usb_tx_int(error_after2);
	m_usb_tx_string("\n"); //line feed

	if(error_after2 > 20){
		set(PORTD,0);
		clear(PORTC,6);
	}

	else if(error_after2 < -20){
		set(PORTC,6);
		clear(PORTD,0);
	}

	else{
		clear(PORTD,0);
		clear(PORTC,6);
	}
	set(TCCR0B,CS02);clear(TCCR0B,CS01);set(TCCR0B,CS00); // set the prescaler as 1024
	set(TCCR0A,WGM01);set(TCCR0A,WGM00); // up to 0xFF, PWM
	set(TCCR0A,COM0A1);set(TCCR0A,COM0A0); // set at OCR0A
	Speed2 = Kp2*abs(error_after2);
	Speed2 += abs(Ki2*sum_error2);
	Speed2 += abs(Kd2*(error_after2-error_before2));
	OCR1A = Speed2;
	m_usb_tx_string("  Speed2 ");
	m_usb_tx_int(Speed2);
	sum_error2 += error_after2;
	error_before2 = error_after2;
}

int main(void){
	//setup ADC
	clear(ADMUX,REFS1);set(ADMUX,REFS0); // set reference voltage as Vcc
	set(ADCSRA, ADPS2);set(ADCSRA, ADPS1);set(ADCSRA, ADPS0); // set ADC prescaler as 128
	set(ADCSRA, ADEN);
    m_usb_init(); //initialize
	for(;;){
		joint_1();
		joint_2();
	}
}
