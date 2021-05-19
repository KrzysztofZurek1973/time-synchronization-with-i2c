/*************************************************
*	AVR TWI interrupt test, how fast uC enters interrupt routine
*	Is it possible to use TWI IRQ to synchronize time on two uC?
*
*	www.alfa46.com
*	krzzurek@gmail.com
*	Krzysztof Zurek
*	2021, Gdansk
*************************************************/
//#define F_CPU (16000000UL)

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <util/twi.h>

typedef unsigned char bool;
#define true 1
#define false 0
typedef unsigned char u8_t;
typedef signed char c8_t;
typedef unsigned int u16_t;
typedef signed int s16_t;
typedef unsigned long u32_t;
typedef signed long s32_t;

#define SLA_ADR 0x03
#define ADR_MASTER 0x0A

/** This macro saves the global interrupt status. */
#define ENTER_CRITICAL_REGION()         {uint8_t sreg = SREG; cli()
/** This macro restores the global interrupt status. */
#define LEAVE_CRITICAL_REGION()         SREG = sreg;}

u16_t volatile int_irq_counter = 0;
u16_t volatile twi_irq_counter = 0;
bool volatile twi_is_running = false;
u8_t twst;
u8_t twi_buff[3];
u8_t twi_byte_cnt = 0;
bool start_trans = false;
u8_t finish_trans = false;

void SysInit(void);


// ----------------------------------------
int main(void){

	SysInit();
	sei();
	sleep_enable();

  	for(;;){
		if ((twi_is_running == false) && (start_trans == true)){
			twi_is_running = true;
			start_trans = false;
			//start TWI transmittion
			twi_byte_cnt = 0;
			twi_buff[0] = 0xAA;
			//twi_buff[1] = twi_irq_counter;
			//twi_buff[2] = twi_irq_counter >> 8;
			TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE); /* send start condition */
		}
		else if (finish_trans == true){
			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO) | _BV(TWIE);
			finish_trans = false;
			twi_is_running = false;
			EIFR |= (1 << INTF0);	//clear INT flag
   			EIMSK |= (1 << INT0);
   	}
   	PORTD &= ~_BV(PD0);		//pin switch OFF
	sleep_cpu();			//sleep CPU
  	}
  	return 0;
}


// ******************************************
// I2C interrupt
ISR(TWI_vect){
	twi_irq_counter++;
	PORTD |= _BV(PD0); //for synchronization measurement
	twst = TW_STATUS;
	switch (twst){
   	case TW_START:
   		//start transmitted
   		twi_buff[1] = twi_irq_counter;
   		twi_buff[2] = twi_irq_counter >> 8;
   		TWDR = (SLA_ADR << 1) | TW_WRITE;
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE); /* clear interrupt to start transmission */
   		break;
   		
   	case TW_MT_SLA_ACK:
   		//SLA+W transmitted, ACK received
   		twi_byte_cnt = 0;
   		TWDR = twi_buff[0];
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
      	break;
      	
      case TW_MT_DATA_ACK:
      	//data transmitted, ACK received
      	twi_byte_cnt++;
      	if (twi_byte_cnt < 3){
   			TWDR = twi_buff[twi_byte_cnt];
   			TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
   		}
   		else{
   			finish_trans = true;
   			//TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO) | _BV(TWIE); //send STOP
   		}
      	break;

   	case TW_MT_SLA_NACK:
   		//SLA+W transmitted, NACK received
   	case TW_MT_ARB_LOST:
   		//arbitration lost in SLA+W or data
   	default:
   		//SLA+W transmitted, NACK received
   		finish_trans = true;
    }
}


// ******************************************
// button interrupt
ISR(INT0_vect){	
	int_irq_counter++;
	start_trans = true;
	EIMSK &= ~(1 << INT0); //switch off IRQ

}


/********************************************
 Initialization
*********************************************/
void SysInit(void){

   /**********   IO init   *********/
   /* PORTD ----------------------------------*/
   PORTD |= 0xFF;
   DDRD = (1 << PD0); //output
	
	// PORT C
	PORTC = 0xFF;
	DDRC = 0;

   /* external interrupts */
   /* INT0: test signal*/
   EIFR |= (1 << INTF0);	//clear INT flag
   EICRA |= (1 << ISC01); //INT0 - falling edge
   EIMSK |= (1 << INT0);
	
	//other settings to make power consumption as low as possible
	//set analog comparator down
	ACSR |= (1 << ACD);
	//switch off ADC
	ADCSRA &= ~(1 << ADEN);
	//switch off SPI interface
	SPCR &= ~(1 << SPE);
	//switch on I2C interface
	TWSR = 0;				//prescaler 1
	TWBR = 72;				//I2C frequency = 100k
	TWAR = ADR_MASTER << 1;
	TWCR |= (1 << TWEN) | _BV(TWIE);	//enable TWI interface
	//switch off USART
	UCSR0B &= ~((1 << RXEN0)|(1 << TXEN0));
}

