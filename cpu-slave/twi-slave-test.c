/*************************************************
*	AVR TWI interrupt test, how fast uC enters interrupt routine
*	Is it possible to use TWI IRQ to synchronize time on two uC?
*
*	www.alfa46.com
*	krzzurek@gmail.com
*	Krzysztof Zurek
*	2021, Gdansk
*************************************************/
#define F_CPU (16000000UL)

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
static u8_t twst;
static u8_t twi_buff[3];
static u8_t twi_byte_cnt = 0;
static bool twi_data_received = false;
static bool twi_slave_is_running = false;

void SysInit(void);


// ----------------------------------------
int main(void){

	SysInit();
	sei();
	sleep_enable();

  	for(;;){
   		if (twi_data_received == true){
   			twi_data_received = false;
   			twi_slave_is_running = false;
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
   	case TW_SR_SLA_ACK:
   		//SLA+W received, ACK returned
   		twi_byte_cnt = 0;
   		twi_slave_is_running = true;
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
   		break;
   	
   	case TW_SR_DATA_ACK:
   		//data received, ACK returned
   		if (twi_byte_cnt < 3){
	   		twi_buff[twi_byte_cnt] = TWDR;
   			twi_byte_cnt++;
   		}
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
   		break;
   		
   	case TW_SR_STOP:
   		//stop or repeated start condition received while selected
   		twi_data_received = true;
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
   		break;
   		
   	case TW_SR_ARB_LOST_GCALL_ACK:
   		//arbitration lost in SLA+RW, general call received, ACK returned
   		break;
   		
   	case TW_SR_GCALL_ACK:
   		//general call received, ACK returned
   		break;
   		
   	default:
   		twi_data_received = true;
   		TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
    }
}


// ******************************************
// button interrupt
ISR(INT0_vect){	
	int_irq_counter++;
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
	TWAR = SLA_ADR << 1;
	TWCR |= (1 << TWEN) | _BV(TWIE) | _BV(TWEA);	//enable TWI interface
	
	//switch off USART
	UCSR0B &= ~((1 << RXEN0)|(1 << TXEN0));
}


