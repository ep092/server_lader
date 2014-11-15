/*
 *    Filename: lader.c
 *     Version: 0.0.1
 * Description: Regelung für Ladung von riesigen Akkus!
 *     License: GPLv3 or later
 *     Depends: global.h, io.h, interrupt.h
 *
 *      Author: Copyright (C) Philipp Hörauf, Toni
 *        Date: 2014-11-15
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <avr/interrupt.h>

#include "../AtmelLib/global.h"
#include "../AtmelLib/io/io.h"

#define I2C_DDR DDRC
#define I2C_PORT PORTC
#define I2C_PIN PINC
#define SCL PC5
#define SDA PC4
uint8_t ERR=0;
#include "../AtmelLib/io/serial/i2c.h"

#define UBRRH_VALUE 1
#define UBRRL_VALUE 2
#include "../AtmelLib/io/serial/uart.h"


#define LED(x) out(PORTC,PC3,0,x)
#define NT_ON(x) out(PORTB,PB0,0,x)
#define SPKR(x) out(PORTD,PD7,0,x)
#define SS_DAC(x) out(PORTB,PB2,0,x)
#define SS_LCD(x) out(PORTD,PD6,0,x)

void adcInit(void) {
	ADMUX = 1<<REFS0; // Referenz auf Vcc
	ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 5<<ADPS0; // ADC an, Interrupt an, Prescaler=128
	ADCSRB = 0; // nicht verwendet.
}

void adcStart(void) {
	ADCSRA |= 1<<ADSC;
}

// ADC auslesen im Interrupt.



// Drehrad Interrupts
// 1
ISR(INT0_vect, ISR_BLOCK) {
	
}


// 2
ISR(INT1_vect, ISR_BLOCK) {
	
}

// ADC Conversion complete Interrupt
uint16_t uNetzteil=0, uReserve=0, Strom=0;
ISR(ADCxx_vect, ISR_BLOCK) {
	
}


int main(void) {
	DDRB  = 1<<PB0 | 1<<PB2 | 1<<PB3 | 1<<PB5;
	DDRC  = 1<<PC3 | 1<<PC5;
	DDRD  = 1<<PD1 | 1<<PD5 | 1<<PD6 | 1<<PD7;
	// Pullups
	PORTD = 1<<PD4;
	
	// init der Module
	i2c_init();
	uartInit();
	adcInit();
	dacInit();
	lcd_init();
	
	uartTxStrln("Guten Tag!");
	uartTxStrln("REICH TIME");
	uartTxNewline();
	uartTxNewline();
	
	sei();
	
	while(1) {
		
	}
	return 0;
}
