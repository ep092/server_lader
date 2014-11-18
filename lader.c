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

// UART: 19200 Baud, 1 Stopbit, no Parity, 8 Bit/frame
#define UBRRH_VALUE 0
#define UBRRL_VALUE 51
#include "../AtmelLib/io/serial/uart.h"

#define DISPLAY_RS_DDR   DDRD
#define DISPLAY_RS_PORT	 PORTD
#define DISPLAY_RS_PIN	 PD5
#define DISPLAY_CSB_DDR  DDRD
#define DISPLAY_CSB_PORT PORTD
#define DISPLAY_CSB_PIN  PD6
#include "EA_DOGM163L-A-Atmel-Lib/EA_DOGM163L-A.c"


#define LED(x) out(PORTC,PC3,0,x)
#define NT_ON(x) out(PORTB,PB0,0,x)
#define SPKR(x) out(PORTD,PD7,0,x)
#define SS_DAC(x) out(PORTB,PB2,0,x)

void adcInit(void) {
	ADMUX = 1<<REFS0; // Referenz auf Vcc
	ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 5<<ADPS0; // ADC an, Interrupt an, Prescaler=128
	ADCSRB = 0; // nicht verwendet.
}

void adcStart(void) {
	ADCSRA |= 1<<ADSC;
}

void dacSetValue(uint16_t wert, uint8_t mode) {
	uint8_t tempspcr = SPCR; // sichere den Stand von SPCR
	SPCR = 1<<SPE | 1<<MSTR | 1<<CPOL; // SPI MSB first, Clock on falling edge, sample on leading, VOLLGAS
	
	SS_DAC(0); // SS_DAC ist low active
	SPDR = (0b00000011 & mode); // sende PowerDown-Mode
	while (!(SPSR & (1<<SPIF))); // warte auf Transmission complete
	SPDR = (wert>>8); // sende oberen Teil vom Datenwort
	while (!(SPSR & (1<<SPIF)));
	SPDR = (wert & 0xff); // sende unteren Teil vom Datenwort
	while (!(SPSR & (1<<SPIF)));
	SS_DAC(1);
	SPCR = tempspcr;
}

#define RFB 1
#define RG1 1
#define RG2 1
#define DAC_REF 2.5
#define UMAX 61000
#define T 55.44
#define M -4.9

#define K1 1+((float)RFB/(float)RG2)+((float)RFB/(float)RG1)
#define K2 ((float)RFB/(float)RG2)*DAC_REF

void setPowerOutput(uint16_t millivolt) {
	if (millivolt<UMAX) { // Schutz vor Überspannung
		// Berechnung des DAC-wertes aus der sollspannung
		dacSetValue((uint16_t)(((((((float)millivolt / 1000) - T) / M) + K2) * 65536) / (K1 * DAC_REF)), 0);
	} else {
		; // nix. wert wird nicht geändert
	}
}

void timerInit(void) {
	// Timer/Counter1 im CTC modus verwenden. alle 10ms ein Interrupt
	TCCR1A = 0; // CTC (mode 4)
	TCCR1B = 1<<WGM12 | 1<<CS11 | 1<<CS10; // CTC (mode 4), Prescaler = 64 -> 4µs pro Timerschritt
	OCR1A  = 2500; // TOP und Interrupt alle 10ms
}

// Der ADC wird interruptgesteuert
volatile uint8_t counter = 0;
ISR(TIMER1_OVF_vect, ISR_BLOCK) {
	ADMUX = ADMUX & 0b11100000; // lösche selektiv die MUX-Bits
	ADMUX |= (counter%3); // setze ADC-Kanal neu
	ADCSRA |= 1<<ADC; // start conversion
}


// Drehrad Interrupts
volatile uint8_t knopf=0;
// knopf-Datentyp:	7:0 		6:0 		5:STEP_TEMP_LINKS 	4:STEP_TEMP_RECHTS
//			3:STEP_LINKS 	2:STEP_RECHTS 	1:KNOPF-GEDRÜCKT 	0:KNOPF-LOSGELASSEN
// Drehung Pin 1
ISR(INT0_vect, ISR_BLOCK) {
	if(knopf & 1<<5) { // knopf wurde eins nach links gedreht
		sbi(knopf, 3);
		cbi(knopf, 5);
	} else { // knopf wurde eins nach rechts gedreht
		sbi(knopf, 4);
	}
}

// Drehung Pin 2
ISR(INT1_vect, ISR_BLOCK) {
	if(knopf & 1<<4) { // knopf wurde eins nach rechts gedreht
		sbi(knopf, 2);
		cbi(knopf, 4);
	} else { // knopf wurde eins nach links gedreht
		sbi(knopf, 5);
	}	
}

// KNOPFDRUCK oder loslassen
ISR(PCINT2_vect, ISR_BLOCK) {
	if(PIND & 1<<PD4) { // Knopf ist gerade losgelassen worden
		sbi(knopf, 0);
		cbi(knopf, 1); // lösche gedrückt-Bit wieder
	} else { // Knopf gedrückt, warte auf loslassen
		sbi(knopf, 1); // setze gedrückt-Bit
	}
}


// ADC Conversion complete Interrupt
// ADC0 = uNetzteil, ADC1 = uReserve, ADC2 = strom
// Die Werte können noch kalibriert werden, allerdings nicht zur Laufzeit.

#define REFERENZ 4	// externe Referenz, 4096mV also 4mV pro STEP
#define MITTELWERTE 4
volatile uint16_t uNetzteil=0, uReserve=0, strom=0; // U in mV, I in mA

ISR(ADC_vect, ISR_BLOCK) {
	// ADC-Auslesungen und Rechnung mit Gleitmittelwert über 5 Werte
	static uint16_t tabelle[3*MITTELWERTE]; // hier kommen die ADC-werte rein.
	uint16_t temp=0;
	tabelle[counter++] = ADC; // fülle Tabelle mit ADC-Werten
	if (counter == 3*MITTELWERTE) { // setze Counter zurück
		counter = 0;
	}
	switch (counter%3) {
		case 0:
			// uNetzteil wird ausgerechnet
			for(uint8_t i=0; i<5; i++) {
				temp += tabelle[0 + 3*i];
			}
			uNetzteil = ((temp*REFERENZ)<<2);
		
		case 1:
			// uReserve wird ausgerechnet
			for(uint8_t i=0; i<5; i++) {
				temp += tabelle[1 + 3*i];
			}
			uReserve = ((temp*REFERENZ)<<2);
		
		case 2:
			// strom wird ausgerechnet
			for(uint8_t i=0; i<5; i++) {
				temp += tabelle[2 + 3*i];
			}
			strom = (uint16_t)((float)temp*((float)50/(float)3));
	}
}



int main(void) {
	DDRB  = 1<<PB0 | 1<<PB2 | 1<<PB3 | 1<<PB5;
	DDRC  = 1<<PC3 | 1<<PC5;
	DDRD  = 1<<PD1 | 1<<PD5 | 1<<PD6 | 1<<PD7;
	// Pullups
	PORTD = 1<<PD4;
	
	// Pinchange Interrupts für das Drehrad und den Knopf
	// Knopf hängt am PCINT20 (PD4)
	EICRA = 1<<ISC11 | 1<<ISC10 | 1<<ISC01 | 1<<ISC00;
	EIMSK = 1<<INT1 | 1<<INT0;
	PCICR = 1<<PCIE2; // PCINT16 bis 23 aktiv
	PCMSK2 = 1<<PCINT20; // Interrupt auf fallender UND steigender Flanke von PD4
	
	delayms(100); // wait for clock stabilize
	
	// init der Module
	i2c_init();
	uartInit();
	adcInit();
	timerInit();
	spiInit();
	SS_DAC(1); // CS vom DAC ist idle high
	DOGM163_init(); // 3x16 Zeichen reflexives LCD
	
	uartTxStrln("Guten Tag!");
	uartTxStrln("REICH TIME");
	uartTxNewline();
	uartTxNewline();
	
	spi_write_string("   Guten Tag!     gebaut von   Philipp und Toni"); // Begrüßung
	char display[17];
	LED(1);
	delayms(10000);
	
	uint16_t sollspannung=30000, maximalstrom=10000; // Spannung in Millivolt, Strom in Milliampere
	uint16_t tempspannung=0, tempstrom=0; // werte, die der Regler verändern kann.
	// unabhängig von denen, die der User eingibt (maximalwerte)
	
	uint8_t status=0; // Statusvariabel für den Automaten
	
	sei(); // und es seien Interrupts (aktiv)!
	
	SPKR(1);
	delayms(100);
	SPKR(0);
	
	spi_write_string("  NUR ANZEIGE!  ");
	while(1) {
		display_set_row(1);
		sprintf(display, "Uout: %umV", uNetzteil); // Netzteilspannung
		spi_write_string(display);
		
		display_set_row(2);
		sprintf(display, "Uout: %umV", strom); // Laststrom
		spi_write_string(display);
	}
	
	while(1) {
		if (status == 0) { // init state: Netzteil Ausgang AUS
			display_set_row(0);
			spi_write_string("Netzteil INAKTIV");
			uartTxStrln("NT aus");
			NT_ON(0);
			LED(0);
		}
		if (knopf & 1) { // wechsle in status
			if (status == 0) {
				status = 1;
			} else if (status == 2) {
				setPowerOutput(sollspannung);
				NT_ON(1); // aktiviere netzteil
				LED(1);
				status = 3
			}
		}
		if (status == 1) { // spannung einstellen
			display_set_row(0);
			spi_write_string("U_soll stellen..");
			uartTxStrln("U stellen");
			status = 2;
		}
		if (status == 3) { // strom einstellen
			display_set_row(0);
			spi_write_string("I_max stellen..");
			uartTxStrln("I stellen");
			status = 4;
		}
		if ((knopf & 1<<2) || (knopf & 1<<3)) { // knopf gedreht um 1 nach rechts oder links
			if (status == 2) { // spannung verstellen
				display_set_row(1);
				spi_write_string("Sollspannung:   ");
				if (knopf & 1<<2) { // Knopf rechts gedreht
					if(sollspannung<60000) {
						sollspannung = sollspannung+100; // +100mV
					}
				} else if (knopf & 1<<3) { // Knopf links gedreht
					if(sollspannung>12000) {
						sollspannung = sollspannung-100; // -100mV
					}
				}
				display_set_row(3);
				sprintf(display, "Uout: %umV", sollspannung); // Sollspannung neu
				spi_write_string(display);
			}
			if (status == 4) {
				display_set_row(1);
				spi_write_string("Maximalstrom:   ");
				if (knopf & 1<<2) { // Knopf rechts gedreht
					if(maximalstrom<55000) {
						maximalstrom = maximalstrom+100; // +100mV
					}
				} else if (knopf & 1<<3) { // Knopf links gedreht
					if(maximalstrom>0) {
						maximalstrom = maximalstrom-100; // -100mV
					}
				}
				display_set_row(3);
				sprintf(display, "Umax: %umA", maximalstrom); // Sollspannung neu
				spi_write_string(display);
			}
		}
		if (status == 42) { // regler läuft.
			// Regler läuft.
			if (strom > maximalstrom) {
				tempspannung--;
			}
			if (strom < maximalstrom) {
				if (tempspannung < sollspannung) {
					tempspannung++;
				}
			}
			setPowerOutput(tempspannung);
		}
	}
	return 0;
}
