//  *    Filename: lader.c
//  *     Version: 0.1.0
//  * Description: Regelung für Ladung von riesigen Akkus!
//  *     License: GPLv3 or later
//  *     Depends: global.h, io.h, interrupt.h
//  *
//  *      Author: Copyright (C) Philipp Hörauf, Toni
//  *        Date: 2014-11-15
//  *
//  *  This program is free software; you can redistribute it and/or modify
//  *  it under the terms of the GNU General Public License as published by
//  *  the Free Software Foundation; either version 2 of the License, or
//  *  (at your option) any later version.
//  *
//  *  This program is distributed in the hope that it will be useful,
//  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  *  GNU General Public License for more details.
//  *
//  *  You should have received a copy of the GNU General Public License
//  *  along with this program; if not, write to the Free Software
//  *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "../AtmelLib/global.h"
#include "../AtmelLib/io/io.h"

#define I2C_DDR DDRC
#define I2C_PORT PORTC
#define I2C_PIN PINC
#define SCL PC5
#define SDA PC4
uint8_t ERR = 0;
#include "../AtmelLib/io/serial/i2c.h"

// UART: 19200 Baud, 1 Stopbit, no Parity, 8 Bit/frame
#define UBRRH_VALUE 0
#define UBRRL_VALUE 25
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

#define B_KNOPF_LOSGELASSEN (knopf & 1) 
#define RESET_KNOPF_LOSGELASSEN cbi(knopf, 0) 
#define B_KNOPF_GEDRUECKT ((knopf >> 1) & 1) 
#define RESET_KNOPF_GEDRUECKT cbi(knopf, 1) 
#define B_STEP_LINKS ((knopf >>3) & 1) 
#define RESET_STEP_LINKS cbi(knopf, 3) 
#define B_STEP_RECHTS ((knopf >>2) & 1) 
#define RESET_STEP_RECHTS cbi(knopf, 2) 


volatile uint16_t MAXIMALSPANNUNG = 61000; // in mV, ab hier wird der Errorstate betreten!
volatile uint16_t STROMOFFSET = 593; // wird durch das Netzgeraet automatisch kalibriert

typedef enum {
	INIT_STATE, AUSWAHL, MODUS_LADER, MODUS_NETZGERAET, REGELUNG_NETZGERAET,
	LADEN_AKTIV, LADUNG_FERTIG, ERROR_STATE
} state_main;

typedef enum {
	LADESCHLUSSSPANNUNG, MAXIMALSTROM, STROM_ENDE, START, ZURUECK
} state_modus_lader;

typedef enum { // Die verschiedenen Errorgründe.
	OVERVOLTAGE_ERR, OVERCURRENT_ERR, OVERPOWER_ERR, OVERTEMP1_ERR,
	OVERTEMP2_ERR, NONE
} state_errors;

state_main state = INIT_STATE;
state_modus_lader state_lader = LADESCHLUSSSPANNUNG;
state_errors errors = NONE;

uint16_t netzgeraet_spannung = 32000;
uint16_t netzgeraet_strom = 12000;

uint16_t modus_lader_ladeschlussspannung = 59000;
uint16_t modus_lader_maximalstrom = 5000;
uint16_t modus_lader_strom_ende = 300;

char display[17];


volatile uint8_t knopf=0;
// knopf-Datentyp:	7:0 		6:0 		5:STEP_TEMP_LINKS 	4:STEP_TEMP_RECHTS
//			3:STEP_LINKS 	2:STEP_RECHTS 	1:KNOPF-GEDRÜCKT 	0:KNOPF-LOSGELASSEN
// Drehung Pin 1


void adcInit(void) {
	ADMUX = 0; // Referenz auf externe Referenz
	ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIE | 5<<ADPS0; // ADC an, Interrupt an, Prescaler=128
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

#define RFB 10000
#define RG1 3300
#define RG2 10000
#define DAC_REF 2.5
#define T 55.44
#define M -4.9

// K1 = 1+(RFB/RG2)+(RFB/RG1)	// K1 = 5,0303
// K2 = (RFB/RG2)*DAC_REF 	// K2 = 2,5

#define K1 5.0303
#define K2 2.5

inline void setPowerOutput(uint16_t millivolt) {
	// Berechnung des DAC-wertes aus der sollspannung
	dacSetValue((uint16_t)(((((((float)millivolt / 1000) - T) / M) + K2) * 65536) / (K1 * DAC_REF)), 0);
}

void timerInit(void) {
	// Timer/Counter1 im CTC modus verwenden. alle 10ms ein Interrupt
	TCCR1A = 0; // CTC (mode 4)
	TCCR1B = 1<<WGM12 | 1<<CS11 | 1<<CS10; // CTC (mode 4), Prescaler = 64 -> 4µs pro Timerschritt
	TIMSK1 = 1<<OCIE1A; // Interrupt on compare match
	OCR1A  = 2500; // TOP und Interrupt alle 10ms
}

// Der ADC wird interruptgesteuert
volatile uint8_t counter = 0;
ISR(TIMER1_COMPA_vect, ISR_BLOCK) {
	ADMUX = ADMUX & 0b11100000; // lösche selektiv die MUX-Bits
	ADMUX |= (counter%3); // setze ADC-Kanal neu
	ADCSRA |= 1<<ADSC; // start conversion
}


// Drehrad Interrupts
ISR(INT0_vect, ISR_BLOCK) {
	if(knopf & 1<<5) { // Knopf wurde eins nach links gedreht
		sbi(knopf, 3);
		cbi(knopf, 5);
	} else { // knopf wurde eins nach rechts gedreht
		sbi(knopf, 4);
	}
}

// Drehung Pin 2
ISR(INT1_vect, ISR_BLOCK) {
	if(knopf & 1<<4) { // Knopf wurde eins nach rechts gedreht
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
#define ABWEICHUNG 1.000296589  // gemessene Abweichung in % vom exakten Referenzspannungswert
#define MITTELWERTE 4
volatile uint16_t uNetzteil=0, uReserve=0, strom=0; // U in mV, I in mA

ISR (ADC_vect, ISR_BLOCK) {
	// 	uartTxStrln("adc");
	// ADC-Auslesungen und Rechnung mit Gleitmittelwert über 4 Werte
	static uint16_t tabelle[3*MITTELWERTE]; // hier kommen die ADC-werte rein.
	uint16_t temp=0;
	tabelle[counter] = ADC; // fülle Tabelle mit ADC-Werten
	
	switch (counter%3) {
		case 0:
			// uNetzteil wird ausgerechnet
			for(uint8_t i=0; i<MITTELWERTE; i++) {
				temp += tabelle[0 + 3*i];
			}
			uNetzteil = (uint16_t)((temp*REFERENZ)<<2)*ABWEICHUNG;
			break;
			
		case 1:
			// uReserve wird ausgerechnet
			for(uint8_t i=0; i<MITTELWERTE; i++) {
				temp += tabelle[1 + 3*i];
			}
			uReserve = (uint16_t)((temp*REFERENZ)<<2)*ABWEICHUNG;
			break;
			
		case 2:
			// strom wird ausgerechnet
			for(uint8_t i=0; i<MITTELWERTE; i++) {
				temp += tabelle[2 + 3*i];
			}
			if (temp < STROMOFFSET) {
				temp = 0;
			} else {
				temp -= STROMOFFSET;
			}
			strom = (uint16_t)((float)temp*ABWEICHUNG*((float)50/(float)3));
			break;
	}
	counter++;
	if (counter == 3*MITTELWERTE) { // setze Counter zurück
		counter = 0;
	}
}

uint8_t knopf_losgelassen(void) {
	uint8_t return_value = B_KNOPF_LOSGELASSEN;
	if(return_value){
		SPKR(1);
		delayms(10);
		SPKR(0);
		RESET_KNOPF_LOSGELASSEN;
		RESET_KNOPF_GEDRUECKT;
	}
	return return_value;
}

uint8_t knopf_gedrueckt(void) {
	uint8_t return_value = B_KNOPF_GEDRUECKT;
	RESET_KNOPF_GEDRUECKT;
	return return_value;
}

uint8_t step_links(void) {
	uint8_t return_value = B_STEP_LINKS;
	RESET_STEP_LINKS;
	if(return_value){
		SPKR(1);
		delayms(10);
		SPKR(0);
	}
	return return_value;
}

uint8_t step_rechts(void) {
	uint8_t return_value = B_STEP_RECHTS;
	RESET_STEP_RECHTS;
	if(return_value){
		SPKR(1);
		delayms(10);
		SPKR(0);
	}
	return return_value;
}

uint16_t spannungseinstellung(uint16_t lokalspannung) {
	while(knopf_losgelassen()!=1){
		sprintf(display, "Uout: %5umV ", lokalspannung);
		display_set_row(2);
		spi_write_string(display);
		if (step_links()){
			lokalspannung -=100;
		}else if (step_rechts()){
			lokalspannung += 100;
		}
	}
	return lokalspannung;
}

uint16_t stromeinstellung(uint16_t lokalstrom) {
	while(knopf_losgelassen()!=1){
		sprintf(display, "Uout: %5umA ", lokalstrom);
		display_set_row(2);
		spi_write_string(display);
		if (step_links()){
			lokalstrom -=100;
		} else if (step_rechts()){
			lokalstrom += 100;
		}
	}
	return lokalstrom;
}

void netzteil_regulation(void) {
	STROMOFFSET = strom; // Offset wird beim Start des Netzteils rauskalibriert.
	uint8_t regelspannung = 0;
	setPowerOutput(netzgeraet_spannung);
	delayms(200);
	display_set_row(0);
	spi_write_pstr(PSTR("Netzgeraet AKTIV"));
	NT_ON(1); // Output ON
	
	// Betrete Regelschleife, in der die Spannung konstant gehalten wird.
	// wenn Knopf gedrückt oder Error passiert, wieder NT-Modus verlassen
	while ((knopf_losgelassen() == 0) && (state != ERROR_STATE)) {
		
		display_set_row(1);
		sprintf(display, "Uout: %5umV ", uNetzteil); // Netzteilspannung anzeigen
		spi_write_string(display);
		display_set_row(2);
		sprintf(display, "Iout: %5umA ", strom); // Laststrom anzeigen
		spi_write_string(display);
		
		if (strom > (netzgeraet_strom)) {
			NT_ON(0); // Netzteil AUS. Überstrom!
			state = ERROR_STATE;
			errors = OVERCURRENT_ERR;
			SPKR(1);
			delayms(1000); // TÜÜT
			SPKR(0);
			continue;
			
		} if (uNetzteil > MAXIMALSPANNUNG) {
			NT_ON(0); // Netzteil AUS. Überspannung!
			state = ERROR_STATE;
			errors = OVERVOLTAGE_ERR;
			SPKR(1);
			delayms(1000); // TÜÜT
			SPKR(0);
			continue;
			
		} else if (uNetzteil - 100 > netzgeraet_spannung) { // Ausgangsspannung zu hoch
			regelspannung--;
			delayms(10); // Zeitkonstante künstlich erhöht, damit Regelung nicht schwingt.
			
		} else if (uNetzteil + 100 < netzgeraet_spannung) { // Ausgangsspannung zu klein
			regelspannung++;
			delayms(10);
		}
		
		// Normaler Betrieb, Spannung kann verstellt werden
		if (step_links() && (netzgeraet_spannung > 100)) { // Spannung um 100mV runter, minimal 0V
			netzgeraet_spannung = netzgeraet_spannung - 100;
		}
		
		if (step_rechts() && ((netzgeraet_spannung + 100) < MAXIMALSPANNUNG)) { // Spannung um 100mV hoch
			netzgeraet_spannung = netzgeraet_spannung + 100;
		}
		
		uartTxStr("U=");
		uartTxDec(netzgeraet_spannung);
		uartTxStrln(" mV");
		if ((netzgeraet_spannung + regelspannung * 8) > MAXIMALSPANNUNG) {
			; // do nothing.
		} else {
			setPowerOutput(netzgeraet_spannung + regelspannung * 8);
		}
	}
	// ENDE. Netzteil wird abgeschaltet und State verlassen
	NT_ON(0);
	delayms(100);
}


void stateMachine(void) {
	knopf=0;
	while (1) {
		switch (state) {
			case INIT_STATE:
				uartTxStrln("INIT_STATE");
				spi_write_pstr(PSTR("   Guten Tag!     gebaut von    Toni und Philipp")); // Begrüßung
				while (knopf_losgelassen() != 1) {
				}
				state = AUSWAHL;
				//      if (knopf_losgelassen()) state=AUSWAHL;
				break;
				
			case AUSWAHL:
				uartTxStrln("AUSWAHL");
				static uint8_t auswahl = 0; //0=LADER; 1=NETZTEIL
				if (step_links() && (auswahl != 0)) {
					auswahl--;
				} else if( step_rechts() && (auswahl != 1)) {
					auswahl++;
				}
				if (knopf_losgelassen()) {
					if (auswahl==0) {
						state=MODUS_LADER;
						// auswahl=0;
						break;
					} else if (auswahl==1) {
						state=MODUS_NETZGERAET;
						// auswahl=0;
						break;
					}
				}
				display_clear();
				spi_write_pstr(PSTR("Auswahl"));
				display_set_row(1);
				if (auswahl==0) {
					spi_write_pstr(PSTR("Lader"));
				} else if (auswahl==1) {
					spi_write_pstr(PSTR("Netzteil"));
				}
				delayms(100);
				break;
				
			case MODUS_LADER:
				uartTxStrln("MODUS_LADER");
				if (step_links() && (state_lader != 0)) {
					state_lader--;
				} else if( step_rechts() && (state_lader != ZURUECK)) {
					state_lader++;
				}
				delayms(100);
				display_clear();
				spi_write_pstr(PSTR("Modus: Lader"));
				display_set_row(1);
				switch(state_lader) {
					case LADESCHLUSSSPANNUNG:
						spi_write_pstr(PSTR("Ladeschlussspannung"));
						if (knopf_losgelassen()) {
							modus_lader_ladeschlussspannung = spannungseinstellung(modus_lader_ladeschlussspannung);
						}
						break;
						
					case MAXIMALSTROM:
						spi_write_pstr(PSTR("Maximalstrom"));
						if (knopf_losgelassen()) {
							modus_lader_maximalstrom = stromeinstellung(modus_lader_maximalstrom);
						}
						break;
						
					case STROM_ENDE:
						spi_write_pstr(PSTR("Abschaltstrom"));
						if (knopf_losgelassen()) {
							modus_lader_strom_ende = stromeinstellung(modus_lader_strom_ende);
						}
						break;
						
					case START:
						spi_write_pstr(PSTR("Start"));
						if (knopf_losgelassen()) {
							state_lader=0;
							state = LADEN_AKTIV;
						}
						break;
						
					case ZURUECK:
						spi_write_pstr(PSTR("Zurück"));
						if (knopf_losgelassen()) {
							state_lader=0;
							state = AUSWAHL;
						}
						break;
				}
				break;
				
			case MODUS_NETZGERAET:
				uartTxStrln("MODUS_NETZGERÄT");
				static uint8_t modus_netzgeraet_auswahl = 0; // 0=Spannung; 1=Maximalstrom; 2=start; 3=zurück
				delayms(100);
				display_clear();
				spi_write_pstr(PSTR("Modus: Netzgerät"));
				display_set_row(1);
				if (step_links() && (modus_netzgeraet_auswahl != 0)) {
					modus_netzgeraet_auswahl--;
				} else if( step_rechts() && (modus_netzgeraet_auswahl != 3)) {
					modus_netzgeraet_auswahl++;
				}
				switch (modus_netzgeraet_auswahl) {
					case 0:
						spi_write_pstr(PSTR("Sollspannung"));
						if (knopf_losgelassen()) {
							netzgeraet_spannung = spannungseinstellung(netzgeraet_spannung);
						}
						break;
						
					case 1:
						spi_write_pstr(PSTR("Maximalstrom"));
						if (knopf_losgelassen()) {
							netzgeraet_strom = stromeinstellung(netzgeraet_strom);
						}
						break;
						
					case 2:
						spi_write_pstr(PSTR("Start"));
						if (knopf_losgelassen()) {
							state = REGELUNG_NETZGERAET;
						}
						break;
						
					case 3:
						spi_write_pstr(PSTR("zurück"));
						if (knopf_losgelassen()) {
							modus_netzgeraet_auswahl=0;
							state = AUSWAHL;
						}
						break;
				}
				break;
				
			case REGELUNG_NETZGERAET:
				uartTxStrln("REGELUNG_NETZGERÄT");
				netzteil_regulation();
				// Funktion verlassen; Netzteil wird wieder ausgeschaltet.
				state=MODUS_NETZGERAET;
				NT_ON(0);
				delayms(100);
				break;
				
			case LADEN_AKTIV:
				uartTxStrln("LADEN_AKTIV");
				display_clear();
				spi_write_pstr(PSTR("Laden aktiv"));
				if (knopf_losgelassen()) {
					state = MODUS_LADER;
				}
				// TODO jump to ERROR_STATE; LADUNG_FERTIG
				break;
				
			case LADUNG_FERTIG:
				uartTxStrln("LADUNG_FERTIG");
				display_clear();
				spi_write_pstr(PSTR("Laden fertig"));
				if (knopf_losgelassen()) {
					state = MODUS_LADER;
				}
				break;
				
			case ERROR_STATE:
				uartTxStrln("ERROR_STATE");
				NT_ON(0);
				display_clear();
				spi_write_pstr(PSTR("FEHLER!"));
				display_set_row(1);
				switch (errors) {
					case OVERVOLTAGE_ERR:
						spi_write_pstr(PSTR("Ausgangsspannungzu hoch!"));
						break;
					case OVERCURRENT_ERR:
						spi_write_pstr(PSTR("Ausgangsstrom   zu hoch!"));
						break;
					case OVERTEMP1_ERR:
						spi_write_pstr(PSTR("Innentemperatur ueberschritten!"));
						break;
					case OVERTEMP2_ERR:
						spi_write_pstr(PSTR("Stromsensor zu  warm!"));
						break;
					default:
						spi_write_pstr(PSTR("Komischer Fehleraufgetreten..."));
						break;
				}
				errors = NONE; // Lösche Fehlercode
				delayms(100);
				if (knopf_losgelassen()) {
					state = AUSWAHL;
				}
				break;
		}
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
	for(uint8_t i=0; i<3; i++) { // Display braucht noch mehrere Anläufe :D
		DOGM163_init();
		display_clear();
		display_return_home();
	}
	delayms(100);
	
	uartTxStrln("Guten Tag!");
	uartTxStrln("REICH TIME");
	uartTxNewline();
	uartTxNewline();
	
	spi_write_string("   Guten Tag!     gebaut von    Toni und Philipp"); // Begrüßung
	LED(1);
	delayms(1000);
	
// 	uint16_t sollspannung=30000, maximalstrom=10000; // Spannung in Millivolt, Strom in Milliampere
	uint16_t tempspannung=0, tempstrom=0; // werte, die der Regler verändern kann.
	// unabhängig von denen, die der User eingibt (maximalwerte)
	
	sei(); // und es seien Interrupts (aktiv)!
	
	SPKR(1);
	delayms(100);
	SPKR(0);
	stateMachine();
	return 0;
}
