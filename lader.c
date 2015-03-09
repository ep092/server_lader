//  *    Filename: lader.c
//  *     Version: 0.2.4
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
#include <string.h>
#include <avr/wdt.h>

#include "../AtmelLib/global.h"
#include "../AtmelLib/io/io.h"

// UART RX Interrupt wird aktiviert - Gerät kann damit auf Fernsteuerbefehle reagieren
#define FERNSTEUERUNG

#define I2C_DDR DDRC
#define I2C_PORT PORTC
#define I2C_PIN PINC
#define SCL PC5
#define SDA PC4
uint8_t ERR = 0;
#include "../AtmelLib/io/serial/i2c.h"

// UART: 19200 Baud, 1 Stopbit, no Parity, 8 Bit/frame
#define UBRRH_VALUE 0
#define UBRRL_VALUE 51
#include "../AtmelLib/io/serial/uart.h"

// #define OLED

#ifndef OLED
#define DISPLAY_RS_DDR   DDRD
#define DISPLAY_RS_PORT	 PORTD
#define DISPLAY_RS_PIN	 PD5
#define DISPLAY_CSB_DDR  DDRD
#define DISPLAY_CSB_PORT PORTD
#define DISPLAY_CSB_PIN  PD6
#include "EA_DOGM163L-A-Atmel-Lib/EA_DOGM163L-A.c"
#endif

#ifdef OLED
// OLED-Lib einbinden
#endif

#define LED(x) out(PORTC,PC3,0,x)
// #define NT_ON(x) out(PORTB,PB0,0,x)
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

// Umlaute für das Display
#define AE 0b10001110
#define ae 0b10000100
#define OE 0b10011001
#define oe 0b10010100
#define UE 0b10011010
#define ue 0b10000001


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
	OVERTEMP2_ERR, TIEFENTLADEN, AKKU_FALSCH_HOCH, AKKU_FALSCH_TIEF,
	NONE, REGLER_RAMP_UP
} state_errors;

state_main state = INIT_STATE;
state_modus_lader state_lader = LADESCHLUSSSPANNUNG;
state_errors errors = NONE;

uint16_t netzgeraet_spannung = 32000;
uint16_t netzgeraet_strom = 12000;

uint16_t modus_lader_ladeschlussspannung = 21000;
uint16_t modus_lader_maximalstrom = 2000;
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

void ntOn(uint8_t ausgang) {
	// (PORTB,PB0,0,x)
	if (ausgang == 0) {
		cbi(PORTB, PB0);
		wdt_disable();
	} else {
		wdt_enable(WDTO_500MS);
		sbi(PORTB, PB0);
	}
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
	OCR1A  = 1250; // TOP und Interrupt alle 10ms bei 16MHz clockspeed
}

// Der ADC wird interruptgesteuert
volatile uint8_t counter = 0, zaehler = 0;
// nötig für Berechnung der Netzteilleistung und der gelandenen Energiemenge
volatile float leistung = 0, energie = 0, fuellstand = 0;
volatile uint16_t uNetzteil=0, uReserve=0, strom=0; // U in mV, I in mA

ISR (TIMER1_COMPA_vect, ISR_BLOCK) {
	ADMUX = ADMUX & 0b11100000; // lösche selektiv die MUX-Bits
	ADMUX |= (counter%3); // setze ADC-Kanal neu
	ADCSRA |= 1<<ADSC; // start conversion
	zaehler++;
	if (zaehler == 100) { // es ist eine Sekunde vergangen!
		energie += ((float)strom * (float)uNetzteil) /1000; // in Wattsekunden
		fuellstand += (float)strom /3600; // in mAh
		zaehler = 0;
	}
}


// Drehrad Interrupts
ISR (INT0_vect, ISR_BLOCK) {
	if(knopf & 1<<5) { // Knopf wurde eins nach links gedreht
		sbi(knopf, 3);
		cbi(knopf, 5);
	} else { // knopf wurde eins nach rechts gedreht
		sbi(knopf, 4);
	}
}

// Drehung Pin 2
ISR (INT1_vect, ISR_BLOCK) {
	if(knopf & 1<<4) { // Knopf wurde eins nach rechts gedreht
		sbi(knopf, 2);
		cbi(knopf, 4);
	} else { // knopf wurde eins nach links gedreht
		sbi(knopf, 5);
	}	
}

// KNOPFDRUCK oder loslassen
ISR (PCINT2_vect, ISR_BLOCK) {
	if(PIND & 1<<PD4) { // Knopf ist gerade losgelassen worden
		sbi(knopf, 0);
		cbi(knopf, 1); // lösche gedrückt-Bit wieder
	} else { // Knopf gedrückt, warte auf loslassen
		sbi(knopf, 1); // setze gedrückt-Bit
	}
}

// UART Receive Complete Interrupt
volatile char tempstring[21], tempchar = 0;
volatile uint8_t uartAktuell = 0, uartLastChar = 0;
ISR (USART_RX_vect, ISR_BLOCK) {
	static uint8_t position = 0;
	static char empfangsdaten[21];
	tempchar = UDR;
	uartLastChar = 1; // Ein Zeichen empfangen
	
	// nur schreibbare Zeichen in den Puffer übernehmen, wenn das letzte Frame
	// schon verarbeitet wurde.
	if (uartAktuell == 0) {
		if (tempchar >= 0x21) {
			empfangsdaten[position] = tempchar;
			position++;
			
			// remote echo nur für Zeichen, die in den Puffer übernommen werden
			uartTx(tempchar);
		}		
		if ((position == 20) || tempchar == 0x0D || tempchar == '\n') {
			strcpy (tempstring, empfangsdaten);
			for(uint8_t i=0; i<21; i++) {
				empfangsdaten[i] = 0; // lösche empfangsdaten
			}
			position = 0;
			uartAktuell = 1;
			uartLastChar = 0;
		}
	}
}


// ADC Conversion complete Interrupt
// ADC0 = uNetzteil, ADC1 = uReserve, ADC2 = strom
// Die Werte können noch kalibriert werden, allerdings nicht zur Laufzeit.

#define REFERENZ 4	// externe Referenz, 4096mV also 4mV pro STEP
#define ABWEICHUNG 1.000296589  // gemessene Abweichung in % vom exakten Referenzspannungswert
#define MITTELWERTE 4

ISR (ADC_vect, ISR_BLOCK) {
	// uartTxStrln("adc");
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

			strom = (uint16_t)((float)temp*ABWEICHUNG*((float)50/(float)3));
			if (strom < STROMOFFSET) {
				strom = 0;
			} else {
				strom -= STROMOFFSET;
			}
			break;
	}
	counter++;
	if (counter == 3*MITTELWERTE) { // setze Counter zurück
		counter = 0;
	}
	leistung = ((float)uNetzteil * (float)strom) / 1000; // Leistung in Watt
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
	uartTxStrln("Netzteil aktiv");
	uartAktuell = 1;
// 	nötig zur Kalibrierung!
	STROMOFFSET = 0;
	delayms(200);
	cli();
	uint16_t ulokal = uNetzteil, ilokal = strom;
	sei();
	errors = NONE; // lösche Fehlerspeicher
	STROMOFFSET = ilokal; // Offset wird beim Start des Netzteils rauskalibriert.
	int regelspannung = 0;
	uint8_t run = 1;
	// Voreinstellung des DACs, damit dieser keinen zu großen Sprung ausführen muss
	// TODO: prüfen ob das nötig ist.
	if (netzgeraet_spannung > 6000) {
		setPowerOutput(netzgeraet_spannung - 6000);
	} else {
		setPowerOutput(netzgeraet_spannung);
	}
	display_set_row(0);
	spi_write_pstr(PSTR("Netzger"));
	spi_write_char(ae);
	spi_write_pstr(PSTR("t AKTIV "));
	delayms(500);
	uartTxPstrln(PSTR("Netzgerät schaltet Ausgänge an"));
	uartTxPstrln(PSTR("Bedienung: Spannung verringern mit -, erhöhen mit +"));
	uartTxPstrln(PSTR("Verlassen des Netzteilmodus mit Knopfdruck auf x"));
	ntOn(1); // Output ON
	
	// Betrete Regelschleife, in der die Spannung konstant gehalten wird.
	// wenn Knopf gedrückt oder Error passiert, wieder NT-Modus verlassen
	while ((knopf_losgelassen() == 0) && (state != ERROR_STATE) && run) {
		
		cli();
		ulokal = uNetzteil;
		ilokal = strom;
		sei();
		
		display_set_row(1);
		sprintf(display, "U_out: %5u mV ", ulokal); // Netzteilspannung anzeigen
		spi_write_string(display);
		display_set_row(2);
		sprintf(display, "I_out: %5u mA ", ilokal); // Laststrom anzeigen
		spi_write_string(display);
		
		if (ilokal > netzgeraet_strom) {
			ntOn(0); // Netzteil AUS. Überstrom!
			state = ERROR_STATE;
			errors = OVERCURRENT_ERR;
			uartTxPstrln(PSTR("ERROR - Strom weit über eingestelltes Limit gestiegen!"));
			SPKR(1);
			delayms(1000); // TÜÜT
			SPKR(0);
			continue;
			
		}
		if (ulokal > (netzgeraet_spannung + 1500)) {
			ntOn(0); // Netzteil AUS. Überspannung!
			state = ERROR_STATE;
			errors = OVERVOLTAGE_ERR;
			uartTxPstrln(PSTR("ERROR - Überspannung!"));
			SPKR(1);
			delayms(1000); // TÜÜT
			SPKR(0);
			continue;
			
		}
		if (ulokal - 100 > netzgeraet_spannung) { // Ausgangsspannung zu hoch
			if (regelspannung > 0) {
				regelspannung--;
			}
			delayms(1); // Zeitkonstante künstlich erhöht, damit Regelung nicht schwingt.
			
		}
		if (ulokal + 100 < netzgeraet_spannung) { // Ausgangsspannung zu klein
			if (regelspannung < 256) {
				regelspannung++;
			} else {
				// Regler Ramp-up Error!
				ntOn(0);
				state = ERROR_STATE;
				errors = REGLER_RAMP_UP;
				uartTxPstrln(PSTR("Regler zu weit aufgedreht - evtl. Spannungsmessung defekt"));
				SPKR(1);
				delayms(1000); // TÜÜT
				SPKR(0);
				continue;
			}
			delayms(1);
		}
		
		// Normaler Betrieb, Spannung kann verstellt werden
		if (step_links() && (netzgeraet_spannung > 100)) { // Spannung um 100mV runter, minimal 0V
			netzgeraet_spannung = netzgeraet_spannung - 100;
		}
		
		if (step_rechts() && ((netzgeraet_spannung + 100) < MAXIMALSPANNUNG)) { // Spannung um 100mV hoch
			netzgeraet_spannung = netzgeraet_spannung + 100;
		}
		
		if (uartLastChar) {
			uartLastChar = 0;
			cli();
			uint8_t zeichen = tempchar;
			sei();
			switch (zeichen) {
				case 'x':
					run = 0;
				break;
				
				case '+':
					if ((netzgeraet_spannung + 100) < MAXIMALSPANNUNG) {
						netzgeraet_spannung = netzgeraet_spannung + 100;
					}
				break;
				
				case '-':
					if (netzgeraet_spannung > 100) {
						netzgeraet_spannung = netzgeraet_spannung - 100;
					}
				break;
				default :
					;
				break;
			}
		}
		
		// Text überschreibt sich auch hier selber
		uartTxStr("U=");
		uartTxDec(ulokal);
		uartTxStr(" mV, I=");
		uartTxDec(ilokal);
		uartTxStr(" mA, Imax=");
		uartTxDec(netzgeraet_strom);
		uartTxStr(" mA     ");
		uartTx(0x0D);
		
		if (((netzgeraet_spannung - 300) + regelspannung * 8) > MAXIMALSPANNUNG) {
			; // do nothing.
		} else {
			setPowerOutput((netzgeraet_spannung - 300) + regelspannung * 12);
		}
		wdt_reset();
	}
	// ENDE. Netzteil wird abgeschaltet und State verlassen
	if (errors == NONE) {
		state = MODUS_NETZGERAET;
		uartTxPstrln(PSTR("Netzteil wieder ausgeschaltet - kehre zum Menü zurück"));
	}
	ntOn(0);
	delayms(100);
}

void ladung_regulation(void) {
	uartTxPstrln(PSTR("LADEN_AKTIV"));
	uartAktuell = 1;
	// Lokale Schutzgrenzen - 10% über Lademaximalwerten
	cli();
	uint16_t ulokal = uNetzteil, ilokal = 0; // werden aus uNetzteil und strom erzeugt, damit interrupts
	// nicht die Regelung stören.
	sei();
	uint16_t umax = modus_lader_ladeschlussspannung + (modus_lader_ladeschlussspannung/10);
	uint16_t imax = modus_lader_maximalstrom + (modus_lader_maximalstrom/5);
	uint16_t tempspannung = ulokal - 1000; // Anfangsspannung ist 2V unter aktueller Akkuspannung
	energie = 0;
	errors = NONE;
// 	STROMOFFSET = 0;
	delayms(1000);
	ilokal = strom;
	STROMOFFSET = ilokal;
	
	if (ulokal > modus_lader_ladeschlussspannung) { // Akku schon voll oder falsche Einstellungen!
		// ABBRUCH!
		state = ERROR_STATE;
		errors = AKKU_FALSCH_HOCH;
		uartTxPstrln(PSTR("ERROR - Akkuspannung über Ladeschlussspannung"));
		// Meldung bringen a "maybe falscher Akku oder sowas"
		
	}
	if (ulokal < (modus_lader_ladeschlussspannung/3)) { // Akku wohl tiefentladen (NiCd kann unter 50% haben!)
		// ABBRUCH!
		state = ERROR_STATE;
		errors = AKKU_FALSCH_TIEF;
		uartTxPstrln(PSTR("ERROR - Akku ist wohl Tiefentladen"));
		// Meldung: akku tiefentladen oder falsche Ladeschlussspannung eingestellt
		
	} else { // normaler Modus; aktiviere Output und Regelschleife
		// Output wird auf 5V weniger als die aktuell gemessene Spannung gesetzt.
		uartTxPstrln(PSTR("Ladeparameter:"));
		uartTxStr("Uend = ");
		uartTxDec(modus_lader_ladeschlussspannung);
		uartTxNewline();
		uartTxStr("Imax = ");
		uartTxDec(modus_lader_maximalstrom);
		uartTxNewline();
		uartTxStr("Iend = ");
		uartTxDec(modus_lader_strom_ende);
		uartTxNewline();
		
		setPowerOutput(tempspannung);
		delayms(200);
		display_clear();
		display_set_row(0);
		spi_write_pstr(PSTR("Akku l"));
		spi_write_char(ae);
		spi_write_pstr(PSTR("dt auf!"));
		uartTxPstrln(PSTR("Starte Netzgerät und lade den Akku"));
		ntOn(1); // Output ON
		
		// Betrete Regelschleife, in der die Spannung konstant gehalten wird.
		// wenn Knopf gedrückt oder Error passiert, wieder NT-Modus verlassen
		while ((knopf_losgelassen() == 0) && (state == LADEN_AKTIV)) {
			
			cli(); // u und i werden ausgelesne und gecacht
			ulokal = uNetzteil;
			ilokal = strom;
			sei();
			
			display_set_row(1); // U und I
			snprintf(display, 17, "U=%5u I=%5u", ulokal, ilokal);
			spi_write_string(display);
			display_set_row(2); // P[W] und E[Ah]
			snprintf(display, 17, "P=%5u E=%5u", (uint16_t)leistung, (uint16_t)fuellstand);
			spi_write_string(display);
			if (ilokal > imax) {
				state = ERROR_STATE;
				errors = OVERCURRENT_ERR;
				uartTxPstrln(PSTR("ERROR - Überstrom beim Laden"));
				ntOn(0); // Netzteil AUS. Überstrom!
				SPKR(1);
				delayms(1000); // TÜÜT
				SPKR(0);
				
			}
			if (ulokal > umax) {
				state = ERROR_STATE;
				errors = OVERVOLTAGE_ERR;
				uartTxPstrln(PSTR("ERROR - Überspannung beim Laden"));
				ntOn(0); // Netzteil AUS. Überspannung!
				SPKR(1);
				delayms(1000); // TÜÜT
				SPKR(0);
			}
			
			// Regelung so, dass der Sollstrom erreicht wird, aber die Ladeschlussspannung nicht überschritten.
			if (ulokal < (modus_lader_ladeschlussspannung+100)) { // CC modus, Akkus werden vollgedrückt
				if (ilokal < (modus_lader_maximalstrom - 10)) {
					tempspannung +=6;
					delayms(1);
				} else if (ilokal > (modus_lader_maximalstrom + 10)) {
					tempspannung -=6;
					delayms(1);
				}
			} 
			if (ulokal > modus_lader_ladeschlussspannung) { // CV modus, Akkus fast voll.
				// Ladespannung (tempspannung) nicht ändern.
				if (ilokal < modus_lader_strom_ende) { // Akku voll. BEENDE LADUNG
					state = LADUNG_FERTIG;
					errors = NONE;
					ntOn(0);
					uartTxPstrln(PSTR("ERFOLG - Akku fertig aufgeladen"));
					uartTxPstrln(PSTR("Bestätigen mit ok"));
					for (uint8_t i=0; i<5; i++) {
						SPKR(1);
						delayms(100);
						SPKR(0);
						delayms(500);
					}
				}
				if (ulokal > (modus_lader_ladeschlussspannung + 100) || (ilokal > modus_lader_maximalstrom + 100)) {
					// wenn durch transiente Effekte tempspannung oder Ladestrom zu hoch wurde...
					tempspannung -=100;
				}
			}
			if (uartLastChar) {
				uartLastChar = 0;
				if (tempchar == 'x') {
					state = AUSWAHL;
					uartTxPstrln(PSTR("Ladung abgebrochen - kehre zurück ins Hauptmenü"));
					errors = NONE;
					ntOn(0);
					continue;
				}
			}
// 			keine neue Zeile nach der Meldung - Text überschreibt sich selbst.
			uartTxStr("U=");
			uartTxDec(ulokal);
			uartTxStr(" mV, I=");
			uartTxDec(ilokal);
			uartTxStr(" mA, C=");
			uartTxDec((uint32_t)fuellstand);
			uartTxStr(" mAh      ");
			uartTx(0x0D); // wagenrücklauf ohne neue Zeile!!!
			
			if (ulokal > modus_lader_ladeschlussspannung) {
				; // CV mode, noch warten bis I klein genug
			} else {
				setPowerOutput(tempspannung);
				delayms(10); // Erhöhung der Regelzeitkonstate
			}
			wdt_reset();
		}
	}
	// Akku voll oder Error Abbruch
	ntOn(0);
	delayms(100);
	if (state == LADEN_AKTIV) { // Ladungsabbruch durch Knopfdruck
		state = MODUS_LADER;
	}
}


void stateMachine(void) {
	knopf=0;
	while (1) {
		switch (state) {
			case INIT_STATE:
				spi_write_pstr(PSTR("   Guten Tag!     gebaut von    Toni und Philipp")); // Begrüßung
				while (knopf_losgelassen() != 1) {
					if (uartAktuell) {
						break;
					}
				}
				state = AUSWAHL;
				break;
				
			case AUSWAHL:
				;
				static uint8_t auswahl = 0; //0=LADER; 1=NETZTEIL
				if (step_links() && (auswahl != 0)) {
					auswahl--;
				} else if( step_rechts() && (auswahl != 1)) {
					auswahl++;
				}
				if (knopf_losgelassen()) {
					if (auswahl==0) {
						state=MODUS_LADER;
						uartTxPstrln(PSTR("MODUS_LADER"));
						break;
					} else if (auswahl==1) {
						state=MODUS_NETZGERAET;
						uartTxPstrln(PSTR("MODUS_NETZGERÄT"));
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
							uartTxPstrln(PSTR("LADEN_AKTIV"));
						}
						break;
						
					case ZURUECK:
						spi_write_pstr(PSTR("Zur"));
						spi_write_char(ue);
						spi_write_pstr(PSTR("ck"));
						if (knopf_losgelassen()) {
							state_lader=0;
							state = AUSWAHL;
							uartTxPstrln(PSTR("AUSWAHL"));
						}
						break;
				}
				break;
				
			case MODUS_NETZGERAET:
				;
				static uint8_t modus_netzgeraet_auswahl = 0; // 0=Spannung; 1=Maximalstrom; 2=start; 3=zurück
				delayms(100);
				display_clear();
				spi_write_pstr(PSTR("Modus: Netzger"));
				spi_write_char(ae);
				spi_write_pstr(PSTR("t"));

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
							modus_netzgeraet_auswahl=0;
							state = REGELUNG_NETZGERAET;
						}
						break;
						
					case 3:
						spi_write_pstr(PSTR("Zur"));
						spi_write_char(ue);
						spi_write_pstr(PSTR("ck"));
						if (knopf_losgelassen()) {
							modus_netzgeraet_auswahl=0;
							state = AUSWAHL;
						}
						break;
				}
				break;
				
			case REGELUNG_NETZGERAET:
				netzteil_regulation();
				// Funktion verlassen; Netzteilausgang wird wieder ausgeschaltet.
				ntOn(0);
				delayms(100);
				break;
				
			case LADEN_AKTIV:
				ladung_regulation();
				// Funktion verlassen; Netzteilausgang wird wieder ausgeschaltet.
				ntOn(0);
				delayms(100);
				break;
				
			case LADUNG_FERTIG:
				display_set_row(0);
				spi_write_pstr(PSTR(" Ladung fertig! "));
				// Leistungswerte bleiben vorerst bestehen.
				if (knopf_losgelassen()) {
					state = MODUS_LADER;
				}
				break;
				
			case ERROR_STATE:
				;
				static uint8_t w = 0;
				if (w == 0) {
					uartTxPstrln(PSTR("ERROR_STATE"));
					w = 1;
				}
				ntOn(0);
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
						spi_write_pstr(PSTR("Innentemperatur "));
						spi_write_char(ue);
						spi_write_pstr(PSTR("berschritten!"));
						break;
					case OVERTEMP2_ERR:
						spi_write_pstr(PSTR("Stromsensor zu  warm!"));
						break;
					case AKKU_FALSCH_HOCH:
						spi_write_pstr(PSTR("Akku schon voll!"));
						break;
					case AKKU_FALSCH_TIEF:
						spi_write_pstr(PSTR("Akku tiefentladen!"));
						break;
					case REGLER_RAMP_UP:
						spi_write_pstr(PSTR("Regler zu weit  aufgedreht!"));
						break;
					default:
						spi_write_pstr(PSTR("Komischer Fehleraufgetreten..."));
						break;
				}
				delayms(100);
				if (knopf_losgelassen() || uartAktuell) {
					delayms(100);
					uartTxPstrln(PSTR("Lösche Fehlercodes"));
					uartAktuell = 0;
					state = AUSWAHL;
					errors = NONE; // Lösche Fehlercode
					w = 0;
				}
				break;
		}
		// Fernsteuerung
		// beispielhafte Zeile für LADER:    l,560,220,020\n
		// Bedeutung: Modus,Ladeschlussspannung,Ladestrom,Ladeschlussstrom,newline
		
		// beispielhafte Zeile für NETZTEIL: n,240,100\n
		// Bedeutung: Modus,Ausgangsspannung,Ausgangsstrom,newline
		
		// Spannungen werden hierbei in Vielfachen von 100mV, Ströme von 100mA angegeben
		// Werte müssen inclusive führender Nullen eingegeben werden!
		if (uartAktuell) {
			static uint8_t remoteState = 0;
			// 0 = nicht initialisiert, 1 = lader wartet auf bestätigung, 2 = netzteil wartet...
			delayms(100);
			uartAktuell = 0;
			// Test auf Validität des Datenpakets
			if ((tempstring[1] == ',') && (tempstring[5] == ',')) {
				if (tempstring[0] == 'l') {
					modus_lader_ladeschlussspannung = 0;
					modus_lader_ladeschlussspannung += 10000*(tempstring[2]-0x30);
					modus_lader_ladeschlussspannung += 1000 *(tempstring[3]-0x30);
					modus_lader_ladeschlussspannung += 100  *(tempstring[4]-0x30);
					
					modus_lader_maximalstrom = 0;
					modus_lader_maximalstrom += 10000*(tempstring[6]-0x30);
					modus_lader_maximalstrom += 1000 *(tempstring[7]-0x30);
					modus_lader_maximalstrom += 100  *(tempstring[8]-0x30);

					modus_lader_strom_ende = 0;
					modus_lader_strom_ende += 10000*(tempstring[10]-0x30);
					modus_lader_strom_ende += 1000 *(tempstring[11]-0x30);
					modus_lader_strom_ende += 100  *(tempstring[12]-0x30);
					
					// Sende Daten zur Überprüfung zurück
					uartTxStr("l,");
					uartTxDec(modus_lader_ladeschlussspannung);
					uartTx(',');
					uartTxDec(modus_lader_maximalstrom);
					uartTx(',');
					uartTxDec(modus_lader_strom_ende);
					uartTxStrln("?");
					
					remoteState = 1;
					
				} else if (tempstring[0] == 'n') {
					netzgeraet_spannung = 0;
					netzgeraet_spannung += 10000*(tempstring[2]-0x30);
					netzgeraet_spannung += 1000 *(tempstring[3]-0x30);
					netzgeraet_spannung += 100  *(tempstring[4]-0x30);
					
					netzgeraet_strom = 0;
					netzgeraet_strom += 10000*(tempstring[6]-0x30);
					netzgeraet_strom += 1000 *(tempstring[7]-0x30);
					netzgeraet_strom += 100  *(tempstring[8]-0x30);
					
					uartTxStr("n,");
					uartTxDec(netzgeraet_spannung);
					uartTx(',');
					uartTxDec(netzgeraet_strom);
					uartTxStrln("?");
					
					remoteState = 2;
					
				}
				// hier kann man noch kalibrationsroutinen einbauen!
				
			} else if (tempstring[0] == 'y') {
				if (remoteState == 1) {
					state = LADEN_AKTIV;
					remoteState = 0;
				} else if (remoteState == 2) {
					state = REGELUNG_NETZGERAET;
					remoteState = 0;
				}
			} else if (tempstring[0] == 's') { // Statusausgabe
				uartTxNewline();
				uartTxPstr(PSTR("gemessene Spannung: "));
				uartTxDec(uNetzteil);
				uartTxPstrln(PSTR(" mV"));
			}
		}
	}
}

int main(void) {
	DDRB  = 1<<PB0 | 1<<PB2 | 1<<PB3 | 1<<PB5;
	DDRC  = 1<<PC3 | 1<<PC5;
	DDRD  = 1<<PD1 | 1<<PD5 | 1<<PD6 | 1<<PD7;
	// Pullups
	PORTD = 1<<PD4;
	
	// Sicherer Start nach Boot des Geräts:
	ntOn(0);
	
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
	#ifdef FERNSTEUERUNG
	// UART Receive Complete Interrupt für Fernsteuerung
	UCSR0B |= 1<<RXCIE0;
	#endif
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
	
	setPowerOutput(5000); // setze DAC, damit kein Überschwinger beim ersten Start passiert.
	
	uartTxPstrln(PSTR("Guten Tag!"));
// 	uartTxPstrln(PSTR("REICH TIME"));
	uartTxNewline();
	uartTxPstrln(PSTR("Hilfe: die Eingabe muss mittels Tastatur erfolgen, das Zeilenende \
ist ein 0x0D"));
	uartTxNewline();
	
	spi_write_string("   Guten Tag!     gebaut von    Toni und Philipp"); // Begrüßung
	LED(1);
	delayms(1000);
	
	sei(); // und es seien Interrupts (aktiv)!
	
	SPKR(1);
	delayms(100);
	SPKR(0);
	stateMachine();
	return 0;
}
