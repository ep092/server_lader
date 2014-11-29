#define B_KNOPF_LOSGELASSEN (knopf & 1)
#define RESET_KNOPF_LOSGELASSEN cbi(knopf, 1)
#define B_KNOPF_GEDRÜCKT ((knopf >> 1) & 1)
#define RESET_KNOPF_GEDRÜCKT cbi(knopf, 1)
#define B_STEP_LINKS ((knopf >>3) & 1)
#define RESET_STEP_LINKS cbi(knopf, 3)
#define B_STEP_RECHTS ((knopf >>2) & 1)
#define RESET_STEP_RECHTS cbi(knopf, 2)

typedef enum {
	INIT_STATE, AUSWAHL, MODUS_LADER, MODUS_NETZGERÄT, REGELUNG_NETZGERÄT
	SPANNUNGSEINSTELLUNG, STROMEINSTELLUNG, ZAHLENWERT_ÄNDERN,
	LADEN_AKTIV, LADUNG_FERTIG, ERROR_STATE
}state_main;

typedef enum {
	LADESCHLUSSSPANNUNG, MAXIMALSTROM, STROM_ENDE, START, ZURÜCK
} state_modus_lader;

state_main state = INIT_STATE;
state_modus_lader state_lader = LADESCHLUSSSPANNUNG;

//TODO noch zu implementieren
uint16_t spannungseinstellung(uint16_t Spannung);
uint16_t stromeinstellung(uint16_t Strom);

uint16_t netzgerät_spannung = 32000;
uint16_t netzgerät_strom = 12000;

uint16_t modus_lader_ladeschlussspannung = 59000;
uint16_t modus_lader_maximalstrom = 5000;
uint16_t modus_lader_strom_ende = 300;

uint8_t knopf_losgelassen() {
	uint8_t return_value = B_KNOPF_LOSGELASSEN;
	RESET_KNOPF_LOSGELASSEN;
	return return_value;
}

uint8_t knopf_gedrückt() {
	uint8_t return_value = B_KNOPF_GEDRÜCKT;
	RESET_KNOPF_GEDRÜCKT;
	return return_value;
}

uint8_t step_links() {
	uint8_t return_value = B_STEP_LINKS;
	RESET_STEP_LINKS;
	return return_value;
}

uint8_t step_rechts() {
	uint8_t return_value = B_STEP_RECHTS;
	RESET_STEP_RECHTS;
	return return_value;
}

void stateMachine() {
	switch (state) {
		case INIT_STATE:
			uartTxStrln("INIT_STATE");
			spi_write_string("   Guten Tag!     gebaut von    Toni und Philipp"); // Begrüßung
			while (knopf_losgelassen() != 1) {
			}
			state = AUSWAHL;
			//      if (knopf_losgelassen()) state=AUSWAHL;
			break;
			
		case AUSWAHL:
			uartTxStrln("AUSWAHL");
			static uint8_t auswahl = 0; //0=LADER; 1=NETZTEIL
			if (step_links() && (auswahl != 0) {
				auswahl--;
			} else if( step_rechts() && (auswahl != MODUS_NETZGERÄT) {
				auswahl++;
			}
			if (knopf_losgelassen()) {
				if (auswahl==0) {
					state=MODUS_LADER;
					break;
				} else if (auswahl==1) {
					state=MODUS_NETZGERÄT;
					break;
				}
				display_clear();
				spi_write_string("Auswahl");
				display_set_row(1);
				if (auswahl==0) {
					spi_write_string("Lader");
				} else if (auswahl==1) {
					spi_write_string("NETZTEIL");
				}
				delayms(100);
			}
			break;
				
		case MODUS_LADER:
			uartTxStrln("MODUS_LADER");
			if (step_links() && (state_lader != 0) {
				state_lader--;
			} else if( step_rechts() && (state_lader != ZURÜCK) {
				state_lader++;
			}
			delayms(100);
			display_clear();
			spi_write_string("Modus: Lader");
			display_set_row(1);
			switch(state_lader) {
				case LADESCHLUSSSPANNUNG:
					spi_write_string("Ladeschlussspannung");
					if (knopf_losgelassen()) {
						modus_lader_ladeschlussspannung = spannungseinstellung(modus_lader_ladeschlussspannung);
					}
					break;
					
				case MAXIMALSTROM:
					spi_write_string("Maximalstrom");
					if (knopf_losgelassen()) {
						modus_lader_maximalstrom = stromeinstellung(modus_lader_maximalstrom);
					}
					break;
					
				case STROM_ENDE:
					spi_write_string("Abschaltstrom");
					if (knopf_losgelassen()) {
						modus_lader_strom_ende = stromeinstellung(modus_lader_strom_ende);
					}
					break;
					
				case START:
					spi_write_string("Start");
					if (knopf_losgelassen()) {
						state = LADEN_AKTIV;
					}
					break;
					
				case ZURÜCK:
					spi_write_string("Zurück")
					if (knopf_losgelassen()) {
						state = AUSWAHL;
					}
					break;
			}
			break;
			
		case MODUS_NETZGERÄT:
			uartTxStrln("MODUS_NETZGERÄT");
			static uint8_t modus_netzgerät_auswahl = 0; //0=Spannung; 1=Maximalstrom; 2=start; 3=zurück
			delayms(100);
			display_clear();
			spi_write_string("Modus: Netzgerät");
			display_set_row(1);
			if (step_links() && (modus_netzgerät_auswahl != 0) {
				modus_netzgerät_auswahl--;
			} else if( step_rechts() && (modus_netzgerät_auswahl != 3) {
				modus_netzgerät_auswahl++;
			}
			switch (modus_netzgerät_auswahl) {
				case 0:
					spi_write_string("Spannung");
					if (knopf_losgelassen()) {
						netzgerät_spannung = spannungseinstellung(netzgerät_spannung);
					}
					break;
					
				case 1:
					spi_write_string("Strom");
					if (knopf_losgelassen()) {
						netzgerät_strom=stromeinstellung(netzgerät_strom);
					}
					break;
					
				case 2:
					spi_write_string("Start");
					if (knopf_losgelassen()) {
						netzteil_regulation(netzgerät_spannung, netzgerät_strom);
					}
					break;
					
				case 3:
					spi_write_string("zurück");
					if (knopf_losgelassen()) {
						state = AUSWAHL;
					}
					break;
			}
			
			break;
			
		case REGELUNG_NETZGERÄT:
			uartTxStrln("REGELUNG_NETZGERÄT");
			if (knopf_losgelassen()) {
				state=MODUS_NETZGERÄT;
			}
			
			break;
			
		//in Funktionen ausgelagert
		/*      
		 *   case SPANNUNGSEINSTELLUNG:
		 *     break; 
		 * 
		 *   case STROMEINSTELLUNG:
		 *     break; 
		 * 
		 *   case ZAHLENWERT_ÄNDERN:
		 *     break; */
		
		case LADEN_AKTIV:
			uartTxStrln("LADEN_AKTIV");
			display_clear();
			spi_write_string("Laden aktiv");
			if (knopf_losgelassen()) {
				state = MODUS_LADER;
			}
			//TODO jump to ERROR_STATE; LADUNG_FERTIG
			break;
			
		case LADUNG_FERTIG:
			uartTxStrln("LADUNG_FERTIG");
			display_clear();
			spi_write_string("Laden fertig");
			if (knopf_losgelassen()) {
				state = MODUS_LADER;
			}
			break;
			
		case ERROR_STATE:
			uartTxStrln("ERROR_STATE");
			display_clear();
			spi_write_string("Error");
			if (knopf_losgelassen()) {
				state = AUSWAHL;
			}
			break;
			
			
	}
}