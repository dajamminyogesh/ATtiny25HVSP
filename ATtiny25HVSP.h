// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _ATtiny25HVSP_H_
#define _ATtiny25HVSP_H_
#include "Arduino.h"
//add your includes for the project ATtiny25HVSP here


//end of add your includes here


//add your function definitions for the project ATtiny25HVSP here

// Defaults for ATtiny25/45/85
#define	HFUSE					0xDF
#define	LFUSE					0x62
#define FLAG_FLASH		true
#define FLAG_EEPROM		false

#define SIZE_FLASH		2048	//bytes
#define SIZE_EEPROM		128		//bytes
#define PAGE_SIZE			256		//unit
#define UNIT_FLASH		2			//bytes = word
#define UNIT_EEPROM		1			//byte


void setup();
void loop();
void establishContact();
void HVSP_Mode();
int shiftOut2(uint8_t dataPin, uint8_t dataPin1, uint8_t clockPin, uint8_t bitOrder, byte val, byte val1);
void printHex8Bit(uint8_t data);

void chipErase();

void writeFlashOrEEPROM(bool zone, const uint8_t *hex);
void writeFlashOrEEPROM_PGM(bool zone, const uint8_t *hex);
void writeFileToFlashOrEEPROM(bool zone, const char *file);
void writeFileToFlashOrEEPROM_PGM(bool zone, const char *file);

void readFlashOrEEPROM(bool zone, bool print = false);

void writeFuses(uint8_t hfuse = HFUSE, uint8_t lfuse = LFUSE);
void readFuses();

void readSignature();
void readCalibration();


//Do not add code below this line
#endif /* _ATtiny25HVSP_H_ */
