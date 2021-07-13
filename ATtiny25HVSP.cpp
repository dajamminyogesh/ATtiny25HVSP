// Do not remove the include below
#include "ATtiny25HVSP.h"

#include "Arduino.h"
#include "HexArray.h"
#include "HexFile.h"

#include "Servo.h"

#define  RST     13    // Output to level shifter for !RESET from transistor to Pin 1
#define  CLKOUT  12    // Connect to Serial Clock Input (SCI) Pin 2
#define  DATAIN  11    // Connect to Serial Data Output (SDO) Pin 7
#define  INSTOUT 10    // Connect to Serial Instruction Input (SII) Pin 6
#define  DATAOUT  A1   // Connect to Serial Data Input (SDI) Pin 5
#define  VCC      8    // Connect to VCC Pin 8

#define  BLTOUCH_PIN	9
#define  BLTOUCH_PIN_1	10

int inByte = 0;         // incoming serial byte Computer
int inData = 0;         // incoming serial byte AVR
bool hasTask = false;
char taskCode = 0;

Servo blTouch = Servo();
Servo blTouch_1 = Servo();

static void setup_HVSP(){
	// Set up control lines for HV parallel programming
	pinMode(VCC, OUTPUT);
	pinMode(RST, OUTPUT);
	pinMode(DATAOUT, OUTPUT);
	pinMode(INSTOUT, OUTPUT);
	pinMode(CLKOUT, OUTPUT);
	pinMode(DATAIN, OUTPUT);  // configured as input when in programming mode

	// Initialize output pins as needed
	digitalWrite(RST, HIGH);  // Level shifter is inverting, this shuts off 12V
	delay(10);
}

static void setup_blTouch(){
	pinMode(LED_BUILTIN, INPUT_PULLUP);

	pinMode(BLTOUCH_PIN, OUTPUT);
	pinMode(BLTOUCH_PIN_1, OUTPUT);
	blTouch.attach(BLTOUCH_PIN);
	blTouch_1.attach(BLTOUCH_PIN_1);
	delay(10);
}

static void disable_blTouch(){
	blTouch.write(0);
	blTouch_1.write(0);
	blTouch.detach();
	blTouch_1.detach();
	pinMode(BLTOUCH_PIN, INPUT_PULLUP);
	pinMode(BLTOUCH_PIN_1, INPUT_PULLUP);
	delay(10);
}

static void bltouch_write(unsigned int us){
	blTouch.write(us);
	blTouch_1.write(us);
}

void setup(){
	setup_HVSP();
	disable_blTouch();

	Serial.begin(115200);

	taskCode = 0xFF;
	hasTask = false;
}

void loop(){
	if(Serial.available() > 0){
		hasTask = true;

		taskCode = Serial.peek();
		delay(100);		//wait data rx complete.
		while(Serial.available())			Serial.read();

		Serial.println(taskCode);
		Serial.println("---------------------- Begin ---------------------------");
	}else{
		establishContact();
	}

	if(hasTask && taskCode){
		switch(taskCode){
		case '0':
			disable_blTouch();
			setup_HVSP();
			HVSP_Mode();
			break;
		case '1':
			Serial.println("	PUSH BLTOUCH.");
			setup_blTouch();
			bltouch_write(10);
			break;
		case '2':
			Serial.println("	PULL BLTOUCH.");
			setup_blTouch();
			bltouch_write(90);
			break;
		case '3':
			Serial.println("	RELEASE ALARM.");
			setup_blTouch();
			bltouch_write(160);
			break;
		case '4':
			Serial.println("	SELF-TEST MODE.");
			setup_blTouch();
			bltouch_write(120);
			break;
		case '5':
			Serial.println("	TEST MODE.");
			setup_blTouch();
			bltouch_write(60);
			break;
		default:
			Serial.println("	Invalid mode code.");
			break;
		}
		Serial.println("----------------------  End  ---------------------------");
		Serial.println();
		Serial.println();
		hasTask = false;
	}
	return;

	// if we get a valid byte, run:
	if(Serial.available() > 0){
		// get incoming byte:
		inByte = Serial.read();
		Serial.println(byte(inByte));
	}
}

void establishContact(){
	if(!hasTask && Serial.available() <= 0 && taskCode){
		Serial.println("Enter a character to continue");   // send an initial string
		Serial.println("	Type '0' to enter HVSP mode.");
		Serial.println("	Type '1' to PUSH BLTOUCH.");
		Serial.println("	Type '2' to PULL BLTOUCH.");
		Serial.println("	Type '3' to RELEASE ALARM of BLTOUCH.");
		Serial.println("	Type '4' to make BLTOUCH self-test.");
		Serial.println("	Type '5' to TEST BLTOUCH.");
		Serial.print("Please Type a  character: ");
		taskCode = 0;
		delay(100);
	}
}

void HVSP_Mode(){
	Serial.println("Entering programming Mode\n");

	// Initialize pins to enter programming mode
	pinMode(DATAIN, OUTPUT);  //Temporary
	digitalWrite(DATAOUT, LOW);
	digitalWrite(INSTOUT, LOW);
	digitalWrite(DATAIN, LOW);
	digitalWrite(RST, HIGH);  // Level shifter is inverting, this shuts off 12V

	// Enter High-voltage Serial programming mode
	digitalWrite(VCC, HIGH);  // Apply VCC to start programming process
	delayMicroseconds(20);
	digitalWrite(RST, LOW);   //Turn on 12v
	delayMicroseconds(10);
	pinMode(DATAIN, INPUT);   //Release DATAIN
	delayMicroseconds(300);

	//Programming mode

	chipErase();
	readFuses();
	writeFuses();
	readFuses();

//	readSignature();
//	readCalibration();
//
//	readFlashOrEEPROM(FLAG_FLASH, true);
//	readFlashOrEEPROM(FLAG_EEPROM, true);
//
//	readFlashOrEEPROM(FLAG_FLASH);
//	chipErase();
//	writeFlashOrEEPROM_PGM(FLAG_FLASH, ATtiny25HexPGM);
//	readFlashOrEEPROM(FLAG_FLASH, true);
//	chipErase();

	Serial.println("Exiting programming Mode\n");
	digitalWrite(CLKOUT, LOW);
	digitalWrite(VCC, LOW);
	digitalWrite(RST, HIGH);   //Turn off 12v
}

int shiftOut2(uint8_t dataPin, uint8_t dataPin1, uint8_t clockPin, uint8_t bitOrder, byte val, byte val1){
	int i;
	int inBits = 0;
	//Wait until DATAIN goes high
	while(!digitalRead(DATAIN))
		;

	//Start bit
	digitalWrite(DATAOUT, LOW);
	digitalWrite(INSTOUT, LOW);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);

	for(i = 0; i < 8; i++){
		if(bitOrder == LSBFIRST){
			digitalWrite(dataPin, !!(val & (1 << i)));
			digitalWrite(dataPin1, !!(val1 & (1 << i)));
		}else{
			digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			digitalWrite(dataPin1, !!(val1 & (1 << (7 - i))));
		}
		inBits <<= 1;
		inBits |= digitalRead(DATAIN);
		digitalWrite(clockPin, HIGH);
		digitalWrite(clockPin, LOW);
	}

	//End bits
	digitalWrite(DATAOUT, LOW);
	digitalWrite(INSTOUT, LOW);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);
	digitalWrite(clockPin, HIGH);
	digitalWrite(clockPin, LOW);

	return inBits;
}

void printHex8Bit(uint8_t data){
	if(data < 0x10){
		Serial.print("0");
	}
	Serial.print(data, HEX);
}

#define SEND_COMMAND(sdi, sii)	shiftOut2(DATAOUT, INSTOUT, CLKOUT, MSBFIRST, sdi, sii)

void chipErase(){
	//Chip Erase
	Serial.println("Chip Erase");
	SEND_COMMAND(0x80, 0x4C);
	SEND_COMMAND(0x00, 0x64);
	SEND_COMMAND(0x00, 0x6C);
	SEND_COMMAND(0x00, 0x4C);
}

void writeFlashOrEEPROM(bool zone, const uint8_t *hex){
	//Write Flash or EEPROM
	uint16_t H_NUM = 0, L_NUM = 0, E_NUM = 0;

	if(zone == FLAG_FLASH){
		Serial.println("Write Flash");
		SEND_COMMAND(0x10, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_FLASH / UNIT_FLASH) / PAGE_SIZE;
		E_NUM = (SIZE_FLASH / UNIT_FLASH) % PAGE_SIZE;
	}else{
		Serial.println("Write EEPROM");
		SEND_COMMAND(0x11, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_EEPROM / UNIT_EEPROM) / PAGE_SIZE;
		E_NUM = (SIZE_EEPROM / UNIT_EEPROM) % PAGE_SIZE;
	}

	for(uint16_t aH = 0; aH <= H_NUM; aH++){
		if(aH == H_NUM && !E_NUM) break;

		SEND_COMMAND(aH, 0x1C);
		for(uint16_t bL = 0; bL < L_NUM; bL++){
			if(aH == H_NUM && bL >= E_NUM) break;

			SEND_COMMAND(bL, 0x0C);
			SEND_COMMAND(*hex++, 0x2C);
			if(zone == FLAG_FLASH){
				SEND_COMMAND(*hex++, 0x3C);
				SEND_COMMAND(0x00, 0x7D);
				SEND_COMMAND(0x00, 0x7C);
			}else{
				SEND_COMMAND(0x00, 0x6D);
				SEND_COMMAND(0x00, 0x6C);
			}
			SEND_COMMAND(0x00, 0x64);
			SEND_COMMAND(0x00, 0x6C);
		}
	}

	SEND_COMMAND(0x00, 0x4C);

	Serial.println();
	Serial.println();
}

void writeFlashOrEEPROM_PGM(bool zone, const uint8_t *hex){
	//Write Flash or EEPROM
	uint16_t H_NUM = 0, L_NUM = 0, E_NUM = 0;

	if(zone == FLAG_FLASH){
		Serial.println("Write Flash");
		SEND_COMMAND(0x10, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_FLASH / UNIT_FLASH) / PAGE_SIZE;
		E_NUM = (SIZE_FLASH / UNIT_FLASH) % PAGE_SIZE;
	}else{
		Serial.println("Write EEPROM");
		SEND_COMMAND(0x11, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_EEPROM / UNIT_EEPROM) / PAGE_SIZE;
		E_NUM = (SIZE_EEPROM / UNIT_EEPROM) % PAGE_SIZE;
	}

	for(uint16_t aH = 0; aH <= H_NUM; aH++){
		if(aH == H_NUM && !E_NUM) break;

		SEND_COMMAND(aH, 0x1C);
		for(uint16_t bL = 0; bL < L_NUM; bL++){
			if(aH == H_NUM && bL >= E_NUM) break;
			uint16_t index = aH * L_NUM + bL;

			SEND_COMMAND(bL, 0x0C);
			if(zone == FLAG_FLASH){
				SEND_COMMAND(pgm_read_byte(hex + index * 2), 0x2C);
				SEND_COMMAND(pgm_read_byte(hex + index * 2 + 1), 0x3C);
				SEND_COMMAND(0x00, 0x7D);
				SEND_COMMAND(0x00, 0x7C);
			}else{
				SEND_COMMAND(pgm_read_byte(hex + index), 0x2C);
				SEND_COMMAND(0x00, 0x6D);
				SEND_COMMAND(0x00, 0x6C);
			}
			SEND_COMMAND(0x00, 0x64);
			SEND_COMMAND(0x00, 0x6C);

			if(!(bL & 0x0F)) Serial.print("#");
		}
	}

	SEND_COMMAND(0x00, 0x4C);

	Serial.println();
	Serial.println();
}

void writeFileToFlashOrEEPROM(bool zone, const char *file){

}

void writeFileToFlashOrEEPROM_PGM(bool zone, const char *file){
	//Write File to Flash or EEPROM
	if(zone == FLAG_FLASH){
		Serial.println("Write File To Flash");
		SEND_COMMAND(0x10, 0x4C);
	}else{
		Serial.println("Write File To EEPROM");
		SEND_COMMAND(0x11, 0x4C);
	}

	char c, *cchar = file;
	const char token = ':';
	uint8_t len = 0, addr_low = 0, addr_high = 0, type = 0;

	c = pgm_read_byte(cchar++);

//	for(uint16_t aH = 0; aH <= H_NUM; aH++){
//		if(aH == H_NUM && !E_NUM)			break;
//
//		SEND_COMMAND(aH, 0x1C);
//		for(uint16_t bL = 0; bL < L_NUM; bL++){
//			if(aH == H_NUM && bL >= E_NUM)		break;
//			uint16_t index = aH * L_NUM + bL;
//
//			SEND_COMMAND(bL, 0x0C);
//			if(zone == FLAG_FLASH){
//				SEND_COMMAND(pgm_read_byte(hex + index * 2), 0x2C);
//				SEND_COMMAND(pgm_read_byte(hex + index * 2 + 1), 0x3C);
//				SEND_COMMAND(0x00, 0x7D);
//				SEND_COMMAND(0x00, 0x7C);
//			} else {
//				SEND_COMMAND(pgm_read_byte(hex + index), 0x2C);
//				SEND_COMMAND(0x00, 0x6D);
//				SEND_COMMAND(0x00, 0x6C);
//			}
//			SEND_COMMAND(0x00, 0x64);
//			SEND_COMMAND(0x00, 0x6C);
//
//			if(!(bL & 0x0F))
//				Serial.print("#");
//		}
//	}
//
//	SEND_COMMAND(0x00, 0x4C);
//
//	Serial.println();
//	Serial.println();
}

void readFlashOrEEPROM(bool zone, bool print){
	//Read Flash Or EEPROM
	uint16_t H_NUM = 0, L_NUM = 0, E_NUM = 0;

	if(zone == FLAG_FLASH){
		Serial.println("Read Flash");
		SEND_COMMAND(0x02, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_FLASH / UNIT_FLASH) / PAGE_SIZE;
		E_NUM = (SIZE_FLASH / UNIT_FLASH) % PAGE_SIZE;
	}else{
		Serial.println("Read EEPROM");
		SEND_COMMAND(0x03, 0x4C);
		L_NUM = PAGE_SIZE;
		H_NUM = (SIZE_EEPROM / UNIT_EEPROM) / PAGE_SIZE;
		E_NUM = (SIZE_EEPROM / UNIT_EEPROM) % PAGE_SIZE;
	}

	for(uint16_t aH = 0; aH <= H_NUM; aH++){
		if(aH == H_NUM && !E_NUM) break;

		SEND_COMMAND(aH, 0x1C);
		for(uint16_t bL = 0; bL < L_NUM; bL++){
			if(aH == H_NUM && bL >= E_NUM) break;

			if(!(bL & 0x0F)){
				if(print)
					Serial.println();
				else
					Serial.print("#");
			}

			SEND_COMMAND(bL, 0x0C);
			SEND_COMMAND(0x00, 0x68);
			inData = SEND_COMMAND(0x00, 0x6C);
			if(print) printHex8Bit(inData);

			if(zone == FLAG_FLASH){
				SEND_COMMAND(0x00, 0x78);
				inData = SEND_COMMAND(0x00, 0x7C);
				if(print) printHex8Bit(inData);
			}
		}
	}

	Serial.println();
	Serial.println();
}

void writeFuses(uint8_t hfuse, uint8_t lfuse){
	//Write hfuse
	Serial.println("Writing hfuse");
	SEND_COMMAND(0x40, 0x4C);
	SEND_COMMAND(hfuse, 0x2C);
	SEND_COMMAND(0x00, 0x74);
	SEND_COMMAND(0x00, 0x7C);

	//Write lfuse
	Serial.println("Writing lfuse\n");
	SEND_COMMAND(0x40, 0x4C);
	SEND_COMMAND(lfuse, 0x2C);
	SEND_COMMAND(0x00, 0x64);
	SEND_COMMAND(0x00, 0x6C);
}

void readFuses(){
	//Read lfuse
	SEND_COMMAND(0x04, 0x4C);
	SEND_COMMAND(0x00, 0x68);
	inData = SEND_COMMAND(0x00, 0x6C);
	Serial.print("lfuse reads as ");
	printHex8Bit(inData);
	Serial.println();

	//Read hfuse
	SEND_COMMAND(0x04, 0x4C);
	SEND_COMMAND(0x00, 0x7A);
	inData = SEND_COMMAND(0x00, 0x7E);
	Serial.print("hfuse reads as ");
	printHex8Bit(inData);
	Serial.println();

	//Read efuse
	SEND_COMMAND(0x04, 0x4C);
	SEND_COMMAND(0x00, 0x6A);
	inData = SEND_COMMAND(0x00, 0x6E);
	Serial.print("efuse reads as ");
	printHex8Bit(inData);
	Serial.println();
	Serial.println();
}

void readSignature(){
	//Read Signature
	Serial.println("Read Signature");
	for(uint8_t bL = 0; bL < 3; bL++){
		SEND_COMMAND(0x08, 0x4C);
		SEND_COMMAND(bL, 0x0C);
		SEND_COMMAND(0x00, 0x68);
		inData = SEND_COMMAND(0x00, 0x6C);
		printHex8Bit(inData);
	}
	Serial.println();
	Serial.println();
}

void readCalibration(){
	//Read Calibration
	Serial.println("Read Calibration");
	SEND_COMMAND(0x08, 0x4C);
	SEND_COMMAND(0x00, 0x0C);
	SEND_COMMAND(0x00, 0x78);
	inData = SEND_COMMAND(0x00, 0x7C);
	printHex8Bit(inData);
	Serial.println();
	Serial.println();
}

