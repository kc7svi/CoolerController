// Library for math
#include <math.h>

#include <EEPROM.h>

// Libraries for LCD
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>

/* The shield uses the I2C SCL and SDA pins. On classic Arduinos
this is Analog 4 and 5 so you can't use those for analogRead() anymore
However, you can connect other I2C sensors to the I2C bus and share
the I2C bus.
*/

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

/* Walk in cooler thermostat

Controls the temperature of a walk in cooler by turning the compressor
on and off as needed. Adjustable set temp. Reports average compressor duty cycle
based on last 10 cycles. Reports high and low temp. Has minimum run time for compressor.

The circuit:
* LED attached from pin 13 to ground.
* Note: on most Arduinos, there is already an LED on the board
that's attached to pin 13, so no hardware is needed for this example.
 
created 2015
by Philip Housel
*/

// constants won't change. Used here to set a pin numbers, etc :
const int heartbeatPin = 13;	// pin for heartbeat LED
const int compPin =	11;	// the number of the pin for the compressor relay
const int fanLowPin =	10;	// the number of the pin for the low fan relay
const int fanMedPin =	9;	// the number of the pin for the med fan relay
const int fanHighPin =	8;	// the number of the pin for the high fan relay
const int evapThermPin = 0;	// analog pin for the evaporator temp sensor
const int roomThermPin = 1;	// analog pin for the room temp sensor
const int heartbeatOnTime = 100;	// milliseconds on for heartbeat LED
const int heartbeatOffTime = 1900;	// milliseconds off for heartbeat LED

// min and max values
const int minRoom = 32;
const int maxRoom = 60;
const int minEvap = 10;
const int maxEvap = 40;
const int minComp = 5;
const int maxComp = 180;
const int minFan = 0;
const int maxFan = 3;



int valueLR = 1;
int valueTB = 0;
int roomSetHigh; // declare
int roomSetLow; // declare
int evapSetHigh; // declare
int evapSetLow; // declare
unsigned long compMinOnTime; // declare
unsigned long compMinOffTime; // declare
int fanState; // declare
boolean roomState; // declare
int fanIdleSpeed; // declare
int compOnTime = 0;
int compOffTime = 0;


unsigned long prevHeartbeatMillis = 0;				// will store last time heartbeat changed
int heartbeatState = LOW;
int compState = LOW;
unsigned long prevButtonMillis = 0;				// will store last time button was pressed
unsigned long prevCompOnMillis = 0;				// will store last time comp was off
unsigned long prevCompOffMillis = 0;				// will store last time comp was off
unsigned long prevThermMillis = 0;				// will store last time thermistor was read
unsigned long currentMillis = 0;				// will store last time thermistor was read
float evapTempC;
float evapTemp;
float avgEvapTemp;
float roomTempC;
float roomTemp;
float avgRoomTemp;
boolean initEvapLPF = 1;	// Flag used to initialize the Low Pass Filter
boolean initRoomLPF = 1;	// Flag used to initialize the Low Pass Filter

void setup() {
	Serial.begin(9600);
	// set the digital pin as output:
	pinMode(heartbeatPin, OUTPUT);
	pinMode(compPin, OUTPUT);
	pinMode(fanLowPin, OUTPUT);
	pinMode(fanMedPin, OUTPUT);
	pinMode(fanHighPin, OUTPUT);
	analogReference(EXTERNAL);

	// set up the LCD's number of columns and rows: 
	lcd.begin(16, 2);
	lcd.setBacklight(0x1); // Turn on backlight
	lcd.print("Hello Dave!");
	delay(2000);
	lcd.clear();
	lcd.print("Loading Settings");
	delay(2000);
	lcd.clear();

	EEPROM.get(0,roomSetHigh);
	EEPROM.get(2,roomSetLow);
	EEPROM.get(4,evapSetHigh);
	EEPROM.get(6,evapSetLow);
	EEPROM.get(8,compMinOnTime);
	EEPROM.get(12,compMinOffTime);
	EEPROM.get(16,fanIdleSpeed);
	Serial.println(roomSetHigh);
	Serial.println(roomSetLow);
	Serial.println(evapSetHigh);
	Serial.println(evapSetLow);
	Serial.println(compMinOnTime);
	Serial.println(compMinOffTime);
	Serial.println(fanIdleSpeed);
	
	if((roomSetHigh>=minRoom)&&(roomSetHigh<=maxRoom)&&(roomSetLow>=minRoom)&&(roomSetLow<=maxRoom)&&(evapSetHigh>=minEvap)&&(evapSetHigh<=maxEvap)&&(evapSetLow>=minEvap)&&(evapSetLow<=maxEvap)&&(compMinOnTime>=minComp)&&(compMinOnTime<=maxComp)&&(compMinOffTime>=minComp)&&(compMinOffTime<=maxComp)&&(fanIdleSpeed>=minFan)&&(fanIdleSpeed<=maxFan)) {
		lcd.setCursor(0, 0);
		lcd.print("Settings Loaded");
		lcd.setCursor(0, 1);
		lcd.print("Successfully");
		//fanIdleSpeed=6; //error value for testing
		delay(2000);
		lcd.clear();
	} else {
		lcd.setCursor(0, 0);
		lcd.print("Error Detected!");
		lcd.setCursor(0, 1);
		lcd.print("Loading Defaults");
		delay(2000);
		lcd.clear();
		//load default safe values
		roomSetHigh = 40;
		roomSetLow = 38;
		evapSetHigh = 33;
		evapSetLow = 24;
		compMinOnTime = 30;	// the minimum seconds the compressor must run before cycling off
		compMinOffTime = 30;
		fanIdleSpeed = 3;
	}
}

uint8_t i=0;
void loop() {
	currentMillis = millis();
	heartBeat();

	if(currentMillis - prevThermMillis >= 100) {
		prevThermMillis = currentMillis;		// Remember the time
		evapTempC = getTempFloat(evapThermPin);
		evapTemp = (evapTempC * 1.8) + 32;
		roomTempC = getTempFloat(roomThermPin);
		roomTemp = (roomTempC * 1.8) + 32;
		avgEvapTemp = LPF(evapTemp,50,initEvapLPF,evapThermPin);
		initEvapLPF = 0;
		avgRoomTemp = LPF(roomTemp,50,initRoomLPF,roomThermPin);
		initRoomLPF = 0;
	}
	
	uint8_t buttons = lcd.readButtons();
	
	if (buttons) {
		if (currentMillis - prevButtonMillis >= 100 ) { // wait 100 ms between button presses
			prevButtonMillis = currentMillis; // Remember the time
			
			if (buttons & BUTTON_LEFT) {
				if (valueLR > 1) {
					valueLR --;
				}
			}
			
			if (buttons & BUTTON_RIGHT) {
				if (valueLR < 8) {
					valueLR ++;
				}
			}
			
			if (buttons & BUTTON_UP) {
				valueTB ++;
			}

			if (buttons & BUTTON_DOWN) {
				valueTB --;
			}
			
			if (buttons & BUTTON_SELECT) {
				Serial.println(roomSetHigh);
				Serial.println(roomSetLow);
				Serial.println(evapSetHigh);
				Serial.println(evapSetLow);
				Serial.println(compMinOnTime);
				Serial.println(compMinOffTime);
				Serial.println(fanIdleSpeed);
				if((roomSetHigh>=minRoom)&&(roomSetHigh<=maxRoom)&&(roomSetLow>=minRoom)&&(roomSetLow<=maxRoom)&&(evapSetHigh>=minEvap)&&(evapSetHigh<=maxEvap)&&(evapSetLow>=minEvap)&&(evapSetLow<=maxEvap)&&(compMinOnTime>=minComp)&&(compMinOnTime<=maxComp)&&(compMinOffTime>=minComp)&&(compMinOffTime<=maxComp)&&(fanIdleSpeed>=minFan)&&(fanIdleSpeed<=maxFan)){
					//Save to EEPROM
					lcd.clear();
					lcd.print("Saving Settings");
					delay(2000);
					EEPROM.put(0,roomSetHigh);
					EEPROM.put(2,roomSetLow);
					EEPROM.put(4,evapSetHigh);
					EEPROM.put(6,evapSetLow);
					EEPROM.put(8,compMinOnTime);
					EEPROM.put(12,compMinOffTime);
					EEPROM.put(16,fanIdleSpeed);
					lcd.clear();
					lcd.print("Settings Saved");
					lcd.setCursor(0, 1);
					lcd.print("Successfully");
					delay(2000); 
				} else {
					lcd.clear();
					lcd.print("ERROR ERROR");
					lcd.setCursor(0, 1);
					lcd.print("I'm Sorry Dave");
					delay(2000);	
					lcd.clear();
					lcd.print("I'm Afraid");
					lcd.setCursor(0, 1);
					lcd.print("I Can't Do That");
					delay(2000);	
				}
				// lcd.print("SELECT ");
				//		lcd.print(lcdTop);
			}
		}
	}
	//Serial.print(valueLR);
	//Serial.print(" ");
	//Serial.println(valueTB);
	switch (valueLR) {
		case 1:
			// Default Display
			// zero up down button variable
			valueTB = 0;
			lcd.setCursor(0, 0);
			lcd.print("Room ");
			lcd.print(avgRoomTemp);
			lcd.print("F				");			
			lcd.setCursor(0, 1);
			lcd.print("Evap ");
			lcd.print(avgEvapTemp);
			lcd.print("F				");
			break;			
		case 2:
			// Room Temp High
			if ((roomSetHigh > 32) && (valueTB < 0)){
			roomSetHigh = roomSetHigh + valueTB;
			valueTB = 0;
			} else if ((roomSetHigh < 60) && (valueTB > 0)){
			roomSetHigh = roomSetHigh + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Room ");
			lcd.print(avgRoomTemp);
			lcd.print("F				");
			lcd.setCursor(0, 1);
			lcd.print("Set High ");
			lcd.print(roomSetHigh);
			lcd.print("F				");
			break;
		case 3:
			// Room Temp Low
			if ((roomSetLow > 32) && (valueTB < 0)){
			roomSetLow = roomSetLow + valueTB;
			valueTB = 0;
			} else if ((roomSetLow < 60) && (valueTB > 0)){
			roomSetLow = roomSetLow + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Room ");
			lcd.print(avgRoomTemp);
			lcd.print("F				");
			lcd.setCursor(0, 1);
			lcd.print("Set Low ");
			lcd.print(roomSetLow);
			lcd.print("F				");
			break;
		case 4:
			// Evap Temp High
			if ((evapSetHigh > 10) && (valueTB < 0)){
			evapSetHigh = evapSetHigh + valueTB;
			valueTB = 0;
			} else if ((evapSetHigh < 40) && (valueTB > 0)){
			evapSetHigh = evapSetHigh + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Evap ");
			lcd.print(avgEvapTemp);
			lcd.print("F				");
			lcd.setCursor(0, 1);
			lcd.print("Set High ");
			lcd.print(evapSetHigh);
			lcd.print("F				");
			break;		
		case 5:
			// Evap Temp Low
			if ((evapSetLow > 10) && (valueTB < 0)){
			evapSetLow = evapSetLow + valueTB;
			valueTB = 0;
			} else if ((evapSetLow < 40) && (valueTB > 0)){
			evapSetLow = evapSetLow + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Evap ");
			lcd.print(avgEvapTemp);
			lcd.print("F				");
			lcd.setCursor(0, 1);
			lcd.print("Set Low ");
			lcd.print(evapSetLow);
			lcd.print("F				");
			break;
		case 6:
			// Comp On Time
			if ((compMinOnTime > 5) && (valueTB < 0)){
			compMinOnTime = compMinOnTime + valueTB;
			valueTB = 0;
			} else if ((compMinOnTime < 180) && (valueTB > 0)){
			compMinOnTime = compMinOnTime + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Comp ");
			lcd.print(compOnTime);
			lcd.print("Sec			");
			lcd.setCursor(0, 1);
			lcd.print(" Min On ");
			lcd.print(compMinOnTime);
			lcd.print("Sec			");
			break;
		case 7:
			// Comp Off Time
			if ((compMinOffTime > 5) && (valueTB < 0)){
			compMinOffTime = compMinOffTime + valueTB;
			valueTB = 0;
			} else if ((compMinOffTime < 180) && (valueTB > 0)){
			compMinOffTime = compMinOffTime + valueTB;
			valueTB = 0;
			}
			lcd.setCursor(0, 0);
			lcd.print("Comp ");
			lcd.print(compOffTime);
			lcd.print("Sec			");
			lcd.setCursor(0, 1);
			lcd.print(" Min Off ");
			lcd.print(compMinOffTime);
			lcd.print("Sec			");
			break;
		case 8:
			// Idle Fan Speed 
			if ((fanIdleSpeed > 0) && (valueTB < 0)){
				fanIdleSpeed = fanIdleSpeed + valueTB;
				valueTB = 0;
			} else if ((fanIdleSpeed < 3) && (valueTB > 0)){
				fanIdleSpeed = fanIdleSpeed + valueTB;
				valueTB = 0;				
			}
			lcd.setCursor(0, 0);
			lcd.print("Fan Idle Speed	");
			lcd.setCursor(0, 1);
			lcd.print("Speed ");
			switch(fanIdleSpeed) {
				case 0:
					lcd.print("Off");
					break;
				case 1:
					lcd.print("Low");
					break;
				case 2:
					lcd.print("Med");
					break;
				case 3:
					lcd.print("High");
					break;
			}
			lcd.print("					");
			break;
			
	}

	if (avgRoomTemp > roomSetHigh) { // if room is warm
		roomState = true; // remember room was warm
		fanState = 3; // turn evap fan to high
		if (avgEvapTemp > evapSetHigh) { // if evap is thawed
			if ((compState == LOW) && (currentMillis - prevCompOffMillis >= (compMinOffTime * 1000))) { // if comp off and has been long enough
				prevCompOffMillis = currentMillis;
				compState = HIGH; // turn on compressor
			}
		} else if (avgEvapTemp < evapSetLow) { // if evap frozen
			if ((compState == HIGH) && (currentMillis - prevCompOnMillis >= (compMinOnTime * 1000))) { // if comp on and has been long enough
				prevCompOnMillis = currentMillis;
				compState = LOW; // turn off compressor
			}
		}		 
	} else if (avgRoomTemp < roomSetLow) { // if room is cold
		if ((compState == HIGH) && (currentMillis - prevCompOnMillis >= (compMinOnTime * 1000))) { // if comp on and has been long enough
			prevCompOnMillis = currentMillis;
			compState = LOW; // turn off compressor
			roomState = false; // remember room was cold
			fanState = fanIdleSpeed; // turn evap fan to idle speed
		}
	} else if (roomState==true) { // room not cool
		if (avgEvapTemp < evapSetLow) { // if evap frozen
			if ((compState == HIGH) && (currentMillis - prevCompOnMillis >= (compMinOnTime * 1000))) { // if comp on and has been long enough
				prevCompOnMillis = currentMillis;
				compState = LOW; // turn off compressor
			}
		} else if (avgEvapTemp > evapSetHigh) { // if evap is thawed
			if ((compState == LOW) && (currentMillis - prevCompOffMillis >= (compMinOffTime * 1000))) { // if comp off and has been long enough
				prevCompOffMillis = currentMillis;
				compState = HIGH; // turn on compressor
			}
		}
	}
	digitalWrite(compPin, compState); // set compressor relay pin
	switch(fanState) {
		case 0:
			// evap fan off
			digitalWrite(fanHighPin, LOW);
			digitalWrite(fanMedPin, LOW);
			digitalWrite(fanLowPin, LOW);			
			break;		
		case 1:
			// evap fan low
			digitalWrite(fanHighPin, LOW);
			digitalWrite(fanMedPin, LOW);
			digitalWrite(fanLowPin, HIGH);			
			break;
		case 2:
			// evap fan med
			digitalWrite(fanHighPin, LOW);
			digitalWrite(fanLowPin, LOW);			
			digitalWrite(fanMedPin, HIGH);
			break;
		case 3:
			// evap fan high
			digitalWrite(fanMedPin, LOW);
			digitalWrite(fanLowPin, LOW);			
			digitalWrite(fanHighPin, HIGH);
			break;
		default:
			// evap fan high
			digitalWrite(fanMedPin, LOW);
			digitalWrite(fanLowPin, LOW);			
			digitalWrite(fanHighPin, HIGH);
			break;		
	}
}

void heartBeat() {
	// heartbeat LED
	if ( (heartbeatState == HIGH) && (currentMillis - prevHeartbeatMillis >= 900) ) {
		heartbeatState = LOW;	// Turn it off
		prevHeartbeatMillis = currentMillis;	// Remember the time
		digitalWrite(heartbeatPin, heartbeatState);	// Update the actual LED
	}
	else if ( (heartbeatState == LOW) && (currentMillis - prevHeartbeatMillis >= 100) ) {
		heartbeatState = HIGH;	// turn it on
		prevHeartbeatMillis = currentMillis;	 // Remember the time
		digitalWrite(heartbeatPin, heartbeatState);	 // Update the actual LED
	}

}

float getTempFloat (int thermPin)	{

	/*
	This function converts a Thermistor reading into a corresponding temperature in degrees C.
	
	 The Thermistor is incorporated into a Voltage Divider Circuit with Ra=Thermistor and Rb=10K,
	 ASCII circuit diagram, below:
	 
	 +Vref---[Thermistor]---+--[10K]---GND
													|
												 ADC @ thermPin
	 
	 ADC Values were externally calculated from the Thermistor Resistance Table
	 using the formula:	 ADC = 1023 * 10000/(Rtherm+10000)
	 
	 (ADC: Analog to Digital Converter)
	 
	 The lookup table, below, is an array of integer constants containing 
	 the predicted ADC values for all temperatures between -20 deg C to + 69 deg C.
	 The array index starts at zero, which corresponds to a temperature of -20 deg C.
	 
	 The resolution of the LUT itself is 1 degree C, but because there is some difference 
	 between adjacent ADC values, in some cases more than 10 counts, a linear interpolation 
	 between the two closest entries is performed to give a finer output resolution.
	 */
	 
	const int LUT_Therm[90] = {
		105, 110, 116, 121, 127, 133, 139, 145, 152, 159,	 // -20C to -11C 
		165, 173, 180, 187, 195, 203, 211, 219, 227, 236,	 // -10C to -1C 
		245, 254, 264, 273, 283, 293, 303, 313, 324, 334,	 //	0C	to +9C 
		345, 355, 366, 377, 388, 399, 410, 422, 433, 444,	 //	10C to 19C 
		455, 467, 478, 489, 500, 512, 523, 534, 545, 555,	 //	20C to 29C 
		566, 577, 588, 598, 608, 619, 629, 639, 648, 658,	 //	30C to 39C 
		667, 677, 686, 695, 704, 712, 721, 729, 737, 745,	 //	40C to 49C 
		753, 760, 768, 775, 782, 789, 795, 802, 808, 814,	 //	50C to 59C 
		820, 826, 832, 837, 843, 848, 853, 858, 863, 867		//	60C to 69C 
	};

	float _tempC;	// Intermediate results and final Temperature return value
	int ADC_Lo;	 // The Lower ADC matching value
	int ADC_Hi;	 // The Higher ADC matching value
	int Temp_Lo;	// The Lower whole-number matching temperature
	int Temp_Hi;	// The Higher whole-number matching temperature

	// get raw ADC value from Thermistor voltage divider circuit
	int thermValue = analogRead(thermPin);
	thermValue = 1024 - thermValue;

	// Return dummy value if the sensor reading falls outside of the LUT
	if (thermValue < LUT_Therm[0]) 
		_tempC = -999;	// Under-range dummy value
	else if (thermValue > LUT_Therm[89])
		_tempC = 999;	// Over-range dummy value
	else {

		// if Sensor Value is within range...
		for (int i=0; i <= 89; i++){		// Step through the lookup table and look for a match
			if (LUT_Therm[i] > thermValue) {	// Find the closest Higher ADC value
				ADC_Hi = LUT_Therm[i];
				Temp_Hi = i - 20;			// Record the closest Higher whole-number temperature

				// Get the closest Lower whole-number temperature, taking the lower table boundary into account
				if (i != 0)	{
					ADC_Lo = LUT_Therm[i-1];
					Temp_Lo = i - 21;
				}
				else	{
					ADC_Lo = LUT_Therm[i];
					Temp_Lo = i - 20;
				}

				// Interpolate the temperature value for greater precision
				// Note: the Map function does not use floating-point math, so the integer values
				// of temperature are multiplied by 100, and the result is subsequently divided by 100
				_tempC = float( map(thermValue, ADC_Lo, ADC_Hi, Temp_Lo*100, Temp_Hi*100) )/100;
				 break;	// exit for-next loop after the match is detected
			}
		}
	}
	return (_tempC);
}

float LPF(float input, int bufferSize, boolean initialize, int thermPin)	{

#define bufferCap 50	//maximum buffer capacity

	static float buffer[bufferCap][2];
	byte samples;
	float tempSum;
	float output;

	bufferSize = constrain(bufferSize, 2, bufferCap);

	// Initialize buffer if requested
	if (initialize) {
		for ( int i=0; i < bufferSize; i++ ) {
			buffer[i][thermPin] = input;
		} // END For
	}	// END if

	// Push down the buffer Stack to discard oldest reading, making room for the new
	for ( int i=bufferSize - 2; i >= 0; i--) {
		buffer[i+1][thermPin] = buffer[i][thermPin];
	} // END For

	// Record the Current input value
	buffer[0][thermPin] = input;

	// Calculate current stack average
	tempSum = 0;
	for ( int i=0; i < bufferSize; i++ ) {
		tempSum = tempSum + buffer[i][thermPin];
	} // END For

	output = tempSum / bufferSize;

	return(output);


} // End Room function
