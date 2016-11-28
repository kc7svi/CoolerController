// Library for math
#include <math.h>

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
const int heartbeatPin = 13;  // pin for heartbeat LED
const int compPin =  11;  // the number of the pin for the compressor relay
const int fanLowPin =  10;  // the number of the pin for the low fan relay
const int fanMedPin =  9;  // the number of the pin for the med fan relay
const int fanHighPin =  8;  // the number of the pin for the high fan relay
const int evapThermPin = 0;  // analog pin for the evaporator temp sensor
const int roomThermPin = 1;  // analog pin for the room temp sensor
const int heartbeatOnTime = 100;  // milliseconds on for heartbeat LED
const int heartbeatOffTime = 1900;  // milliseconds off for heartbeat LED


int valueLR = 1;
int valueTB = 0;
int roomSetHigh = 40;
int roomSetLow = 38;
int evapSetHigh = 33;
int evapSetLow = 24;
int compMinOnTime = 30;  // the minimum seconds the compressor must run before cycling off
int compMinOffTime = 30;  // the minimum seconds the compressor must be off before cycling on
int compRunTimeOn = 0;
int compRunTimeOff = 0;


unsigned long previousHeartbeatMillis = 0;        // will store last time heartbeat changed
int heartbeatState = LOW;
int compRelayState = LOW;
unsigned long previousEvapMillis = 0;        // will store last time evap temp sampled
unsigned long previousRoomMillis = 0;        // will store last time room temp sampled
unsigned long previousButtonMillis = 0;        // will store last time button was pressed
unsigned long previousCompMillis = 0;        // will store last time comp was off


void setup() {
  Serial.begin(9600);
  // set the digital pin as output:
  pinMode(heartbeatPin, OUTPUT);
  pinMode(compPin, OUTPUT);
  pinMode(fanLowPin, OUTPUT);
  pinMode(fanMedPin, OUTPUT);
  pinMode(fanHighPin, OUTPUT);
  analogReference(EXTERNAL)
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  lcd.setBacklight(0x1); // Turn on backlight
  lcd.print("Hello Dave!");
  delay(5000);
  lcd.clear();
}

uint8_t i=0;
void loop() {
  unsigned long currentMillis = millis();
  heartBeat();
  // evap temp sample
  
  if(currentMillis - previousEvapMillis >= evapSampleRate) {
    previousEvapMillis = currentMillis;    // Remember the time
    evapSample10 = evapSample09;
    evapSample09 = evapSample08;
    evapSample08 = evapSample07;
    evapSample07 = evapSample06;
    evapSample06 = evapSample05;
    evapSample05 = evapSample04;
    evapSample04 = evapSample03;
    evapSample03 = evapSample02;
    evapSample02 = evapSample01;
    evapSample01 = analogRead(evapPin);
    evapAverage = (evapSample01 + evapSample02 + evapSample03 + evapSample04 + evapSample05 + evapSample06 + evapSample07 + evapSample08 + evapSample09 + evapSample10)/10;
    // convert to resistance
    evapResistance = 10000.0 / ((1023.0 / evapAverage) - 1.0);
    // convert to degrees F
    evapDegF = evapResistance / 10000.0; //(R/Ro)
    evapDegF = log(evapDegF); // ln(R/Ro)
    evapDegF *= 0.000253; // 1/B * ln(R/Ro)
    evapDegF += .003354; // + (1/To)
    evapDegF = 1.0 / evapDegF; // Invert
    evapDegF -= 274.95; // Deg C with 1.8 offset tested in ice water
    evapDegF = (evapDegF * 1.8) + 32; // Deg F    lcd.setCursor(0, 0);
  }
  
  // room temp sample
//  currentMillis = millis();
  
  if(currentMillis - previousRoomMillis >= roomSampleRate) {
    previousRoomMillis = currentMillis;    // Remember the time
    roomSample10 = roomSample09;
    roomSample09 = roomSample08;
    roomSample08 = roomSample07;
    roomSample07 = roomSample06;
    roomSample06 = roomSample05;
    roomSample05 = roomSample04;
    roomSample04 = roomSample03;
    roomSample03 = roomSample02;
    roomSample02 = roomSample01;
    roomSample01 = analogRead(roomPin);
    roomAverage = (roomSample01 + roomSample02 + roomSample03 + roomSample04 + roomSample05 + roomSample06 + roomSample07 + roomSample08 + roomSample09 + roomSample10)/10;
    Serial.println("room");
    Serial.print("ADC: ");
    Serial.println(roomAverage);
    // convert to resistance
    roomResistance = 10000.0 / ((1023.0 / roomAverage) - 1.0);
    Serial.print("Res: ");
    Serial.println(roomResistance);
    // convert to degrees F
    roomDegF = roomResistance / 10000.0; //(R/Ro)
    roomDegF = log(roomDegF); // ln(R/Ro)
    roomDegF *= 0.000253; // 1/B * ln(R/Ro)
    roomDegF += .003354; // + (1/To)
    roomDegF = 1.0 / roomDegF; // Invert
    roomDegF -= 274.95; // Deg C with 1.8 offset tested in ice water
    roomDegF = (roomDegF * 1.8) + 32; // Deg F
    Serial.print(roomDegF);
    Serial.println(" to Deg F");
    Serial.println();
  }

//  currentMillis = millis();
  
  uint8_t buttons = lcd.readButtons();
  
  if (buttons) {
    if (currentMillis - previousButtonMillis >= 250 ) { // wait 250 ms between button presses
      previousButtonMillis = currentMillis; // Remember the time
      
      if (buttons & BUTTON_LEFT) {
        if (valueLR > 1) {
          valueLR -= 1;
        }
      }
      
      if (buttons & BUTTON_RIGHT) {
        if (valueLR < 6) {
          valueLR += 1;
        }
      }
      
      if (buttons & BUTTON_UP) {
        valueTB += 1;
      }

      if (buttons & BUTTON_DOWN) {
        valueTB -= 1;
      }
      
      if (buttons & BUTTON_SELECT) {
        // lcd.print("SELECT ");
        //    lcd.print(lcdTop);
      }
    }
  }
  
  switch (valueLR) {
    case 1:
      // Room Temp High
      lcd.setCursor(0, 0);
      lcd.print("Room ");
      lcd.print(roomDegF);
      lcd.print("F        ");
      lcd.setCursor(0, 1);
      lcd.print("Set High");
      lcd.print(roomSetHigh);
      lcd.print("F");
      break;
    case 2:
      // Room Temp Low
      lcd.setCursor(0, 0);
      lcd.print("Room ");
      lcd.print(roomDegF);
      lcd.print("F");
      lcd.setCursor(0, 1);
      lcd.print("Set Low ");
      lcd.print(roomSetLow);
      lcd.print("F");
      break;
    case 3:
      // Evap Temp High
      lcd.setCursor(0, 0);
      lcd.print("Evap ");
      lcd.print(evapDegF);
      lcd.print("F        ");
      lcd.setCursor(0, 1);
      lcd.print("Set High ");
      lcd.print(evapSetHigh);
      lcd.print("F");
      break;    
    case 4:
      // Evap Temp Low
      lcd.setCursor(0, 0);
      lcd.print("Evap ");
      lcd.print(evapDegF);
      lcd.print("F");
      lcd.setCursor(0, 1);
      lcd.print("Set Low ");
      lcd.print(evapSetLow);
      lcd.print("F");
      break;
    case 5:
      // Comp On Time
      lcd.setCursor(0, 0);
      lcd.print("Comp ");
      lcd.print(compRunTimeOn);
      lcd.print("Sec      ");
      lcd.setCursor(0, 1);
      lcd.print(" Min On ");
      lcd.print(compMinOnTime);
      lcd.print("Sec");
      break;
    case 6:
      // Comp Off Time
      lcd.setCursor(0, 0);
      lcd.print("Comp ");
      lcd.print(compRunTimeOff);
      lcd.print("Sec      ");
      lcd.setCursor(0, 1);
      lcd.print(" Min Off ");
      lcd.print(compMinOffTime);
      lcd.print("Sec");
      break;
  }

  if (roomDegF > roomSetHigh) {
    if (evapDegF > evapSetHigh) {
      if ((compRelayState == LOW) && (currentMillis - previousCompMillis >= (compMinOffTime * 1000))) {
        compRelayState == HIGH;
        digitalWrite(compRelayPin, compRelayState);
      }
    }
    else if (evapDegF < evapSetLow) {
      if ((compRelayState == HIGH) && (currentMillis - previousCompMillis >= (compMinOnTime * 1000))) {
        compRelayState == LOW;
        digitalWrite(compRelayPin, compRelayState);
      }
    }     
  }
  else if (roomDegF < roomSetLow) {
    if ((compRelayState == HIGH) && (currentMillis - previousCompMillis >= (compMinOnTime * 1000))) {
      compRelayState == LOW;
      digitalWrite(compRelayPin, compRelayState);
    }
  }
}

void heartBeat() {
  // heartbeat LED
  if ( (heartbeatState == HIGH) && (currentMillis - previousHeartbeatMillis >= 900) ) {
    heartbeatState = LOW;  // Turn it off
    previousHeartbeatMillis = currentMillis;  // Remember the time
    digitalWrite(heartbeatPin, heartbeatState);  // Update the actual LED
    // Serial.println("Heartbeat");
  }
  else if ( (heartbeatState == LOW) && (currentMillis - previousHeartbeatMillis >= 100) ) {
    heartbeatState = HIGH;  // turn it on
    previousHeartbeatMillis = currentMillis;   // Remember the time
    digitalWrite(heartbeatPin, heartbeatState);   // Update the actual LED
  }

}
