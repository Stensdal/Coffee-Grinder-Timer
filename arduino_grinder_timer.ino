/* 
 *  Coffee-Grinder-Timer.ino
 *  
 *  Copyright (c) 2019, t.stensdal@gmail.com
 *  All rights reserved.
 *  
 *  Helpfull links:
 *  https://playground.arduino.cc/Code/HoldButton
 *  https://www.pjrc.com/teensy/td_libs_Encoder.html
 *  https://github.com/olikraus/u8g2/wiki/u8g2reference
 *  
 *  
*/

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Encoder.h>

/*
 * User Config
 * 
 * Change this if you have another display type or use other pin configurations
*/

// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
//U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

Encoder encoder(2, 3); // Setup encoder with correct pins, swap pins if wrong direction - Use pin 2 and 3 for best performance (These are interrupt pins)
const int ledPin = 13; // Status led
const int grinderPin = 12; // Pin for relay controlling the grinder
const int buttonPin = 10; // Button on rotary encoder or single shot button
//const int buttonPin2 = 11; // double shot button

const bool grinderInverted = false; // if output for relay should be inverted

/*
 * End of user config
 */


// the current state of the LED / Grinder
int ledState = LOW; // LOW means off, HIGH means on
int grinderState = (grinderInverted ? HIGH : LOW); // LOW means off, HIGH means on, reversed if grinderInverted is set to true

double totalRuntime;
unsigned int totalShotsGrinded;
double total_kg_Grinded;
double gramCoffeePerSec = 0.7;

// the current and previous readings from the input pin
int buttonState;
int thisButtonState = HIGH; // High is unpressed
int lastButtonState = HIGH;
// time is measured in milliseconds and will quickly exceed limitations of an integer, so we use a long for these two
unsigned long lastDebounceTime = 0;  // the time the button state last switched
unsigned long debounceDelay = 50;    // the state must remain the same for this many millis to register the button press
bool buttonHold = false;

// Rotary Encoder variables
const int ROTARYMIN = 20; // = 1 second
const int ROTARYMAX = 500; // = 25 seconds
int pos = 300; // = load saved value
int lastPos = pos;

// Countdown variables
int sec; // seconds
int dsec;  // desisecond (tenth of second)
int secPaused;
int dsecPaused;
uint32_t startTime, endTime;
int sLeft, mLeft;
bool countdownPaused = false;


/*--------------------
  ----- Functions ------
  --------------------*/

void saveRuntimeToEeprom(int _runtime) {
  static int _savedRuntime = 0;
  
  if (_savedRuntime != _runtime) {
    _savedRuntime = _runtime;
    EEPROM.write(100, _runtime);
  }
}

void saveTotalRuntimeToEeprom(double _totalRuntime) {
  static double _savedTotalRuntime = 0;
  
  if (_savedTotalRuntime != _totalRuntime) {
    _savedTotalRuntime = _totalRuntime;
      double _temp = _totalRuntime;    
      byte _Mega_Byte = _temp/65535;
      byte _High_Byte = (_temp - (_Mega_Byte*65535)) / 255;
      byte _Low_Byte =  _temp - (_Mega_Byte*65535) - (_High_Byte*255);
      EEPROM.write(110, _Mega_Byte);
      EEPROM.write(111, _High_Byte);
      EEPROM.write(112, _Low_Byte);
  }
}

//void addRuntimeToTotal(int sec, int dsec) {
// unfinished function
//  double _temp   
//}

void drawTimer( uint8_t s, uint8_t m) {
  char m_str[4];
  sprintf(m_str, "%02d.%01d", s, m);

  u8g2.firstPage();
  do {
//    if (grinderState == LOW) { // if grinding
//        // status countdown bar
//    }
//    else if (countdownPaused) { // if paused
//      u8g2.setFont(u8g2_font_ncenR10_tr);
//      u8g2.drawStr(48, 12, "Pause");
//    } else {                    // if idle
//      u8g2.setFont(u8g2_font_ncenR10_tr);
//      u8g2.drawStr(0, 12, "#12");
//      u8g2.drawStr(64, 12, "T:23m");
//    }
//    u8g2.drawHLine(0, 20, 128);
    u8g2.setFont(u8g2_font_osr35_tn);
    u8g2.drawStr(14, 63, m_str);
  } while ( u8g2.nextPage() );
}

void pauseCountdown() {
  ledState = LOW;
  grinderState = (grinderInverted ? HIGH : LOW);
  digitalWrite(grinderPin, grinderState); // sluk relæ
  digitalWrite(ledPin, ledState); // sluk LED
  
  countdownPaused = true;
  Serial.println("Paused");
  buttonState = LOW;
  secPaused = sLeft;
  dsecPaused = mLeft;
  drawTimer(secPaused, dsecPaused);
  while (digitalRead(buttonPin) == false) {
    delay(1);
  }
  Serial.print(secPaused);
  Serial.print(".");
  Serial.println(dsecPaused);
}


void runCountdown() {
  Serial.println("start countdown");
  //delay(1000);

  startTime = millis();
  //Serial.print("Start time: ");
  //Serial.println(startTime);
  ledState = HIGH;
  grinderState = (grinderInverted ? LOW : HIGH); // on
  digitalWrite(grinderPin, grinderState); // Tænd relæ
  digitalWrite(ledPin, ledState); // Tænd LED

 
  if (countdownPaused) { // if paused resume from ramaining time 
    endTime = startTime + (secPaused * 1000) + (dsecPaused * 100);
    countdownPaused = false;
    Serial.println("Resumed from pause");
    //Serial.print(secPaused);
    //Serial.print(".");
    //Serial.println(dsecPaused);
  } else { // use normal 
    endTime = startTime + (sec * 1000) + (dsec * 100);
    //Serial.print("End time: ");
    //Serial.println(endTime);
  }
  
  while (millis() < endTime && countdownPaused == false) {
    sLeft = ( endTime - millis() ) / 1000; // sekunder tilbage
    mLeft = (( endTime - millis() ) % 1000) / 100; // tiendedele af et sekund tilbage(0-99)
    drawTimer(sLeft, mLeft);
    //Serial.print("Time left: ");
    //Serial.print(sLeft);
    //Serial.println(mLeft);

    if (digitalRead(buttonPin) == false) { // pressed
      pauseCountdown();
      Serial.println("Returning to main loop");
      return;
    }
    delay(10);
  }
  ledState = LOW;
  grinderState = (grinderInverted ? HIGH : LOW); // off
  digitalWrite(grinderPin, grinderState); // sluk relæ
  digitalWrite(ledPin, ledState); // sluk LED
  //countdownPaused = false;
  drawTimer(0, 0);
  //addRuntimeToTotal(sec, dsec);
  delay(1500);
  drawTimer(sec, dsec);
}

/*--------------------
  ----- Main Loops -----
  --------------------*/

void setup(void) {
  pinMode(grinderPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  digitalWrite(grinderPin, grinderState); // sluk relæ / HIGH
  digitalWrite(ledPin, ledState); // sluk led // LOW

  delay(1500);
  u8g2.begin();
  delay(500);
  Serial.begin(115200);
  
  encoder.write(pos); // set the start value  
  dsec = (pos/2) % 10; // tiendedele
  sec = (pos/2) / 10;
  drawTimer(sec, dsec);

  if (EEPROM.read(99) == 1) { // if eeprom is configured
    Serial.println("Loading saved state");
    // Load totalRuntime
    double temp = EEPROM.read(110);
    totalRuntime = temp*65535;
    temp = EEPROM.read(111);
    totalRuntime = totalRuntime + (temp*255);
    temp = EEPROM.read(112);
    totalRuntime = totalRuntime + temp;

    // Load last runtime
    pos = EEPROM.read(100)*2;
    lastPos = pos;
  
  } else {
    // First run
    Serial.println("First run, Configuring EEPROM");
    
    totalRuntime = 0;
    totalShotsGrinded = 0;
    EEPROM.write(99, 1);
    saveRuntimeToEeprom(pos/2);
    saveTotalRuntimeToEeprom(0);
  }

  
}

void loop(void) {

  // encoder logic
  
  pos = encoder.read(); // get the current physical position and calc the logical position

  if (lastPos != pos) {
    // Keep position/time inside configured limits
    if (pos < ROTARYMIN) {
      encoder.write(ROTARYMIN);
      pos = ROTARYMIN;
    } else if (pos > ROTARYMAX) {
      encoder.write(ROTARYMAX);
      pos = ROTARYMAX;
    }

    // Convert pos to time and display
    if (!countdownPaused) {
      dsec = (pos/2) % 10;
      sec = (pos/2) / 10;
      drawTimer(sec, dsec);
      lastPos = pos;
      Serial.print(pos);
      Serial.println();
    } else {
      encoder.write(lastPos);
    }
  }

  // Button logic
  
  thisButtonState = digitalRead(buttonPin); // read button state, LOW when pressed, HIGH when not

  if (thisButtonState == LOW) { // if button pressed
    if (lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) { // check debounce
      lastDebounceTime = millis(); // reset the debounce timer
      if (buttonHold) {
      buttonHold = false;
      }
    }
  }

  // if the current state does not match the previous state
  // the button was just pressed/released, or is transition noise

  // if debounce delay have elapsed, if the state remains the same, register the press
  if ((millis() - lastDebounceTime) > debounceDelay && !buttonHold) {

    // on release
    if (thisButtonState == HIGH && lastButtonState == LOW) {
      Serial.println("Countdown starting");
      saveRuntimeToEeprom(pos/2);
      runCountdown();
    }  
    
    // knappen holdt nede i minimum 1,5 sekunder
    if (thisButtonState == LOW && (millis() - lastDebounceTime) > 1500) {
      if (countdownPaused) {
        countdownPaused = false;
        drawTimer(sec, dsec);
        Serial.println("Pause canceled");
      } else {
        //Serial.println("Enter menu");
        //drawMenu();
      }
      
      
      buttonHold = true;
    }  
  }
  
  // persist for next loop iteration
  lastButtonState = thisButtonState;
}
