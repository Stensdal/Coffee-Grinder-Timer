/* 
 *  https://playground.arduino.cc/Code/HoldButton
 *  http://www.mathertel.de/Arduino/RotaryEncoderLibrary.aspx
 *  https://github.com/olikraus/u8g2/wiki/u8g2reference
 *  
 *  
*/

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <U8g2lib.h>
#include <Encoder.h>
#include <Wire.h>


// U8g2 Contructor List (Picture Loop Page Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
//U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_2_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

Encoder encoder(2, 3); // D5 + D6 - Setup encoder with correct pins
const int ledPin = 13; // D4
const int grinderPin = 12; //D7 - High is off low on (Low level relay)
const int buttonPin = 10; // D3 


// the current state of the LED / Grinder
int ledState = LOW; // LOW means off, HIGH means on
int grinderState = HIGH; // LOW means on, HIGH means off

// the current and previous readings from the input pin
int buttonState;
int thisButtonState = HIGH; // High is unpressed
int lastButtonState = HIGH;
// time is measured in milliseconds and will quickly exceed limitations of an integer, so we use a long for these two
unsigned long lastDebounceTime = 0;  // the time the button state last switched
unsigned long debounceDelay = 50;    // the state must remain the same for this many millis to register the button press
bool buttonHold = false;

// Rotary Encoder variables
const int ROTARYMIN = 20; // =1 sekund
const int ROTARYMAX = 600; // =30 sekunder
int pos = 300; // =15 sekunder
int lastPos = pos;

// Countdown variables
int sec; // sekunder
int msec;  // hundrededele sekund
int secPaused;
int msecPaused;
uint32_t startTime, endTime;
int sLeft, mLeft;
bool pauseCountdown = false;


/*--------------------
  ----- Functions ------
  --------------------*/

void drawTimer( uint8_t s, uint8_t m) {
  char m_str[4];
  sprintf(m_str, "%02d.%01d", s, m);

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_logisoso24_tn);
    u8g2.drawStr(32, 31, m_str);
  } while ( u8g2.nextPage() );
}

void fpauseCountdown() {
  ledState = LOW;
  grinderState = HIGH;
  digitalWrite(grinderPin, grinderState); // sluk relæ
  digitalWrite(ledPin, ledState); // sluk LED
  
  pauseCountdown = true;
  Serial.println("Paused");
  buttonState = LOW;
  secPaused = sLeft;
  msecPaused = mLeft;
  while (digitalRead(buttonPin) == false) {
    delay(1);
  }
  Serial.print(secPaused);
  Serial.print(".");
  Serial.println(msecPaused);
}

void runCountdown() {
  Serial.println("start countdown");
  //delay(1000);

  startTime = millis();
  Serial.print("Start time: ");
  Serial.println(startTime);
  ledState = HIGH;
  grinderState = LOW;
  digitalWrite(grinderPin, grinderState); // Tænd relæ
  digitalWrite(ledPin, ledState); // Tænd LED

 
  if (pauseCountdown) { // if paused resume from ramaining time 
    endTime = startTime + (secPaused * 1000) + (msecPaused * 100);
    pauseCountdown = false;
    Serial.println("Resumed from pause");
    Serial.print(secPaused);
    Serial.print(".");
    Serial.println(msecPaused);
  } else { // use normal 
    endTime = startTime + (sec * 1000) + (msec * 100);
    Serial.print("End time: ");
    Serial.println(endTime);
  }
  
  while (millis() < endTime && pauseCountdown == false) {
    sLeft = ( endTime - millis() ) / 1000; // sekunder tilbage
    mLeft = (( endTime - millis() ) % 1000) / 100; // tiendedele af et sekund tilbage(0-99)
    drawTimer(sLeft, mLeft);
    //Serial.print("Time left: ");
    //Serial.print(sLeft);
    //Serial.println(mLeft);

    if (digitalRead(buttonPin) == false) { // pressed
      fpauseCountdown();
      Serial.println("Returning to main loop");
      return;
    }
    delay(10);
  }
  ledState = LOW;
  grinderState = HIGH;
  digitalWrite(grinderPin, grinderState); // sluk relæ
  digitalWrite(ledPin, ledState); // sluk LED
  pauseCountdown = false;
  drawTimer(0, 0);
  delay(1000);
  drawTimer(sec, msec);
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
  //u8g2.setFlipMode(1);
  delay(500);
  Serial.begin(115200);
  
  encoder.write(pos); // set the start value  
  msec = (pos/2) % 10; // tiendedele
  sec = (pos/2) / 10;
  drawTimer(sec, msec);
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
    if (!pauseCountdown) {
      msec = (pos/2) % 10;
      sec = (pos/2) / 10;
      drawTimer(sec, msec);
      lastPos = pos;
      Serial.print(pos);
      Serial.println();
    } else {
      encoder.write(lastPos);
    }
  }

  // Button logic
  thisButtonState = digitalRead(buttonPin); // read button state, LOW when pressed, HIGH when not

  if (thisButtonState == LOW && lastButtonState == HIGH && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis(); // reset the debounce timer
    if (buttonHold) {
     buttonHold = false;
    }
  }

  // if the current state does not match the previous state
  // the button was just pressed/released, or is transition noise

  // once delay millis have elapsed, if the state remains the same, register the press
  if ((millis() - lastDebounceTime) > debounceDelay && !buttonHold) {

    // on release
    if (thisButtonState == HIGH && lastButtonState == LOW) {
      Serial.println("Countdown starting");
      runCountdown();
    }  
    
    // knappen holdt nede i minimum 1,5 sekunder
    if (thisButtonState == LOW && (millis() - lastDebounceTime) > 1500) {
      pauseCountdown = false;
      drawTimer(sec, msec);
      Serial.println("Pause canceled");
      buttonHold = true;
    }  
  }
  
  // persist for next loop iteration
  lastButtonState = thisButtonState;
}


