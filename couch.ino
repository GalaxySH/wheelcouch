/*
McT's note  https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads

 Mimics the fade example but with an extra parameter for frequency. It should dim but with a flicker 
 because the frequency has been set low enough for the human eye to detect. This flicker is easiest to see when 
 the LED is moving with respect to the eye and when it is between about 20% - 60% brighness. The library 
 allows for a frequency range from 1Hz - 2MHz on 16 bit timers and 31Hz - 2 MHz on 8 bit timers. When 
 SetPinFrequency()/SetPinFrequencySafe() is called, a bool is returned which can be tested to verify the 
 frequency was actually changed.
 */

#include <Arduino.h>
#include <PWM.h>
//#define RECORD_GAP_MICROS 12000
#include "IRremote.hpp"

/*-----( Global Constants )-----*/
int32_t frequency = 750;//frequency (in Hz)
bool success;
int REVERSE = 0;
int NEUTRAL = 127;
int FORWARD = 255;
const int IR_RECEIVE_PIN = 2;// Signal Pin of IR receiver to Arduino Digital Pin 11
//const int LED_BUILTIN = 13;
int motors[2][2] {{9, 10}, {5, 6}};// left forward, left rear, right forward, right rear
int speeds[2] {NEUTRAL, NEUTRAL};// left, right
int fpow = 0;// initial forward power  (vector with values -1, 0, and 1)
int lpow = 0;// initial lateral turning power (vector with values -1, 0, and 1)

/*
 * Helper macro for getting a macro definition as string
 */
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

/*-----( Function )-----*/
void translateIR() {
  uint32_t rawDat = IrReceiver.decodedIRData.decodedRawData;
  Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
  
  switch(rawDat){
    case 0xFFA25D: Serial.println("POWER"); break;
    case 0xFFE21D: Serial.println("FUNC/STOP"); break;
    case 0xFF629D: Serial.println("VOL+"); break;
    case 0xFF22DD: Serial.println("FAST BACK");    break;
    case 0xFF02FD: Serial.println("PAUSE");    break;
    case 0xFFC23D: Serial.println("FAST FORWARD");   break;
    case 0xFFE01F: Serial.println("DOWN");    break;
    case 0xFFA857: Serial.println("VOL-");    break;
    case 0xFF906F: Serial.println("UP");    break;
    case 0xFF9867: Serial.println("EQ");    break;
    case 0xFFB04F: Serial.println("ST/REPT");    break;
    case 0xFF6897: Serial.println("0");    break;
    case 0xFF30CF: Serial.println("1");    break;
    case 0xFF18E7: Serial.println("2");    break;
    case 0xFF7A85: Serial.println("3");    break;
    case 0xFF10EF: Serial.println("4");    break;
    case 0xFF38C7: Serial.println("5");    break;
    case 0xFF5AA5: Serial.println("6");    break;
    case 0xFF42BD: Serial.println("7");    break;
    case 0xFF4AB5: Serial.println("8");    break;
    case 0xFF52AD: Serial.println("9");    break;
    case 0xFFFFFFFF: Serial.println(" REPEAT");break;  

  default: 
    Serial.print(" other button   ");
    IrReceiver.printIRResultRawFormatted(&Serial, true);

  }// End Case

} //END translateIR

void left() {
  if (fpow == -1) {
    speeds[0] = NEUTRAL;
  } else {
    speeds[1] = FORWARD;
  }
}

void right() {
  if (fpow == -1) {
    speeds[1] = NEUTRAL;
  } else {
    speeds[0] = FORWARD;
  }
}

void setup() {
  Serial.begin(9600);
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 

  //sets the frequency for the specified pin
  for (size_t i = 0; i < 2; i++)
  {
    for (size_t i2 = 0; i2 < 2; i2++)
    {
      success = SetPinFrequencySafe(motors[i][i2], frequency);
    }
  }
  
  //if the pin frequency was set successfully, turn pin 13 on
  if(success) {
	  pinMode(13, OUTPUT);
	  digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
  }

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
//  Serial.println(F("at pin " + STR(IR_RECEIVE_PIN)));
  //IrReceiver.enableIRIn();// Start the receiver
}

void loop() {
  if (IrReceiver.decode()) {
    IrReceiver.printIRResultShort(&Serial);
//    translateIR();
//    delay(500);// Do not get immediate repeat
    IrReceiver.resume();// receive the next value
  }
  
//  if (IrReceiver.decodeHashOld())   // have we received an IR signal?
//  {
//    Serial.println("hash old raw:" + IrReceiver.decodedIRData.decodedRawData);
//    IrReceiver.resume();// receive the next value
//  }

  switch (fpow) {
    case -1: speeds[0] = FORWARD; speeds[1] = FORWARD; break;
    case 0: speeds[0] = NEUTRAL; speeds[1] = NEUTRAL; break;
    case 1: speeds[0] = REVERSE; speeds[1] = REVERSE; break;
  }
  switch (lpow) {
    case -1: left(); break;
    case 1: right(); break;
  }
  for (size_t i = 0; i < 2; i++)
  {
    for (size_t i2 = 0; i2 < 2; i2++)
    {
      pwmWrite(motors[i][i2], speeds[i]);
    }
  }

//  Serial.println(String("left: ") + String(speeds[0]) + String(" right: ") + String(speeds[1]));
}
