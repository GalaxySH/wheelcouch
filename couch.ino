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
//#include <PWM.h>
#include <SoftPWM.h>
//#define RECORD_GAP_MICROS 12000
#include "IRremote.hpp"
/*
 * Define macros for input and output pin etc.
 */
#include "PinDefinitionsAndMore.h"

/*-----( Global Constants )-----*/
int32_t frequency = 750;//frequency (in Hz)
bool success;
int REVERSE = 0;
int NEUTRAL = 127;
int FORWARD = 255;
//const int IR_RECEIVE_PIN = 2;// Signal Pin of IR receiver to Arduino Digital Pin 11
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
  Serial.println(rawDat, HEX);
  
  switch(rawDat){
    case 0xBA45FF00: Serial.println("POWER"); break;
    case 0xB847FF00: Serial.println("FUNC/STOP"); break;
    case 0xB946FF00: Serial.println("VOL+"); break;
    case 0xBB44FF00: Serial.println("FAST BACK");    break;
    case 0xBF40FF00: Serial.println("PAUSE");    break;
    case 0xBC43FF00: Serial.println("FAST FORWARD");   break;
    case 0xF807FF00: Serial.println("DOWN");    break;
    case 0xEA15FF00: Serial.println("VOL-");    break;
    case 0xF609FF00: Serial.println("UP");    break;
    case 0xE619FF00: Serial.println("EQ");    break;
    case 0xF20DFF00: Serial.println("ST/REPT");    break;
    case 0xE916FF00: Serial.println("0");    break;
    case 0xF30CFF00: Serial.println("1");    break;
    case 0xE718FF00: Serial.println("2");    break;
    case 0xA15EFF00: Serial.println("3");    break;
    case 0xF708FF00: Serial.println("4");    break;
    case 0xE31CFF00: Serial.println("5");    break;
    case 0xA55AFF00: Serial.println("6");    break;
    case 0xBD42FF00: Serial.println("7");    break;
    case 0xAD52FF00: Serial.println("8");    break;
    case 0xB54AFF00: Serial.println("9");    break;
    case 0x0: Serial.println(" REPEAT");break;  

  default: 
    Serial.print(" other button   ");
//    IrReceiver.printIRResultRawFormatted(&Serial, true);

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
  //sets the frequency for the specified pin
//  for (size_t i = 0; i < 2; i++)
//  {
//    for (size_t i2 = 0; i2 < 2; i2++)
//    {
//      success = SetPinFrequencySafe(motors[i][i2], frequency);
//    }
//  }
  
  //if the pin frequency was set successfully, turn pin 13 on
//  if(success) {
//	  pinMode(13, OUTPUT);
//	  digitalWrite(13, HIGH);
//    delay(500);
//    digitalWrite(13, LOW);
//  }

//  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
//  Serial.print(F("Ready to receive IR signals of protocols: "));
//  printActiveIRProtocols(&Serial);
//  Serial.println(F("at pin " + STR(IR_RECEIVE_PIN)));
  //IrReceiver.enableIRIn();// Start the receiver
  
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(9600);   // Status message will be sent to PC at 9600 baud
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

    // infos for receive
    Serial.print(RECORD_GAP_MICROS);
    Serial.println(F(" us is the (minimum) gap, after which the start of a new IR packet is assumed"));
    Serial.print(MARK_EXCESS_MICROS);
    Serial.println(F(" us are subtracted from all marks and added to all spaces for decoding"));
    //Serial.println(LED_BUILTIN);
}

void loop() {
    if (IrReceiver.decode()) {  // Grab an IR code
        // Check if the buffer overflowed
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            Serial.println(F("Overflow detected"));
            Serial.println(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
            // see also https://github.com/Arduino-IRremote/Arduino-IRremote#modifying-compile-options-with-sloeber-ide
        } else {
            Serial.println();                               // 2 blank lines between entries
            Serial.println();
//            IrReceiver.printIRResultShort(&Serial);
//            Serial.println();
//            Serial.println(F("Raw result in internal ticks (50 us) - with leading gap"));
//            IrReceiver.printIRResultRawFormatted(&Serial, false); // Output the results in RAW format
//            Serial.println(F("Raw result in microseconds - with leading gap"));
//            IrReceiver.printIRResultRawFormatted(&Serial, true);  // Output the results in RAW format
//            Serial.println();                               // blank line between entries
//            Serial.print(F("Result as internal ticks (50 us) array - compensated with MARK_EXCESS_MICROS="));
//            Serial.println(MARK_EXCESS_MICROS);
//            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, false); // Output the results as uint8_t source code array of ticks
//            Serial.print(F("Result as microseconds array - compensated with MARK_EXCESS_MICROS="));
//            Serial.println(MARK_EXCESS_MICROS);
//            IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true); // Output the results as uint16_t source code array of micros
//            IrReceiver.printIRResultAsCVariables(&Serial);  // Output address and data as source code variables
//
//            IrReceiver.compensateAndPrintIRResultAsPronto(&Serial);

            /*
             * Example for using the compensateAndStorePronto() function.
             * Creating this String requires 2210 bytes program memory and 10 bytes RAM for the String class.
             * The String object itself requires additional 440 bytes RAM from the heap.
             * This values are for an Arduino UNO.
             */
//        Serial.println();                                     // blank line between entries
//        String ProntoHEX = F("Pronto HEX contains: ");        // Assign string to ProtoHex string object
//        if (int size = IrReceiver.compensateAndStorePronto(&ProntoHEX)) {   // Dump the content of the IReceiver Pronto HEX to the String object
//            // Append compensateAndStorePronto() size information to the String object (requires 50 bytes heap)
//            ProntoHEX += F("\r\nProntoHEX is ");              // Add codes size information to the String object
//            ProntoHEX += size;
//            ProntoHEX += F(" characters long and contains "); // Add codes count information to the String object
//            ProntoHEX += size / 5;
//            ProntoHEX += F(" codes");
//            Serial.println(ProntoHEX.c_str());                // Print to the serial console the whole String object
//            Serial.println();                                 // blank line between entries
//        }
          translateIR();
        }
        IrReceiver.resume();                            // Prepare for the next value
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
      //Serial.print("spd");
//      pwmWrite(motors[i][i2], speeds[i]);
    }
  }
//  Serial.println(String("left: ") + String(speeds[0]) + String(" right: ") + String(speeds[1]));
}
