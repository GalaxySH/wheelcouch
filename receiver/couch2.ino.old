#include <Arduino.h>
#include <SoftPWM.h>
#include "IRremote.hpp"
#include "PinDefinitionsAndMore.h"

/*-----( Global Constants )-----*/
int32_t frequency = 750;//frequency (in Hz)
bool success;
int REVERSE = 0;
int NEUTRAL = 127;// confirm that this is actually a neutral value
int FORWARD = 255;
int motors[2][2] {{9, 10}, {5, 6}};// left forward, left rear, right forward, right rear
int speeds[2] {NEUTRAL, NEUTRAL};// left, right
int fpow = 0;// initial forward power  (vector with values -1, 0, and 1)
int lpow = 0;// initial lateral turning power (vector with values -1, 0, and 1)
String lastPressed = "";

/*
   Helper macro for getting a macro definition as string
*/
#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

/* pins_arduino.h defines the pin-port/bit mapping as PROGMEM so
   you have to read them with pgm_read_xxx(). That's generally okay
   for ordinary use, but really bad when you're writing super fast
   codes because the compiler doesn't treat them as constants and
   cannot optimize them away with sbi/cbi instructions.

   Therefore we have to tell the compiler the PORT and BIT here.
   Hope someday we can find a way to workaround this.

   Check the manual of your MCU for port/bit mapping.

   The following example demonstrates setting channels for all pins
   on the ATmega328P or ATmega168 used on Arduino Uno, Pro Mini,
   Nano and other boards. */
//SOFTPWM_DEFINE_CHANNEL(0, DDRD, PORTD, PORTD0);  //Arduino pin 0
//SOFTPWM_DEFINE_CHANNEL(1, DDRD, PORTD, PORTD1);  //Arduino pin 1
//SOFTPWM_DEFINE_CHANNEL(2, DDRD, PORTD, PORTD2);  //Arduino pin 2
//SOFTPWM_DEFINE_CHANNEL(2, DDRD, PORTD, PORTD3);  //Arduino pin 3
//SOFTPWM_DEFINE_CHANNEL(4, DDRD, PORTD, PORTD4);  //Arduino pin 4
SOFTPWM_DEFINE_CHANNEL(0, DDRD, PORTD, PORTD5);  //Arduino pin 5
SOFTPWM_DEFINE_CHANNEL(1, DDRD, PORTD, PORTD6);  //Arduino pin 6
//SOFTPWM_DEFINE_CHANNEL(7, DDRD, PORTD, PORTD7);  //Arduino pin 7
//SOFTPWM_DEFINE_CHANNEL(8, DDRB, PORTB, PORTB0);  //Arduino pin 8
SOFTPWM_DEFINE_CHANNEL(2, DDRB, PORTB, PORTB1);  //Arduino pin 9
SOFTPWM_DEFINE_CHANNEL(3, DDRB, PORTB, PORTB2);  //Arduino pin 10
//SOFTPWM_DEFINE_CHANNEL(11, DDRB, PORTB, PORTB3);  //Arduino pin 11
//SOFTPWM_DEFINE_CHANNEL(12, DDRB, PORTB, PORTB4);  //Arduino pin 12
//SOFTPWM_DEFINE_CHANNEL(13, DDRB, PORTB, PORTB5);  //Arduino pin 13
//SOFTPWM_DEFINE_CHANNEL(14, DDRC, PORTC, PORTC0);  //Arduino pin A0
//SOFTPWM_DEFINE_CHANNEL(15, DDRC, PORTC, PORTC1);  //Arduino pin A1
//SOFTPWM_DEFINE_CHANNEL(16, DDRC, PORTC, PORTC2);  //Arduino pin A2
//SOFTPWM_DEFINE_CHANNEL(17, DDRC, PORTC, PORTC3);  //Arduino pin A3
//SOFTPWM_DEFINE_CHANNEL(18, DDRC, PORTC, PORTC4);  //Arduino pin A4
//SOFTPWM_DEFINE_CHANNEL(19, DDRC, PORTC, PORTC5);  //Arduino pin A5

/* Here you make an instance of desired channel counts you want
   with the default 256 PWM levels (0 ~ 255). */
SOFTPWM_DEFINE_OBJECT(4);

/*-----( Function )-----*/
String translateIR() {
  uint32_t rawDat = IrReceiver.decodedIRData.decodedRawData;
  Serial.println(rawDat, HEX);

  switch (rawDat) {
    case 0xBA45FF00: return "POWER";
    case 0xB847FF00: return "FUNC/STOP";
    case 0xB946FF00: return "VOL+";
    case 0xBB44FF00: return "FAST BACK";
    case 0xBF40FF00: return "PAUSE";
    case 0xBC43FF00: return "FAST FORWARD";
    case 0xF807FF00: return "DOWN";
    case 0xEA15FF00: return "VOL-";
    case 0xF609FF00: return "UP";
    case 0xE619FF00: return "EQ";
    case 0xF20DFF00: return "ST/REPT";
    case 0xE916FF00: return "0";
    case 0xF30CFF00: return "1";
    case 0xE718FF00: return "2";
    case 0xA15EFF00: return "3";
    case 0xF708FF00: return "4";
    case 0xE31CFF00: return "5";
    case 0xA55AFF00: return "6";
    case 0xBD42FF00: return "7";
    case 0xAD52FF00: return "8";
    case 0xB54AFF00: return "9";
    case 0x0: return "REPEAT";

    default:
      return "OTHER";
  }
}

void controlExec(String cmd) {
  if (cmd == "VOL+") {
    fpow = 1;
  } else if (cmd == "FAST BACK") {
    lpow = -1;
  } else if (cmd == "FAST FORWARD") {
    lpow = 1;
  } else if (cmd == "VOL-") {
    fpow = -1;
  } else if (cmd == "POWER") {
    allStop();
  } else if (cmd == "FUNC/STOP") {
    channelTest();
  } else {
    return;
  }
}

//LOOP THROUGH AND FADE IN CHANNELS
static volatile uint8_t v = 0;
void channelTest() {
  for (uint8_t i = 0; i < Palatis::SoftPWM.size(); ++i) {
    Serial.print(micros());
    Serial.print(" loop(): ");
    Serial.println(i);

    unsigned long const WAIT = 1000000 / Palatis::SoftPWM.PWMlevels() / 2;
    unsigned long nextMicros = 0;
    for (int v = 0; v < Palatis::SoftPWM.PWMlevels() - 1; ++v) {
      while (micros() < nextMicros);
      nextMicros = micros() + WAIT;
      Palatis::SoftPWM.set(i, v);
    }
    for (int v = Palatis::SoftPWM.PWMlevels() - 1; v >= 0; --v) {
      while (micros() < nextMicros);
      nextMicros = micros() + WAIT;
      Palatis::SoftPWM.set(i, v);
    }
  }
}

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

void allStop() {
  lpow = 0;
  fpow = 0;
}

void setup() {
  Serial.begin(9600);   // Status message will be sent to PC at 9600 baud
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
  delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  //SOFTPWM
  Palatis::SoftPWM.begin(60);
  Palatis::SoftPWM.printInterruptLoad();

  //IRRECEIVER
  pinMode(LED_BUILTIN, OUTPUT);

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
      String cmd = translateIR();
      controlExec(cmd);
    }
    IrReceiver.resume();                            // Prepare for the next value
  }

  switch (fpow) {
    case -1: speeds[0] = FORWARD; speeds[1] = FORWARD; break;
    case 0: speeds[0] = NEUTRAL; speeds[1] = NEUTRAL; break;
    case 1: speeds[0] = REVERSE; speeds[1] = REVERSE; break;
  }
  switch (lpow) {
    case -1: left(); break;
    case 1: right(); break;
    default: break;
  }
  Palatis::SoftPWM.set(0, speeds[0]);
  Palatis::SoftPWM.set(1, speeds[0]);
  Palatis::SoftPWM.set(2, speeds[1]);
  Palatis::SoftPWM.set(3, speeds[1]);
  //  Serial.println(String("left: ") + String(speeds[0]) + String(" right: ") + String(speeds[1]));

}
