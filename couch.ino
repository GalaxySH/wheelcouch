#include <Arduino.h>
#include <SoftPWM.h>
#include "PinDefinitionsAndMore.h"

#define IRMP_PROTOCOL_NAMES              1 // Enable protocol number mapping to protocol strings - needs some FLASH. Must before #include <irmp*>
#define IRMP_USE_COMPLETE_CALLBACK       1 // Enable callback functionality
#define IRMP_ENABLE_PIN_CHANGE_INTERRUPT   // Enable interrupt functionality

#define IRMP_SUPPORT_NEC_PROTOCOL               1       // NEC + APPLE + ONKYO  >= 10000                 ~300 bytes
#define IRMP_SUPPORT_SAMSUNG_PROTOCOL           1       // Samsung + Samsg32    >= 10000                 ~300 bytes
#define IRMP_SUPPORT_KASEIKYO_PROTOCOL          1       // Kaseikyo             >= 10000                 ~250 bytes
#define IRMP_SUPPORT_RC6_PROTOCOL               1       // RC6 & RC6A           >= 10000                 ~250 bytes
#define IRMP_SUPPORT_DENON_PROTOCOL             1       // DENON, Sharp         >= 10000                 ~250 bytes

#include <irmp.hpp>

/*-----( Global Constants )-----*/
int32_t frequency = 750;//frequency (in Hz)
bool success;
int REVERSE = 0;
int NEUTRAL = 128;//127// confirm that this is actually a neutral value
int FORWARD = 255;
int motors[2][2] {{9, 10}, {5, 6}};// left forward, left rear, right forward, right rear
int speeds[2] {NEUTRAL, NEUTRAL};// left, right
int fpow = 0;// initial forward power  (vector with values -1, 0, and 1)
int lpow = 0;// initial lateral turning power (vector with values -1, 0, and 1)
String lastPressed = "";
IRMP_DATA irmp_data;
bool sJustReceived;

void handleReceivedIRData();
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
SOFTPWM_DEFINE_CHANNEL(4, DDRB, PORTB, PORTB3);  //Arduino pin 11
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
SOFTPWM_DEFINE_OBJECT(5);

/*-----( Function )-----*/
void handleReceivedIRData() {
    irmp_get_data(&irmp_data);
#if defined(ARDUINO_ARCH_MBED) || defined(ESP32)
    sJustReceived = true; // Signal new data for main loop, this is the recommended way for handling a callback :-)
#else
    interrupts(); // enable interrupts
    irmp_result_print(&irmp_data); // this is not allowed in ISR context for any kind of RTOS
#endif
}

String translateIR() {
//  uint32_t rawDat = IrReceiver.decodedIRData.decodedRawData;
  Serial.println(irmp_data.command, HEX);

  switch (irmp_data.command) {
    case 0x45: return "POWER";
    case 0x46: return "VOL+";
    case 0x47: return "FUNC/STOP";
    case 0x44: return "FAST BACK";
    case 0x40: return "PAUSE";
    case 0x43: return "FAST FORWARD";
    case 0x7: return "DOWN";
    case 0x15: return "VOL-";
    case 0x9: return "UP";
    case 0x19: return "EQ";
    case 0xD: return "ST/REPT";
    case 0x16: return "0";
    case 0xC: return "1";
    case 018: return "2";
    case 0x5E: return "3";
    case 0x8: return "4";
    case 0x1C: return "5";
    case 0x5A: return "6";
    case 0x42: return "7";
    case 0x52: return "8";
    case 0x4A: return "9";
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
    //Serial.print(micros());
    //Serial.print(" loop(): ");
    //Serial.println(i);

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
//  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  //SOFTPWM
  //Palatis::SoftPWM.begin(500);
  Palatis::SoftPWM.printInterruptLoad();

  //IRRECEIVER
  pinMode(LED_BUILTIN, OUTPUT);

  irmp_init();
  irmp_irsnd_LEDFeedback(true); // Enable receive signal feedback at LED_BUILTIN
  irmp_register_complete_callback_function(&handleReceivedIRData);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  irmp_print_active_protocols(&Serial);
  Serial.println(F("at pin " STR(IRMP_INPUT_PIN)));
}

void loop() {
  if (sJustReceived) {
      sJustReceived = false;
      irmp_result_print(&irmp_data); // this is not allowed in ISR context for any kind of RTOS
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
  Palatis::SoftPWM.set(4, speeds[1]);
  //  Serial.println(String("left: ") + String(speeds[0]) + String(" right: ") + String(speeds[1]));

}
