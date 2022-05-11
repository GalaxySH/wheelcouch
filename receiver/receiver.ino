//master
#include <Arduino.h>
#include "PinDefinitionsAndMore.h"
#include <SoftwareSerial.h>

#define IRMP_PROTOCOL_NAMES              1 // Enable protocol number mapping to protocol strings - needs some FLASH. Must before #include <irmp*>
#define IRMP_USE_COMPLETE_CALLBACK       1 // Enable callback functionality
//#define IRMP_ENABLE_PIN_CHANGE_INTERRUPT   // Enable interrupt functionality

#define IRMP_SUPPORT_NEC_PROTOCOL               1       // NEC + APPLE + ONKYO  >= 10000                 ~300 bytes
#define IRMP_SUPPORT_SAMSUNG_PROTOCOL           1       // Samsung + Samsg32    >= 10000                 ~300 bytes
#define IRMP_SUPPORT_KASEIKYO_PROTOCOL          1       // Kaseikyo             >= 10000                 ~250 bytes
#define IRMP_SUPPORT_RC6_PROTOCOL               1       // RC6 & RC6A           >= 10000                 ~250 bytes
#define IRMP_SUPPORT_DENON_PROTOCOL             1       // DENON, Sharp         >= 10000                 ~250 bytes
//#define IRMP_INPUT_PIN 2

#include <irmp.hpp>

/*-----( VARS )-----*/
SoftwareSerial mySerial(5,6);  //rx pin,tx pin
int fpow = 0;// initial forward power  (vector with values -1, 0, and 1)
int lpow = 0;// initial lateral turning power (vector with values -1, 0, and 1)
int sounding = 0;
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

/*-----( FUNCS )-----*/
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
    case 0x18: return "2";
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
    return;//make functionality test function
  } else if (cmd == "EQ") {
    if (sounding) {
      sounding = 0;
    } else {
      sounding = 1;
    }
  } else {
    return;
  }
}

void allStop() {
  lpow = 0;
  fpow = 0;
}

void sendData() {
  mySerial.print("<");
  //  for(int i = 0; i < sizeof(someArray) / sizeof(someArray[0]; i++)
  //  {
  //     Serial.print(someArray[i]);
  //     Serial.print(",");
  //  }
  mySerial.print(fpow);
  mySerial.print(",");
  mySerial.print(lpow);
  mySerial.print(",");
  mySerial.print(sounding);
  mySerial.print(">");
}

void setup() {
  Serial.begin(9600);   // Status message will be sent to PC at 9600 baud
  mySerial.begin(9600);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
  delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

  // Just to know which program is running on my Arduino
//  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

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
  sendData();
  //  Serial.println(String("left: ") + String(speeds[0]) + String(" right: ") + String(speeds[1]));
}
