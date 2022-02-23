/*
  Based on the SendDemo example from the RC Switch library
  https://github.com/sui77/rc-switch/
*/

#include <RCSwitch.h>
#include <Bounce2.h>
RCSwitch mySwitch = RCSwitch();

uint8_t   ouvrePortail_pin  = PD2;
Bounce oup = Bounce();

String ouvreG = "4199746"; // open gaarage
char* ouvreGB = "010000000001010101000010";
String b4 = "4199752"; // button 4
char* b4B     = "010000000001010101001000";


void setup() {
  oup.attach(ouvrePortail_pin, INPUT_PULLUP);
  oup.interval(10); // debounce interval in ms
  Serial.begin(9600);

  // Transmitter is connected to Arduino Pin #12
  mySwitch.enableTransmit(12);

  // Optional set pulse length.
  mySwitch.setPulseLength(320);

  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(5);
  Serial.println("start tx");
}

void loop() {
  oup.update();
  if (oup.fell()) {
    Serial.println("press button");
    mySwitch.send(b4B);
    delay(1000);
  }
}
