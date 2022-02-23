/*
  Example for receiving

  https://github.com/sui77/rc-switch/

  If you want to visualize a telegram copy the raw data and
  paste it into http://test.sui.li/oszi/
*/

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2

  Serial.println("start");
}
String converter(uint8_t *str) {
  return String((char *)str);
}
void loop() {
  if (mySwitch.available()) {
    //        output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());

    Serial.print("value: " );
    Serial.println(mySwitch.getReceivedValue());
    Serial.print("Length: " );
    Serial.println(mySwitch.getReceivedBitlength());
    Serial.print("Delay: " );
    Serial.println(mySwitch.getReceivedDelay());
    //String mys = char(mySwitch.getReceivedRawdata());
    Serial.print("Rawdata0: ");
    Serial.println(mySwitch.getReceivedRawdata()[1]);
    //    Serial.print("Rawdata1: ");
    //    Serial.println(mySwitch.getReceivedRawdata()[1]);
    Serial.print("Protocol: " );
    Serial.println(mySwitch.getReceivedProtocol());

    //, mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    mySwitch.resetAvailable();
  }
}
