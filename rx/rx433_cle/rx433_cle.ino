/*
to analyse keys sent by 433 command
*/

#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();

void setup() {
  Serial.begin(115200);
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
delay(1);
    //, mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    mySwitch.resetAvailable();
  }
}
