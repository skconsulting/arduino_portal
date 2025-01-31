// test tx433 as per portail commands with temperature/humidity
//define RH_ASK_MAX_MESSAGE_LEN 100  //67
#include <RH_ASK.h>
//#define RH_ASK_MAX_PAYLOAD_LEN 80  //67
//#include <SPI.h> // Not actually used but needed to compile
RH_ASK driver;  // data on 12
//RH_ASK driver(2000, 11, 10);
const char *msg = "ouvreP";

String stringTCo = String("ouvreP");  // portail ouvert
String stringTCf = String("fermeP");  //portail ferme
String stringGAo = String("ouvreG");  // garage ouvert
String stringGAf = String("fermeG");  // garage ferme

int delaybs = 3000;

void sendmessage(String instr) {
  int n = instr.length();
  char msg[n + 1];
  strcpy(msg, instr.c_str());
  driver.send((uint8_t *)msg, strlen(msg));
  Serial.print("send message 433mhz " + String(strlen(msg)) + ": ");
  Serial.println(msg);
  driver.waitPacketSent();
  //driver.send((uint8_t *)msg, strlen(msg));
  //driver.waitPacketSent();
  //driver.send((uint8_t *)msg, strlen(msg));
  //driver.waitPacketSent();
}
String alignT(float t, String C) {
  String strt;
  strt = String(t, 1);
  if (t >= 10) {
    strt = C + "+" + strt;
  } else if (t >= 0) {
    strt = C + "+0" + strt;
  } else if (t <= -10) {
    strt = C + strt;
  } else {
    strt = C + "-0" + strt.substring(1, 4);
  }
  return strt;
}

void setup() {

  Serial.begin(9600);  // Debugging only
  Serial.println("init send tx433");
  if (!driver.init())
    Serial.println("init failed");
  //sendmessage("in rot:");
  //sendmessage("1234567890123456");
}
void loop() {
  sendmessage("1234567890123456");
  delay(2000);
  sendmessage("abcdefghijklmnop");
  //sendmessage("123456");
  delay(2000);
  sendmessage("123456");
  delay(2000);
  sendmessage("abcdefghijklmnop");
  delay(2000);


  //sendmessage("in rot: 252");
  //delay(1000);
}

void loop1() {
  float humid = 40.21;
  float tempera = -12.36;
  String strtemp = alignT(tempera, "T");
  sendmessage(strtemp);
  sendmessage(stringGAo);
  delay(delaybs);
  strtemp = alignT(humid, "H");
  sendmessage(stringTCo);
  sendmessage(strtemp);
  delay(delaybs);
  Serial.println("ouvreG and ouvreP");

  humid = 10.10;
  tempera = 25.82;
  strtemp = alignT(tempera, "T");
  sendmessage(strtemp);
  sendmessage(stringGAf);
  delay(delaybs);
  strtemp = alignT(humid, "H");
  sendmessage(strtemp);
  sendmessage(stringTCf);
  Serial.println("fermeG and fermeP");
  delay(delaybs);
}
void loop2() {
  //sendmessage(stringGAo);
  // delay(delaybs);
  sendmessage(stringTCo);
  //Serial.println("ouvreG and ouvreP");
  Serial.println("ouvreP");
  delay(delaybs);
  //sendmessage(stringGAf);
  // delay(delaybs);
  sendmessage(stringTCf);
  //Serial.println("fermeG and fermeP");
  Serial.println("fermeP");
  delay(delaybs);
}
