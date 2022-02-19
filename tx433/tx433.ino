
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile 
RH_ASK driver;
//RH_ASK driver(2000, 11, 10);
const char *msg = "ouvreP";

String stringTCo = String("ouvreP"); // portail ouvert
String stringTCf = String("fermeP"); //portail ferme
String stringGAo = String("ouvreG");// garage ouvert
String stringGAf = String("fermeG"); // garage ferme


void sendmessage(String instr) {
  int n = instr.length();
  char msg[n + 1];
  strcpy(msg, instr.c_str());
  driver.send((uint8_t *)msg, strlen(msg));
  Serial.print("send message 433mhz: ");
  Serial.println(msg);
  driver.waitPacketSent();
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();
  driver.send((uint8_t *)msg, strlen(msg));
  driver.waitPacketSent();

}

void setup()
{

  Serial.begin(9600);    // Debugging only
  Serial.println("init send");
  if (!driver.init())
    Serial.println("init failed");
}

void loop()
{
  sendmessage(stringGAo);
  sendmessage(stringTCo);
  Serial.println("ouvreG and ouvreP");
  delay(1000);

  sendmessage(stringGAf);
  sendmessage(stringTCf);
  Serial.println("fermeG and fermeP");
  delay(1000);

}
