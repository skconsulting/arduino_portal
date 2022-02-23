
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile 
#include <RCSwitch.h>
#include <Bounce2.h>

uint8_t   ouvrePortail_pin  = PD2;
Bounce oup = Bounce();


RH_ASK driver;
RCSwitch mySwitch = RCSwitch();

//RH_ASK driver(2000, 11, 10);
const char *msg = "ouvreP";
String ouvreG = "4199746"; // open gaarage
char* ouvreGB = "010000000001010101000010";
String b4 = "4199752"; // button 4
char* b4B     = "010000000001010101001000";

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
  oup.attach(ouvrePortail_pin, INPUT_PULLUP);
  oup.interval(10); // debounce interval in ms
  Serial.begin(9600);    // Debugging only
  Serial.println("init start");
  if (!driver.init())
    Serial.println("init failed");
  mySwitch.enableTransmit(12);

  // Optional set pulse length.
  mySwitch.setPulseLength(320);

  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Optional set number of transmission repetitions.
  mySwitch.setRepeatTransmit(5);
  Serial.println("start tx with garage");
}
void loop()
{
  oup.update();
  if (oup.fell()) {
    Serial.println("press button");
    mySwitch.send(b4B);
  }
  sendmessage(stringGAo);
  sendmessage(stringTCo);
  Serial.println("ouvreG and ouvreP");
  delay(1000);

  sendmessage(stringGAf);
  sendmessage(stringTCf);
  Serial.println("fermeG and fermeP");
  delay(1000);

}
