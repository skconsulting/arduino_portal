/* detecteur garage ouvert/ferme et ouvre bluetooth
 25/2/2020
 */
 */
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile 
#include <RCSwitch.h>
#include <SoftwareSerial.h>
#include <Bounce2.h>


#define led 7                                    // affectation des broches
#define RX 2
#define TX 3
word octet_recu;
String palabra = "";
uint8_t   ouvrePortail_pin  = PD4;
Bounce oup = Bounce();


RH_ASK driver;
RCSwitch mySwitch = RCSwitch();
SoftwareSerial bluetooth(RX, TX);

//RH_ASK driver(2000, 11, 10);
const char *msg = "ouvreP";
//String ouvreG = "4199746"; // open garage D
char* ouvreGGB = "010000000001010101000010";
//String ouvreD = "4199748"; // open garage G
char* ouvreGDB = "010000000001010101000100";
//String ouvreP = "4199745"; // open portail
char* ouvrePB = "010000000001010101000001";
//String b4 = "4199752"; // button 4
char* b4B     = "010000000001010101001000";

String stringTCo = String("ouvreP"); // portail ouvert
String stringTCf = String("fermeP"); //portail ferme
String stringGAo = String("ouvreG");// garage ouvert
String stringGAf = String("fermeG"); // garage ferme

const uint8_t sendPOC = 30; //send info portail every 30 seconds
unsigned long startPOC;
unsigned long curPOC;


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
void recevoir()
{
  if (bluetooth.available())
  {
    octet_recu = bluetooth.read();
    palabra += (char)octet_recu;
  }
}

void setup()
{
  startPOC = millis();  //initial start time
  oup.attach(ouvrePortail_pin, INPUT_PULLUP);
  oup.interval(10); // debounce interval in ms
  Serial.begin(9600);    // Debugging only
  bluetooth.begin(9600);                    // initialisation connexion série Bluetooth à 9600 bauds

  Serial.println("init start");
  if (!driver.init())
    Serial.println("init failed");

  // protocol for garage command
  // Transmitter is connected to Arduino Pin #12
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
  curPOC = millis();
  if ((curPOC - startPOC > sendPOC * 1000)) {
    startPOC = millis();
    if (!oup.read()) {
      sendmessage(stringGAf);
    }
    if (oup.read()) {
      sendmessage(stringGAo);
    }
  }
  //  sendmessage(stringGAo);
  //  sendmessage(stringTCo);
  //  Serial.println("ouvreG and ouvreP");
  //  delay(1000);
  //
  //  sendmessage(stringGAf);
  //  sendmessage(stringTCf);
  //  Serial.println("fermeG and fermeP");
  //  delay(1000);

  recevoir();      // appel de la procédure recevoir
  //  if (palabra != palabra_old ) {
  //    Serial.println(palabra);
  //    palabra_old = palabra;
  //  }
  if (palabra == "Portail") {
    Serial.println("Portail");
    mySwitch.send(ouvrePB);
    palabra = "";
    delay(1000);
  }
  if (palabra == "GarageG") {
    Serial.println("GarageG");
    mySwitch.send(ouvreGGB);
    palabra = "";
    delay(1000);
  }

  if (palabra == "GarageD") {
    Serial.println("GarageD");
    mySwitch.send(ouvreGDB);
    palabra = "";
    delay(1000);
  }
}
