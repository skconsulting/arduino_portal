/**********************************************************
  receive my protocol
*/
//#include <Bounce2.h>
//#include <Wire.h>
#include "WiFiEsp.h"
#include "SoftwareSerial.h"
#include <RH_ASK.h> // (fait partie de Radiohead)

#include <LiquidCrystal_I2C.h>

boolean debug = false;
boolean wifil = true;


const String Pgarage =  "Wcd5KPhDxCObUdw6";
const String Pportail = "P5IbZrIejfJPLjH5";
const String Ptemp =    "juuWGLJh4kgFfPOm";
const String Phum =     "xaQJN6hjY4XS8XGB";

const String stringTCo = String("ouvreP"); // portail ouvert
const String stringTCf = String("fermeP"); //portail ferme
const String stringGAo = String("ouvreG");// garage ouvert
const String stringGAf = String("fermeG"); // garage ferme


const int sizeStack = 6; // size of stack
String stack[sizeStack] ;
int pointStack = -1;

const char ssid[] = "baleinou";            // your network SSID (name)
const char pass[] = "tagada1956";        // your network password
const String content = "EIOT-AuthToken : 2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj";   //

const unsigned long postingInterval = 2000; // delay between updates, in milliseconds
unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
unsigned long diffReceive = 1000;         // last time you connected to the server, in milliseconds
unsigned long endReceive = 0;         // last time you connected to the server, in milliseconds
unsigned long startMillisP;  //timer portail
unsigned long startMillisG;  //timer garage
unsigned long startMillisS; // timer sensor
unsigned long currentMillis;
const unsigned long period = 60000;  //timeout in milliseconds 60000                                                                                         0secondes

String mysold = "old";
boolean lcdclear = true;

const uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
const uint8_t buflen = sizeof(buf);
int status = WL_IDLE_STATUS;     // the Wifi radio's status
#define POF   PD2    //  Portail fermé
#define DEFPO PD3     //  defaillance portail
#define POO   PD4     //  Portail ouvert
#define GAF   PD5     //  garage fermé
#define DEFGA PD6     //   defaillance garage
#define GAO   PD7     //  garage ouvert

SoftwareSerial esp8266(9, 8);
//Bounce trig = Bounce();
WiFiEspClient client;
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
RH_ASK driver;

void pushString(String ins)
{
  if (pointStack < sizeStack - 1) {
    pointStack += 1;
    stack[pointStack] = ins;
  }
  else {
    for (int i = 0 ; i < pointStack; i ++) {
      stack[i] = stack[i + 1];
    }
    stack[pointStack] = ins;
  }
}

String pullString()
{ String res = "";
  if (pointStack >= 0) {
    res = stack[0];
    for (int i = 0 ; i < pointStack; i ++) {
      //Serial.println(i);
      stack[i] = stack[i + 1];
    }
    pointStack -= 1;
  }
  else {
    res = "Empty";
  }
  return res;
}

void dumpString()
{
  Serial.println("start dump");
  for (int i = 0 ; i <= pointStack; i++) {
    //Serial.println(i);
    Serial.println(stack[i]);
  }
  Serial.println("end of pile");
}
boolean lookForString(String ins)
{
  boolean res = false;
  for (int i = 0 ; i <= pointStack; i++) {
    //Serial.println(i);
    if (ins == stack[i]) {
      res = true;
      break;
    }
  }
  return res;
}
void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void setup()
{
  Serial.begin(115200);
  Serial.println("rx433_ccomplet");
  if (wifil) {
    esp8266.begin(9600);
    WiFi.init(&esp8266);
    // check for the presence of the shield
    if (WiFi.status() == WL_NO_SHIELD) {
      //Serial.println("WiFi shield not present");
      // don't continue
      while (true);
    }
    // attempt to connect to WiFi network
    while ( status != WL_CONNECTED) {
      Serial.println("Attempting to connect to WPA SSID ");
      // Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);
    }
    Serial.println("You're connected to the network");
    printWifiStatus();
  }
  pinMode(POO, OUTPUT); // Portail ouvert
  pinMode(POF, OUTPUT);// portail fermé
  pinMode(GAO, OUTPUT); //garage ouvert
  pinMode(GAF, OUTPUT); // garage fermé
  pinMode(DEFPO, OUTPUT); // defaillance portail
  pinMode(DEFGA, OUTPUT); // defaillance garage
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
  lcd.clear();
  lcd.setCursor(6, 1); // set the cursor to column 15, line 0
  lcd.print("Start");
  digitalWrite(GAO, HIGH);
  delay(200);
  digitalWrite(GAO, LOW);
  delay(200);
  digitalWrite(DEFGA, HIGH);
  delay(200);
  digitalWrite(DEFGA, LOW);
  delay(200);
  digitalWrite(GAF, HIGH);
  delay(200);
  digitalWrite(GAF, LOW);
  delay(200);
  digitalWrite(POO, HIGH);
  delay(200);
  digitalWrite(POO, LOW);
  delay(200);
  digitalWrite(DEFPO, HIGH);
  delay(200);
  digitalWrite(DEFPO, LOW);
  delay(200);
  digitalWrite(POF, HIGH);
  delay(200);
  digitalWrite(POF, LOW);
  delay(200);
  digitalWrite(DEFPO, HIGH);
  digitalWrite(DEFGA, HIGH);
  //trig.attach(clk, INPUT_PULLUP);
  //trig.interval(1); // debounce interval in ms
  if (!driver.init()) {
    Serial.println(F("Echec de l'initialisation de Radiohead"));
    lcd.print("NO SIGNAL");
  }
  httpRequestFormat(Pportail, "2");
  httpRequestFormat(Pgarage, "2");
  httpRequestFormat(Phum, "0");
  httpRequestFormat(Ptemp, "0");
  startMillisP = millis();  //initial start time
  startMillisG = millis();  //initial start time
  startMillisS = millis(); //init sensor
}

void sendmyString(String inp) {
  String pid = inp.substring(0, inp.indexOf('_'));
  String command = inp.substring(inp.indexOf('_') + 1, inp.length());
  httpRequest(pid , command.toFloat());
}


void httpRequest(String Pid , float command)
{
  const char server[] = "cloud.iot-playground.com";
  const String content = "EIOT-AuthToken : 2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj";   //
  if (debug) {
    Serial.println(Pid);
    Serial.println(command);
  }
  if (wifil) {
    //  close any connection before send a new request
    // this will free the socket on the WiFi shield
    client.stop();
    // if there's a successful connection
    if (client.connect(server, 40404)) {
      //Serial.println(F("Connecting..."));

      // send the HTTP PUT request
      client.println("POST /RestApi/v1.0/Parameter/" + Pid + "/Value/" + String(command) + " HTTP/1.1");
      client.println("Host: http://cloud.iot-playground.com:40404");
      client.println("Accept: application/json; indent=4");
      client.println("Content-Length: " + String(content.length()));
      client.println(F("Content-Type: application/json"));
      client.println(content);
      client.println();

      lastConnectionTime = millis();
      //client.stop();
    }
    else {
      // if you couldn't make a connection
      Serial.println(F("Connection failed"));
    }
  }
}
void httpRequestFormat(String Pid , String command)
{
  if (debug) {
    Serial.println("Pid:" + String(Pid));
    Serial.println("command:" + String(command));
  }
  String tempo = Pid + "_" + command;
  if (!lookForString(tempo)) {
    pushString(tempo);
  }
}

void commande (String sub, boolean ht) {
  if (sub[0] == 'T') {
    if (debug) {
      Serial.print("temperature: ");
      Serial.println(sub.substring(1, 6) + " C");
    }
    //    if (ht) {
    //      httpRequest(Ptemp, sub.substring(1, 6));
    //    }
    sub.replace("+0", "+ ");
    sub.replace("-0", "- ");
    lcd.setCursor(0, 0); // set the cursor to column 0, line 0
    lcd.print("Temp: " + sub.substring(1, 6) + "C");
    lcdclear = true;
    startMillisS = millis();
  }
  else if (sub[0] == 'H') {
    if (debug) {
      Serial.print("humidity: ");
      Serial.println(sub.substring(1, 6) + " %");
    }
    lcd.setCursor(0, 1); // set the cursor to column 15, line 0
    sub.replace("+0", "+ ");
    lcd.print("Humi: " + sub.substring(1, 6) + "%");
    lcdclear = true;
    if (ht) {
      httpRequestFormat(Phum, sub.substring(2, 6));
    }
    startMillisS = millis();
  }
  else  if (sub == stringTCo) { // portail ouvert
    if (debug) {
      Serial.println("this is Po");
    } // portail ouvert
    digitalWrite(POO, HIGH);
    digitalWrite(POF, LOW);
    digitalWrite(DEFPO, LOW);
    if (ht) {
      httpRequestFormat(Pportail, "1");
    }
    startMillisP = millis();
  }
  else if (sub == stringTCf) {// portail fermé
    if (debug) {
      Serial.println("this is Pf");
    } // portail fermé
    digitalWrite(POO, LOW);
    digitalWrite(POF, HIGH);
    digitalWrite(DEFPO, LOW);
    if (ht) {
      httpRequestFormat(Pportail, "0");
    }
    startMillisP = millis();
  }
  else if (sub == stringGAo) { // garage ouvert
    if (debug) {
      Serial.println("this is Go");
    }  // garage ouvert
    digitalWrite(GAO, HIGH);
    digitalWrite(GAF, LOW);
    digitalWrite(DEFGA, LOW);
    if (ht) {
      httpRequestFormat(Pgarage, "1");
    }
    startMillisG = millis();
  }
  else if (sub == stringGAf) { // garage ferme
    if (debug) {
      Serial.println("1 this is Gf"); // garage ferme
    }
    digitalWrite(GAO, LOW);
    digitalWrite(GAF, HIGH);
    digitalWrite(DEFGA, LOW);
    if (ht) {
      httpRequestFormat(Pgarage, "0");
    }
    startMillisG = millis();
  }
  else {
    if (debug) {
      Serial.println("this is unk");
    }
    digitalWrite(POO, LOW);
    digitalWrite(POF, LOW);
    digitalWrite(GAO, LOW);
    digitalWrite(GAF, LOW);
    digitalWrite(DEFPO, HIGH);
    digitalWrite(DEFGA, HIGH);
    if (ht) {
      httpRequestFormat(Pgarage, "2");
      httpRequestFormat(Pportail, "2");
    }
  }
}
String converter(uint8_t *str) {
  return String((char *)str);
}

void loop() {
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)

  if (((currentMillis - lastConnectionTime ) > postingInterval) & ((currentMillis - endReceive)  > diffReceive)) {
    if (pointStack >= 0) {
      if (debug) {
        dumpString();
      }
      sendmyString(pullString());

      //      if (debug) {
      //      Serial.print ("pointStack: ");
      //      Serial.println (pointStack);
      //      }
    }
  }

  if (currentMillis - startMillisP >= period)  //test whether the period has elapsed for portail
  {
    digitalWrite(DEFPO, HIGH);
    digitalWrite(POO, LOW);
    digitalWrite(POF, LOW);
    httpRequestFormat(Pportail, "2");
    startMillisP = millis();
  }
  if (currentMillis - startMillisG >= period)  //test whether the period has elapsed for Garage
  {
    digitalWrite(DEFGA, HIGH);
    digitalWrite(GAO, LOW);
    digitalWrite(GAF, LOW);
    httpRequestFormat(Pgarage, "2");
    startMillisG = millis();
  }
  if (currentMillis - startMillisS >= period)  //test whether the period has elapsed for sensor
  {
    if (lcdclear) {
      lcd.clear();
      lcdclear = false;
      lcd.setCursor(0, 0); // set the cursor to column 0, line 0
      lcd.print("NO SIGNAL");
      httpRequestFormat(Ptemp, "0");
      httpRequestFormat(Phum, "0");
      startMillisS = millis();
    }
  }

  if (driver.recv(buf, &buflen))
  {
    endReceive = millis();
    //    Serial.println("Receive message");
    //for (i = 0; i < buflen; i++)
    //    {
    //      Serial.write(buf[i]);
    //    }
    //    Serial.println("");
    String mys = converter(buf);
    String sub = mys.substring(0, 6);
    //    Serial.print("Receive message");
    //    Serial.println(sub);
    //    Serial.print("old Receive message: ");
    //    Serial.println(mysold);
    //    Serial.print("diff:");
    //    Serial.println(sub != mysold);
    //    Serial.println("sub :" + sub);

    if (sub != mysold) {
      if (debug) {
        Serial.println("New Message recu: " + sub);
        Serial.println("------");
      }
      mysold = sub;
      commande(sub, true);

    }
    //    else {
    //      //           Serial.println("Old Message  ");
    //      //           Serial.println("------");
    //    }
  }
}
