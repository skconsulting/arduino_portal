/**********************************************************
  my protocol transmit
  rx433 with my i2c protocol 
*/

#include <RH_ASK.h> // (fait partie de Radiohead)
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

boolean debug = true;


const String Pgarage =  "Wcd5KPhDxCObUdw6";
const String Pportail = "P5IbZrIejfJPLjH5";
const String Ptemp =    "juuWGLJh4kgFfPOm";
const String Phum =     "xaQJN6hjY4XS8XGB";

const String stringTCo = String("ouvreP"); // portail ouvert
const String stringTCf = String("fermeP"); //portail ferme
const String stringGAo = String("ouvreG");// garage ouvert
const String stringGAf = String("fermeG"); // garage ferme

const String startOfHeading =    "00000001";
const String endOfTransmission = "00000100";

unsigned long startMillisP;  //timer portail
unsigned long startMillisG;  //timer garage
unsigned long startMillisS; // timer sensor
unsigned long currentMillis;
const unsigned long period = 60000;  //timeout in milliseconds 60000                                                                                         0secondes
String mysold = "old";
boolean lcdclear = true;

const uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
const uint8_t buflen = sizeof(buf);

//const int sizeStack = 5; // size of stack
//String stack[sizeStack] ;
//int pointStack = -1;

#define POF   PD2    //  Portail fermé
#define DEFPO PD3     //  defaillance portail
#define POO   PD4     //  Portail ouvert
#define GAF   PD5     //  garage fermé
#define DEFGA PD6     //   defaillance garage
#define GAO   PD7     //  garage ouvert

//A4 SDA
//A5 SCL
//const int  clk = 8;    // clock
//const int data = 9;       // data

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
RH_ASK driver;

//SSVQueueStackArray <String> storage (QUEUE_Storage, //QUEUE_Storage (FIFO) or STACK_Storage (LIFO)
//                                     PTFSA_Overwrite,  //PTFSA_Resize, PTFSA_Overwrite, PTFSA_Ignore
//                                     5 );           //init size;

String converter(uint8_t *str) {
  return String((char *)str);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("rx4323_myprot");
  Wire.begin();

  //  pinMode(clk, OUTPUT);
  //  pinMode(data, OUTPUT);
  //  digitalWrite(clk, HIGH);
  //  digitalWrite(data, HIGH);

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

  if (!driver.init()) {
    Serial.println(F("Echec de l'initialisation de Radiohead"));
    lcd.print("NO SIGNAL");
  }
  httpRequest(Pportail, "2");
  httpRequest(Pgarage, "2");
  httpRequest(Phum, "0");
  httpRequest(Ptemp, "0");
  startMillisP = millis();  //initial start time
  startMillisG = millis();  //initial start time
  startMillisS = millis(); //init sensor
}

void commande (String sub, boolean ht) {
  if (sub[0] == 'T') {
    if (debug) {
      Serial.print("temperature: ");
      Serial.println(sub.substring(1, 6) + " C");
    }
    if (ht) {
      httpRequest(Ptemp, sub.substring(1, 6));
    }
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
      httpRequest(Phum, sub.substring(2, 6));
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
      httpRequest(Pportail, "1");
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
      httpRequest(Pportail, "0");
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
      httpRequest(Pgarage, "1");
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
      httpRequest(Pgarage, "0");
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
      httpRequest(Pgarage, "2");
      httpRequest(Pportail, "2");
    }
  }
}

void httpRequest(String Pid , String command)
{
  if (debug) {
    Serial.println("Pid:" + String(Pid));
    Serial.println("command:" + String(command));
  }
  String tempo = Pid + "_" + command;
  char buffer[32];
  tempo.toCharArray(buffer, 32);
  Wire.beginTransmission(0x25);
  Wire.write(buffer);
  Wire.endTransmission();
}

void loop1() {
  //  sendbeacon(startOfHeading, false);
  //  String tempo = "z";
  //  for (int i = 0; i < tempo.length(); i++)
  //  { seriew(tempo[i], false);
  //  }
  //  sendbeacon(endOfTransmission, false);
  httpRequest(Pgarage, "1");
  delay(5000);
}

void loop() {
  //  int is = 0;
  //  while (pointStack >=0) {
  //      is += 1;
  //      Serial.println (pullString());
  //      Serial.println(is);
  //   }
  //  is = 0;
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillisP >= period)  //test whether the period has elapsed for portail
  {
    digitalWrite(DEFPO, HIGH);
    digitalWrite(POO, LOW);
    digitalWrite(POF, LOW);
    httpRequest(Pportail, "2");
    startMillisP = millis();
  }
  if (currentMillis - startMillisG >= period)  //test whether the period has elapsed for Garage
  {
    digitalWrite(DEFGA, HIGH);
    digitalWrite(GAO, LOW);
    digitalWrite(GAF, LOW);
    httpRequest(Pgarage, "2");
    startMillisG = millis();
  }
  if (currentMillis - startMillisS >= period)  //test whether the period has elapsed for sensor
  {
    if (lcdclear) {
      lcd.clear();
      lcdclear = false;
      lcd.setCursor(0, 0); // set the cursor to column 0, line 0
      lcd.print("NO SIGNAL");
      httpRequest(Ptemp, "0");
      httpRequest(Phum, "0");
      startMillisS = millis();
    }
  }

  if (driver.recv(buf, &buflen))
  {
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
