/**********************************************************
  Radiohead_reception_433MHz
  rx 433 with led and temperature/humidity
*/

#include <RH_ASK.h> // (fait partie de Radiohead)
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

RH_ASK driver;
//A4 SDA
//A5 SCL
#define POF   PD2    //  Portail fermé
#define DEFPO PD3     //  defaillance portail
#define POO   PD4     //  Portail ouvert
#define GAF   PD5     //  garage fermé
#define DEFGA PD6     //   defaillance garage
#define GAO   PD7     //  garage ouvert

//D11 radio input

unsigned long startMillisP;  //timer portail
unsigned long startMillisG;  //timer garage
unsigned long startMillisS; // timer sensor
unsigned long currentMillis;
const unsigned long period = 60000;  //timeout in milliseconds 60000                                                                                         0secondes

String converter(uint8_t *str) {
  return String((char *)str);
}
const String stringTCo = String("ouvreP"); // portail ouvert
const String stringTCf = String("fermeP"); //portail ferme
const String stringGAo = String("ouvreG");// garage ouvert
const String stringGAf = String("fermeG"); // garage ferme


uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
uint8_t buflen = sizeof(buf);

boolean lcdclear = true;
void setup()
{
  Serial.begin(115200);

  pinMode(POO, OUTPUT); // Portail ouvert
  pinMode(POF, OUTPUT);// portail fermé
  pinMode(GAO, OUTPUT); //garage ouvert
  pinMode(GAF, OUTPUT); // garage fermé
  pinMode(DEFPO, OUTPUT); // defaillance portail
  pinMode(DEFGA, OUTPUT); // defaillance garage


  Serial.println("init receive rx433");

  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
  lcd.clear();
  lcd.setCursor(6, 1); // set the cursor to column 15, line 0
  lcd.print("Start");

  digitalWrite(GAO, HIGH);
  delay(500);
  digitalWrite(GAO, LOW);
  delay(500);
  digitalWrite(DEFGA, HIGH);
  delay(500);
  digitalWrite(DEFGA, LOW);
  delay(500);
  digitalWrite(GAF, HIGH);
  delay(500);
  digitalWrite(GAF, LOW);
  delay(500);

  digitalWrite(POO, HIGH);
  delay(500);
  digitalWrite(POO, LOW);
  delay(500);

  digitalWrite(DEFPO, HIGH);
  delay(500);
  digitalWrite(DEFPO, LOW);
  delay(500);

  digitalWrite(POF, HIGH);
  delay(500);

  digitalWrite(POO, LOW);
  digitalWrite(POF, LOW);
  digitalWrite(GAO, LOW);
  digitalWrite(GAF, LOW);
  digitalWrite(DEFPO, HIGH);
  digitalWrite(DEFGA, HIGH);

  if (!driver.init()) {
    Serial.println("Echec de l'initialisation de Radiohead");
    digitalWrite(DEFPO, HIGH);
    digitalWrite(DEFGA, HIGH);
    lcd.print("NO SIGNAL");
  }

  startMillisP = millis();  //initial start time
  startMillisG = millis();  //initial start time
  startMillisS = millis(); //init sensor

}

void loop()
{
  int i ;
    digitalWrite(GAO, HIGH);
    delay(500);
    digitalWrite(GAO, LOW);
    delay(500);
    digitalWrite(DEFGA, HIGH);
    delay(500);
    digitalWrite(DEFGA, LOW);
    delay(500);
  
    digitalWrite(GAF, HIGH);
    delay(500);
    digitalWrite(GAF, LOW);
    delay(500);
  
    digitalWrite(POO, HIGH);
    delay(500);
    digitalWrite(POO, LOW);
    delay(500);
  
    digitalWrite(DEFPO, HIGH);
    delay(500);
    digitalWrite(DEFPO, LOW);
    delay(500);
  
    digitalWrite(POF, HIGH);
    delay(500);
    digitalWrite(POF, LOW);
    delay(500);

  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillisP >= period)  //test whether the period has elapsed for portail
  {
    digitalWrite(DEFPO, HIGH);
    digitalWrite(POO, LOW);
    digitalWrite(POF, LOW);
    startMillisP = millis();
  }
  if (currentMillis - startMillisG >= period)  //test whether the period has elapsed for Garage
  {
    digitalWrite(DEFGA, HIGH);
    digitalWrite(GAO, LOW);
    digitalWrite(GAF, LOW);
    startMillisG = millis();
  }
  if (currentMillis - startMillisS >= period)  //test whether the period has elapsed for sensor
  {
    if (lcdclear) {
      lcd.clear();
      lcdclear = false;
      lcd.setCursor(0, 0); // set the cursor to column 0, line 0
      lcd.print("NO SIGNAL");
    }
    startMillisS = millis();
  }

  if (driver.recv(buf, &buflen))
  {
    Serial.println("Receive message");
    for (i = 0; i < buflen; i++)
    {
      Serial.write(buf[i]);
    }
    Serial.println("");
    String mys = converter(buf);
    String sub = mys.substring(0, 6);
    //    Serial.println(sub.substring(0, 1));
    Serial.println("Message recu: " + sub);


    if (sub.substring(0, 1) == "T") {
      Serial.print("temperature: ");
      Serial.println(sub.substring(1, 6) + " C");

      lcd.setCursor(0, 0); // set the cursor to column 0, line 0
      sub.replace("+0", "+ ");
      sub.replace("-0", "- ");

      lcd.print("Temp: " + sub.substring(1, 6) + "C");
      lcdclear = true;
      startMillisS = millis();
    }
    else if (sub.substring(0, 1) == "H") {
      Serial.print("humidity: ");
      Serial.println(sub.substring(1, 6) + " %");
      lcd.setCursor(0, 1); // set the cursor to column 15, line 0
      sub.replace("+0", "+ ");
      lcd.print("Humi: " + sub.substring(1, 6) + "%");
      lcdclear = true;
      startMillisS = millis();
    }
    else   if (sub == stringTCo) {
      Serial.println("this is Po"); // portail ouvert
      digitalWrite(POO, HIGH);
      digitalWrite(POF, LOW);
      digitalWrite(DEFPO, LOW);
      startMillisP = millis();
    }
    else if (sub == stringTCf) {
      Serial.println("this is Pf"); // portail fermé
      digitalWrite(POO, LOW);
      digitalWrite(POF, HIGH);
      digitalWrite(DEFPO, LOW);
      startMillisP = millis();
    }
    else if (sub == stringGAo) { // garage ouvert
      Serial.println("this is Go");
      digitalWrite(GAO, HIGH);
      digitalWrite(GAF, LOW);
      digitalWrite(DEFGA, LOW);
      startMillisG = millis();
    }
    else if (sub == stringGAf) {
      Serial.println("1 this is Gf"); // garage ferme
      digitalWrite(GAO, LOW);
      digitalWrite(GAF, HIGH);
      digitalWrite(DEFGA, LOW);
      startMillisG = millis();
    }
    else {
      Serial.println("this is unk");
      digitalWrite(POO, LOW);
      digitalWrite(POF, LOW);
      digitalWrite(GAO, LOW);
      digitalWrite(GAF, LOW);
      digitalWrite(DEFPO, HIGH);
      digitalWrite(DEFGA, HIGH);
    }
  }
}
