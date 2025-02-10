/**********************************************************
  Radiohead_reception_433MHz
  rx 433 test for portail
  31 01 2024
*/
#define RH_ASK_MAX_MESSAGE_LEN 25  //67

#include <RH_ASK.h>  // (fait partie de Radiohead)
//#define RH_ASK_MAX_PAYLOAD_LEN 20  //67
//#include <SPI.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>
//A4 SDA
//A5 SCL

LiquidCrystal_I2C lcd(0x3F, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
RH_ASK driver;
/*
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
uint8_t buflen = sizeof(buf);
*/
const String stringCAo = String("ouvreCo");
const String stringCAf = String("fermeCo");
//D11 radio input
bool b = true;
String myst = "";
String mysf = "";
int j = 0;

String converter(uint8_t *str) {
  return String((char *)str);
}




void setup() {
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);


  Serial.println("init receive rx433 test for portail");
  //Serial.println("max length message:" + String(buflen));


  if (!driver.init()) {
    Serial.println("Echec de l'initialisation de Radiohead");
  }
  lcd.init();       //initialize the lcd
  lcd.backlight();  //open the backlight
  lcd.clear();
  lcd.setCursor(0, 0);  // set the cursor to column 15, line 0
  lcd.print("receive rx433 test for portail");
}

String padStringTo16(String ins) {
  String str = ins;
  if (str.length() < 17) {
    while (str.length() < 17) {
      str = str + " ";
    }
  }
  return str;
}
void loop1() {
  lcd.clear();
  delay(1000);
  lcd.setCursor(0, 0);  // set the cursor to column 0, line 0
  lcd.print("col0 line 0");
  delay(1000);
  lcd.setCursor(0, 1);  // set the cursor to column 0, line 0
  lcd.print("col");

  delay(1000);
}
void loop() {
  int i;
  String res = "";
  String newins = "";

  uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
  uint8_t buflen = sizeof(buf);

  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(200);

  if (driver.recv(buf, &buflen)) {
    Serial.print("Receive message:" + String(j) + " length: " + String(buflen) + ": ");
    for (i = 0; i < buflen; i++) {
      Serial.write(buf[i]);
      res = res + char(buf[i]);
    }
    

    //b = !b;


    if ((stringCAo == res) || (stringCAf == res)) {
      newins = padStringTo16(res);
      lcd.setCursor(0, 0);  // set the cursor to column 0, line 0
      lcd.print(String(j)+": "+newins);
    } else {
      lcd.setCursor(0, 1);  // set the cursor to column 0, line 0
      newins = padStringTo16(res);
      lcd.print(String(j)+": "+newins);
    }
    Serial.println("Message recu: " + res);
    j++;
    /*
    String subt = myst.substring(0, 6);
    //    Serial.println(sub.substring(0, 1));
    
    String subf = mysf.substring(0, 6);
    //    Serial.println(sub.substring(0, 1));
    Serial.println("Message recuf: " + subf);
    */
  }
}
