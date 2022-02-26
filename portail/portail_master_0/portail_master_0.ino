// 26 02 2022  LAST UPDATE
#include <RH_ASK.h>
#include "DHT.h"
#include <SPI.h> // Not actually used but needed to compile 
#include <Bounce2.h>
RH_ASK driver;

// pins
uint8_t   countrot_pin  = PD2; // counter of rotation pull up
uint8_t   trig_pin      = PD3; // change rotation , trigger action pull up
uint8_t   EMA       = PD5; // pwm motor
uint8_t   IN1       = PD6; // command motor 1
uint8_t   IN2       = PD7; // command motor 2
uint8_t   POC_pin       = 8; // detection portail ouvert/ferme on pin D8 PB0
#define DHTPIN 4     // what pin we're connected to thermal sensor D10
//D12 TX433mhz data

//A4 SDA
//A5 SCL
uint8_t   refsensor    = A6; // overloadsensor reference
uint8_t   sensor    = A7; // overloadsensor

#define DHTTYPE DHT22   // DHT 22  (AM2302)

Bounce countrot = Bounce();
Bounce trig = Bounce();
Bounce POC = Bounce();


// include the library code
//#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// var config
// for right portail , seen from inside court
const uint8_t rotStop = 20; // stop rotation after 20 seconds
const uint8_t sendPOC = 30; //send info portail every 30 seconds
const int opendelay = 1000; // 1 second afte command to open
const int closedelay = 0; // 0 second after command to close
const int delayCurrentDrive = 500; // delay before read motor current in ms
const uint8_t minmotor = 150; // value when approaching end of run

// var util
uint8_t stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
uint8_t calibre ;
int maxrotationopen, maxrotationclose;
//int halfopen, halfclose;
int minmotorpos, maxmotorpos;
bool inrotation = false;
double VoltageRef = 3.3; // ref Voltage from potentiometre
int countrotation = 0;

double meas, meas0, meas1, meas2, measold, measref ;

double Voltage = 0;

int mVperAmp;
int ACSoffset;

const char *msg = "ouvreP";
unsigned long startPOC;
unsigned long curPOC;
unsigned long startROT;
unsigned long curROT;

String stringCAo = String("ouvreC");
String stringCAf = String("fermeC");
String stringPCo = String("ouvreP");
String stringPCf = String("fermeP");

DHT dht(DHTPIN, DHTTYPE);

void pressedcountrot (void) // each time there is a counter increase
{
  if (stateportail == 3) {//ferme
    countrotation++;
    if (countrotation == maxrotationclose) {
      Serial.print("min rotation reached! completement ferme: ");
      Serial.println(countrotation);
      stateportail = 0;
      actionportail();
    }
    if (countrotation == minmotorpos) {
      Serial.println("min motor reached! ");
      //      Serial.println( minmotorpos, DEC);
      //      Serial.print("countrotation : ");
      //      Serial.println(countrotation);
      analogWrite(EMA, minmotor);
    }
  }
  if (stateportail == 1) {//ouvre
    countrotation++;
    if (countrotation == maxrotationopen) {
      Serial.print("maxrotation reached! completement ouvert!: ");
      Serial.println(countrotation);
      stateportail = 2;
      actionportail();
    }
    if (countrotation == maxmotorpos) {
      //      Serial.print("max motor reached!: ");
      //      Serial.println(maxmotorpos);
      //      Serial.print("countrotation : ");
      //      Serial.println(countrotation);
      analogWrite(EMA, minmotor);
    }
  }
  //  Serial.print("rotation count");
  //  Serial.println(countrotation);
  //lcd.setCursor(6, 1); // set the cursor to column 15, line 0
  //lcd.print("o");
  //lcd.setCursor(7, 1);
  //lcd.print(maxrotationopen);
  //lcd.setCursor(11, 1); // set the cursor to column 15, line 0
  //lcd.print("f");
  //lcd.setCursor(12, 1);
  //lcd.print(maxrotationclose);
  //lcd.setCursor(0, 1); // set the cursor to column 15, line 0
  //lcd.print("   ");
  //lcd.setCursor(0, 1); // set the cursor to column 15, line 0
  //lcd.print(countrotation);
}

void ouvreportail()
{
  inrotation = true;
  startROT = millis();
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  delay(10);
  analogWrite(EMA, 255);
}
void fermeportail()
{
  inrotation = true;
  startROT = millis();
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  delay(10);
  analogWrite(EMA, 255);
}
void reposportail() {
  inrotation = false;
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
}

void actionportail() {// 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  //lcd.setCursor(0, 0); // set the cursor to column 15, line 0
  switch (stateportail) {
    case 0: {
        Serial.println("repos ferme!");
        //        lcd.print("repferme");
        countrotation = 0;
        reposportail();
        break;
      }
    case 1: {
        Serial.println("ouvre!");
        //lcd.print("ouvre!  ");
        delay(opendelay);
        ouvreportail();
        break;
      }
    case 2: {
        Serial.println("repos ouvert!");
        //lcd.print("rep ouve");
        reposportail();
        countrotation = 0;
        break;
      }
    case 3: {
        Serial.println("ferme!");
        //        lcd.print("ferme!  ");
        delay(closedelay);
        fermeportail();
        break;
      }
  }
}
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

String alignT(float t, String C) {
  String strt;
  strt = String(t, 1);
  if (t >= 10) {
    strt = C + "+" + strt;
  } else if (t >= 0) {
    strt = C + "+0" + strt;
  }
  else if (t <= -10) {
    strt = C + strt;
  }
  else {
    strt = C + "-0" + strt.substring(1, 4);
  }
  return strt;
}

void triggeraction (void) { // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  // run each time action button is pressed
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("old stateportail: ");
  Serial.println( stateportail);
  switch (stateportail) {
    case 0: {
        stateportail = 1;

        sendmessage(stringCAo);
        break;
      }
    case 1: {
        stateportail = 3;
        sendmessage(stringCAf);
        break;
      }
    case 2 : {
        stateportail = 3;
        sendmessage(stringCAf);
        break;
      }
    case 3: {
        stateportail = 1;
        sendmessage(stringCAo);
        break;
      }
  }
  Serial.print("new stateportail: ");
  Serial.println( stateportail);
  actionportail();
}

void anaread ()
//stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
{
  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000; // Gets you mV
  meas0 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(5); // 5 ms
  meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(5); // 5 ms
  meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  // Serial.println(meas );
  if ((meas - measold > 5) || (measold - meas > 5)) {
    //delay(500);
    //Serial.print("measure = : ");
    //Serial.println(meas * (5.0 / 1023.0));
    Voltage = (meas / 1024.0) * 5000; // Gets you mV
    //    Serial.print("mV = "); // shows the voltage measured
    //    Serial.println(Voltage );
    //    Serial.print("Ref mV = "); // shows the voltage measured
    //    Serial.println(VoltageRef );


    measold = meas;

    //    if (Voltage < VoltageRef) {
    //      delay(5); // 100 ms
    //      meas0 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
    //      delay(5); // 10 ms
    //      meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
    //      delay(5); // 10 ms
    //      meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
    //      meas = (meas0 + meas1 + meas2) / 3;
    //      Voltage = (meas / 1024.0) * 5000; // Gets you mV

    if (Voltage < VoltageRef) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("overdrive : ");
      Serial.println(Voltage);
      Serial.print("over VoltageRef: ");
      Serial.println(VoltageRef);
      Serial.print("countrotation : ");
      Serial.println(countrotation);
      Serial.print("calibration maxrotation: ");
      Serial.println(maxrotationopen);

      if (calibre == 2) {
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        //minmotorpos = maxrotationclose - 1;
        stateportail = 0;
        actionportail();
        calibre = 0;
        //          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //          lcd.print(" cal fer");
        Serial.print("calibration maxrotationoclose: ");
        Serial.println(maxrotationclose);
      }
      if (calibre == 1) {
        maxrotationopen = countrotation;
        //minmotorpos = int(0.1 * maxrot
        //          halfopen = int(0.5 * maxrotationopen);
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
        //          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //          lcd.print(" cal ope");
        Serial.print("calibration maxrotationopen: ");
        Serial.println(maxrotationopen);
      }
      if (stateportail == 1) {
        //          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //          lcd.print(" 1/2oove");
        Serial.println("overdrive with a moitie ouvert!");
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        stateportail = 2;
        countrotation = maxrotationclose - countrotation;
        reposportail();
      }
      if (stateportail == 3) {
        //          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //          lcd.print(" 1/2fove");
        Serial.println("overdrive with a moitie ferme!");
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        stateportail = 0;
        countrotation = maxrotationclose - countrotation;
        reposportail();
      }
    }
  }
}
void setup() {
  dht.begin();
  //  lcd.init(); //initialize the lcd
  //  lcd.backlight(); //open the backlight
  //  lcd.clear();
  //  lcd.setCursor(0, 0); // set the cursor to column 15, line 0
  //  lcd.print("init and ferme!");

  Serial.begin(9600);

  Serial.println("***** portail droit*********");
  Serial.println("***** Master*********");

  countrot.attach(countrot_pin, INPUT_PULLUP);
  countrot.interval(10); // debounce interval in ms
  trig.attach(trig_pin, INPUT_PULLUP);
  trig.interval(10); // debounce interval in ms
  POC.attach(POC_pin, INPUT_PULLUP);
  POC.interval(10); // debounce interval in ms

  pinMode (EMA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!driver.init())
    Serial.println("init failed");

  Serial.print("opendelay ms: ");
  Serial.println(opendelay);
  Serial.print("closedelay ms: ");
  Serial.println(closedelay);
  Serial.println("init ferme!");
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(EMA, 0);

  stateportail = 0;
  countrotation = 0;
  maxrotationopen = 500;
  maxrotationclose = 500;
  minmotorpos = 500;
  maxmotorpos = 500;
  calibre = 1;
  inrotation = false;
  measold = 0;
  startPOC = millis();  //initial start time
}

void loop() {
  countrot.update();
  trig.update();
  POC.update();
  curPOC = millis();
  if ((curPOC - startPOC > sendPOC * 1000) & !inrotation)
  {
    startPOC = millis();
    float humid = dht.readHumidity();
    //    // Read temperature as Celsius
    float tempera = dht.readTemperature();
    if (isnan(humid) || isnan(tempera)) {
      Serial.println("Failed to read from DHT sensor!");
    } else {
      //      Serial.print("Humidity: ");
      //      Serial.print(humid);
      //      Serial.print(" %\t");
      //      Serial.print("Temperature: ");
      //      Serial.print(tempera);
      //      Serial.println(" *C ");
      String strtemp = alignT(tempera, "T");
      sendmessage(strtemp);
      strtemp = alignT(humid, "H");
      sendmessage(strtemp);
    }
    //    sendmessage(stringPCf);
    //    delay(500); // 1000 ms
    //    sendmessage(stringPCo);
    //      delay(500); // 1000 ms

    if (!POC.read()) {
      sendmessage(stringPCf);
    }
    if (POC.read()) {
      sendmessage(stringPCo);
    }
  }
  if ( inrotation ) {
    // Serial.println("in rotation");
    curROT = millis();
    if (curROT - startROT > delayCurrentDrive) {// measure after 0.5 sec
      anaread();
    }
    if (countrot.fell())
    {
      Serial.println("count rotation pressed");
      pressedcountrot();
    }
    if (curROT - startROT > rotStop * 1000) { //stop after rotstop
      inrotation = false;
      if (calibre == 2) {
        Serial.print("calibration maxrotationoclose: ");
        Serial.println(maxrotationclose);
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        //      halfclose = int(0.5 * maxrotationclose);
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        //minmotorpos = maxrotationclose - 1;
        stateportail = 0;
        actionportail();
        calibre = 0;
        //        lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //        lcd.print(" cal fer");
        Serial.print("New calibration maxrotation: ");
        Serial.println(maxrotationclose);
      }
      if (calibre == 1) {
        Serial.print("calibration maxrotationopen: ");
        Serial.println(maxrotationopen);
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        maxrotationopen = countrotation;
        //minmotorpos = int(0.1 * maxrot
        //      halfopen = int(0.5 * maxrotationopen);
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
        //        lcd.setCursor(8, 0); // set the cursor to column 15, line 0
        //        lcd.print(" cal ope");
  
      }
      Serial.println("stop motor after 20s");
      //      lcd.setCursor(8, 0); // set the cursor to column 15, line 0
      //      lcd.print(" timeove");
      Serial.print("countrotation : ");
      Serial.println(countrotation);
      Serial.print("calibration maxrotation: ");
      Serial.println(maxrotationopen);
      if (stateportail == 1) {
        stateportail = 2;
      }
      if (stateportail == 3) {
        stateportail = 0;
      }
      actionportail();
    }
  }

  // change mode by pressing remote controller
  if (trig.fell()) {
    Serial.println(" action pressed!");
    triggeraction();
    //delay(1000); // 1000 ms
  }
  //delay(delayloop); // 100 ms
}
