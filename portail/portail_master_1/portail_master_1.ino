// 16 03 2022  LAST UPDATE THE ONE USED AT THIS DATE
#include <RH_ASK.h>
#include "DHT.h"
//#include <SPI.h> // Not actually used but needed to compile
#include <Bounce2.h>
RH_ASK driver;

// pins
const byte   countrot_pin  = PD2; // counter of rotation pull up
const byte   trig_pin      = PD3; // change rotation , trigger action pull up
const uint8_t   EMA       = PD5; // pwm motor
const uint8_t   IN1       = PD6; // command motor 1
const uint8_t   IN2       = PD7; // command motor 2
const uint8_t   POC_pin       = 8; // detection portail ouvert/ferme on pin D8 PB0
#define DHTPIN 4     // what pin we're connected to thermal sensor D10
//D12 TX433mhz data

const uint8_t   refsensor    = A6; // overloadsensor reference
const uint8_t   sensor    = A7; // overloadsensor overload if sensor < ref sensor

#define DHTTYPE DHT22   // DHT 22  (AM2302)

Bounce POC = Bounce();

// var config
// for right portail , seen from inside court

//const unsigned long postingInterval = 2000; // delay between updates, in milliseconds
const unsigned long rotStop = 20000; // stop rotation after 20 seconds
const unsigned long sendPOC = 30000; //send info portail every 30 seconds
const unsigned long opendelay = 1000; // 1 second afte command to open
const unsigned long closedelay = 0; // 0 second after command to close
const unsigned long delayCurrentDrive = 500; // delay before read motor current in ms
const unsigned long delaybetweentrig = 1000; // delay before 2 trigger action
const uint8_t minmotor = 150; // value when approaching end of run

// var util
volatile boolean triggerFlag = false;
volatile boolean counterFlag = false;

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

unsigned long humtempPOC; // delay for humidity and temperature sensor send data
unsigned long trigROT; // delay between 2 triggers
unsigned long curTime; // current time
unsigned long startROT; // start of rotation
//unsigned long curROT ;

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
  Serial.print("rotation count");
  Serial.println(countrotation);
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

void triggeractionISR (void) {
  triggerFlag = true;
}


void counteractionISR (void) {
  counterFlag = true;
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
  delay(1); // 5 ms
  meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1); // 5 ms
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
  Serial.begin(115200);
  Serial.println("***** portail droit*****Master****");
  Serial.println(" with interrupt portail_master_1");

  pinMode(countrot_pin, INPUT_PULLUP);
  pinMode(trig_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(countrot_pin), counteractionISR, RISING);
  attachInterrupt(digitalPinToInterrupt(trig_pin), triggeractionISR, FALLING);

  //  countrot.attach(countrot_pin, INPUT_PULLUP);
  //  countrot.interval(10); // debounce interval in ms
  //  trig.attach(trig_pin, INPUT_PULLUP);
  //  trig.interval(10); // debounce interval in ms
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
  humtempPOC = millis();  //initial start time
  trigROT = millis();
}

void loop() {
  //  countrot.update();
  //  trig.update();
  POC.update();
  curTime = millis();
  if ((curTime - humtempPOC > sendPOC) & !inrotation)
  {
    humtempPOC = millis();
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
 
    if (!POC.read()) {
      sendmessage(stringPCf);
    }
    if (POC.read()) {
      sendmessage(stringPCo);
    }
  }
  if ( inrotation ) {
    // Serial.println("in rotation");
    if (curTime - startROT > delayCurrentDrive) {// measure after 0.5 sec
      anaread();
    }
    if (curTime - startROT > rotStop ) { //stop after rotstop
      inrotation = false;
      if (calibre == 2) {
        Serial.print("calibration maxrotationoclose: ");
        Serial.println(maxrotationclose);
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        //minmotorpos = maxrotationclose - 1;
        stateportail = 0;
        actionportail();
        calibre = 0;
        Serial.print("New calibration maxrotation: ");
        Serial.println(maxrotationclose);
      }
      if (calibre == 1) {
        Serial.print("calibration maxrotationopen: ");
        Serial.println(maxrotationopen);
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        maxrotationopen = countrotation;
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
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
  if (triggerFlag) {
    Serial.println(" action pressed!");
    if (curTime - trigROT > delaybetweentrig ) {
      triggeraction();
      trigROT = millis();
    }
    triggerFlag = false;
  }
  if ( counterFlag) {
    Serial.println(" counter pressed!");
    if ( inrotation ) {
      pressedcountrot();
    }
    counterFlag = false;
  }
}
