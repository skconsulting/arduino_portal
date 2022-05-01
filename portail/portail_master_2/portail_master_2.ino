// 28 04 2022  LAST UPDATE THE ONE TO be USED AT THIS DATE
#include <RH_ASK.h>
#include "DHT.h"
#include <Bounce2.h>
RH_ASK driver;
const boolean debug = true;
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
// for right portail master , seen from inside court

const unsigned long messageDelay = 60000; // message duration  after 60 seconds
const unsigned long rotStop = 20000; // stop rotation after 20 seconds
const unsigned long sendPOC = 30000; //send info portail every 30 seconds
const unsigned long opendelay = 1000; // 1 second afte command to open
const unsigned long closedelay = 0; // 0 second after command to close
const unsigned long delayCurrentDrive = 100; // delay before read motor current in ms
const unsigned long delaybetweentrig = 2000; // delay before 2 trigger action
const unsigned long delaybetweencount = 100; // delay before 2 rot action

const uint8_t minmotor = 150; // value when approaching end of run

// var util

volatile boolean triggerFlag = false;
volatile boolean counterFlag = false;

boolean messageFlag = false; // 0 pas de message, 1 ouvre, 2 repos ouvert, 3 ferme , repos ferme, 5 ouvre puis ferme, 6 ferme puis ouvre, 7 overdrive, 8 overtime
uint8_t stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
uint8_t calibre ;
int maxrotationopen, maxrotationclose;
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
unsigned long trigCOUNT; // delay between rotation
unsigned long curTime; // current time
unsigned long startROT; // start of rotation
unsigned long startMessage; // start of message
const String stringCAo = String("ouvreC");
const String stringCAf = String("fermeC");
const String stringPCo = String("ouvreP");
const String stringPCf = String("fermeP");

const String message1 = String("messa1");
//const String message2 = String("messa2");
const String message3 = String("messa3");
//const String message4 = String("messa4");
const String message5 = String("messa5");
const String message6 = String("messa6");
const String message7 = String("messa7");
const String message8 = String("messa8");

DHT dht(DHTPIN, DHTTYPE);

void pressedcountrot (void) // each time there is a counter increase
{
  if (stateportail == 3) {//ferme
    countrotation++;
    if (countrotation == maxrotationclose) {
      if (debug) {
        Serial.print(F("min rotation reached! completement ferme: "));
        Serial.println(countrotation);
      }
      stateportail = 0;
      actionportail();
    }
    if (countrotation == minmotorpos) {
      if (debug) {
        Serial.println(F("min motor reached! "));
      }
      analogWrite(EMA, minmotor);
    }
  }
  if (stateportail == 1) {//ouvre
    countrotation++;
    if (countrotation == maxrotationopen) {
      if (debug) {
        Serial.print(F("maxrotation reached! completement ouvert!: "));
        Serial.println(countrotation);
      }
      stateportail = 2;
      actionportail();
    }
    if (countrotation == maxmotorpos) {
      if (debug) {
        Serial.print(F("max motor reached!: "));
        Serial.println(maxmotorpos);
        Serial.print(F("countrotation : "));
        Serial.println(countrotation);
      }
      analogWrite(EMA, minmotor);
    }
  }
  if (debug) {
    Serial.print(F("rotation count"));
    Serial.println(countrotation);
  }
}

void ouvreportail()
{
  inrotation = true;
  startROT = millis();
  analogWrite(EMA, 0);
  delay(2);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  delay(2);
  analogWrite(EMA, 255);
}
void fermeportail()
{
  inrotation = true;
  startROT = millis();
  analogWrite(EMA, 0);
  delay(2);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  delay(2);
  analogWrite(EMA, 255);
}
void reposportail() {
  inrotation = false;
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
}

void actionportail() {// 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  switch (stateportail) {
    case 0: {
        if (debug) {
          Serial.println(F("repos ferme!"));
        }
        countrotation = 0;
        reposportail();
        break;
      }
    case 1: {
        if (debug) {
          Serial.println(F("ouvre!"));
        }
        delay(opendelay);
        ouvreportail();
        break;
      }
    case 2: {
        if (debug) {
          Serial.println(F("repos ouvert!"));
        }
        reposportail();
        countrotation = 0;
        break;
      }
    case 3: {
        if (debug) {
          Serial.println(F("ferme!"));
        }
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
  if (debug) {
    Serial.print(F("send message 433mhz: "));
    Serial.println(msg);
  }
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
  //  if (debug) {
  //    Serial.print(F("old stateportail: "));
  //    Serial.println( stateportail);
  //  }
  switch (stateportail) {
    case 0: {
        stateportail = 1;
        sendmessage(stringCAo);
        sendStatus(message1);
        break;
      }
    case 1: {
        stateportail = 3;
        sendmessage(stringCAf);
        sendStatus(message5);
        break;
      }
    case 2 : {
        stateportail = 3;
        sendmessage(stringCAf);
        sendStatus(message3);
        break;
      }
    case 3: {
        stateportail = 1;
        sendmessage(stringCAo);
        sendStatus(message6);
        break;
      }
  }
  if (debug) {
    Serial.print(F("new stateportail: "));
    Serial.println( stateportail);
  }
  actionportail();
}
void sendStatus (String ins) {
  sendmessage(ins);
  startMessage = millis();
  messageFlag=true;
}

void anaread ()
//stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
{
  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000; // Gets you mV
  meas0 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  // delay(1); // 1 ms
  meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  //delay(1); // 1 ms
  meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  Voltage = (meas / 1024.0) * 5000; // Gets you mV
  // Serial.println(meas );

  //    measold = meas;
  if (abs(meas - measold) > 5) {
    measold = meas;
    if (debug) {
      Serial.print(F("mV = ")); // shows the voltage measured
      Serial.println(Voltage );
      Serial.print(F("Ref mV = ")); // shows the voltage measured
      Serial.println(VoltageRef );
    }
  }

  if (Voltage < VoltageRef) {

    digitalWrite(LED_BUILTIN, HIGH);
    if (debug) {
      Serial.print(F("overdrive : "));
      Serial.println(Voltage);
      Serial.print(F("over VoltageRef: "));
      Serial.println(VoltageRef);
      Serial.print(F("countrotation : "));
      Serial.println(countrotation);
      Serial.print(F("calibration maxrotation: "));
      Serial.println(maxrotationopen);
    }
    if (calibre == 2) {
      maxrotationclose = max(countrotation, maxrotationopen);
      maxrotationopen = maxrotationclose;
      minmotorpos = int(0.9 * maxrotationclose);
      maxmotorpos = int(0.9 * maxrotationopen);
      stateportail = 0;
      actionportail();
      calibre = 0;
      if (debug) {
        Serial.print(F("calibration maxrotationoclose: "));
        Serial.println(maxrotationclose);
      }
    }
    if (calibre == 1) {
      maxrotationopen = countrotation;
      maxmotorpos = int(0.9 * maxrotationopen);
      stateportail = 2;
      actionportail();
      calibre = 2;
      if (debug) {
        Serial.print(F("calibration maxrotationopen: "));
        Serial.println(maxrotationopen);
      }
    }
    if (stateportail == 1) {
      if (debug) {
        Serial.println(F("overdrive when open!"));
        Serial.print(F("countrotation : "));
        Serial.println(countrotation);
      }
      stateportail = 2;
      countrotation = maxrotationclose - countrotation;
      reposportail();
    }
    if (stateportail == 3) {
      if (debug) {
        Serial.println(F("overdrive with a moitie ferme!"));
        Serial.print(F("countrotation : "));
        Serial.println(countrotation);
      }
      stateportail = 0;
      countrotation = maxrotationclose - countrotation;
      reposportail();
    }
  }
  //}
}
void setup() {
  dht.begin();
  Serial.begin(115200);
  Serial.println(F("***** portail droit*****Master****"));
  Serial.println(F("with interrupt portail_master_1"));

  pinMode(countrot_pin, INPUT_PULLUP);
  pinMode(trig_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(countrot_pin), counteractionISR, RISING);
  attachInterrupt(digitalPinToInterrupt(trig_pin), triggeractionISR, FALLING);

  POC.attach(POC_pin, INPUT_PULLUP);
  POC.interval(10); // debounce interval in ms

  pinMode (EMA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  if (!driver.init())
    Serial.println("init failed");

  Serial.print(F("opendelay ms: "));
  Serial.println(opendelay);
  Serial.print(F("closedelay ms: "));
  Serial.println(closedelay);
  Serial.println(F("init ferme!"));
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
  triggerFlag = false;
  counterFlag = false;
  delay(1000);
  humtempPOC = millis();  //initial start time
  trigROT = millis();
  startMessage = millis();
  trigCOUNT = millis();
}
//
void loop1() {
  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000; // Gets you mV
  meas0 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1); // 5 ms
  meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1); // 5 ms
  meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  Voltage = (meas / 1024.0) * 5000; // Gets you mV

  Serial.print("mV = "); // shows the voltage measured
  Serial.println(Voltage );
  Serial.print("Ref mV = "); // shows the voltage measured
  Serial.println(VoltageRef );
  // delay(1000);
}

void loop() {
  //  countrot.update();
  //  trig.update();
  POC.update();
  curTime = millis();
  if ((curTime - startMessage) > messageDelay) {
    messageFlag = false;
  }
  if ((curTime - humtempPOC > sendPOC) & !inrotation  & !messageFlag)
  {
    humtempPOC = millis();
    float humid = dht.readHumidity();
    //float humid = 10.;
    //    // Read temperature as Celsius
    float tempera = dht.readTemperature();
    //float tempera = 45.3;
    if (isnan(humid) || isnan(tempera)) {
      if (debug) {
        Serial.println("Failed to read from DHT sensor!");
      }
    } else {
      if (debug) {
        Serial.print(F("Humidity: "));
        Serial.print(humid);
        Serial.print(" %\t");
        Serial.print(F("Temperature: "));
        Serial.print(tempera);
        Serial.println(" *C ");
      }
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
    if (curTime - startROT > delayCurrentDrive) {// measure after 0.5 sec
      //Serial.println("start measure");
      anaread();
    }
    if (curTime - startROT > rotStop ) { //stop after rotstop
      sendStatus(message8);
      inrotation = false;
      if (calibre == 2) {
        if (debug) {
          Serial.print(F("calibration maxrotationoclose: "));
          Serial.println(maxrotationclose);
          Serial.print(F("countrotation : "));
          Serial.println(countrotation);
        }
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        //minmotorpos = maxrotationclose - 1;
        stateportail = 0;
        actionportail();
        calibre = 0;
        if (debug) {
          Serial.print(F("New calibration maxrotation: "));
          Serial.println(maxrotationclose);
        }
      }
      if (calibre == 1) {
        if (debug) {
          Serial.print(F("calibration maxrotationopen: "));
          Serial.println(maxrotationopen);
          Serial.print(F("countrotation : "));
          Serial.println(countrotation);
        }
        maxrotationopen = countrotation;
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
      }
      if (debug) {
        Serial.print(F("stop motor after 20s, countrotation : "));
        Serial.println(countrotation);
        Serial.print(F("calibration maxrotation: "));
        Serial.println(maxrotationopen);
      }
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
    triggerFlag = false;
    //if (debug) {
    Serial.print(F(" action pressed!.. "));
    //}
    if (curTime - trigROT > delaybetweentrig ) {
      //if (debug) {
      Serial.println(F("OK!"));
      //}
      trigROT = millis();
      triggeraction();
    }
    else {
      //if (debug) {
      Serial.println(F("NOT OK"));
      //}
    }
    triggerFlag = false;
  }

  if ( counterFlag) {
    counterFlag = false;
    if (debug) {
      Serial.println(F("counter pressed!"));
    }
    if (curTime - trigCOUNT > delaybetweencount ) {
      trigCOUNT = millis();
      if ( inrotation ) {
        pressedcountrot();

      }
    }
  }
  counterFlag = false;
}
