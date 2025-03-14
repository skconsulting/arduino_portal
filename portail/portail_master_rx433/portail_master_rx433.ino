// 7 mar 2025 Portail Master with rxtx433
//#define RH_ASK_MAX_MESSAGE_LEN 80  //67
#include <RH_ASK.h>
#include "DHT.h"
//#include <Bounce2.h>
RH_ASK driver;
//int i = 0;

const boolean debug = true;
const boolean noRadio = false;
const boolean noDTH = true;
// pins
const byte countrot_pin = PD2;  // counter of rotation pull up
const byte trig_pin = PD3;      // trigger action pull up
const uint8_t EMA = PD5;        // pwm motor
const uint8_t IN1 = PD6;        // command motor 1
const uint8_t IN2 = PD7;        // command motor 2
const uint8_t POC_pin = 8;      // detection portail ouvert/ferme on pin D8 PB0
#define DHTPIN 4                // what pin we're connected to thermal sensor D10
//D12 TX433mhz data

const uint8_t refsensor = A6;  // overloadsensor reference
const uint8_t sensor = A7;     // overloadsensor overload if sensor < ref sensor

#define DHTTYPE DHT22  // DHT 22  (AM2302)

//Bounce POC = Bounce();

// var config
// for right portail master , seen from inside court

const unsigned long messageDelay = 30000;     // message info delay  after 30 seconds
const unsigned long rotStop = 20000;          // stop rotation after 20 seconds
const unsigned long opendelay = 1000;         // 1 second afte command to open
const unsigned long closedelay = 0;           // 0 second after command to close
const unsigned long delayCurrentDrive = 100;  // delay before read motor current in ms
const unsigned long delaybetweentrig = 2000;  // delay before 2 trigger action
const unsigned long delaybetweencount = 50;   // delay before 2 rot action

const uint8_t minmotor = 150;  // value when approaching end of run

// var util

volatile boolean countFlag = false;
volatile boolean triggerFlag = false;
//boolean messageFlag = false;  // 0 pas de message, 1 ouvre, 2 repos ouvert, 3 ferme , repos ferme, 5 ouvre puis ferme, 6 ferme puis ouvre, 7 overdrive, 8 overtime
uint8_t stateportail = 0;  // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
uint8_t calibre;
int maxrotationopen, maxrotationclose;
int minmotorpos, maxmotorpos;
bool inrotation = false;
double VoltageRef = 3.3;  // ref Voltage from potentiometre
int countrotation = 0;

double meas, meas0, meas1, meas2, measold, measref;

double Voltage = 0;

int mVperAmp;
int ACSoffset;

const char *msg = "ouvreP";

unsigned long trigROT;       // delay between 2 triggers
unsigned long trigCOUNT;     // delay between rotation
unsigned long startROT;      // start of rotation
unsigned long startMessage;  // start of message

const String stringCAo = String("ouvreCo");
const String stringCAf = String("fermeCo");

const String stringPCo = String("ouvreP");  // portail ouvert
const String stringPCf = String("fermeP");  // portail ferme

const String message0 = String("STIRO");    // stop after 20 se run mode ouvre
const String message1 = String("STIRF");    // stop after 20 se run mode ferme
const String message2 = String("SODCO: ");  // stop after overdrivese calibre mode ouvre
const String message3 = String("SODCF: ");  // stop after overdrivese calibre mode ferme
const String message4 = String("SODRO: ");  // stop after overdrivese run mode ouvre
const String message5 = String("SODRF: ");  // stop after overdrivese run mode ferme
const String message6 = String("STICO: ");  // stop after 20S  calibre mode ouvre
const String message7 = String("STICF: ");  // stop after 20S  calibre mode ferme
const String message8 = String("MACRO: ");  // max counter reach run mode ouvre
const String message9 = String("MACRF: ");  // max counter reached run mode ferme

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println(F("***** portail droit*****Master****"));
  Serial.println(F("with interrupt portail_master_rx433"));

  pinMode(EMA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(countrot_pin, INPUT_PULLUP);
  pinMode(trig_pin, INPUT_PULLUP);
  pinMode(POC_pin, INPUT_PULLUP);
  /*
  POC.attach(countrot_pin, INPUT_PULLUP);
  POC.attach(trig_pin, INPUT_PULLUP);
  POC.interval(5);  // debounce interval in ms
  POC.setPressedState(LOW);
*/
  attachInterrupt(digitalPinToInterrupt(countrot_pin), counteractionISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(trig_pin), triggeractionISR, FALLING);


  // POC.setPressedState(Low);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(EMA, 0);

  if (debug) {
    Serial.println(F("start DTH"));
  }
  if (!noDTH) {
    dht.begin();
  }
  if (!noRadio) {
    if (!driver.init())
      Serial.println("Starting TX433 failed");
  }

  Serial.print(F("opendelay ms: "));
  Serial.println(opendelay);
  Serial.print(F("closedelay ms: "));
  Serial.println(closedelay);
  Serial.println(F("init ferme!"));

  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000;  // Gets you mV
  meas0 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1);                                // 5 ms
  meas1 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1);                                // 5 ms
  meas2 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  Voltage = (meas / 1024.0) * 5000;  // Gets you mV

  Serial.print("sensor mV = ");  // shows the voltage measured
  Serial.println(Voltage);
  Serial.print("Ref mV = ");  // shows the voltage measured
  Serial.println(VoltageRef);


  stateportail = 0;
  countrotation = 0;
  maxrotationopen = 500;
  maxrotationclose = 500;
  minmotorpos = 500;
  maxmotorpos = 500;
  calibre = 1;
  inrotation = false;
  measold = 0;
  delay(1000);

  trigROT = millis();
  startMessage = millis();
  trigCOUNT = millis();
  startROT = millis();

  sendStatus("init ferme", 5);
}

void pressedcountrot(void)  // each time there is a counter increase
{
  if (stateportail == 3) {  //ferme
    countrotation++;
    if (countrotation == maxrotationclose) {
      if (debug) {
        Serial.print(F("min rotation reached! completement ferme: "));
        Serial.println(countrotation);
      }
      stateportail = 0;
      actionportail();
      sendStatus(message9 + String(countrotation) + " on: " + String(maxrotationclose), 2);
    }
    if (countrotation == minmotorpos) {
      if (debug) {
        Serial.println(F("min motor reached! "));
      }
      analogWrite(EMA, minmotor);
    }
  }
  if (stateportail == 1) {  //ouvre
    countrotation++;
    if (countrotation == maxrotationopen) {
      if (debug) {
        Serial.print(F("maxrotation reached! completement ouvert!: "));
        Serial.println(countrotation);
      }
      stateportail = 2;
      actionportail();
      sendStatus(message8 + String(countrotation) + " on: " + String(maxrotationopen), 2);
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
    Serial.println(String(countrotation));
  }
  //sendStatus("in rot: " + String(countrotation));
}

void ouvreportail() {
  inrotation = true;
  startROT = millis();
  analogWrite(EMA, 0);
  delay(2);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  delay(2);
  analogWrite(EMA, 255);
}
void fermeportail() {
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
  startROT = millis();
  inrotation = false;
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
}

void actionportail() {  // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  switch (stateportail) {
    case 0:
      {
        if (debug) {
          Serial.println(F("repos ferme!"));
        }
        countrotation = 0;
        reposportail();
        break;
      }
    case 1:
      {
        if (debug) {
          Serial.println(F("ouvre!"));
        }
        delay(opendelay);
        ouvreportail();
        break;
      }
    case 2:
      {
        if (debug) {
          Serial.println(F("repos ouvert!"));
        }
        reposportail();
        countrotation = 0;
        break;
      }
    case 3:
      {
        if (debug) {
          Serial.println(F("ferme!"));
        }
        delay(closedelay);
        fermeportail();
        break;
      }
  }
}
/*
void sendmessage(String instr) {
  if (!noRadio) {
    //String newins = "";
    //if (debug) {
    //  Serial.print(F("send message: "));
    //  Serial.println(instr + String(instr.length()));
    //}
    /*
    newins = padStringTo16(instr);
    if (debug) {
      Serial.print(F("new string "));
      Serial.println(newins + String(newins.length()));
    }
    */
/*
    int n = instr.length();
    char msg[n + 1];
    strcpy(msg, instr.c_str());
    if (debug) {
      Serial.print(F("send message 433mhz: "));
      Serial.println(String(msg) + " " + strlen(msg));
    }
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
  }
}
*/
String alignT(float t, String C) {
  String strt;
  strt = String(t, 1);
  if (t >= 10) {
    strt = C + "+" + strt;
  } else if (t >= 0) {
    strt = C + "+0" + strt;
  } else if (t <= -10) {
    strt = C + strt;
  } else {
    strt = C + "-0" + strt.substring(1, 4);
  }
  return strt;
}


void counteractionISR(void) {
  countFlag = true;
}
void triggeractionISR(void) {
  triggerFlag = true;
}


void triggeraction(void) {  // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  // run each time action button is pressed
  digitalWrite(LED_BUILTIN, LOW);
  //  if (debug) {
  //    Serial.print(F("old stateportail: "));
  //    Serial.println( stateportail);
  //  }
  switch (stateportail) {
    case 0:
      {
        stateportail = 1;
        sendStatus(stringCAo, 3);
        break;
      }
    case 1:
      {
        stateportail = 3;
        sendStatus(stringCAf, 3);
        break;
      }
    case 2:
      {
        stateportail = 3;
        sendStatus(stringCAf, 3);
        break;
      }
    case 3:
      {
        stateportail = 1;
        sendStatus(stringCAo, 3);
        break;
      }
  }
  if (debug) {
    Serial.print(F("new stateportail: "));
    Serial.println(stateportail);
  }
  actionportail();
}
/*
String padStringTo16(String ins) {
  String str = ins;
  if (str.length() < 16) {
    while (str.length() < 16) {
      str = str + " ";
    }
  }
  return str;
}
*/
void sendStatus(String instr, uint8_t numb) {
  if (!noRadio) {
    //String newins = "";
    //if (debug) {
    //  Serial.print(F("send message: "));
    //  Serial.println(instr + String(instr.length()));
    //}
    /*
    newins = padStringTo16(instr);
    if (debug) {
      Serial.print(F("new string "));
      Serial.println(newins + String(newins.length()));
    }
    */
    int n = instr.length();
    char msg[n + 1];
    strcpy(msg, instr.c_str());
    if (debug) {
      Serial.print(F("send message 433mhz: "));
      Serial.println(String(msg) + " " + strlen(msg));
    }
    for (int i = 0; i < numb; i++) {
      driver.send((uint8_t *)msg, strlen(msg));
      driver.waitPacketSent();
      /*
      if (debug) {
        Serial.println("send number " + String(i) + "on: " + String(numb));
      }
      
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    */
    }
    startMessage = millis();
    //messageFlag = true;
  }
}

void anaread()
//stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
{
  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000;  // Gets you mV
  meas0 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  // delay(1); // 1 ms
  meas1 = analogRead(sensor);  // Converts and read the analog input value (value from 0.0 to 1.0)
  //delay(1); // 1 ms
  meas2 = analogRead(sensor);  // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  Voltage = (meas / 1024.0) * 5000;  // Gets you mV
  // Serial.println(meas );

  //    measold = meas;
  if (abs(meas - measold) > 5) {
    measold = meas;
    if (debug) {
      Serial.print(F("mV = "));  // shows the voltage measured
      Serial.println(Voltage);
      Serial.print(F("Ref mV = "));  // shows the voltage measured
      Serial.println(VoltageRef);
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
    if (calibre == 0) {
      if (stateportail == 1) {
        if (debug) {
          Serial.println(F("overdrive when open!"));
          Serial.print(F("countrotation : "));
          Serial.println(countrotation);
        }
        sendStatus(message4 + String(countrotation) + " on: " + String(maxrotationclose), 3);
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
        sendStatus(message5 + String(countrotation) + " on: " + String(maxrotationclose), 3);
        stateportail = 0;
        countrotation = maxrotationclose - countrotation;
        reposportail();
      }
    } else if (calibre == 2) {
      maxrotationclose = max(countrotation, maxrotationopen);
      maxrotationopen = maxrotationclose;
      minmotorpos = int(0.9 * maxrotationclose);
      maxmotorpos = int(0.9 * maxrotationopen);
      stateportail = 0;
      sendStatus(message3 + String(countrotation) + " on: " + String(maxrotationclose), 3);
      actionportail();
      calibre = 0;
      if (debug) {
        Serial.print(F("calibration maxrotationoclose: "));
        Serial.println(maxrotationclose);
      }
    } else if (calibre == 1) {
      maxrotationopen = countrotation;
      maxmotorpos = int(0.9 * maxrotationopen);
      stateportail = 2;
      sendStatus(message2 + String(maxrotationopen), 3);
      actionportail();
      calibre = 2;
      if (debug) {
        Serial.print(F("calibration maxrotationopen: "));
        Serial.println(maxrotationopen);
      }
    }
  }
  //}
}

//
void loop1() {
  sendStatus("1234567890123", 3);
  Serial.println("send1");
  delay(100);
  sendStatus("1234567890123", 3);
  Serial.println("send2");
  delay(100);
}
void loop2() {

  /*
  measref = analogRead(refsensor);
  VoltageRef = (measref / 1024.0) * 5000;  // Gets you mV
  meas0 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1);                                // 5 ms
  meas1 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(1);                                // 5 ms
  meas2 = analogRead(sensor);              // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  Voltage = (meas / 1024.0) * 5000;  // Gets you mV

  Serial.print("mV = ");  // shows the voltage measured
  Serial.println(Voltage);
  Serial.print("Ref mV = ");  // shows the voltage measured
  Serial.println(VoltageRef);

  i++;
  Serial.println("eteint tout"+i);
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  delay(500);
  Serial.println("allume in1");
  digitalWrite(IN1, 1);
  delay(500);
  Serial.println("allume in2");
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  delay(500);
  Serial.println("allume ema");
  analogWrite(EMA, 200);
  digitalWrite(IN2, 0);
  delay(500);
    */
}

void loop() {

  float humid = 1;
  float tempera = 0;

  if (triggerFlag) {
    if (millis() - trigROT > delaybetweentrig) {
      if (debug) {
        Serial.println(F("Trigger action OK!"));
      }
      triggeraction();
      trigROT = millis();
    } else {
      if (debug) {
        Serial.println(F("NOT OK delay trigger"));
      }
    }
    triggerFlag = false;
  }

  if (countFlag) {
    /*
    if (debug) {
      Serial.println(F("counter pressed!"));
    }*/
    //curTime = millis();
    if (inrotation && (millis() - trigCOUNT) > delaybetweencount) {
      trigCOUNT = millis();
      if (debug) {
        Serial.println(F("counter pressed OK!"));
      }
      pressedcountrot();
    }
    countFlag = false;
  }

  if (inrotation) {
    if (millis() - startROT > delayCurrentDrive) {  // measure after 0.5 sec
      //Serial.println("start measure");
      anaread();
    }
    //curTime = millis();
    if (millis() - startROT > rotStop) {  //stop after rotstop
      inrotation = false;
      if (calibre == 0) {

        if (stateportail == 1) {
          sendStatus(message0 + String(countrotation) + " on: " + String(maxrotationopen), 3);
          stateportail = 2;
        }
        if (stateportail == 3) {
          sendStatus(message1 + String(countrotation) + " on: " + String(maxrotationopen), 3);
          stateportail = 0;
        }
        actionportail();
      } else if (calibre == 2) {
        if (debug) {
          Serial.print(F("calibration maxrotationoclose: "));
          Serial.println(maxrotationclose);
          Serial.print(F("countrotation : "));
          Serial.println(countrotation);
        }
        sendStatus(message7 + String(countrotation) + " on: " + String(maxrotationopen), 3);
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
      } else if (calibre == 1) {
        if (debug) {
          Serial.print(F("calibration maxrotationopen: "));
          Serial.println(maxrotationopen);
          Serial.print(F("countrotation : "));
          Serial.println(countrotation);
        }
        maxrotationopen = countrotation;
        sendStatus(message6 + String(countrotation), 3);
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
      startMessage = millis();
    }
  } else {
    //curTime = millis();
    if (millis() - startMessage > messageDelay) {
      if (!noDTH) {
        if (debug) {
          Serial.println("read DTH");
        }
        humid = dht.readHumidity();
        //float humid = 10.;
        tempera = dht.readTemperature();  //Read temperature as Celsius
      } else {
        if (debug) {
          Serial.println("read DTH dummy values");
        }
        float humid = 52.0;
        float tempera = 90.0;
      }
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
        sendStatus(strtemp, 10);
        strtemp = alignT(humid, "H");
        sendStatus(strtemp, 10);
      }
      if (!digitalRead(POC_pin)) {
        sendStatus(stringPCf, 10);
      } else {
        sendStatus(stringPCo, 10);
      }
      startMessage = millis();
    }
  }
}
