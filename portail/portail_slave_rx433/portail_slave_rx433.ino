// 2094 fev 2025 Portail Slave with Lora
#define RH_ASK_MAX_MESSAGE_LEN 24  //67

#include <RH_ASK.h>  // (fait partie de Radiohead)

const boolean debug = true;
const boolean noRadio = false;
// pins
const byte countrot_pin = PD2;  // counter of rotation pull up
//const byte trig_pin = PD3;      // trigger action pull up
const uint8_t EMA = PD5;  // pwm motor
const uint8_t IN1 = PD6;  // command motor 1
const uint8_t IN2 = PD7;  // command motor 2
//D12 TX433mhz data

const uint8_t refsensor = A6;  // overloadsensor reference
const uint8_t sensor = A7;     // overloadsensor overload if sensor < ref sensor
// var config
// for right portail master , seen from inside court

const unsigned long rotStop = 20000;          // stop rotation after 20 seconds
const unsigned long opendelay = 0;            // 0 second afte command to open
const unsigned long closedelay = 1000;        // 1 second after command to close
const unsigned long delayCurrentDrive = 100;  // delay before read motor current in ms
const unsigned long delaybetweentrig = 1500;  // delay before 2 trigger action
const unsigned long delaybetweencount = 50;   // delay before 2 rot action

const uint8_t minmotor = 150;  // value when approaching end of run

// var util
volatile boolean countFlag = false;

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

unsigned long trigCOUNT;  // delay between rotation
unsigned long trigROT;    // delay between 2 triggers
unsigned long startROT;   // start of rotation

const String stringCAo = String("ouvreCo");
const String stringCAf = String("fermeCo");
RH_ASK driver;

void setup() {
  Serial.begin(9600);
  Serial.println(F("***** portail gauche*****slave****"));
  Serial.println(F("with interrupt portail_slave_rx433"));

  pinMode(EMA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(countrot_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(countrot_pin), counteractionISR, RISING);

  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(EMA, 0);

  if (!noRadio) {
    Serial.println("init receive rx433 test for portail");
    //Serial.println("max length message:" + String(buflen));

    if (!driver.init()) {
      Serial.println("Echec de l'initialisation de Radiohead");
    }
  }

  Serial.print(F("opendelay ms: "));
  Serial.println(opendelay);
  Serial.print(F("closedelay ms: "));
  Serial.println(closedelay);
  Serial.println(F("init ferme!"));


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

  trigCOUNT = millis();
  trigROT = millis();
  startROT = millis();
}
/*
void onReceive(int packetSize) {
  String inString = "";  // string to hold input

  for (int i = 0; i < packetSize; i++) {
    char incoming = (char)LoRa.read();
    inString += (char)incoming;
  }
  finalString = inString;
  if (debug) {
    Serial.print("Received packet: '");
    Serial.println(finalString);
  }
  intFlag = true;
}
*/
void trigAction(String finalString) {
  if ((finalString == stringCAo) || (finalString == stringCAf)) {

    if (finalString == stringCAo) {
      stateportail = 1;
      if (debug) {
        Serial.println(F(" ouvre!.. "));
      }
    }
    if (finalString == stringCAf) {
      stateportail = 3;
      if (debug) {
        Serial.println(F(" ferme!.. "));
      }
    }
    actionportail();
  }
}

void counteractionISR(void) {
  countFlag = true;
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

void counteractionISR(void) {
  unsigned long interrupt_time = millis();
 
  if (debug) {
    Serial.println(F("counter pressed!"));
  }
  if (interrupt_time - trigCOUNT > delaybetweencount) {
    if (inrotation) {
      trigCOUNT = millis();
      pressedcountrot();
    } else {
      if (debug) {
        Serial.println(F("not in rotation"));
      }
    }
  } else {
    if (debug) {
      Serial.println(F("NOT OK delay counter"));
    }
  }
  //counterFlag = false;
}

*/
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
        break;
      }
    case 1:
      {
        stateportail = 3;
        break;
      }
    case 2:
      {
        stateportail = 3;
        break;
      }
    case 3:
      {
        stateportail = 1;
        break;
      }
  }
  if (debug) {
    Serial.print(F("new stateportail: "));
    Serial.println(stateportail);
  }
  actionportail();
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

//
void loop1() {
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
  */
  Serial.println("eteint tout");
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
}

void loop() {
  //unsigned long curTime = millis();  // current time
  if (millis() - trigROT > delaybetweentrig) {
    trigROT = millis();
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);
    String res = "";
    if (driver.recv(buf, &buflen)) {
      if (debug) {
        Serial.print("Receive message length: " + String(buflen) + ": ");
      }
      for (int i = 0; i < buflen; i++) {
        if (debug) {
          Serial.write(buf[i]);
        }
        res = res + char(buf[i]);
      }
      if (debug) {
        Serial.println("");
      }
      trigAction(res);
    }
  }
  /*
  if (intFlag) {
    trigAction();
    finalString = "";
    intFlag = false;
  }
  */
  //curTime = millis();
  if (countFlag) {
    if (inrotation && (millis() - trigCOUNT) > delaybetweencount) {
      trigCOUNT = millis();
      if (debug) {
        Serial.println(F("counter pressed!"));
      }
      pressedcountrot();
    }
    countFlag = false;
  }
  //curTime = millis();
  if (inrotation) {
    if (millis() - startROT > delayCurrentDrive) {  // measure after 0.5 sec
      //Serial.println("start measure");
      anaread();
    }
    //curTime = millis();
    if (millis() - startROT > rotStop) {  //stop after rotstop
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
}
