
// 16 03 2022 with radio for trigger
#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile 
#include <Bounce2.h>
RH_ASK driver;

// pins
uint8_t   countrot_pin  = PD2; // counter of rotation pull up
uint8_t   trig_pin      = PD3; // change rotation , trigger action pull up (back up only, normally radio controlled)
uint8_t   EMA       = PD5; // pwm motor
uint8_t   IN1       = PD6; // command motor 1
uint8_t   IN2       = PD7; // command motor 2
uint8_t   sensor    = A7; // overloadsensor
int   refsensor    = A6; // overloadsensor reference

Bounce countrot = Bounce();
Bounce trig = Bounce();


// var config
// for left portail , seen from inside court
const int rotStop = 20; // stop rotation after 20 seconds
const int opendelay = 0; // 1 second afte command to open
const int closedelay = 1000; // 0 second after command to close
const int delayCurrentDrive = 500; // delay before read motor current in ms
const int minmotor = 150; // value when approaching end of run

// var util
int stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
int maxrotationopen, maxrotationclose, calibre ;
double VoltageRef = 2.0; // ref Voltage from potentiometre

int minmotorpos, maxmotorpos;
bool inrotation;

int countrotation = 0;
double meas, meas0, meas1, meas2, measold , measref;
double Voltage = 5;
int mVperAmp;
int ACSoffset;

uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
uint8_t buflen = sizeof(buf);

const char *msg = "ouvreP";
unsigned long startROT;
unsigned long curROT;

String stringCAo = String("ouvreC");
String stringCAf = String("fermeC");


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
      Serial.print("min motor reached!: ");
      Serial.println( minmotorpos, DEC);
      Serial.print("countrotation : ");
      Serial.println(countrotation);
      analogWrite(EMA, minmotor);
    }
  }
  if (stateportail == 1) {//ouvre
    countrotation++;
    if (countrotation == maxrotationopen) {
      Serial.print("maxrotation reached! completement ouvert!: ");
      Serial.println(maxrotationopen);
      stateportail = 2;
      actionportail();
    }
    if (countrotation == maxmotorpos) {
      Serial.print("max motor reached!: ");
      Serial.println(maxmotorpos);
      Serial.print("countrotation : ");
      Serial.println(countrotation);
      analogWrite(EMA, minmotor);
    }
  }
  Serial.print("rotation count");
  Serial.println(countrotation);
}

void ouvreportail()
{
  inrotation = true;
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  delay(20);
  analogWrite(EMA, 255);
}
void fermeportail()
{
  inrotation = true;
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  delay(20);
  analogWrite(EMA, 255);
}
void reposportail() {
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  inrotation = false;
}

void actionportail() {// 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  switch (stateportail) {
    case 0: {
        Serial.println("repos ferme!");
        countrotation = 0;
        reposportail();
        break;
      }
    case 1: {
        Serial.println("ouvre!");
        delay(opendelay);
        ouvreportail();
        break;
      }
    case 2: {
        Serial.println("repos ouvert!");
        reposportail();
        countrotation = 0;
        break;
      }
    case 3: {
        Serial.println("ferme!");
        delay(closedelay);
        fermeportail();
        break;
      }
  }
}

void triggeraction (void) { // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme
  // run each time action button is pressed
  digitalWrite(LED_BUILTIN, LOW);
  startROT = millis();
  Serial.print("old stateportail: ");
  Serial.println( stateportail);
  switch (stateportail) {
    case 0: {
        stateportail = 1;
        break;
      }
    case 1: {
        stateportail = 3;
        break;
      }
    case 2 : {
        stateportail = 3;
        break;
      }
    case 3: {
        stateportail = 1;
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
  delay(10); // 10 ms
  meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  delay(10); // 10 ms
  meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  meas = (meas0 + meas1 + meas2) / 3;
  // Serial.println(meas );
  if ((meas - measold > 5) || (measold - meas > 5)) {
    //delay(500);
    //Serial.print("measure = : ");
    //Serial.println(meas * (5.0 / 1023.0));
    Voltage = (meas / 1024.0) * 5000; // Gets you mV

    Serial.print("mV = "); // shows the voltage measured
    Serial.println(Voltage );
    Serial.print("Ref mV = "); // shows the voltage measured
    Serial.println(VoltageRef );

    measold = meas;

    if (Voltage < VoltageRef) {
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("overdrive : ");
      Serial.println(VoltageRef);
      Serial.print("over VoltageRef: ");
      Serial.println(Voltage);

      if (calibre == 2) {
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        //minmotorpos = maxrotationclose - 1;
        stateportail = 0;
        actionportail();
        calibre = 0;
        Serial.print("calibration maxrotationoclose: ");
        Serial.println(maxrotationclose);
      }
      if (calibre == 1) {
        maxrotationopen = countrotation;
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
        Serial.print("calibration maxrotationopen: ");
        Serial.println(maxrotationopen);
      }
      if (stateportail == 1) {
        Serial.println("overdrive with a moitie ouvert!");
        Serial.print("countrotation : ");
        Serial.println(countrotation);
        stateportail = 2;
        countrotation = maxrotationclose - countrotation;
        reposportail();
      }
      if (stateportail == 3) {
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
String converter(uint8_t *str) {
  return String((char *)str);
}
void setup() {
  Serial.begin(9600);
  Serial.println("***** module slave *********");
  Serial.println("***** portail gauche*********");

  countrot.attach(countrot_pin, INPUT_PULLUP);
  countrot.interval(10); // debounce interval in ms
  trig.attach(trig_pin, INPUT_PULLUP);
  trig.interval(10); // debounce interval in ms


  pinMode (EMA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  if (!driver.init())
    Serial.println("init failed");
  countrotation = 0;
  maxrotationopen = 500;
  maxrotationclose = 500;
  minmotorpos = 500;
  maxmotorpos = 500;


  Serial.print("opendelay ms: ");
  Serial.println(opendelay);
  Serial.print("closedelay ms: ");
  Serial.println(closedelay);
  stateportail = 0;
  Serial.println("init ferme!");
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  analogWrite(EMA, 0);
  calibre = 1;
  inrotation = false;
  measold = 0;
}

void loop() {
  //  meas = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
  //  Voltage = (meas / 1024.0) * 5000; // Gets you mV
  //  Serial.println(Voltage);

  countrot.update();
  trig.update();

  if (driver.recv(buf, &buflen))
  {
    String mys = converter(buf);
    String sub = mys.substring(0, 6);
    if (sub == stringCAo) {
      if (stateportail == 0 || stateportail == 3) {
        Serial.println("trigger open");
        triggeraction();
      }
    }
    else if (sub == stringCAf) {
      if (stateportail == 2 || stateportail == 1 ) {
        Serial.println("trigger close");
        triggeraction();
      }
    }
  }
  if ( inrotation ) {
    curROT = millis();
    if (curROT - startROT > delayCurrentDrive) {// measure after 0.5 sec
      anaread();
    }
    if (countrot.fell())
    {
      Serial.println("count rotation pressed");
      pressedcountrot();
    }
    if (curROT - startROT > rotStop * 1000) {
      inrotation = false;
      if (calibre == 2) {
        maxrotationclose = max(countrotation, maxrotationopen);
        maxrotationopen = maxrotationclose;
        minmotorpos = int(0.9 * maxrotationclose);
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 0;
        actionportail();
        calibre = 0;
        Serial.print("calibration maxrotationoclose: ");
        Serial.println(maxrotationclose);
      }
      if (calibre == 1) {
        maxrotationopen = countrotation;
        maxmotorpos = int(0.9 * maxrotationopen);
        stateportail = 2;
        actionportail();
        calibre = 2;
        Serial.print("calibration maxrotationopen: ");
        Serial.println(maxrotationopen);


      }
      Serial.println("stop motor after 20s");
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
  }
}
