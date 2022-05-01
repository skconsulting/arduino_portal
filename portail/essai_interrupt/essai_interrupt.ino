// 28 04 2022  LAST UPDATE THE ONE TO be USED AT THIS DATE


int counti = 0;
int acti = 0;
const byte   countrot_pin  = PD2; // counter of rotation pull up
const byte   trig_pin      = PD3; // change rotation , trigger action pull up

unsigned long delaybetweentrig = 2000; // delay before 2 trigger action
unsigned long delaybetweencount = 100; // delay before 2 rot action

volatile boolean triggerFlag = false;
volatile boolean counterFlag = false;

unsigned long trigROT; // delay between 2 triggers
unsigned long curTime; // current time
unsigned long trigCOUNT;

const uint8_t   refsensor    = A7; 
const uint8_t   sensor    = A6; 
double meas, measref, measold, measoldref ;

void triggeractionISR (void) {
  triggerFlag = true;
}

void counteractionISR (void) {
  counterFlag = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("***** essai interrupt****");
  pinMode(countrot_pin, INPUT_PULLUP);
  pinMode(trig_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(countrot_pin), counteractionISR, RISING);
  attachInterrupt(digitalPinToInterrupt(trig_pin), triggeractionISR, RISING);

  trigROT = millis();
  trigCOUNT = millis();
}
void loop() {
  measref = analogRead(refsensor);
  meas = analogRead(sensor);

  delaybetweentrig = measref * 10;
  delaybetweencount = meas * 10;

  if (abs(meas - measold) > 5) {
    Serial.println("count delay: ");
    Serial.println(delaybetweencount);
    measold = meas;
  }

  if (abs(measref - measoldref) > 5) {
    Serial.println("trig delay: ");
    Serial.println(delaybetweentrig);
    measoldref = measref;
  }
  curTime = millis();

  // change mode by pressing remote controller
  if (triggerFlag) {
    triggerFlag = false;
    //Serial.print(" action pressed!.. ");
    if (millis() - trigROT > delaybetweentrig ) {
      trigROT = millis();
      acti += 1;
      Serial.println(acti);
    }
    else {
      //Serial.println("NOT OK");
    }
  }
  if ( counterFlag) {
    counterFlag = false;
    //Serial.print(" counter pressed!...");
    if (millis() - trigCOUNT > delaybetweencount ) {
      trigCOUNT = millis();
      counti += 1;
      Serial.println(counti);
    }
    else {
      //Serial.println("NOT OK COUNTER");
    }
  }
  //delay(1000);
}
