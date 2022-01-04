// 10Nov 2019  LAST UPDATE

// pins
int   countrot  = PD2; // counter of rotation pull up
int   trig      = PD3; // change rotation , trigger action pull up
int   EMA       = PD5; // pwm motor
int   IN1       = PD6; // command motor 1
int   IN2       = PD7; // command motor 2
int   sensor    = A7; // overloadsensor

//A5 SCL
//A4 SDA
// include the library code
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

// var config
int delayloop = 10; // 10ms
double coefd = 1; // 100 for debug, normally 1
int dg = 0
         ; // 1 for left portail, 0 for right, seen from inside court


// var util
int stateportail = 0; // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme,4 ouvert a moitie 5 ferme a moitie
int maxrotationopen, maxrotationclose, calibre, inrotation;
int halfopen, halfclose;
int opendelay, closedelay, minmotorpos, maxmotorpos, minmotor;

int countrotation = 0;
int new_pb, old_pb = 1;
int new_pb1, old_pb1 = 1;
double meas, mesa0, meas1, meas2, measold, maxVCC ;
int varCompteur = 0; // La variable compteur
int topn = 0;
double Voltage = 0;
double Amps = 0;
int mVperAmp;
int ACSoffset;


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
  lcd.setCursor(6, 1); // set the cursor to column 15, line 0
  lcd.print("o");
  lcd.setCursor(7, 1);
  lcd.print(maxrotationopen);
  lcd.setCursor(11, 1); // set the cursor to column 15, line 0
  lcd.print("f");
  lcd.setCursor(12, 1);
  lcd.print(maxrotationclose);
  lcd.setCursor(0, 1); // set the cursor to column 15, line 0
  lcd.print("   ");
  lcd.setCursor(0, 1); // set the cursor to column 15, line 0
  lcd.print(countrotation);
}

void ouvreportail()
{
  varCompteur = 1;
  inrotation = 1;
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  delay(10);
  analogWrite(EMA, 255);
}
void fermeportail()
{
  varCompteur = 1;
  inrotation = 1;
  analogWrite(EMA, 0);
  delay(10);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  delay(10);
  analogWrite(EMA, 255);
}
void reposportail() {
  analogWrite(EMA, 0);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  inrotation = 0;
  varCompteur = 0;
}

void actionportail() {// 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme,4 ouvert a moitie, 5 ferme a moitie
  lcd.setCursor(0, 0); // set the cursor to column 15, line 0
  switch (stateportail) {
    case 0: {
        Serial.println("repos ferme!");
        lcd.print("repferme");
        countrotation = 0;
        reposportail();
        break;
      }
    case 1: {
        Serial.println("ouvre!");
        lcd.print("ouvre!  ");
        delay(opendelay);
        ouvreportail();
        break;
      }
    case 2: {
        Serial.println("repos ouvert!");
        lcd.print("rep ouve");
        reposportail();
        countrotation = 0;
        break;
      }
    case 3: {
        Serial.println("ferme!");
        lcd.print("ferme!  ");
        delay(closedelay);
        fermeportail();
        break;
      }
    case 4: {
        //        if (countrotation < halfopen)
        //        {
        //        Serial.println("a moitie ouvert!");
        //        lcd.print("1/2ouver");
        //        reposportail();
        //        countrotation = maxrotationclose - countrotation;
        //        }
        //        else {
        Serial.println("1/2 ouvert et ferme!");
        lcd.print("1/2o fe!");
        stateportail = 3;
        fermeportail();
        countrotation = maxrotationclose - countrotation;
        //        }
        break;
      }
    case 5: {
        //        if (countrotation < halfclose) {
        //        Serial.println("a moitie ferme!");
        //        lcd.print("1/2ferme");
        //        reposportail();
        //        countrotation = maxrotationopen - countrotation;
        //        }
        //        else {
        Serial.println("a moitie fermé et ouvre!");
        lcd.print("1/2f ou!");
        stateportail = 1;
        ouvreportail();
        countrotation = maxrotationclose - countrotation;
        //        }
        break;
      }
  }
}

void triggeraction (void) { // 0 repos ferme, 1 ouvre, 2 repos ouvert, 3 ferme,4 ouvert a moitie, 5 ferme a moitie
  // run each time action button is pressed
  digitalWrite(LED_BUILTIN, LOW);
  switch (stateportail) {
    case 0: {
        stateportail = 1;
        break;
      }
    case 1: {
        stateportail = 4;
        break;
      }
    case 2 : {
        stateportail = 3;
        break;
      }
    case 3: {
        stateportail = 5;
        break;
      }
    case 4: {
        stateportail = 3;
        break;
      }
    case 5: {
        stateportail = 1;
        break;
      }
  }
  Serial.print("new stateportail: ");
  Serial.println( stateportail);
  actionportail();
}

void anaread ()
{ int
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

    measold = meas;

    if (Voltage < maxVCC) {
      delay(70); // 100 ms
      meas0 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
      delay(120); // 10 ms
      meas1 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
      delay(20); // 10 ms
      meas2 = analogRead(sensor); // Converts and read the analog input value (value from 0.0 to 1.0)
      meas = (meas0 + meas1 + meas2) / 3;
      Voltage = (meas / 1024.0) * 5000; // Gets you mV

      if (Voltage < maxVCC) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.print("overdrive : ");
        Serial.println(maxVCC);
        Serial.print("under maxVCC: ");
        Serial.println(Voltage);


        if (calibre == 2) {
          maxrotationclose = countrotation;
          halfclose = int(0.5 * maxrotationclose);
          minmotorpos = int(0.9 * maxrotationclose);
          //minmotorpos = maxrotationclose - 1;
          stateportail = 0;
          actionportail();
          calibre = 0;
          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
          lcd.print(" cal fer");
          Serial.print("calibration maxrotationoclose: ");
          Serial.println(maxrotationclose);
        }
        if (calibre == 1) {
          maxrotationopen = countrotation;
          //minmotorpos = int(0.1 * maxrot
          halfopen = int(0.5 * maxrotationopen);
          maxmotorpos = int(0.9 * maxrotationopen);
          stateportail = 2;
          actionportail();
          calibre = 2;
          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
          lcd.print(" cal ope");
          Serial.print("calibration maxrotationopen: ");
          Serial.println(maxrotationopen);
        }
        if (stateportail == 1) {
          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
          lcd.print(" 1/2oove");
          Serial.println("overdrive with a moitie ouvert!");
          Serial.print("countrotation : ");
          Serial.println(countrotation);
          stateportail = 4;
          countrotation = maxrotationclose - countrotation;
          reposportail();
        }
        if (stateportail == 3) {
          lcd.setCursor(8, 0); // set the cursor to column 15, line 0
          lcd.print(" 1/2fove");
          Serial.println("overdrive with a moitie ferme!");
          Serial.print("countrotation : ");
          Serial.println(countrotation);
          stateportail = 5;
          countrotation = maxrotationclose - countrotation;
          reposportail();
        }
      }
    }
  }
}
void setup() {
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight
  lcd.clear();
  lcd.setCursor(0, 0); // set the cursor to column 15, line 0
  lcd.print("init and ferme!");
  minmotor = 150; // value when approaching end of run
  Serial.begin(9600);
  Serial.println(dg);
  if (dg == 1) {
    Serial.println("***** portail gauche*********");
    closedelay = 1000;
    opendelay = 0; // 0 second
    maxVCC = 2000;//
  }
  else {
    Serial.println("***** portail droit*********");
    opendelay = 1000; // 1.5 second
    closedelay = 0;
    maxVCC = 2000;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode (countrot, INPUT_PULLUP);
  pinMode (trig, INPUT_PULLUP);
  pinMode (EMA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  Serial.println("moniteur pret");
  countrotation = 0;
  maxrotationopen = 500;
  maxrotationclose = 500;
  minmotorpos = int(0.8 * maxrotationclose);
  maxmotorpos = int(0.8 * maxrotationopen);
  Serial.print("min motor: ");
  Serial.println(minmotorpos);
  Serial.print("max motor: ");
  Serial.println( maxmotorpos);

  Serial.print("min voltage overdrive: ");
  Serial.print(maxVCC);
  Serial.println("mV");

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
  inrotation = 0;
  measold = 0;
}

void loop() {
  /// icurrent read after 1000ms
  if (varCompteur == 0 and inrotation > 0 ) {
    anaread();
  }
  if (varCompteur > 0 ) {
    varCompteur++;
  }
  if (varCompteur > 1000.0 / delayloop) { //1000ms
    varCompteur = 0;

  }
  // stop rotation after 25 second
  if (inrotation > 0 ) {
    inrotation++;
  }
  topn = inrotation / 2.4 / delayloop ;
  if (topn > 0) {
    //Serial.println(topn);
    lcd.setCursor(9, 0); // set the cursor to column 15, line 0
    lcd.print(topn);
    lcd. print(" s");
  }

  if (inrotation > 6800.0 * coefd / delayloop) { //24 second =8000 for coefd=1 and 10ms delay
    inrotation = 0;

    if (calibre == 2) {
      maxrotationclose = countrotation;
      halfclose = int(0.5 * maxrotationclose);
      minmotorpos = int(0.9 * maxrotationclose);
      //minmotorpos = maxrotationclose - 1;
      stateportail = 0;
      actionportail();
      calibre = 0;
      lcd.setCursor(8, 0); // set the cursor to column 15, line 0
      lcd.print(" cal fer");
      Serial.print("calibration maxrotationoclose: ");
      Serial.println(maxrotationclose);
    }
    if (calibre == 1) {
      maxrotationopen = countrotation;
      //minmotorpos = int(0.1 * maxrot
      halfopen = int(0.5 * maxrotationopen);
      maxmotorpos = int(0.9 * maxrotationopen);
      stateportail = 2;
      actionportail();
      calibre = 2;
      lcd.setCursor(8, 0); // set the cursor to column 15, line 0
      lcd.print(" cal ope");
      Serial.print("calibration maxrotationopen: ");
      Serial.println(maxrotationopen);
    }
    Serial.println("stop motor after 25s");
    lcd.setCursor(8, 0); // set the cursor to column 15, line 0
    lcd.print(" timeove");
    if (stateportail == 1) {
      stateportail = 2;
    }
    if (stateportail == 3) {
      stateportail = 0;
    }
    actionportail();
  }
  // count rotation
  if (inrotation > 0 ) {
    new_pb = digitalRead(countrot);
    if ((new_pb == 0) && (old_pb == 1)) {
      //delay(10);//delay(10); // 10 ms
      //new_pb = digitalRead(countrot);
      //if (new_pb == 0) {
      //Serial.println("count rotation pressed");
      pressedcountrot();
      //}
    }
    old_pb = new_pb;
  }

  // change mode by pressing remote controller
  new_pb1 = digitalRead(trig);
  //Serial.println(new_pb1);
  if ((new_pb1 == 0) && (old_pb1 == 1)) {
    Serial.println(" action pressed!");
    lcd.clear();
    triggeraction();
    delay(100); // 100 ms

  }
  old_pb1 = new_pb1;

  delay(delayloop); // 100 ms

}
