/*
  Created by Igor Jarc
  See http://iot-playground.com for details
  Please use community fourum on website do not contact author directly

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
*/

#include <ESP8266WiFi.h>
#include "EIoTCloudRestApiV1.0.h"
#include <Wire.h>

#define DEBUG_PROG

#ifdef DEBUG_PROG
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)    Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

EIoTCloudRestApi eiotcloud;
int oldInputState;

#define INPUT_PIN        2


// change those lines
//#define AP_USERNAME "sylvain.kritter@gmail.com"
#define AP_USERNAME "baleinou"
//#define AP_PASSWORD "2@LbFD7H4X7aqyh"
#define AP_PASSWORD "tagada1956"

#define TOKEN "2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj"



String moduleId = "5";
String parameterId = "Wcd5KPhDxCObUdw6";


void setup() {
  Serial.begin(9600);
  DEBUG_PRINTLN("Start...");
  eiotcloud.begin(AP_USERNAME, AP_PASSWORD);
  eiotcloud.SetToken(TOKEN);
  pinMode(INPUT_PIN, INPUT_PULLUP);
  oldInputState = !digitalRead(INPUT_PIN);
}


void loop() {
  int inputState = digitalRead(INPUT_PIN);;

  if (inputState != oldInputState)
  {
    bool valueRet = eiotcloud.SetParameterValue(parameterId, String(inputState));
    DEBUG_PRINT("SetParameterValue: ");
    DEBUG_PRINTLN(valueRet);
    oldInputState = inputState;
  }
}
