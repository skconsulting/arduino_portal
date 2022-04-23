/**********************************************************
  receive my protocol esp8266
*/
//#include <Bounce2.h>
#include <ESP8266WiFi.h>
#include "EIoTCloudRestApiV1.0.h"

#define clk        2
#define data        0
#define AP_USERNAME "baleinou"
#define AP_PASSWORD "tagada1956"
#define TOKEN "2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj"

boolean debug = true;
#define DEBUG_PROG

#ifdef DEBUG_PROG
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)    Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

EIoTCloudRestApi eiotcloud;

int clknew = 1;
const int pow2[] = {1, 2, 4, 8, 16, 32, 64};
const String startOfHeading =    "00000001";
const String endOfTransmission = "00000100";

const int sizeStack = 8; // size of stack
String stack[sizeStack] ;
int pointStack = -1;

bool StartPayload = false;
String res = "";
String finalString = "";
int numreceived = -1;
int clk_old = 1;

const unsigned long postingInterval = 5000; // delay between updates, in milliseconds
unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
unsigned long diffReceive = 800;         // last time you connected to the server, in milliseconds
unsigned long endReceive = 0;         // last time you connected to the server, in milliseconds


void pushString(String ins)
{
  if (pointStack < sizeStack - 1) {
    pointStack += 1;
    stack[pointStack] = ins;
  }
  else {
    for (int i = 0 ; i < pointStack; i ++) {
      stack[i] = stack[i + 1];
    }
    stack[pointStack] = ins;
  }
}

String pullString()
{ String res = "";
  if (pointStack >= 0) {
    res = stack[0];
    for (int i = 0 ; i < pointStack; i ++) {
      //Serial.println(i);
      stack[i] = stack[i + 1];
    }
    pointStack -= 1;
  }
  else {
    res = "Empty";
  }
  return res;
}

void dumpString()
{
  DEBUG_PRINTLN("start dump");
  for (int i = 0 ; i <= pointStack; i++) {
    //Serial.println(i);
    DEBUG_PRINTLN(stack[i]);
  }
  DEBUG_PRINTLN("end of pile");
}

void setup()
{
  pinMode(data, INPUT_PULLUP);
  pinMode(clk, INPUT_PULLUP);
  Serial.begin(74880);
  eiotcloud.begin(AP_USERNAME, AP_PASSWORD);
  eiotcloud.SetToken(TOKEN);
  DEBUG_PRINTLN("esp8266_complet");
  delay(2000);
}

boolean lookForString(String ins)
{
  boolean res = false;
  for (int i = 0 ; i <= pointStack; i++) {
    //Serial.println(i);
    if (ins == stack[i]) {
      res = true;
      break;
    }
  }
  return res;
}
void senstring(String inp) {
  String pid = inp.substring(0, inp.indexOf('_'));
  String command = inp.substring(inp.indexOf('_') + 1, inp.length());
  httpRequest(pid , command);
}

void httpRequest(String parameterId , String command)
{
//  if (debug) {
//    DEBUG_PRINTLN(parameterId);
//    DEBUG_PRINTLN(command);
//  }
  //  String id = "xaQJN6hjY4XS8XGB";
  //  String co = "40.2";
  //  bool valueRet = eiotcloud.SetParameterValue(id, co);
  bool valueRet = eiotcloud.SetParameterValue(parameterId, command);
  lastConnectionTime = millis();
}

void loop1() {
  clknew = digitalRead(clk);
  if (clknew == 0 and clk_old == 1) {
    int the_bit ;
    //trig.update();
    //if (trig.fell()) {
    numreceived += 1;
    if (debug) {
      DEBUG_PRINT("data ");
      DEBUG_PRINTLN(digitalRead(data));
    }
  }
  clk_old = clknew;
}

void loop()
{
  if ((millis() - lastConnectionTime > postingInterval) & (millis() - endReceive  > diffReceive)) {
    if (pointStack >= 0) {
      if (debug) {
        dumpString();
      }
      senstring(pullString());
      //      if (debug) {
      //      Serial.print ("pointStack: ");
      //      Serial.println (pointStack);
      //      }
    }
  }
  clknew = digitalRead(clk);
  if (clknew == 0 and clk_old == 1) {
    int the_bit ;
    //trig.update();
    //if (trig.fell()) {
    numreceived += 1;
    //    if (debug) {
    //      Serial.print("data ");
    //      Serial.println(digitalRead(data));
    //    }
    if (digitalRead(data)) {
      res = '1' + res;
    }
    else {
      res =  '0' + res;
    }
    if (!StartPayload) {
      if (res[0] != startOfHeading[7 - numreceived]) {
        //        if (debug) {
        //          Serial.println("Wrong startOfHeading ");
        //        }
        numreceived = -1;
        res = "";
      }
    }
    //Serial.println(res);
    if (res.length() == 8) {
      //      if (debug) {
      //        DEBUG_PRINT("received : ");
      //        DEBUG_PRINTLN(res);
      //      }
      int dest_char = 0; // the output, 'H' -- must initialize all bits to 0
      for (int source_bit_pos = 7; source_bit_pos >= 0; source_bit_pos--) // start from rightmost position
      {
        if (res[source_bit_pos] == '0') {
          the_bit = 0;
        } else {
          the_bit = 1;
        }
        dest_char += the_bit * pow2[7 - source_bit_pos];
      }

      if (dest_char == 4) {
        StartPayload = false;
        if (debug) {
          DEBUG_PRINT("Final String:");
          DEBUG_PRINTLN(finalString);
        }
        endReceive = millis();
        if (!lookForString(finalString)) {
          pushString(finalString);
        }
        finalString = "";
        numreceived = -1;
      }
      if (StartPayload) {
        if ((dest_char > 31) & (dest_char < 127)) {
          finalString += (char)dest_char;
        }
        else {
          finalString = "";
          numreceived = -1;
          StartPayload = false;
        }
        //Serial.println((char)dest_char);
      }
      if (dest_char == 1 & !StartPayload) {
        // Serial.println((millis() - startReceive) / 1000.);
        StartPayload = true;
      }
      res = "";
    }
  }
  clk_old = clknew;
}
