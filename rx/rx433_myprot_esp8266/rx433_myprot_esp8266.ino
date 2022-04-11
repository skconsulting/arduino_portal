/**********************************************************
  receive my protocol
*/
//#include <Bounce2.h>
#include "WiFiEsp.h"
#include "SoftwareSerial.h"
//#include "SSVQueueStackArray.h"
boolean debug = false;
boolean wifil = true;
const int  clk = 2;    // clock
const int data = 3;       // data
int clknew = 1;
const int pow2[] = {1, 2, 4, 8, 16, 32, 64};
const String startOfHeading =    "00000001";
const String endOfTransmission = "00000100";

const int sizeStack = 5; // size of stack
String stack[sizeStack] ;
int pointStack = -1;

bool StartPayload = false;
String res = "";
String finalString = "";
int numreceived = -1;
int clk_old = 1;

const char ssid[] = "baleinou";            // your network SSID (name)
const char pass[] = "tagada1956";        // your network password
const String content = "EIOT-AuthToken : 2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj";   //

const unsigned long postingInterval = 1000; // delay between updates, in milliseconds
unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
unsigned long diffReceive = 1000;         // last time you connected to the server, in milliseconds
unsigned long endReceive = 0;         // last time you connected to the server, in milliseconds


int status = WL_IDLE_STATUS;     // the Wifi radio's status

SoftwareSerial esp8266(9, 8);
//Bounce trig = Bounce();
WiFiEspClient client;
//SSVQueueStackArray <String> storage (QUEUE_Storage, //QUEUE_Storage (FIFO) or STACK_Storage (LIFO)
//                                     PTFSA_Overwrite,  //PTFSA_Resize, PTFSA_Overwrite, PTFSA_Ignore
//                                     5 );           //init size;


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
  Serial.println("start dump");
  for (int i = 0 ; i <= pointStack; i++) {
    //Serial.println(i);
    Serial.println(stack[i]);
  }
  Serial.println("end of pile");
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
void setup()
{
  pinMode(data, INPUT_PULLUP);
  pinMode(clk, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println("mycontrl receive");
  //trig.attach(clk, INPUT_PULLUP);
  //trig.interval(1); // debounce interval in ms
  esp8266.begin(9600);
  WiFi.init(&esp8266);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.println("Attempting to connect to WPA SSID ");
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("You're connected to the network");
  printWifiStatus();
}

void senstring(String inp) {
  String pid = inp.substring(0, inp.indexOf('_'));
  String command = inp.substring(inp.indexOf('_') + 1, inp.length());
  httpRequest(pid , command.toFloat());
}

void httpRequest(String Pid , float command)
{
  const char server[] = "cloud.iot-playground.com";
  const String content = "EIOT-AuthToken : 2uUINeYK2uaTUuAPfe1TqK0p1nlAaqcYsKrFFffj";   //
  if (debug) {
    Serial.println(Pid);
    Serial.println(command);
  }
  if (wifil) {
    //  close any connection before send a new request
    // this will free the socket on the WiFi shield
    client.stop();
    // if there's a successful connection
    if (client.connect(server, 40404)) {
      //Serial.println(F("Connecting..."));

      // send the HTTP PUT request
      client.println("POST /RestApi/v1.0/Parameter/" + Pid + "/Value/" + String(command) + " HTTP/1.1");
      client.println("Host: http://cloud.iot-playground.com:40404");
      client.println("Accept: application/json; indent=4");
      client.println("Content-Length: " + String(content.length()));
      client.println(F("Content-Type: application/json"));
      client.println(content);
      client.println();

      lastConnectionTime = millis();
      //client.stop();
    }
    else {
      // if you couldn't make a connection
      Serial.println(F("Connection failed"));
    }
  }
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
  //    trig.update();
  //    if (trig.fell()) {
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

      //      Serial.print("received : ");
      //      Serial.println(res);
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
          Serial.print("Final String:");
          Serial.println(finalString);
        }
        endReceive = millis();
        pushString(finalString);
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
