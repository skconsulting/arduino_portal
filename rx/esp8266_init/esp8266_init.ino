/**

   init esp8266 to 9600 bauds

   This program will create a pass-through serial connection to the ESP module. This makes it possible to send AT commands to the device.
   For a new device, you might have to try several connections speeds! (see comment at line 54: "your esp's baud rate might be different")
   Open the Serial Monitor (Tools>Serial Monitor) to send AT-commands.

   Build circuit as detailed here: https://home.et.utwente.nl/slootenvanf/2018/03/17/connect-esp8266-control-blynk/
   Make sure the ESP module gets its own separate power supply of 3.3V

   In the serial monitor set the speed to 115200 (to match the speed set at line 52).
   This program will try sending the command AT+GMR first.
   Does that produce readable information? Then proceed to "Initialize a new device".
   If not, adjust the speed at line 54, upload again and see what happens.

   Initialize a new device:
   Send:
   AT+RST                         # Reset
   AT+UART_DEF=9600,8,1,0,0       # Default baud to 9600,N,8,1,no flow ctl

   After this, set the speed at line 54 to 9600 (only if you changed it), and upload again to send the remaining commands:

   AT+CWMODE=1                    # Station mode (client)
   ATE0                           # Command echo off

   Other commands (which you do not neeed now!)
   Show version info: AT+GMR
   Set it to different speed: AT+UART_DEF=9600,8,1,0,0 (then update sketch) (https://arduino.stackexchange.com/questions/38289/at-command-is-not-responding-on-serial-monitor)
   Search Access Points: AT+CWLAP
   Connect to WIFI: AT+CWJAP="Your Wifi Name","Your Wifi Password"
   Check Allocated Ip: AT+CIFSR

   If this returns an error, the module maybe configured as an access point, try searching for Wifi networks on your laptop, it might show up there, eg. as "AI-THINKER_A3ECE2"
   To turn it into a client and connect to a Wifi network:

   http://www.electronicslovers.com/2017/11/learn-how-to-setup-wifi-module-EspSerial.html

   Code based on https://forum.arduino.cc/index.php?topic=406354.0
   AT command reference: https://room-15.github.io/blog/2015/03/26/EspSerial-at-command-reference/

*/

String NomduReseauWifi = "baleinou"; // Garder les guillements
String MotDePasse      = "tagada1956"; // Garder les guillements


// Connect the TX line from the ESP module to the Arduino's pin 9
// and the RX line from the ESP module to the Arduino's pin 8
// Emulate EspSerial on pins 8/9 if not present

#include "SoftwareSerial.h"
SoftwareSerial ESP8266(9, 8); // RXPin, TXpin seen from arduino TX/RX for esp8266


long int baudrate[] = {9600, 57600, 76800, 115200};

void setup() {

  Serial.begin(9600);
  Serial.println("Start esp8266 init  ...");
  char readReplay[] = "version";
  delay(2000);
  //
  // when bauderate found:
  long int baudrate0 = 9600;
  ESP8266.begin(baudrate0);
  delay(2000);
  Serial.println("Trying to send AT+GMR ...");
  ESP8266.println("AT+GMR");
  delay(3000);
  if (ESP8266.find(readReplay))
  {
    Serial.println("Success " + String(baudrate0));
    ESP8266.println("AT+UART_DEF=9600,8,1,0,0");
    // break;
  }
  else {
    Serial.println("Fail " + String(baudrate0));
  }
  ESP8266.println("AT+UART_DEF=9600,8,1,0,0");
  //delay(2000);
  // rsearch of baudrate
  //  for ( int i = 0; i < 4; i++)
  //  {
  //    Serial.println("try baudrate  ..." + String(baudrate[i]));
  //    ESP8266.begin(baudrate[i]); // your esp's baud rate might be different 9600, 57600, 76800 or 115200
  //    delay(2000);
  //    Serial.println("Trying to send AT+GMR ...");
  //    ESP8266.println("AT+GMR");
  //    //delay(2000);
  //    if (ESP8266.find(readReplay))
  //    {
  //      Serial.println("Success " + String(baudrate[i]));
  //      // break;
  //    }
  //    else {
  //      Serial.println("Fail " + String(baudrate[i]));
  //    }
  //  }

  //initESP8266();
}

void recoitDuESP8266(const int timeout)
{
  String reponse = "";
  long int time = millis();
  while ( (time + timeout) > millis())
  {
    while (ESP8266.available())
    {
      char c = ESP8266.read();
      reponse += c;
    }
  }
  Serial.print(reponse);
}
void initESP8266()
{
  Serial.println("**********************************************************");
  Serial.println("**************** DEBUT DE L'INITIALISATION ***************");
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+RST");
  recoitDuESP8266(2000);
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+CWMODE=1");
  recoitDuESP8266(5000);
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+CWJAP=\"" + NomduReseauWifi + "\",\"" + MotDePasse + "\"");
  recoitDuESP8266(10000);
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+CIFSR");
  recoitDuESP8266(1000);
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+CIPMUX=1");
  recoitDuESP8266(1000);
  Serial.println("**********************************************************");
  envoieAuESP8266("AT+CIPSERVER=1,80");
  recoitDuESP8266(1000);
  Serial.println("**********************************************************");
  Serial.println("***************** INITIALISATION TERMINEE ****************");
  Serial.println("**********************************************************");
  Serial.println("");
}

/****************************************************************/
/*        Fonction qui envoie une commande à l'ESP8266          */
/****************************************************************/
void envoieAuESP8266(String commande)
{
  ESP8266.println(commande);
}
void loop() {

  //  ESP8266.println("AT+GMR");
  //    delay(5000);

  if (ESP8266.available()) // check if the ESP module is sending a message
  {
    while (ESP8266.available())
    {
      // The esp has data so display its output to the serial window
      char c = ESP8266.read(); // read the next character.
      Serial.write(c);
    }
  }

  if (Serial.available()) // check if connection through Serial Monitor from computer is available
  {
    // the following delay is required because otherwise the arduino will read the first letter of the command but not the rest
    // In other words without the delay if you use AT+RST, for example, the Arduino will read the letter A send it, then read the rest and send it
    // but we want to send everything at the same time.
    delay(1000);

    String command = "";

    while (Serial.available()) // read the command character by character
    {
      // read one character
      command += (char)Serial.read();
    }
    ESP8266.println(command); // send the read character to the Esp module
  }
}
