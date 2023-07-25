/*
   power down detection
  Rui Santos
  Complete project details at:
   - ESP32: https://RandomNerdTutorials.com/esp32-send-email-smtp-server-arduino-ide/
   - ESP8266: https://RandomNerdTutorials.com/esp8266-nodemcu-send-email-smtp-server-arduino/

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
  Example adapted from: https://github.com/mobizt/ESP-Mail-Client
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP_Mail_Client.h>

#define WIFI_SSID "baleinou"
#define WIFI_PASSWORD "tagada1956"

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
//Week Days
String weekDays[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//Month names
String months[12] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};


/** The smtp host name e.g. smtp.gmail.com for GMail or smtp.office365.com for Outlook or smtp.mail.yahoo.com */
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "sylvain.kri.alerte@gmail.com"
#define AUTHOR_PASSWORD "fxcyezuduiwjfcjf"

/* Recipient's email*/
#define RECIPIENT_EMAIL "sylvain.kri.alerte@gmail.com"
SMTP_Message message;
Session_Config config;

#define INPUT_PIN        0
#define    led_pin     2
int oldInputState;
int oldtime = 0;
int timeToSend = 13;

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status);

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP8266 Power 1 start");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(7200);


  pinMode(INPUT_PIN, INPUT);
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  /*  Set the network reconnection option */
  MailClient.networkReconnect(true);

  /** Enable the debug via Serial port
     0 for no debugging
     1 for basic level debugging

     Debug port can be changed via ESP_MAIL_DEFAULT_DEBUG_PORT in ESP_Mail_FS.h
  */
  smtp.debug(1);

  /* Set the callback function to get the sending results */
  smtp.callback(smtpCallback);

  /* Declare the Session_Config for user defined session credentials */
  // Session_Config config;

  /* Set the session config */
  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";


  /*
    Set the NTP config time
    For times east of the Prime Meridian use 0-12
    For times west of the Prime Meridian add 12 to the offset.
    Ex. American/Denver GMT would be -6. 6 + 12 = 18
    See https://en.wikipedia.org/wiki/Time_zone for a list of the GMT/UTC timezone offsets
  */
  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 2;
  config.time.day_light_offset = 0;
  /* Connect to the server */
  if (!smtp.connect(&config)) {
    ESP_MAIL_PRINTF("Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!smtp.isLoggedIn()) {
    Serial.println("\nNot yet logged in.");
  }
  else {
    if (smtp.isAuthenticated())
      Serial.println("\nSuccessfully logged in.");
    else
      Serial.println("\nConnected with no Auth.");
  }


  /* Declare the message class */


}

void sendmail(String txtMessage, String importance) {
  Serial.print("send message: ");
  Serial.println(txtMessage);
  message.sender.name = F("ESP");
  message.sender.email = AUTHOR_EMAIL;
  message.subject = txtMessage;
  message.addRecipient(F("Sylvain"), RECIPIENT_EMAIL);
  message.text.content = txtMessage.c_str();
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  if (importance == "high" ) {
    message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;
  }
  else {
    message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  }
  //message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;


  /* Connect to the server */
  if (!smtp.connect(&config)) {
    ESP_MAIL_PRINTF("Connection error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!smtp.isLoggedIn()) {
    Serial.println("\nNot yet logged in.");
  }
  else {
    if (smtp.isAuthenticated())
      Serial.println("\nSuccessfully logged in.");
    else
      Serial.println("\nConnected with no Auth.");
  }

  /* Start sending Email and close the session */
  if (!MailClient.sendMail(&smtp, &message))
    ESP_MAIL_PRINTF("Error, Status Code: %d, Error Code: %d, Reason: %s", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
}
void loop() {
  delay(5000);
  timeClient.update();
  String currentDate;

  time_t epochTime = timeClient.getEpochTime();
  //  Serial.print("Epoch Time: ");
  //  Serial.println(epochTime);

  String formattedTime = timeClient.getFormattedTime();
  //  Serial.print("Formatted Time: ");
  //  Serial.println(formattedTime);

  int currentHour = timeClient.getHours();
  //  Serial.print("Hour: ");
  //  Serial.println(currentHour);


  int currentMinute = timeClient.getMinutes();
  //  Serial.print("Minutes: ");
  //  Serial.println(currentMinute);

  int currentSecond = timeClient.getSeconds();
  //  Serial.print("Seconds: ");
  //  Serial.println(currentSecond);

  String weekDay = weekDays[timeClient.getDay()];
  //  Serial.print("Week Day: ");
  //  Serial.println(weekDay);

  //Get a time structure
  struct tm *ptm = gmtime ((time_t *)&epochTime);

  int monthDay = ptm->tm_mday;
  //  Serial.print("Month day: ");
  //  Serial.println(monthDay);

  int currentMonth = ptm->tm_mon + 1;
  //  Serial.print("Month: ");
  //  Serial.println(currentMonth);

  String currentMonthName = months[currentMonth - 1];
  //  Serial.print("Month name: ");
  //  Serial.println(currentMonthName);

  int currentYear = ptm->tm_year + 1900;
  //  Serial.print("Year: ");
  //  Serial.println(currentYear);

  //Print complete date:
  int inputState = digitalRead(INPUT_PIN);
  if (inputState == 0) {
    currentDate = "Power down at: " + String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + "--" + formattedTime;
  }
  else {
    currentDate = "Power up at: " + String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + "--" + formattedTime;
  }

  //  String currentDate = "Awake at: " + String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay) + "--" + formattedTime;
  Serial.println(currentDate);
  Serial.println("");

  if (currentHour == timeToSend && oldtime == 0) {
    oldtime = 1;
    sendmail(currentDate, "low");
  }
  if (currentHour == timeToSend + 1 && oldtime == 1) {
    oldtime = 0;
  }

  if (inputState != oldInputState) {
    if (inputState == 0) {
      sendmail("ALERTE POWER DOWN", "high");
      digitalWrite(led_pin, HIGH);
    } else {
      sendmail("ALERTE POWER IS BACK", "high");
      digitalWrite(led_pin, LOW);
    }
    oldInputState = inputState;
  }
}

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status) {
  /* Print the current status */
  Serial.println(status.info());

  /* Print the sending result */
  if (status.success()) {
    // ESP_MAIL_PRINTF used in the examples is for format printing via debug Serial port
    // that works for all supported Arduino platform SDKs e.g. AVR, SAMD, ESP32 and ESP8266.
    // In ESP8266 and ESP32, you can use Serial.printf directly.

    Serial.println("----------------");
    ESP_MAIL_PRINTF("Message sent success: %d\n", status.completedCount());
    ESP_MAIL_PRINTF("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++)
    {
      /* Get the result item */
      SMTP_Result result = smtp.sendingResult.getItem(i);

      // In case, ESP32, ESP8266 and SAMD device, the timestamp get from result.timestamp should be valid if
      // your device time was synched with NTP server.
      // Other devices may show invalid timestamp as the device time was not set i.e. it will show Jan 1, 1970.
      // You can call smtp.setSystemTime(xxx) to set device time manually. Where xxx is timestamp (seconds since Jan 1, 1970)

      ESP_MAIL_PRINTF("Message No: %d\n", i + 1);
      ESP_MAIL_PRINTF("Status: %s\n", result.completed ? "success" : "failed");
      ESP_MAIL_PRINTF("Date/Time: %s\n", MailClient.Time.getDateTimeString(result.timestamp, "%B %d, %Y %H:%M:%S").c_str());
      ESP_MAIL_PRINTF("Recipient: %s\n", result.recipients.c_str());
      ESP_MAIL_PRINTF("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");

    // You need to clear sending result as the memory usage will grow up.
    smtp.sendingResult.clear();
  }
}
