/*
 * ----------------------------------------------------------------------------
 *            _____ _           _                   _
 *           | ____| | ___  ___| |_ _ __ ___  _ __ (_) ___
 *           |  _| | |/ _ \/ __| __| '__/ _ \| '_ \| |/ __|
 *           | |___| |  __/ (__| |_| | | (_) | | | | | (__
 *           |_____|_|\___|\___|\__|_|  \___/|_| |_|_|\___|
 *            ____                   _   ____
 *           / ___|_      _____  ___| |_|  _ \ ___  __ _ ___
 *           \___ \ \ /\ / / _ \/ _ \ __| |_) / _ \/ _` / __|
 *            ___) \ V  V /  __/  __/ |_|  __/  __/ (_| \__ \
 *           |____/ \_/\_/ \___|\___|\__|_|   \___|\__,_|___/
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <pontus@sweetpeas.se> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Pontus Oldberg
 * ----------------------------------------------------------------------------
 */

#define UTE_TEMP

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <BME280_MOD-1022.h>
#include <Opt3001.h>
#include <PubSubClient.h>
#include<stdlib.h>
#ifdef UTE_TEMP
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#define USE_USCI_B1 
 
#define LED_PIN          5
#define LED_ON           1
#define LED_OFF          0
#define setLed(x)        digitalWrite(LED_PIN,x)
 
#define ONE_SEC          1000000
#define ONE_MIN          (60*ONE_SEC)
 
#define SDA_PIN          2
#define SCL_PIN          14

#ifdef UTE_TEMP
#define ONE_WIRE_BUS 5  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
#endif

 // Update these with values suitable for your network.

#include "mySSID.h"

WiFiClient espClient;
PubSubClient client(espClient);

// Default is that we sleep for this amount of time before waking up again.
uint32_t  deepSleepTime = ONE_MIN * 5; // 5 minutes interval
char msg[50];
 
// The OPT3001 instance
Opt3001 opt3001;
 
// The client instance
//WiFiClient client;

unsigned long tm = 0L;

void setup_wifi() {
  bool bOn = false;
  
  setLed(LED_ON);
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    setLed(bOn ? LED_ON : LED_OFF);
    bOn = !bOn;
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  setLed(LED_OFF);
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void mqtt_connect() {
  // Loop until we're reconnected
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect((char*) clientName.c_str()), mqtt_user, mqtt_passw) {
    //if (client.connect("EnvClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //sprintf(msg, "Hello World #%d", xx++);
      //client.publish("nodeit/environment", msg);
      // ... and resubscribe
      //client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for(int n=0; n<12; n++) {
        setLed(LED_ON);
        delay(200);
        setLed(LED_OFF);
        delay(200);
      }
    }
  }
}

#define MAX_NTEMP 5

void setup() {
  float temp[MAX_NTEMP+1];
  float humidity;
  float pressure;
  uint32_t light;
  int cnt;

  tm = millis();
  
  pinMode(LED_PIN, OUTPUT);
  setLed(LED_OFF);
 
  // First disable watchdog, in case it was enabled above.
  //wdt_disable();

  Wire.begin(SDA_PIN, SCL_PIN);
 
  // Debug out put on the USB Serial port
  Serial.begin(115200);
 
  Serial.println();
  Serial.println();
 
  // We start by connecting to a WiFi network
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  delay(500);
  mqtt_connect();
 
  // Initialize optical sensor
  opt3001.begin();
  DS18B20.begin();
 
  // Make sure we found the BME280
  Serial.print("ChipID = 0x");
  Serial.println(BME280.readChipId(), HEX);
 
  // And do the same for the OPT3001  
  Serial.print("Manufacturer ID = 0x"); 
  Serial.println(opt3001.readManufacturerId(), HEX);
 
  // need to read the NVM compensation parameters
  BME280.readCompensationParams();
 
  // Need to turn on 1x oversampling, default is os_skipped, which means it doesn't measure anything
  BME280.writeOversamplingPressure(os1x);  // 1x over sampling (ie, just one sample)
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
 
  // example of a forced sample.  After taking the measurement the chip goes back to sleep
  BME280.writeMode(smForced);
  delay(50);
  while (BME280.isMeasuring()) {
    Serial.println("Measuring...");
    delay(50);
  }
  Serial.println("Done!");
 
  // read out the data - must do this before calling the getxxxxx routines
  BME280.readMeasurements();
 
  temp[0] = BME280.getTemperature();
  Serial.print("Temp = ");
  Serial.println(temp[0]);  // must get temp first
 
  humidity = BME280.getHumidity();
  Serial.print("Humidity = ");
  Serial.println(humidity);
 
  pressure = BME280.getPressure();
  Serial.print("Pressure = ");
  Serial.println(pressure);
 
  // Read the result from the opt3001
  light = opt3001.readResult();
  Serial.print("LUX = ");
  Serial.println(light);


  if (client.connected()) {
    char topic[32];
    client.loop();
    // Indicate that we are doing the transfer
    setLed(LED_ON); 
    Serial.println(F("WiFi client connected to MQTT !"));

#ifdef UTE_TEMP
    cnt = DS18B20.getDeviceCount();
    tm = millis() - tm;
    sprintf(msg, "cnt = %d, %ld", cnt, tm);
    client.publish("nodeit/status", msg);
    DS18B20.requestTemperatures(); 
    for( int i=0; i < cnt && i < MAX_NTEMP; i++ ) {
      temp[i+1] = DS18B20.getTempCByIndex(i);
    }
    cnt++;
#else
    cnt = 1; // Just one temp ...
#endif

    dtostrf(pressure,1,1,msg);
    client.publish("nodeit/status/pressure", msg);
    for(int i=0; i<cnt; i++) {
      dtostrf(temp[i],1,1,msg);
      sprintf(topic, "nodeit/status/temp%d", i+1);
      client.publish(topic, msg);
    }
    sprintf(msg, "%d", light);
    client.publish("nodeit/status/light", msg);
    dtostrf(humidity,1,1,msg);
    client.publish("nodeit/status/humidity", msg);
   }
  else
  {
    Serial.println(F("Connection to MQTT Failed, trying again in 15 seconds."));
    deepSleepTime = ONE_SEC * 15;
  }
  setLed(LED_OFF);
  delay(100);
 
  // Now we need to adjust the deepSleepTime with the time that it has taken to
  // connect to the WLAN and the time it took to send all data and to actually get
  // to this point.
  uint32_t cTime = micros();
  // Need to make sure that the time elapsed is not bigger than the repeat time
  if (cTime < deepSleepTime) {
    deepSleepTime -= cTime;
  } else {
    deepSleepTime = ONE_SEC * 10;
  }
  Serial.print(F("Going down for deep sleep to wake again in "));
  Serial.print(deepSleepTime);
  Serial.println(F(" microseconds."));
 
  // Now sleep for a while
  ESP.deepSleep(deepSleepTime, WAKE_RF_DEFAULT);
}
 
void loop() {
  // We will never get here.....
}
