/*
 
 === LICENCE NOTICE ===

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.

 === DESCRIPTION ===

 This sketch demonstrates how to use an ESP32 board to publish sensor data 
 to a MQTT server and subscribe to the same MQTT server in order to receive
 commands.
 
 A Bosch environmental sensor (BME280) is used to measure temperature
 pressure and humidity which is then published to an MQTT server.

 A 5V powered relay is used in order to open or close an external circuit 
 (e.g. for opening/closing a garage door)

 The sketch is developed and tested on a GOOUUU-ESP32 board, but will work
 with any other ESP32 board like ESP-WROOM32 Dev Kit etc.


 === PREREQUISITES ===

 An arduino IDE (https://www.arduino.cc)
 Arduino ESP32 development board (e.g. https://www.espressif.com/en/products/hardware/esp-wroom-32/overview) 
 Arduino core for the ESP32 (https://github.com/espressif/arduino-esp32#installation-instructions)
 Bosch Environmental Sensor Unit (BME280) 
 A 5V relay (e.g. SRD-05VDC-SL-C)
 

 === WIRING DIAGRAM ===

                 +----------------------------------+    
                 |                                  |    +--------------+
                 |  +-------------------------+     +----| VCC          |
                 +--| 3v3                 GND |----------| GND          |
                    | EN                   23 |    +-----| SLC    BME   |     
                    | VP                   22 |----+  +--| SDA    280   |
                    | VN   ESP-WROOM-32    TX |       |  | CSB          |
                    | 34                   RX |       |  | SDO          |
                    | 35                   21 |-------+  +--------------+
                    | 32                  GND |--------+
                    | 33                   19 |------+ |
                    | 25                   18 |      | |
                    | 26                    5 |   +--|-|---+
                    | 27                   17 |   | Switch |
                    | 14                   16 |   +--------+
              +-----| 12                    4 |  
   5V Relay   | +---| GND                   0 |  
   +------+   | |   | 13                    2 |
   |    S |---+ |   | D2                   15 |
   |    - |-----+   | D3                   D1 |
   |    + |----+    | CM?    +-------+     D0 |
   + -----+    +----| 5V     |  USB  |    CLK |
                    +- -------+-------+--------+

 === Code summar
 
  - connect to a wireless network 
  - establish an encrypted connection to an MQTT server
  - authenticates to MQTT server 
  - read temperature, pressure and humidity, and publish values to
    mqtt topics (/myhome/garage/temperature etc.}
  - read switch state (0-closed, 1-open), and publish the value to
    mqtt topic /myhome/garage/door 
  - subscribes mqtt topic /myhome/garage/door/set, and control the
    relay according to the command (0=close, 1=open, other=trigger open for 500 ms)
 

 === CREDITS ===

 www.espressif.com (https://github.com/espressif/arduino-esp32)
 Tyler Glen (https://github.com/finitespace/BME280/)
 Nicholas O'Leary (https://github.com/knolleary/pubsubclient)
 
*/

#include <Wire.h>             // Part of Arduino / ESP 32
#include <WiFi.h>             // Part of Arduino / ESP 32
#include <WiFiClientSecure.h> // Part of Arduino / ESP 32
#include <BME280I2C.h>        // Tools -> Manage Libraries... -> BME280 by Tyler Glen
#include <PubSubClient.h>     // Tools -> Manage Libraries... -> PubSubClient by Nick O'Leary
#include "arduino_secrets.h"

// SETTINGS
const char* wifi_ssid = SECRET_WIFI_SSID;
const char* wifi_pass = SECRET_WIFI_PASS;
const char* mqtt_srvr = SECRET_MQTT_SRVR;
const char* mqtt_user = SECRET_MQTT_USER;
const char* mqtt_pass = SECRET_MQTT_PASS;
const char* mqtt_cert = SECRET_MQTT_CERT;
int         mqtt_port = 8883;

const char* mqtt_topic_temperature = "myhome/garage/temperature";
const char* mqtt_topic_pressure =    "myhome/garage/pressure";
const char* mqtt_topic_humidity =    "myhome/garage/humidity";
const char* mqtt_topic_relay_set =   "myhome/garage/door/set";
const char* mqtt_topic_switch =      "myhome/garage/door";
const char* mqtt_topic_heartbeat =   "myhome/garage/heartbeat";
const char* mqtt_topic_wifiquality = "myhome/garage/wifiquality";


int mqtt_heartbeat=0;

int RELAY_PIN = 12;
int SWITCH_PIN= 19;

#define MS_BETWEEN_UPDATES (2000)
#define MSG_BUFFER_SIZE  (256)

WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(wifiClientSecure);
BME280I2C bmeSensor; 

unsigned long lastMsg = 0;
char buf[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {
  delay(10);
  Serial.print("Enable WiFi station mode, disable access point mode.");
  WiFi.mode(WIFI_STA);    
  delay(10);  
  Serial.print("Connecting to ");  
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_tls() {
  wifiClientSecure.setCACert(mqtt_cert);    
}

void setup_bme() {
  delay(10);    // need some time to settle before enabling I2C
  Wire.begin();  
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);    
  while(!bmeSensor.begin()){
    Serial.println("BME280: not found!");
    delay(250);
  }
  Serial.println("BME280: connected successfully.");
}

void mqtt_publish()
{
  float temp(NAN), hum(NAN), pres(NAN), altitude(NAN), dewPoint(NAN);
  bmeSensor.read(pres, temp, hum);
  pres = pres / 100000;

  int dBm = WiFi.RSSI();
  int wifiqual;
  if(dBm <= -100)
      wifiqual = 0;
  else if(dBm >= -50)
      wifiqual = 100;
  else
      wifiqual = 2 * (dBm + 100);  

  char humidity[10];
  char pressure[10];
  char temperature[10];
  char heartbeat[10];
  char switchstate[2];
  char wifiquality[10];
  switchstate[1]='\0';
  if (digitalRead(SWITCH_PIN) == HIGH) {
    switchstate[0] = '1';
  } else {
    switchstate[0] = '0';
  }
  sprintf(wifiquality, "%d", wifiqual);
  sprintf(humidity, "%.1f", hum);
  sprintf(pressure, "%.4f", pres);
  sprintf(temperature, "%.1f", temp);
  sprintf(heartbeat, "%d", mqtt_heartbeat++);
  


  sprintf(buf,"Prepare MQTT publish of heartbeat: %s, temperature: %s°C, humidity: %s %%, pressure: %s atm, switchstate: %s, wifiquality: %s %%",heartbeat, temperature, humidity, pressure, switchstate, wifiquality);
  Serial.println(buf);
  Serial.flush();
  
  mqttClient.publish(mqtt_topic_humidity, humidity);
  mqttClient.publish(mqtt_topic_pressure, pressure);
  mqttClient.publish(mqtt_topic_temperature, temperature);
  mqttClient.publish(mqtt_topic_heartbeat, heartbeat);
  mqttClient.publish(mqtt_topic_switch, switchstate);
  mqttClient.publish(mqtt_topic_wifiquality, wifiquality);

  Serial.println("MQTT publish complete.");
  
}

void mqtt_subscribe(char* topic, byte* payload, unsigned int length) {
  Serial.print("Subscribed MQTT message arrived: [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[0] == '0') {
    digitalWrite(RELAY_PIN, LOW);  
  } else if ((char)payload[0] == '1') {
    digitalWrite(RELAY_PIN, HIGH); 
  } else 
  {
    digitalWrite(RELAY_PIN, HIGH);
    delay(500);
    digitalWrite(RELAY_PIN, LOW);
  }
}

void mqttConnect() {
  int numRetries=0;
  // Loop until connected with mqtt server
  while (!mqttClient.connected()) {
    Serial.print(++numRetries);
    Serial.println(". (Re)trying to connect to MQTT server. ");
    // Set mqtt username as client identifier prefix
    String clientId = mqtt_user;
    // Create a random mqtt suffix (TODO: change with mac address)
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish(mqtt_topic_heartbeat, "0");
      // ... and resubscribe
      if (!mqttClient.subscribe(mqtt_topic_relay_set)) Serial.println("Subscription failed!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      if (numRetries>=10) {
        Serial.println(" WILL RESTART IN 5 SECONDS");
        delay(5000);
        ESP.restart();
      } 
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      
    }
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT); 
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  setup_bme();
  setup_wifi();
  setup_tls();
  mqttClient.setServer(mqtt_srvr, mqtt_port);
  mqttClient.setCallback(mqtt_subscribe);
}

void loop() {
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  unsigned long now = millis();
  if (now - lastMsg > MS_BETWEEN_UPDATES) {  
    lastMsg = now;
    mqtt_publish();
  }
}
