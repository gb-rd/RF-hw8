#include <ArduinoMqttClient.h>
#include <WiFi.h>

#include "settings.h"

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = MQTT_BROKER;
int        port     = MQTT_PORT;

const char topicADC[]    = "test/esp32/light";
const char topicDHT_T[]  = "test/esp32/temperature";
const char topicDHT_H[]  = "test/esp32/humidity";
const char topicBTN[]    = "test/esp32/button";
const char topicLED[]    = "test/esp32/led";

const long interval = 1000;
unsigned long previousMillis = 0;

void mqtt_setup() {
  //Initialize serial and wait for port to open:
  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

  // You can provide a unique client ID, if not set the library uses Arduino-millis()
  // Each client must have a unique client ID
  mqttClient.setId(MQTT_ID);

  // You can provide a username and password for authentication
  // mqttClient.setUsernamePassword("username", "password");

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }
  
  mqttClient.onMessage(onMqttMessage);
  mqttClient.subscribe(topicLED);

  Serial.println("Connected to the MQTT broker!");
  Serial.println();
}

void mqtt_send() {
  // call poll() regularly to allow the library to send MQTT keep alives which
  // avoids being disconnected by the broker
  mqttClient.poll();

  // to avoid having delays in loop, we'll use the strategy from BlinkWithoutDelay
  // see: File -> Examples -> 02.Digital -> BlinkWithoutDelay for more info
  unsigned long currentMillis = millis();
  
  if ((currentMillis - previousMillis >= interval) && (valuesUpdated)) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topicBTN);
    mqttClient.print(btnValue);
    mqttClient.endMessage();
    
    mqttClient.beginMessage(topicADC);
    mqttClient.print(adcValue);
    mqttClient.endMessage();

    mqttClient.beginMessage(topicDHT_T);
    mqttClient.print(dhtTValue);
    mqttClient.endMessage();
    
    mqttClient.beginMessage(topicDHT_H);
    mqttClient.print(dhtHValue);
    mqttClient.endMessage();

    valuesUpdated = false;
  }
  
//  if (btnValue != prevBtnValue) {
//
//    prevBtnValue = btnValue;
//  }
}

void mqtt_read() {
    mqttClient.poll();
}

void onMqttMessage(int messageSize) {
  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    ledValue = mqttClient.parseInt();
  }
  
  ledUpdated = true;
}
