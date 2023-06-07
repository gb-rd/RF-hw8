#include <Arduino.h>

uint8_t adcValue = 0;
uint8_t dhtTValue = 0;
uint8_t dhtHValue = 0;
uint8_t btnValue = 0;
uint8_t prevBtnValue = 0;
uint8_t ledValue = 0;

boolean ledUpdated = false;
boolean valuesUpdated = false;

void setup() {
  serial_setup();
  mqtt_setup();
}

void loop() {
  serial_read();
  mqtt_send();
  mqtt_read();
  if (ledUpdated == true) {
    serial_write();
    ledUpdated = false;
  }
}
