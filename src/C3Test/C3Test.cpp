// Most up to date code as of 9/18/23
#include <ESP32SPISlave.h>
#include <Adafruit_NeoPixel.h>
#include <ble_ota.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "pin_config.h"
#include "PWM.hpp"
#include "esp_log.h"

static const char *TAG = "DebugTest";

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10


void setup()
{
    Serial.begin(115200);
    pinMode(SLOT_TP1pin, OUTPUT);
    // BLE Configuration
    String slot = "SLOT_";
    String mac = WiFi.macAddress();
    ota_dfu_ble.begin(slot + mac);

}


void loop()
{
    digitalWrite(SLOT_TP1pin, HIGH);
    RGBled.setPixelColor(0,RGBled.Color(200,0,0));
    RGBled.show();
    delay(500);
    RGBled.setPixelColor(0,RGBled.Color(0,200,0));
    RGBled.show();
    delay(500);
    RGBled.setPixelColor(0,RGBled.Color(0,0,200));
    RGBled.show();
    delay(500);
    digitalWrite(SLOT_TP1pin, LOW);
}