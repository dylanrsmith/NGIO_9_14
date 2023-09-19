#include<Arduino.h>
#include<ble_ota_dfu.hpp>

void setup(){
    Serial.begin(115200);

    ota_dfu_ble.begin("SLOT_9");
}