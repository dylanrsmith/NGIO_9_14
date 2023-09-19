// Most up to date code as of 9/18/23
#include <ESP32SPISlave.h>
#include <Adafruit_NeoPixel.h>
#include <ble_ota.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "pin_config.h"
#include "PWM.hpp"
#include "ResetReason.h"
#include "esp_log.h"
// #include <RBD_Timer.h>

#include "esp_log.h"
static const char *TAG = "DebugTest";

const int slot_swversion1_id = 0;
const int slot_swversion2_id = 1;

// freq out settings
bool DEBUG_FLAG = false;
bool new_type = false;

const int pwmFreq = 500;      // needs to work at 0, max is 20000
const int pwmResolution = 10; // 3;
bool low_speed_freq = false;
int pwmState = LOW;
unsigned long time_now = 0;
unsigned long previousMillis = 0;
unsigned long previousMillisPWM = 0;

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
ESP32SPISlave slave;

byte newLEDdata[3];
boolean newDataAvail = false;
static constexpr uint32_t BUFFER_SIZE{8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

int slot_number; // Possible future use
int slot_type1_id;
int slot_type2_id;
int slot_type3_id;
int slot_type4_id;
int slot_hwversion1_id;
int slot_hwversion2_id;

uint32_t primaryColors[10] = {
    // SWAP RED AND GREEN
    RGBled.Color(0, 0, 0),       // off
    RGBled.Color(0, 255, 0),     // Red
    RGBled.Color(255, 255, 255), // White
    RGBled.Color(0, 128, 128),   // Purple
    RGBled.Color(165, 255, 0),   // Orange
    RGBled.Color(0, 0, 255),     // Blue
    RGBled.Color(255, 0, 0),     // Green
    RGBled.Color(255, 255, 0),   // Yellow
    RGBled.Color(169, 169, 169), // Grey
    RGBled.Color(69, 139, 19)    // Brown
};

uint16_t slot_data1_out;
Adafruit_MCP4725 dac;
PWM my_pwm(SLOT_IO0pin);

bool first_run;
int duty = 128;

uint16_t checksumCalculator(uint8_t *data, uint16_t length);

struct Config
{
    int slot_number_json;
    int slot_type1_id_json;
    int slot_type2_id_json;
    int slot_type3_id_json;
    int slot_type4_id_json;
    int slot_hwversion1_id_json;
    int slot_hwversion2_id_json;
    int slot_swversion3_id_json;
    int slot_swversion4_id_json;
};

Config config;
const char *filename = "/config.txt";

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

void loadConfiguration(const char *filename, Config &config)
{
    Serial.println(F("Loading Config File"));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "r");
        StaticJsonDocument<3000> doc;
        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            Serial.println(F("Failed to read file..."));
        }
        config.slot_number_json = doc["slotNumber"];
        config.slot_type1_id_json = doc["slotType1ID"];
        config.slot_type2_id_json = doc["slotType2ID"];
        config.slot_type3_id_json = doc["slotType3ID"];
        config.slot_type4_id_json = doc["slotType4ID"];
        config.slot_hwversion1_id_json = doc["slotHWVersion1ID"];
        config.slot_hwversion2_id_json = doc["slotHWVersion2ID"];
    }
    else
    {
        Serial.println(F("SPIFFS FAULT"));
    }
}

void saveConfiguration(const char *filename, const Config &config)
{
    Serial.println(F("saving config file..."));
    if (SPIFFS.begin(true))
    {
        File file = SPIFFS.open(filename, "w");
        if (!file)
        {
            Serial.println(F("Failed to save config file..."));
            return;
        }
        StaticJsonDocument<2000> doc;
        doc["slotNumber"] = config.slot_number_json;
        doc["slotType1ID"] = config.slot_type1_id_json;
        doc["slotType2ID"] = config.slot_type2_id_json;
        doc["slotType3ID"] = config.slot_type3_id_json;
        doc["slotType4ID"] = config.slot_type4_id_json;
        doc["slotHWVersion1ID"] = config.slot_hwversion1_id_json;
        doc["slotHWVersion2ID"] = config.slot_hwversion2_id_json;
        if (serializeJson(doc, file) == 0)
        {
            Serial.println(F("Failed to write to file"));
        }
        file.close();
    }
}

void setConfigAndReboot(int type)
{
    Serial.println(type);
    if (type != 9)
    {
        config.slot_type1_id_json = type;
        slot_type1_id = type;
        saveConfiguration(filename, config);
    }
    // first_run = true;
    Serial.println("Rebooting...");
    delay(100);
    ESP.restart();
}

void set_buffer()
{
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        spi_slave_tx_buf[i] = 0;
    }
    memset(spi_slave_rx_buf, 0, BUFFER_SIZE);
}

void task_wait_spi(void *pvParameters)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // block until the transaction comes from master
        slave.wait(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

        xTaskNotifyGive(task_handle_process_buffer);
    }
}

void task_process_buffer(void *pvParameters)
{
    while (1)
    {
        // digitalWrite(SLOT_TP1pin, HIGH);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        slot_data1_out = 5000;
        spi_slave_tx_buf[0] = spi_slave_rx_buf[0];
        spi_slave_tx_buf[1] = slot_data1_out & 0xFF;
        spi_slave_tx_buf[2] = (slot_data1_out >> 8) & 0xFF;
        // Last two bytes are check sum 
        uint16_t checkSum = checksumCalculator(spi_slave_tx_buf,6);
        spi_slave_tx_buf[6] = checkSum & 0xFF;
        spi_slave_tx_buf[7] = (checkSum >> 8) & 0xFF;

        
        uint16_t checkSumFromS3 = checksumCalculator(spi_slave_rx_buf, 6);

        // if checksum is valid
        if(checkSumFromS3 == (spi_slave_rx_buf[6] | (spi_slave_rx_buf[7] << 8)) && checkSumFromS3 != 0)
        {
            if (spi_slave_rx_buf[5] != slot_type1_id && spi_slave_rx_buf[5]< 9)
            {
                slot_type1_id = spi_slave_rx_buf[5];
            }
        }
        slave.pop();

        xTaskNotifyGive(task_handle_wait_spi);
        // digitalWrite(SLOT_TP1pin, LOW);
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(DIGI_OUTpin, OUTPUT);
    pinMode(SLOT_TP1pin, OUTPUT);
    esp_log_level_set(TAG, ESP_LOG_INFO); // Set the log level (you can use ESP_LOG_VERBOSE, ESP_LOG_DEBUG, ESP_LOG_INFO, ESP_LOG_WARN, or ESP_LOG_ERROR)
    // Log a message
    ESP_LOGI(TAG, "Hello, world!");
    first_run = true;

    loadConfiguration(filename, config);
    slot_number = config.slot_number_json;
    slot_type1_id = config.slot_type1_id_json;
    slot_type2_id = config.slot_type2_id_json;
    slot_type3_id = config.slot_type3_id_json;
    slot_type4_id = config.slot_type4_id_json;

    Serial.printf("Slot_type_ids: %d.%d.%d.%d\n", slot_type1_id, slot_type2_id, slot_type3_id, slot_type4_id);
    Serial.printf("Slot version: %d.%d SW:%d.%d\n", slot_hwversion1_id, slot_hwversion2_id, slot_swversion1_id, slot_swversion2_id);
    // Serial.printf("the slot_number is: %d\n", slot_number);

    // BLE Configuration
    String slot = "SLOT_";
    String mac = WiFi.macAddress();
    ota_dfu_ble.begin(slot + mac);

    // S3 SPI
    slave.setDataMode(SPI_MODE0);
    gpio_set_drive_capability((gpio_num_t)MISO_PIN, GPIO_DRIVE_CAP_1);

    // UNIVERSAL IO V1
    slave.begin(SPI2_HOST, CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN); // SCLK, MISO, MOSI, SS
    // slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
    set_buffer();

    xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
    xTaskNotifyGive(task_handle_wait_spi);

    xTaskCreatePinnedToCore(task_process_buffer, "task_process_buffer", 2048, NULL, 2, &task_handle_process_buffer, CORE_TASK_PROCESS_BUFFER);

    // TBD CORE_DEBUG_LEVEL=3
    Serial.printf("##############Value:%d", slot_type1_id);
    switch (slot_type1_id)
    {

    case 1: // DO-0101	Digital Output
        pinMode(SLOT_IO0pin, OUTPUT);
        break;
    case 2: // DI-0200	Digital Input
        pinMode(SLOT_IO0pin, INPUT);
        break;
    case 3: // DI-0300	Analog Input
        pinMode(SLOT_IO0pin, INPUT);
        break;
    case 4: // AO-0400	Analog Output
        // dac.setVoltage(500, false);
        break;
    case 5: // PWM
        // pinMode(SLOT_IO0pin, INPUT);
        my_pwm.begin(true);
        // TBD High bit choses high/low
        break;
    case 6:                       // FREQ
        ledcSetup(0, pwmFreq, 8); // channel,freq,pwm  FOR PWM ONLY
        // ledcAttachPin(DIGI_OUTpin, 0);    // pin, channel , need a pin?
        ledcAttachPin(SLOT_TP1pin, 0);
        break;
    case 7: // RL-0700	CAN	CAN	7
        // TWIA return can count.
        // test send one message with pdo value
        break;
    case 8: // RL-0800	Current input
        // Future Analog read+scale
        break;
    default:
        break;
    }
}

void low_freq(const int pin, const int freq)
{
    unsigned long currentMillisPWM = millis();

    // 500 / freq???
    float period = 1000.0 / freq; // 1000 since we are using milliseconds to keep track
    // float period = 1.0/freq;
    // Serial.printf("period: %f\n",period);
    if (currentMillisPWM - previousMillisPWM >= period)
    {
        // save the last time you blinked the LED
        previousMillisPWM = currentMillisPWM;

        // if the LED is off turn it on and vice-versa:
        if (pwmState == LOW && freq != 0)
        {
            pwmState = HIGH;
        }
        else
        {
            pwmState = LOW;
        }

        // set the LED with the ledState of the variable:
        digitalWrite(pin, pwmState);
    }
}
boolean toggle = false;

unsigned long SerialpreviousMillis = 0;
const unsigned long interval = 1000; // 1 second

void loop()
{
    digitalWrite(SLOT_TP1pin, HIGH);
    // toggle = !toggle;

    if (first_run)
    {
        first_run = false;
        Serial.printf("type: %d",slot_type1_id);
        // TBD use non blocking library Run only on first loop
        for (int i = 0; i < 10; i++)
        {
            // RGBled.setPixelColor(0, RGBled.Color(red, green, blue));
            RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
            RGBled.show();
            delay(100);
            RGBled.setPixelColor(0, primaryColors[int(slot_type1_id)]);
            RGBled.show();
            delay(100);
        }
    }

    RGBled.setPixelColor(0, primaryColors[slot_type1_id]);    
    RGBled.show();

    switch (slot_type1_id)
    {
    case 1: // DO-0101	Digital Output
        // if (slot_type2_id == 1)
        //     digitalWrite(DIGI_OUTpin, spi_slave_rx_buf[1]); // PDO Data
        // else
        //     digitalWrite(DIGI_OUTpin, !spi_slave_rx_buf[1]);
        // Serial.printf("Digital Output: active\n");
        break;
    case 2: // DI-0200	Digital Input
        slot_data1_out = digitalRead(SLOT_IO0pin);
        Serial.printf("Digital Input: %d\n", slot_data1_out);
        break;
    case 3: // DI-0300	Analog Input
        slot_data1_out = analogRead(SLOT_IO0pin);
        Serial.printf("Analog Input: %d\n", slot_data1_out);
        break;
    case 4: // AO-0400	Analog Output
        // dac.setVoltage(500, false);
        // Serial.printf("Analog Output: %d\n", slot_data1_out);
        // TBD use upper bit to persist
        break;
    case 5: // PWM
        // slot_data1_out = my_pwm.getValue();
        // SLOT_IO0pin
        // Serial.print("AGE: ");
        // Serial.println(my_pwm.getAge());
        // Serial.printf("PWM Input: %d\n", my_pwm.getValue());
        break;
    case 6: // FREQ
        // if pdo in <> frequout we are sending
        // setup iwth lcd attached. Frequncy value 2 bytes pwm on time 2 bytes.
        // handle low speed using ms
        // output value.
        ledcWrite(0, duty);
        // Serial.printf("Freq Output duty: %d\n", duty);
        break;
    case 7: // RL-0700	CAN	CAN	7
        // TWIA message count, send message, spi slave.
        // Serial.printf("CAN TBD: %d\n", slot_data1_out);
        break;
    case 8: // RL-0800	Current input
        // Serial.printf("Current Input: %d\n", slot_data1_out);
    case 9: // RL-0800	Current input
        // Serial.printf("Slot_type_ids: %d.%d.%d.%d\n", slot_type1_id, slot_type2_id, slot_type3_id, slot_type4_id);
        // Serial.printf("Slot version: %d.%d SW:%d.%d\n", slot_hwversion1_id, slot_hwversion2_id, slot_swversion1_id, slot_swversion2_id);
        DEBUG_FLAG = !DEBUG_FLAG;
        break;
    default:
        RGBled.setPixelColor(0, primaryColors[1]);
        RGBled.show();
        break;
    }

    if (Serial.available() > 0)
    {
        int y = Serial.parseInt();
        slot_type1_id = y;
        if (slot_type1_id != 9)
        {
            config.slot_type1_id_json = slot_type1_id;
            saveConfiguration(filename, config);
            delay(100);
            first_run = true;
        }
        else if (slot_type1_id == 9)
        {
            Serial.println("Rebooting...");
            delay(100);
            ESP.restart();
        }
    }

    // if (FEIDEBUG)
    // {
    //     unsigned long currentMillis = millis();
    //     if (currentMillis - SerialpreviousMillis >= interval)
    //     {
    //         SerialpreviousMillis = currentMillis;
    //         // Investigate LOGE
    //         Serial.printf("Test turning off logging, for code to execute properly\n");
    //     }
    // }
    digitalWrite(SLOT_TP1pin, LOW);
}