#include <ESP32SPISlave.h>
#include <Adafruit_NeoPixel.h>
#include <ble_ota.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "PWM.hpp"
#include "driver/ledc.h"

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
ESP32SPISlave slave;
PWM SYNC_PIN(9);  // PWM INPUT PIN
Adafruit_MCP4725 dac;


byte newLEDdata[3];
boolean newDataAvail = false;
static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

int slot_number;
int slot_type1_id_json;

uint32_t primaryColors[] = {
  RGBled.Color(255, 0, 0),     // Red
  RGBled.Color(0, 255, 0),     // Green
  RGBled.Color(0, 0, 255),     // Blue
  RGBled.Color(255, 255, 0),   // Yellow
  RGBled.Color(128, 0, 128),   // Purple
  RGBled.Color(255, 165, 0),   // Orange
  RGBled.Color(255, 255, 255), // White
  RGBled.Color(139, 69, 19),   // Brown
  RGBled.Color(169, 169, 169)  // Grey
};


const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b

// freq out settings
const int pwmFreq = 20000;     // needs to work at 0
const int pwmResolution = 10; // 3;
// const int IO_0 = 0;
bool low_speed_freq = false;
int pwmState = LOW;
unsigned long time_now = 0;
unsigned long previousMillis = 0;
unsigned long previousMillisPWM = 0;


uint16_t slot_data1_out;

bool first_run;

int red = 0;
int green = 0;
int blue = 0;

uint16_t checksumCalculator(uint8_t *data, uint16_t length);

struct Config {
  int slot_number_json;
  int slot_type_id_json;
};

Config config;
const char *filename = "/config.txt";

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

void loadConfiguration(const char *filename, Config &config) {
  Serial.println(F("Checking Config File"));
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open(filename, "r");
    StaticJsonDocument<3000> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println(F("Failed to read file..."));
    }
    config.slot_number_json = doc["slotNumber"];
    config.slot_type_id_json = doc["slotTypeID"];
  } else {
    Serial.println(F("SPIFFS FAULT"));
  }
}

void saveConfiguration(const char *filename, const Config &config) {
  Serial.println(F("saving config file..."));
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open(filename, "w");
    if (!file) {
      Serial.println(F("Failed to save config file..."));
      return;
    }
    StaticJsonDocument<2000> doc;
    doc["slotNumber"] = config.slot_number_json;
    doc["slotTypeID"] = config.slot_type_id_json;
    if (serializeJson(doc, file) == 0) {
      Serial.println(F("Failed to write to file"));
    }
    file.close();
  }
}

void set_buffer()
{
  for (uint32_t i = 0; i < BUFFER_SIZE; i++)
  {
    spi_slave_tx_buf[i] = (0xFF - i) & 0xFF;
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // show received data
    //    for (size_t i = 0; i < BUFFER_SIZE; ++i) {
    //      printf("%c ", spi_slave_rx_buf[i]);
    //    }
    //    printf("\n");
    uint16_t checkSum = checksumCalculator(spi_slave_rx_buf, 5);
    spi_slave_tx_buf[0] = checkSum;
    spi_slave_tx_buf[1] = checkSum >> 8;
    spi_slave_tx_buf[2] = slot_data1_out & 0xFF;
    spi_slave_tx_buf[3] = (slot_data1_out >> 8) & 0xFF;

    if (spi_slave_rx_buf[0] == '$' && spi_slave_rx_buf[4] == '\n')
    {
      newDataAvail = true;
      newLEDdata[0] = spi_slave_rx_buf[1];
      newLEDdata[1] = spi_slave_rx_buf[2];
      newLEDdata[2] = spi_slave_rx_buf[3];
    }

    slave.pop();

    xTaskNotifyGive(task_handle_wait_spi);
  }
}


void setup()
{
  Serial.begin(115200);

  first_run = true;

  // JSON configuration
  config.slot_number_json = 1;
  config.slot_type_id_json = 4;
  saveConfiguration(filename,config);
  
  loadConfiguration(filename, config);
  slot_number = config.slot_number_json;
  slot_type1_id_json = config.slot_type_id_json;
  
  Serial.printf("the slot_type_id is: %d\n", slot_type1_id_json);
  Serial.printf("the slot_number is: %d\n", slot_number);

  // Analog Input 
  pinMode(0,INPUT);

  // BLE Configuration
  String slot = "SLOT_";
  String mac = WiFi.macAddress();
  ota_dfu_ble.begin(slot+mac);

  // S3 SPI 
  slave.setDataMode(SPI_MODE0);
  gpio_set_drive_capability((gpio_num_t)ESP_D5, GPIO_DRIVE_CAP_1);
  slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
  set_buffer();

  xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
  xTaskNotifyGive(task_handle_wait_spi);

  xTaskCreatePinnedToCore(task_process_buffer,"task_process_buffer",2048,NULL,2,&task_handle_process_buffer,CORE_TASK_PROCESS_BUFFER);

  switch(slot_type1_id_json){
      case 1:  //DO-0101	Digital Output
        // type 1
        break;
      case 2:   //DI-0200	Digital Input
        break;
      case 3: //DI-0300	Analog Input
        break;
      case 4: //AO-0400	Analog Output        
        break;
    }
  dac.begin(0x62);          
}

void loop()
{
  if(first_run)
  {
    // Run only on first loop
    first_run = false;
    for(int i=0;i<10;i++){
      RGBled.setPixelColor(0, primaryColors[slot_type1_id_json]);
      RGBled.show();  
      delay(100);
      RGBled.setPixelColor(0, primaryColors[0]);
      RGBled.show();  
      delay(100);
    }
    RGBled.setPixelColor(0, primaryColors[slot_type1_id_json]);
    RGBled.show();
  }

  switch(slot_type1_id_json){
    case 1:   //DO-0101	Digital Output
      // type 1
      break;
    case 2:   //DI-0200	Digital Input
      break;
    case 3:   //DI-0300	Analog Input
      break;
    case 4:   //AO-0400	Analog Output
      // dac.setVoltage(pgm_read_word(&(DACLookup_FullSine_9Bit[i])), false);        
      break;
  }

  dac.setVoltage(500, false);        
  if (newDataAvail)
  {
    newDataAvail = false;
    RGBled.setPixelColor(0, RGBled.Color(newLEDdata[0], newLEDdata[1], newLEDdata[2])); // Moderately bright green color.
    RGBled.show();
  }

  // analog_input = analogRead(0);  
  slot_data1_out = 5000;
  Serial.printf("Analog Input: %d\n", slot_data1_out);
}