#include <ESP32SPISlave.h>
#include <Adafruit_NeoPixel.h>
#include <ble_ota.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>


Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800); // on pin10
ESP32SPISlave slave;

byte newLEDdata[3];
boolean newDataAvail = false;
static constexpr uint32_t BUFFER_SIZE{32};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];



const int ESP_D1 = 3; // sck io16b
const int ESP_D2 = 5; // mosi io16a
const int ESP_D4 = 4; // ss io13a
const int ESP_D5 = 2; // miso io13b
uint16_t checksumCalculator(uint8_t *data, uint16_t length);

struct Config {
  int slotNumber;
  int slotTypeID;
};

Config config;
const char *filename = "/config.txt";


void loadConfiguration(const char *filename, Config &config) {
  Serial.println(F("Checking Config File"));
  if (SPIFFS.begin(true)) {
    File file = SPIFFS.open(filename, "r");
    StaticJsonDocument<3000> doc;
    DeserializationError error = deserializeJson(doc, file);
    if (error) {
      Serial.println(F("Failed to read file..."));
    }
    config.slotNumber = doc["slotNumber"];
    config.slotTypeID = doc["slotTypeID"];
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
    doc["slotNumber"] = config.slotNumber;
    doc["slotTypeID"] = config.slotTypeID;
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

constexpr uint8_t CORE_TASK_SPI_SLAVE{0};
constexpr uint8_t CORE_TASK_PROCESS_BUFFER{0};

static TaskHandle_t task_handle_wait_spi = 0;
static TaskHandle_t task_handle_process_buffer = 0;

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
    // spi_slave_tx_buf[2] = 0x05;
    // spi_slave_tx_buf[3] = 0xE5;
    // spi_slave_tx_buf[4] = 0x52;

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

  // JSON configuration
  // loadConfiguration(filename, config);

  RGBled.begin();
  RGBled.setPixelColor(0, RGBled.Color(100, 0, 0));
  RGBled.show();
  delay(2000);
  RGBled.setPixelColor(0, RGBled.Color(0, 100, 0));
  RGBled.show();
  delay(100);
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 100));
  RGBled.show();
  delay(100);
  RGBled.setPixelColor(0, RGBled.Color(0, 0, 0));
  RGBled.show();
  delay(2000);

  String slot = "SLOT_";
  String mac = WiFi.macAddress();
  ota_dfu_ble.begin(slot+mac);
  // ota_dfu_ble.begin("SLOT_6");
  // begin() after setting
  // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
  // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
  slave.setDataMode(SPI_MODE0);
  //  pinMode(ESP_D1, INPUT);//sck io16b
  //  pinMode(ESP_D2, INPUT);//mosi io16a
  //  pinMode(ESP_D4, INPUT);//ss io13a
  //  pinMode(ESP_D5, INPUT);//miso io13b
  gpio_set_drive_capability((gpio_num_t)ESP_D5, GPIO_DRIVE_CAP_1);
  slave.begin(SPI2_HOST, ESP_D1, ESP_D5, ESP_D2, ESP_D4); // SCLK, MISO, MOSI, SS
  set_buffer();

  xTaskCreatePinnedToCore(task_wait_spi, "task_wait_spi", 2048, NULL, 2, &task_handle_wait_spi, CORE_TASK_SPI_SLAVE);
  xTaskNotifyGive(task_handle_wait_spi);

  xTaskCreatePinnedToCore(
      task_process_buffer,
      "task_process_buffer",
      2048,
      NULL,
      2,
      &task_handle_process_buffer,
      CORE_TASK_PROCESS_BUFFER);
}

void loop()
{
  if (newDataAvail)
  {
    newDataAvail = false;
    RGBled.setPixelColor(0, RGBled.Color(newLEDdata[0], newLEDdata[1], newLEDdata[2])); // Moderately bright green color.
    RGBled.show();
  }
}
