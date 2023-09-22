#include <Arduino.h>
#include <ESP32SPISlave.h>
#include <ble_ota.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "pin_config.h"
#include <atomic>

Adafruit_NeoPixel RGBled = Adafruit_NeoPixel(1, 10, NEO_GRB + NEO_KHZ800);
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

byte slot_type1_id;
static uint16_t slot_data1_out;
std::atomic<int> data_out(0);


uint16_t checksumCalculator(uint8_t *data, uint16_t length);

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

void recvSerial()
{
  if (Serial.available() > 0)
  {
    int y = Serial.parseInt();
    slot_type1_id = y;
    if (slot_type1_id != 9)
    {
      // config.slot_type1_id_json = slot_type1_id;
      // saveConfiguration(filename, config);
      // delay(100);
      // first_run = true;
    }
    else if (slot_type1_id == 9)
    {
      Serial.println("Rebooting...");
      delay(100);
      ESP.restart();
    }
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
    uint16_t slot_data1_out = data_out.load();
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

  // BLE Configuration
  String slot = "SLOT_";
  String mac = WiFi.macAddress();
  ota_dfu_ble.begin(slot + mac);

  slave.setDataMode(SPI_MODE0);
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
    // my_pwm.begin(true);
    // TBD High bit choses high/low
    break;
  case 6: // FREQ
    // ledcSetup(0, pwmFreq, 8);
    // ledcAttachPin(SLOT_TP1pin, 0);
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

  pinMode(SLOT_TP1pin, OUTPUT);
}

void loop()
{
  digitalWrite(SLOT_TP1pin, HIGH);
  if (newDataAvail)
  {
    newDataAvail = false;
    RGBled.setPixelColor(0, RGBled.Color(newLEDdata[0], newLEDdata[1], newLEDdata[2])); // Moderately bright green color.
    RGBled.show();
  }

      switch (slot_type1_id)
    {
    case 1: // DO-0101	Digital Output
        slot_data1_out = 1000;
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
        slot_data1_out = 5000;
        // dac.setVoltage(500, false);
        // Serial.printf("Analog Output: %d\n", slot_data1_out);
        // TBD use upper bit to persist
        break;
    case 5: // PWM
        slot_data1_out = 4000;
        // slot_data1_out = my_pwm.getValue();
        // SLOT_IO0pin
        // Serial.print("AGE: ");
        // Serial.println(my_pwm.getAge());
        // Serial.printf("PWM Input: %d\n", my_pwm.getValue());
        break;
    case 6: // FREQ
        slot_data1_out = 6000;
        // ledcWrite(0, duty);
        break;
    case 7: // RL-0700	CAN	CAN	7
        slot_data1_out = 7000;  
        break;
    case 8: // RL-0800	Current input
        slot_data1_out = 8000;
    default:
        RGBled.setPixelColor(0, RGBled.Color(255,255,255));
        RGBled.show();
        break;
    }

  data_out.store(slot_data1_out);

  recvSerial();

  digitalWrite(SLOT_TP1pin, LOW);
}
