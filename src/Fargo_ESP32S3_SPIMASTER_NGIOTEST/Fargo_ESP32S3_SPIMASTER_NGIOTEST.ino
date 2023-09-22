#include <Arduino.h>
#include <EasyCAT.h>
#include <SPI.h>
#include "Wire.h"
#include "XPowersLib.h" //https://github.com/lewisxhe/XPowersLib
XPowersAXP2101 PMU;
void writeSlotBoard(byte slotNumber, byte redCol, byte greenCol, byte blueCol);
uint16_t checksumCalculator(uint8_t *data, uint16_t length);
const int logic_pin = 18;
// FPGA PINS
#define PIN_BTN 0
#define VSPI HSPI // previously fspi, change to hspi
#define PIN_IIC_SDA 38
#define PIN_IIC_SCL 39
#define PIN_PMU_IRQ 40
#define PIN_LED 46
const int ESP_D0 = 1;
const int ESP_D1 = 3;
const int ESP_D2 = 5;
const int ESP_D3 = 6;
const int ESP_D4 = 4;
const int ESP_D5 = 2;
const char SpiCS_Pin = 10; // ethercat SS
static const uint32_t spiClk = 1000000;
SPIClass *vspi = NULL;
bool passFailArray[100];
int current = 0;
int previous = 0;
uint8_t testData = 0;
uint8_t redColor = 255;
uint8_t greenColor = 255;
uint8_t blueColor = 255;
int count = 0;

byte command = 1;
byte slot_type = 1;
void etherCAT_read(){
  // Get values of PDO outputs
  if(EasyCAT_BufferOut.Cust.type != 0){
    slot_type = EasyCAT_BufferOut.Cust.type;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("Start ethercat");
  // EtherCAT SETUP
  pinMode(40, OUTPUT); // EC RST pin
  digitalWrite(40, HIGH);
  delay(100);
  EasyCAT_Init(SpiCS_Pin, ASYNC);
  pinMode(logic_pin, OUTPUT);
  Serial.println("start pmu");
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  bool result = PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, PIN_IIC_SDA, PIN_IIC_SCL);
  if (result == false)
  {
    // Serial.println("PMU is not online...");
    while (1)
      delay(50);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }

  PMU.setDC4Voltage(1200);   // Here is the FPGA core voltage. Careful review of the manual is required before modification.
  PMU.setALDO1Voltage(3300); // BANK0 area voltage
  PMU.setALDO2Voltage(3300); // BANK1 area voltage
  PMU.setALDO3Voltage(3300); // BANK2 area voltage
  PMU.setALDO4Voltage(3300); // BANK3 area voltage

  PMU.enableALDO1();
  PMU.enableALDO2();
  PMU.enableALDO3();
  PMU.enableALDO4();
  PMU.disableTSPinMeasure();
  // delay(1000);
  // vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  pinMode(ESP_D0, OUTPUT);
  pinMode(ESP_D1, OUTPUT); // sck
  pinMode(ESP_D2, OUTPUT); // mosi
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT); // ss
  pinMode(ESP_D5, INPUT);  // miso

  //  digitalWrite(ESP_D1, HIGH);
  //  digitalWrite(ESP_D2, HIGH);
  //  digitalWrite(ESP_D3, HIGH);
  //  digitalWrite(ESP_D4, HIGH);
  //  digitalWrite(ESP_D0, HIGH);
  digitalWrite(ESP_D0, LOW);

  vspi = new SPIClass(VSPI);
  //  vspi->begin(ESP_D1, ESP_D5, ESP_D2, ESP_D4); //SCLK, MISO, MOSI, SS
  //  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
  // Serial.println("Starting...");

  pinMode(ESP_D1, OUTPUT);
  pinMode(ESP_D2, OUTPUT);
  pinMode(ESP_D3, OUTPUT);
  pinMode(ESP_D4, OUTPUT);
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW); // ALL LOW is SMART with OE Enabled

  digitalWrite(ESP_D0, HIGH); // all is applied on rising edge
  digitalWrite(ESP_D1, LOW);
  digitalWrite(ESP_D2, LOW);
  digitalWrite(ESP_D3, LOW);
  digitalWrite(ESP_D4, LOW);
  digitalWrite(ESP_D0, LOW);

  Serial.println("start VSPI");

  vspi->begin(ESP_D1, ESP_D5, ESP_D2);
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));

  Serial.println("END OF SETUP");
}

void loop()
{
  // Serial.println("Start of loop");

  EasyCAT_MainTask();
  // etherCAT_read();

  current = millis();
  int numberOfSlots = 1;
  if (current - previous >= 1000)
  { 
    digitalWrite(logic_pin, HIGH);
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));    
    previous = current;
    
    slot_type++;
    if(slot_type>8){slot_type=1;}
    
    for (int i = 1; i <= numberOfSlots; i++){
      writeSlotBoard(i, command, slot_type);
    }
    digitalWrite(logic_pin, LOW);
  }
  delayMicroseconds(10);
}   