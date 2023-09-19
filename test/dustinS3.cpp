#include <ESP32SPISlave.h>

static constexpr uint8_t VSPI_SS{10}; // default: GPIO 5
SPIClass master(HSPI);

static const uint32_t BUFFER_SIZE{8};
uint8_t spi_master_tx_buf[BUFFER_SIZE]{0};
uint8_t spi_master_rx_buf[BUFFER_SIZE]{0};

static const uint32_t BUFFER_SIZE_SPI = 8; // For SPI Protocol from S3 and C3

void setup()
{
  Serial.begin(115200);
  // delay(10000);
  printf("Master Slave Test\n");

  // SPI Master
  // VSPI = CS: 5, CLK: 18, MOSI: 23, MISO: 19
  pinMode(VSPI_SS, OUTPUT);
  digitalWrite(VSPI_SS, HIGH);
  master.begin(12, 13, 11, 10);
}

void addChecksum(uint8_t *data, uint8_t Master_Checksum)
{
  uint16_t checksum = 0;

  data[0] = Master_Checksum;

  // Calculate the checksum for the first 6 bytes (0 to 5)
  for (int i = 0; i < 6; i++)
  {
    checksum += data[i];
  }
 
  // Store the checksum as the last two bytes (6 and 7)
  data[6] = checksum >> 8;   // Upper byte
  data[7] = checksum & 0xFF; // Lower byte
}

bool validateChecksum(uint8_t *data)
{
  uint16_t receivedChecksum = (data[6] << 8) | data[7]; // Combine the received checksum bytes into a 16-bit value
  uint16_t calculatedChecksum = 0;

  // Calculate the checksum for the first 6 bytes (0 to 5)
  for (int i = 0; i < 6; i++)
  {
    calculatedChecksum += data[i];
  }

  // Compare the received checksum with the calculated checksum
  return (receivedChecksum == calculatedChecksum);
}

bool isAllZerosOrFFs(uint8_t* array, size_t size) {
  for (size_t i = 0; i < size; i++) {
    if (array[i] != 0x00 && array[i] != 0xFF) {
      // If an element is neither 0x00 nor 0xFF, return false
      return false;
    }
  }
  // If all elements are either 0x00 or 0xFF, return true
  return true;
}

byte value = 0;
unsigned long mastererrors = 0;
unsigned long slaveerrors = 0;
unsigned long mxvalue = 0;
unsigned long msgcount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second interval in milliseconds
int Hz = 100000;
int usdelay = 250;

void loop()
{

  unsigned long currentMillis = millis();
  Hz=4000000;
  master.beginTransaction(SPISettings(Hz, MSBFIRST, SPI_MODE0));

  digitalWrite(VSPI_SS, LOW);
  mxvalue++;
  spi_master_tx_buf[1] = mxvalue & 0xFF;
  spi_master_tx_buf[2] = (mxvalue >> 8) & 0xFF;
  spi_master_tx_buf[3] = (mxvalue >> 16) & 0xFF;
  spi_master_tx_buf[4] = (mxvalue >> 24) & 0xFF;
  addChecksum(spi_master_tx_buf, 2);
  master.transferBytes(spi_master_tx_buf, spi_master_rx_buf, BUFFER_SIZE);

  // Or you can transfer like this
  // for (size_t i = 0; i < BUFFER_SIZE; ++i)
  //     spi_master_rx_buf[i] = master.transfer(spi_master_tx_buf[i]);
  digitalWrite(VSPI_SS, HIGH);
  master.endTransaction();

  // printf(" slave master ->: ");
  // for (size_t i = 0; i < 8; ++i) {
  //     printf("%02X ", spi_master_rx_buf[i]);
  // }
  // printf("\n");

  if (!validateChecksum(spi_master_rx_buf))
    slaveerrors++;

  if (spi_master_rx_buf[0] == 0 || isAllZerosOrFFs(spi_master_rx_buf, BUFFER_SIZE))
  {
    mastererrors++;
  }

  msgcount++;

  if (currentMillis - previousMillis >= interval)
  {

    previousMillis = currentMillis;

    // printf(" slave -> master : ");
    // for (size_t i = 0; i < 8; ++i) {
    //     printf("%02X ", spi_master_rx_buf[i]);
    // }
    // printf("\n");
    //usdelay = 200;

    printf(" MasterC?:%d SlaveC?:%d Hz: %d Count:%d DelayUs:%d\n", mastererrors, slaveerrors, Hz, msgcount, usdelay);

    if (Hz<1000000)
    {
          Hz = Hz + 100000;
    }
    else{
          Hz = Hz + 1000000;
    }
    mastererrors = 0;
    slaveerrors = 0;
    if (Hz >= 70000000)
    {
      Hz = 1000000;
    }
    msgcount = 0;

    usdelay=usdelay-5;
    if (usdelay == 0)
    {
      usdelay = 250;
    }    
  }
  // delay(10000); // 100
  //usdelay = 50;
    delayMicroseconds(usdelay);
}



// bool areAllElementsIdentical(int arr[], int size, int referenceElement) {
//     if (size <= 1) {
//         // If the array has 0 or 1 element, they are considered identical.
//         return true;
//     }

//     int referenceElement = arr[0]; // Choose the first element as the reference.

//     for (int i = 1; i < size; ++i) {
//         if (arr[i] != referenceElement) {
//             // If any element is different from the reference, return false.
//             return false;
//         }
//     }

//     // If all elements match the reference, return true.
//     return true;
// }


