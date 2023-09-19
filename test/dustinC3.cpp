//Dustin C3
#include <ESP32SPISlave.h>

static constexpr uint8_t VSPI_SS {4};  // default: GPIO 5
ESP32SPISlave slave;

static const uint32_t BUFFER_SIZE {8};
uint8_t spi_slave_tx_buf[BUFFER_SIZE] {0};
uint8_t spi_slave_rx_buf[BUFFER_SIZE] {0};
uint8_t spi_slave_rx_buf_previous[BUFFER_SIZE] {0};
uint8_t spi_slave_rx_buf_previous2[BUFFER_SIZE] {0};

static const uint32_t BUFFER_SIZE_SPI = 8; // For SPI Protocol from S3 and C3

void setup() {
    Serial.begin(115200);
    delay(2000);

    // SPI Slave
    // HSPI = CS: 15, CLK: 14, MOSI: 13, MISO: 12 -> default
    // VSPI = CS:  5, CLK: 18, MOSI: 23, MISO: 19
    slave.setDataMode(SPI_MODE0);
    slave.begin(HSPI,3,2,5,4);
    // connect same name pins each other
    // CS - CS, CLK - CLK, MOSI - MOSI, MISO - MISO

    printf("Master Slave Polling Test Start\n");
}

void addChecksum(uint8_t* data, uint8_t Master_Checksum) {
  uint16_t checksum = 0;

  data[0] = Master_Checksum;

  // Calculate the checksum for the first 6 bytes (0 to 5)
  for (int i = 0; i < 6; i++) {
    checksum += data[i];
  }

  // Store the checksum as the last two bytes (6 and 7)
  data[6] = checksum >> 8;   // Upper byte
  data[7] = checksum & 0xFF; // Lower byte
}


bool validateChecksum(uint8_t* data) {
  uint16_t receivedChecksum = (data[6] << 8) | data[7]; // Combine the received checksum bytes into a 16-bit value
  uint16_t calculatedChecksum = 0;

  // Calculate the checksum for the first 6 bytes (0 to 5)
  for (int i = 0; i < 6; i++) {
    calculatedChecksum += data[i];
  }

  // Compare the received checksum with the calculated checksum
  return (receivedChecksum == calculatedChecksum);
}

bool lastchecksum_result = true;
unsigned long value = 0x00;

void loop() {
    // just queue transaction
    // if transaction has completed from master, buffer is automatically updated
    slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);

    // if slave has received transaction data, available() returns size of received transactions
    while (slave.available()) {

        //printf("slave -> master: ");
        for (size_t i = 0; i < slave.size(); ++i) {

            spi_slave_tx_buf[1] = value & 0xFF;
            spi_slave_tx_buf[2] = (value >> 8) & 0xFF;
            spi_slave_tx_buf[3] = (value >> 16) & 0xFF;
            spi_slave_tx_buf[4] = (value >> 24) & 0xFF;
            // memcpy(spi_slave_tx_buf,spi_slave_rx_buf_previous,8);            
            // spi_slave_tx_buf[1]=spi_slave_tx_buf[1]+1;
            addChecksum(spi_slave_tx_buf,validateChecksum(spi_slave_rx_buf));
            //printf("%02X ", spi_slave_tx_buf[i]);
        }
        //printf("------\n"); 

        // printf("master -> slave: ");
        // for (size_t i = 0; i < slave.size(); ++i) {
        //     printf("%02X ", spi_slave_rx_buf[i]);
        //     //memcpy(spi_slave_rx_buf_previous,spi_slave_tx_buf,8);     
        // }
        //printf("\n");
        value++;
        slave.pop();
    }
}
