// #include <EasyCAT.h>
#include <Arduino.h>

void writeSlotBoard(byte slotNumber, byte command, byte redCol, byte blueCol) {

  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);


  char slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; //select the slot
  //  for (int i = 0; i < 4; i++)
  //    Serial.print(slotSelect[i], BIN);
  //  Serial.println("");
  //vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  //vspi->transfer(&slotSelect, 4);//
  for (int i = 0; i < 4; i++) {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);
  // vspi->endTransaction();
  //delayMicroseconds(5);
  byte colorData[] = {command, redCol, blueCol, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  //byte colorData[] = {'$', 0x50, 0x51, 0x52, '\n', 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t checkSum = checksumCalculator(colorData, 4);
  colorData[6] = checkSum;
  colorData[7] = checkSum >> 8;
  //char buf3[] = {'$', 0, 0, 100, '\n', 0xFF, 0xFF, 0xFF, 0xFF};
  //vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  char misoData[10];
  for (int i = 0; i < 8; i++) {
    misoData[i] = vspi->transfer(colorData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  Serial.print("From: #");
  Serial.print(slotNumber);
  Serial.print(" - ");
  //  for (int i = 0; i < 4; i++) {
  //    Serial.print(misoData[i],HEX);
  //  }

  //delayMicroseconds(100);
  for (int i = 0; i < 4; i++) {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  byte ackData[] = {'%', '\n', 0xFF, 0xFF, 0xFF, 0xFF};
  for (int i = 0; i < 6; i++) {
    misoData[i] = vspi->transfer(ackData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  Serial.print(" [C3 0x");
  uint16_t checksumFromC3 = misoData[0] + (misoData[1] << 8);
  if(slotNumber == 1)
  {
    // uint32_t slot1_val =  misoData[4] + (misoData[3] << 8) + (misoData[2] << 16);   
    uint16_t slot1_val = (static_cast<uint16_t>(misoData[3]) << 8) | misoData[2];
    // uint8_t slot1_val = misoData[3];

    EasyCAT_BufferIn.Cust.Slot1 = slot1_val;
  }
  Serial.print(checksumFromC3, HEX);
  Serial.print(" ] [S3 0x");
  Serial.print(checkSum, HEX);
  for (int i = 0; i < 99; i++) {
    passFailArray[i] = passFailArray[i + 1];
  }
  if (checkSum == checksumFromC3 && checkSum != 0) {
    Serial.print("]  PASS  ");
    passFailArray[99] = true;
    EasyCAT_BufferIn.Cust.Slot1_valid++;
  }
  else {
    Serial.print("]  FAIL  ");
    passFailArray[99] = false;
    EasyCAT_BufferIn.Cust.Slot1_missed++;
  }

  int passCount = 0;
  for (int i = 0; i < 100; i++) {
    if (passFailArray[i])
      passCount++;
  }
  Serial.print(passCount);
  Serial.println("/100");


  //delay(1);
  //  vspi->endTransaction();

}