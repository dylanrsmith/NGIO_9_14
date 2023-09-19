
// Write to the C3 from the S3

void writeSlotBoard(byte slotNumber, byte command, byte type, byte blueCol) {

  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);


  char slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; //select the slot

  for (int i = 0; i < 4; i++) {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);
  
  byte colorData[] = {command, blueCol, 0xFF, 0xFF, 0xFF, type, 0xFF, 0xFF};

  uint16_t checkSum = checksumCalculator(colorData, 6);
  colorData[6] = checkSum & 0xFF;
  colorData[7] = (checkSum >> 8) & 0xFF;
  
  byte misoData[8];
  for (int i = 0; i < 8; i++) {
    vspi->transfer(colorData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  // Serial.print("From: #");
  // Serial.print(slotNumber);
  // Serial.print(" - ");

  //delayMicroseconds(100);
  for (int i = 0; i < 4; i++) {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  // byte ackData[] = {'%', '\n', 0xFF, 0xFF, 0xFF, 0xFF};
  byte ackData[] = {0x00, blueCol, 0xFF, 0xFF, 0xFF, type, 0xFF, 0xFF};
  for (int i = 0; i < 8; i++) {
    misoData[i] = vspi->transfer(ackData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  // Serial.print(" [C3 0x");
  checkSum = checksumCalculator(misoData, 6);
  uint16_t checksumFromC3 = (misoData[6] | (misoData[7] << 8));
  // if(slotNumber == 1 && (checkSum == checksumFromC3 && checksumFromC3 != 0))
  if((checkSum == checksumFromC3) && (checksumFromC3 != 0))
  {
    uint16_t slot1_val = (misoData[1] | (misoData[2] << 8));
    EasyCAT_BufferIn.Cust.Slot1 = slot1_val;
    EasyCAT_BufferIn.Cust.Slot1_valid++;

    if(misoData[1] != 0x88 || misoData[2] != 0x13)
    {      
      for (int i = 0; i < 8; i++) {
        Serial.printf("%d ", misoData[i]);
      }
      Serial.println();
    }
  }
  else
  {
    EasyCAT_BufferIn.Cust.Slot1_missed++;
  }
  // Serial.print(checksumFromC3, HEX);
  // Serial.print(" ] [S3 0x");
  // Serial.print(checkSum, HEX);
  // for (int i = 0; i < 99; i++) {
  //   passFailArray[i] = passFailArray[i + 1];
  // }
  // if (checkSum == checksumFromC3 && checkSum != 0) {
  //   Serial.print("]  PASS  ");
  //   passFailArray[99] = true;
  //   EasyCAT_BufferIn.Cust.Slot1_valid++;
  // }
  // else {
  //   Serial.print("]  FAIL  ");
  //   passFailArray[99] = false;
  //   EasyCAT_BufferIn.Cust.Slot1_missed++;
  // }

  // int passCount = 0;
  // for (int i = 0; i < 100; i++) {
  //   if (passFailArray[i])
  //     passCount++;
  // }
  // Serial.print(passCount);
  // Serial.println("/100");


  //delay(1);
  //  vspi->endTransaction();

}