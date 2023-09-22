void writeSlotBoard(byte slotNumber, byte command, byte slot_type) {

  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);

  byte slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; //select the slot

  char misoData[100];
  for (int i = 0; i < 4; i++) {
    vspi->transfer(slotSelect[i]);
  }
  //delay(1);
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);
  // vspi->endTransaction();
  //delayMicroseconds(5);
  byte colorData[] = {'$', command, slot_type, 5, '\n', 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t checkSum = checksumCalculator(colorData, 5);
  // colorData[5] = checkSum;
  // colorData[6] = checkSum >> 8;

  for (int i = 0; i < 11; i++) {
    misoData[i] = vspi->transfer(colorData[i]);
    // vspi->transfer(colorData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  //delay(1);
  digitalWrite(ESP_D4, LOW);

  Serial.print("From: #");
  Serial.print(slotNumber);
  Serial.print(" - ");

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

  // added by dylan
  // for (int i = 0; i < 4; i++) {
  //   Serial.print(misoData[i],HEX);
  // }

  Serial.print(" [C3 0x");
  uint16_t checksumFromC3 = misoData[0] + (misoData[1] << 8);
  Serial.print(checksumFromC3, HEX);
  Serial.print(" ] [S3 0x");
  Serial.print(checkSum, HEX);
  for (int i = 0; i < 99; i++) {
    passFailArray[i] = passFailArray[i + 1];
  }
  if (checkSum == checksumFromC3) {
    Serial.print("]  PASS  ");
    passFailArray[99] = true;
  }
  else {
    Serial.print("]  FAIL  ");
    passFailArray[99] = false;
  }

  int passCount = 0;
  for (int i = 0; i < 100; i++) {
    if (passFailArray[i])
      passCount++;
  }
  Serial.print(passCount);
  Serial.println("/100");
}
