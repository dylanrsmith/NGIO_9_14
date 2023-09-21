
// Write to the C3 from the S3

void writeSlotBoardOnce(byte slotNumber, byte command, byte type, bool doREAD, bool processData)
{
  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);

  char slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; // select the slot

  for (int i = 0; i < 4; i++)
  {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  
  byte misoData[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  if (!doREAD)
  {
    byte colorData[] = {command, 0xFF, 0xFF, 0xFF, 0xFF, type, 0xFF, 0xFF};

 
  
    // uint16_t checkSum = 0;
    // int index;
    // for(index = 0; index < 6; index = index+1)
    // {
    //     checkSum = checkSum + colorData[index];
    //     Serial.printf("CheckSum: %d %d %d ..", checkSum, index, colorData[index]);
    // }
    // Serial.println();

    uint16_t checkSum = checksumCalculatorFEI(colorData, 6);
    colorData[6] = checkSum & 0xFF;
    colorData[7] = (checkSum >> 8) & 0xFF;

    Serial.println(checkSum);
    Serial.println("Sending:");
    for (int i = 0; i < 8; i++)
    {
      Serial.printf("%d ", colorData[i]);
    }
    for (int i = 0; i < 8; i++)
    {
      if(processData)
      {
        misoData[i] = vspi->transfer(colorData[i]);
      }
      else
      {
        vspi->transfer(colorData[i]); 
      }
    }
    digitalWrite(ESP_D4, HIGH);
    // delay(1);
    digitalWrite(ESP_D4, LOW);
  }
  else
  {
    // delayMicroseconds(100);
    // for (int i = 0; i < 4; i++)
    // {
    //   vspi->transfer(slotSelect[i]);
    // }
    // digitalWrite(ESP_D4, HIGH);
    // // delay(1);
    // digitalWrite(ESP_D4, LOW);

    // Sending CMD as 0x00 on purpose do that C3 can ignore the message.
    byte ackData[] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, type, 0xFF, 0xFF};
    for (int i = 0; i < 8; i++)
    {
      misoData[i] = vspi->transfer(ackData[i]);
      // Serial.printf("%d ", misoData[i]);
    }
    //Serial.println();
    digitalWrite(ESP_D4, HIGH);
    // delay(1);
    digitalWrite(ESP_D4, LOW);

    processData = true;
  }

  if(processData)
  {
    Serial.println("Receiving:");
    for (int i = 0; i < 8; i++)
    {
      Serial.printf("%d ", misoData[i]);
    }
    Serial.println();

    uint16_t checkSum = checksumCalculatorFEI(misoData, 6);
    uint16_t checksumFromC3 = (misoData[6] | (misoData[7] << 8));
    Serial.printf("calculated cs: %d received cs: %d", checkSum, checksumFromC3);
    Serial.println();
    if (slotNumber == 1) // && (checkSum == checksumFromC3 && checksumFromC3 != 0))
    {
      if ((checkSum == checksumFromC3) && (checksumFromC3 != 0))
      {
        //Serial.println("Checksum Pass");
        uint16_t slot1_val = (misoData[1] | (misoData[2] << 8));
        EasyCAT_BufferIn.Cust.Slot1 = slot1_val;
        EasyCAT_BufferIn.Cust.Slot1_valid++;
        if (misoData[1] != 0x88 || misoData[2] != 0x13)
        {
          // Serial.println("Receiving:");
          // for (int i = 0; i < 8; i++) {
          //   Serial.printf("%d ", misoData[i]);
          // }
          // Serial.println();
        }
      }
      else
      {
        //Serial.println("checksum fail");
        EasyCAT_BufferIn.Cust.Slot1_missed++;
      }
    }
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

  // delay(1);
  //   vspi->endTransaction();
}

// Write to the C3 from the S3

void writeSlotBoard(byte slotNumber, byte command, byte type, byte blueCol)
{

  uint32_t slotMask = 0xFFFFFFFF;
  bitClear(slotMask, slotNumber - 1);

  byte slotSelect[] = {byte(slotMask >> 24), byte(slotMask >> 16), byte(slotMask >> 8), byte(slotMask)}; // select the slot

  for (int i = 0; i < 4; i++)
  {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  // uint8_t colorData[] = {command, 255, 255, 255, 255, type, 255, 255};
  uint8_t colorData[] = {8,7,6,5,4,3,2,1};
  

  uint16_t checkSum = checksumCalculatorCRC(colorData, 6);
  // colorData[6] = checkSum & 0xFF;
  // colorData[7] = (checkSum >> 8) & 0xFF;

  Serial.println("Sending:");
  for (int i = 0; i < 8; i++)
  {
    Serial.printf("%d ", colorData[i]);
  }
  for (int i = 0; i < 8; i++)
  {
    vspi->transfer(colorData[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  delayMicroseconds(100);
  for (int i = 0; i < 4; i++)
  {
    vspi->transfer(slotSelect[i]);
  }
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  // uint8_t ackData[] = {command, 255, 255, 255, 255, type, 255, 255};
  byte ackData[] = {8,7,6,5,4,4,2,1};
  // ackData[6] = checkSum & 0xFF;  // ADD CHECKSUM TO ACK DATA
  // ackData[7] = (checkSum >> 8) & 0xFF;
  
  uint8_t misoData[8];
  for (int i = 0; i < 8; i++)
  {
    misoData[i] = vspi->transfer(ackData[i]);
    // Serial.printf("%d ", misoData[i]);
  }
  Serial.println();
  digitalWrite(ESP_D4, HIGH);
  // delay(1);
  digitalWrite(ESP_D4, LOW);

  Serial.println("Receiving:");
  for (int i = 0; i < 8; i++)
  {
    Serial.printf("%d ", misoData[i]);
  }
  Serial.println();

  checkSum = checksumCalculatorCRC(misoData, 6);
  uint16_t checksumFromC3 = (misoData[6] | (misoData[7] << 8));
  Serial.printf("calculated cs: %d received cs: %d", checkSum, checksumFromC3);
  if (slotNumber == 1) // && (checkSum == checksumFromC3 && checksumFromC3 != 0))
  {
    // if ((checkSum == checksumFromC3) && (checksumFromC3 != 0))
    {
      Serial.println("Checksum Pass");
      uint16_t slot1_val = (misoData[1] | (misoData[2] << 8));
      EasyCAT_BufferIn.Cust.Slot1 = slot1_val;
      EasyCAT_BufferIn.Cust.Slot1_valid++;
      if (misoData[1] != 0x88 || misoData[2] != 0x13)
      {
        // Serial.println("Receiving:");
        // for (int i = 0; i < 8; i++) {
        //   Serial.printf("%d ", misoData[i]);
        // }
        // Serial.println();
      }
    }
    // else
    // {
    //   Serial.println("checksum fail");
    //   EasyCAT_BufferIn.Cust.Slot1_missed++;
    // }
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

  // delay(1);
  //   vspi->endTransaction();
}


// // checksum from sravan
// uint16_t checksumCalculatorFEI(uint8_t * data, uint16_t length)
// {
//    // Our command should not be zero except for ACK from S3 to C3.
//    if(data[0] == 0)
//    {
//       return 0;
//    }
//    // if(data[0] == 0xFF)
//    // {
//    //    return 0;
//    // }
//    uint16_t checkSum = 0;
//    int index;
//    for(index = 0; index < length; index = index+1)
//    {
//       checkSum = checkSum + data[index];
//       // Serial.printf("CheckSum: %d %d %d ..", checkSum, index, data[index]);
//    }
//   //  Serial.println();
   
//    return checkSum;
//    //return ((checkSum >> 8) & 0xFF) | checkSum & 0xFF;
//    //return checkSum & 0xFF | ((checkSum >> 8) & 0xFF);
// }