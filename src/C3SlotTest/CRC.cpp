#include <Arduino.h>
#include <stdint.h>

// CRC From kevin
uint16_t checksumCalculatorCRC(uint8_t * data, uint16_t length)
{
   // Our command should not be zero except for ACK from S3 to C3.
   if(data[0] == 0)//|| data[0] == 255)
   {
      return 0;
   }
   uint16_t curr_crc = 0x0000;
   uint8_t sum1 = (uint8_t) curr_crc;
   uint8_t sum2 = (uint8_t) (curr_crc >> 8);
   int index;
   for(index = 0; index < length; index = index+1)
   {
      sum1 = (sum1 + data[index]) % 255;
      sum2 = (sum2 + sum1) % 255;
   }
   return (sum2 << 8) | sum1;
}

// checksum from sravan
uint16_t checksumCalculatorFEI(uint8_t * data, uint16_t length)
{
   // Our command should not be zero except for ACK from S3 to C3.
   if(data[0] == 0)
   {
      return 0;
   }
   // if(data[0] == 0xFF)
   // {
   //    return 0;
   // }
   uint16_t checkSum = 0;
   int index;
   for(index = 0; index < length; index = index+1)
   {
      checkSum = checkSum + data[index];
      // Serial.printf("CheckSum: %d %d %d ..", checkSum, index, data[index]);
   }
  //  Serial.println();
   
   return checkSum;
   //return ((checkSum >> 8) & 0xFF) | checkSum & 0xFF;
   //return checkSum & 0xFF | ((checkSum >> 8) & 0xFF);
}