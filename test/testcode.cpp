#include <Wire.h>
#include <Adafruit_MCP4725.h>

// Analog OUTPUT

Adafruit_MCP4725 dac;

#define DAC_RESOLUTION (12)

int sda = 8;
int scl = 9;

void setup(void)
{
  Serial.begin(115200);
  Wire.begin(sda, scl, 400000);
  dac.begin(0x62);
}

int x = 0;
double offset=4095/12;
void loop(void)
{
  uint16_t i;
  delay(1);
  x++;
  if (x == 4095)
  {
    delay(5000);
    x = 0;
    dac.setVoltage(x, false);
    delay(5000);
  }
  dac.setVoltage(x, false);
  Serial.printf("x = %f\n", x/offset);
}