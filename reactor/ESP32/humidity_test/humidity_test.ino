#include <Arduino.h>
#include "Wire.h"
#include "CHT8305.h"

CHT8305 CHT;


void setup()
{
  Serial.begin(115200);

  CHT.begin(0x40);   //  CHT8305_DEFAULT_ADDRESS = 0x40

  delay(1000);
}


void loop()
{
  if (millis() - CHT.lastRead() >= 1000)
  {
    // READ DATA
    CHT.read();

    Serial.print(millis());
    Serial.print('\t');
    Serial.print(CHT.getHumidity());
    Serial.print('\t');
    Serial.println(CHT.getTemperature());
  }
}
