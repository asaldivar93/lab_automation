#include <Arduino.h>
#include <Wire.h>
#include "bioprocess.h"

//MPRLS mprls_0(0, 25, 6.89476);
//SEN0322 sen_ox_0(SEN0322_DEFAULT_ADDRESS);
SEN0546 humidity_0 = SEN0546();

void setup() {
  Serial.begin(230400);
  Wire.begin();
}

void loop() {
  Serial.println(humidity_0.read_humidity(), 3);
}
