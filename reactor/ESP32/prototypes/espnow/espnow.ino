#include <WiFi.h>

void setup() {
  Serial.begin(230400);
  WiFi.mode(WIFI_STA);
  Serial.println("");
  Serial.println(WiFi.macAddress());
}

void loop() {
  // put your main code here, to run repeatedly:

}
