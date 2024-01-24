#include <Arduino.h>
#include <AccelStepper.h>
#include "Adafruit_MPRLS.h"

boolean         newCommand = false;
String          ADDRESS = "r102";
String          inputString = "";
float           setpoint[4] = {0, 0, 0, 0};

#define MS1       26
#define MS2       25
#define MS3       33
#define STEP      32
#define DIR       35

#define MAX_SPEED 1000

AccelStepper PUMP_0(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(230400);
  //Serial2.begin(230400, SERIAL_8N1, 16, 17);
  mpr.begin();

  pinMode(STEP, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  digitalWrite(MS3, HIGH);

  PUMP_0.setMaxSpeed(MAX_SPEED);
  PUMP_0.setSpeed(setpoint[0]);

}


void loop() {
  PUMP_0.runSpeed();

  parseSerial();
  parseString(inputString);
  inputString = "";
}

void parseSerial(void){
 while(Serial2.available()){
   char inChar = char(Serial.read());
   inputString += inChar;
   if(inChar == '!'){
     newCommand = true;
     break;
   }
 }
}

void parseString(String inputString){
 if(newCommand){
   String ADDR = inputString.substring(0, inputString.indexOf(' '));

   if(ADDR == ADDRESS){
     int firstcomma = inputString.indexOf(',');
     String cmd = inputString.substring(inputString.indexOf(' '), firstcomma);
     int command = cmd.toInt();

     if(command == 1){
       int lastcomma = firstcomma;
       for (byte i = 0; i < 4; i++){
         int nextcomma = inputString.indexOf(',', lastcomma + 1);
         String val = inputString.substring(lastcomma + 1, nextcomma);
         float value = val.toFloat();
         setpoint[i] = value;
         lastcomma = nextcomma;
       }
       PUMP_0.setSpeed(setpoint[0]);
     }
     else if (command == 2){
       Serial.println("115!");
     }
     else if (command == 3){
       Serial.println("115!");
     }
   }

   inputString = "";
   newCommand = false;
 }
}
