#include <Arduino.h>
#include <AccelStepper.h>

boolean         newCommand = false;
String          ADDRESS = "r102";
String          inputString = "";
float           setpoint[4] = {0, 0, 0, 0};

#define ENABLE          13
#define MS1             12
#define MS2             14
#define MS3             27
#define PUMP_STEP_PIN_0 26
#define DIR_PIN         25

#define MAX_SPEED 1000

AccelStepper PUMP_0(AccelStepper::DRIVER, PUMP_STEP_PIN_0, DIR_PIN);

void setup() {
  Serial.begin(230400);

  pinMode(ENABLE, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  digitalWrite(ENABLE, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  
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
  while(Serial.available()){
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
        Serial.println("115!");
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
