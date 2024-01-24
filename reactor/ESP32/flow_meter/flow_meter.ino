#include <Arduino.h>
#include <AccelStepper.h>
#include "Adafruit_MPRLS.h"

boolean         newCommand = false;
String          ADDRESS = "r102";
String          inputString = "";
float           setpoint[4] = {0, 0, 0, 0};

uint32_t        sample_per_second = 2;
uint32_t        sample_time = 1000000 / sample_per_second;
float           sample_time_seconds = 1000000 / (float) sample_per_second / 1000000;

// Pressure read vars
uint16_t        PSI_min = 0;
uint16_t        PSI_max = 25;
float           P_factor = 0.068046; //corversion factor psi to atm
float           pressure;
uint16_t        pulses = 0;

//Pins for MPRLS sensor
#define RESET_PIN -1
#define EOC_PIN   -1
// MPRLS SDA_PIN  21
// MPRLS SCL_PIN  22

#define SOLENOID 13

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN, PSI_min, PSI_max, 10, 90, P_factor);

// Timer Callback function
esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;
void read_temp(void *p) {
  READING = true;
}

void setup() {
  Serial.begin(230400);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);
  mpr.begin();

  pinMode(SOLENOID, OUTPUT)

  // Start Timer
  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_st
}


void loop() {
  if(!READING){
    pressure = mpr.readPressure();
    if(pressure > 1.5){
      pulses = pulses + 1;
      digitalWrite(SOLENOID, HIGH);
      digitalWrite(SOLENOID, LOW);
    }
  }

  if(READING){
    Serial.println(pulses);
    pulses = 0;
  }
//  parseSerial();
//  parseString(inputString);
//  inputString = "";
}

//void parseSerial(void){
//  while(Serial.available()){
//    char inChar = char(Serial.read());
//    inputString += inChar;
//    if(inChar == '!'){
//      newCommand = true;
//      break;
//    }
//  }
//}
//
//void parseString(String inputString){
//  if(newCommand){
//    String ADDR = inputString.substring(0, inputString.indexOf(' '));
//
//    if(ADDR == ADDRESS){
//      int firstcomma = inputString.indexOf(',');
//      String cmd = inputString.substring(inputString.indexOf(' '), firstcomma);
//      int command = cmd.toInt();
//
//      if(command == 1){
//        int lastcomma = firstcomma;
//        for (byte i = 0; i < 4; i++){
//          int nextcomma = inputString.indexOf(',', lastcomma + 1);
//          String val = inputString.substring(lastcomma + 1, nextcomma);
//          float value = val.toFloat();
//          setpoint[i] = value;
//          lastcomma = nextcomma;
//        }
//        PUMP_0.setSpeed(setpoint[0]);
//        Serial.println("115!");
//      }
//      else if (command == 2){
//        Serial.println("115!");
//      }
//      else if (command == 3){
//        Serial.println("115!");
//      }
//    }
//
//    inputString = "";
//    newCommand = false;
//  }
//}
