#include <Arduino.h>
#include "MCP_ADC.h"

// Serial buffer
boolean         newCommand = false;
String          ADDRESS = "PIC0";
String          inputString = "";

// Analog read buffer
unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0};
float           reads[8];

// Analog read vars
float           inputVoltage = 3.3;
float           refVoltage = 3.3;

// Analog read parameters
float           sample_number = 0;
uint32_t        sample_time = 1000000/10;
boolean         READING = false;

// Pressure read vars
uint16_t        PSI_min = 0; 
uint16_t        PSI_max = 25;
float           P_factor = 0.068046; //corversion factor psi to atm
float           p = 0;
float           pressure;
float           methane;

//Pins for MCP_3208
#define MCP_DOUT 19
#define MCP_DIN  23
#define MCP_CLK  18
#define CS1       5

//Pins for PWM output
#define PMW_CHN_VALVE_SENSOR 0
#define PMW_CHN_PUMP_SENSOR  1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
#define LEDC_CHANNEL_4 4
#define LEDC_CHANNEL_5 5
#define LEDC_CHANNEL_6 6
#define LEDC_CHANNEL_7 7

#define LEDC_BIT 8

#define LEDC_BASE_FREQ 5000

#define VALVE_SENSOR 13
#define PUMP_SENSOR  12
#define HEATER_PIN_2 14
#define HEATER_PIN_3 27
#define HEATER_PIN_4 26
#define HEATER_PIN_5 25
#define HEATER_PIN_6 33
#define HEATER_PIN_7 32

//Pins for MPRLS sensor
#define RESET_PIN -1
#define EOC_PIN -1

MCP3208 ADC1(MCP_DOUT, MCP_DIN, MCP_CLK);

esp_timer_create_args_t create_args;                                  
esp_timer_handle_t timer_handle;

void read_temp(void *p) {
  READING = true;                                                            // Change flag to enable print
}

void setup() {
  Serial.begin(9600);

  ADC1.begin(CS1); 
  ADC1.setSPIspeed(1000000);

  ledcSetup(PMW_CHN_VALVE_SENSOR, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup( PMW_CHN_PUMP_SENSOR, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_4, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_5, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_6, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_7, LEDC_BASE_FREQ, LEDC_BIT);
  
  ledcAttachPin(VALVE_SENSOR, PMW_CHN_VALVE_SENSOR);
  ledcAttachPin( PUMP_SENSOR,  PMW_CHN_PUMP_SENSOR);
  ledcAttachPin(HEATER_PIN_2, LEDC_CHANNEL_2);
  ledcAttachPin(HEATER_PIN_3, LEDC_CHANNEL_3);
  ledcAttachPin(HEATER_PIN_4, LEDC_CHANNEL_4);
  ledcAttachPin(HEATER_PIN_5, LEDC_CHANNEL_5);
  ledcAttachPin(HEATER_PIN_6, LEDC_CHANNEL_6);
  ledcAttachPin(HEATER_PIN_7, LEDC_CHANNEL_7);
  
  ledcWrite(PMW_CHN_VALVE_SENSOR, 0);
  ledcWrite( PMW_CHN_PUMP_SENSOR, 0);
  ledcWrite(LEDC_CHANNEL_2, 0);
  ledcWrite(LEDC_CHANNEL_3, 0);
  ledcWrite(LEDC_CHANNEL_4, 0);
  ledcWrite(LEDC_CHANNEL_5, 0);
  ledcWrite(LEDC_CHANNEL_6, 0);
  ledcWrite(LEDC_CHANNEL_7, 0);
  delay(500);

  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);


  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {

  if (!READING){
    for (byte i = 0; i < ADC1.channels(); i++) {
      analog[i] += ADC1.analogRead(i);
    }
    sample_number += 1; 
  }

  if (READING) {
    for (byte i = 0; i < ADC1.channels(); i++) {
      float a = analog[i] / sample_number;
      reads[i] = a;
      analog[i] = 0;
    }
    
    sample_number = 0;
    methane = reads[1];
    //printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f\n", reads[0], reads[1], reads[2], reads[3], reads[4], reads[5], reads[6], reads[7], pressure);
    printf("%.2f\n", methane);
    esp_timer_start_once(timer_handle, sample_time);
    READING = false; 
  }
  
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

      //Change Setpoint
      if(command == 1){
//        int lastcomma = firstcomma;
//        for (byte i = 0; i < 8; i++){
//          int nextcomma = inputString.indexOf(',', lastcomma + 1);
//          String val = inputString.substring(lastcomma + 1, nextcomma);
//          float value = val.toFloat();
//          setpoint[i] = value;
//          lastcomma = nextcomma;
//        }
        Serial.println("115!");
      }
      else if (command == 2){
        String val = inputString.substring(inputString.indexOf(',')+1);
        int value = val.toInt();
        Serial.println(value);
        ledcWrite(PMW_CHN_VALVE_SENSOR, value);
      }
      else if (command == 3){
        String val = inputString.substring(inputString.indexOf(',')+1);
        int value = val.toInt();
        Serial.println(value);
        ledcWrite(PMW_CHN_PUMP_SENSOR, value);
      }
    }
    
    inputString = "";
    newCommand = false;
  }
}
