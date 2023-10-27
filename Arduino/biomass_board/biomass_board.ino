#include <Arduino.h>
#include "MCP_ADC.h"
#include "QuickPID.h"

boolean         newCommand = false;
String          ADDRESS = "bms";
String          inputString = "";

unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0};
float           biomass[8];
float           sample_number = 0;
uint32_t        sample_time = 1000000/4;
boolean         READING = false;
float           inputVoltage = 3.3;
float           refVoltage = 3.3;
int             resistorReference = 10000;

#define MCP_DOUT 21
#define MCP_DIN  22
#define MCP_CLK  19
#define CS1      23

#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
#define LEDC_CHANNEL_4 4
#define LEDC_CHANNEL_5 5
#define LEDC_CHANNEL_6 6
#define LEDC_CHANNEL_7 7

#define LEDC_BIT 8

#define LEDC_BASE_FREQ 5000

#define HEATER_PIN_0 13
#define HEATER_PIN_1 12
#define HEATER_PIN_2 14
#define HEATER_PIN_3 27
#define HEATER_PIN_4 26
#define HEATER_PIN_5 25
#define HEATER_PIN_6 33
#define HEATER_PIN_7 32

//MCP3208 ADC1(MCP_DOUT, MCP_DIN, MCP_CLK);

esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;

void read_temp(void *p) {
  READING = true;                                                            // Change flag to enable print
}

void setup() {
  //Serial.begin(9600);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);

  //ADC1.begin(CS1);
  //ADC1.setSPIspeed(1000000);

  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_4, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_5, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_6, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_7, LEDC_BASE_FREQ, LEDC_BIT);

  ledcAttachPin(HEATER_PIN_0, LEDC_CHANNEL_0);
  ledcAttachPin(HEATER_PIN_1, LEDC_CHANNEL_1);
  ledcAttachPin(HEATER_PIN_2, LEDC_CHANNEL_2);
  ledcAttachPin(HEATER_PIN_3, LEDC_CHANNEL_3);
  ledcAttachPin(HEATER_PIN_4, LEDC_CHANNEL_4);
  ledcAttachPin(HEATER_PIN_5, LEDC_CHANNEL_5);
  ledcAttachPin(HEATER_PIN_6, LEDC_CHANNEL_6);
  ledcAttachPin(HEATER_PIN_7, LEDC_CHANNEL_7);

  ledcWrite(LEDC_CHANNEL_0, 255);
  ledcWrite(LEDC_CHANNEL_1, 255);
  ledcWrite(LEDC_CHANNEL_2, 255);
  ledcWrite(LEDC_CHANNEL_3, 255);
  ledcWrite(LEDC_CHANNEL_4, 255);
  ledcWrite(LEDC_CHANNEL_5, 255);
  ledcWrite(LEDC_CHANNEL_6, 255);
  ledcWrite(LEDC_CHANNEL_7, 255);
  delay(500);

  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {

  if (!READING){
    //for (byte i = 0; i < ADC1.channels(); i++) {
    //  analog[i] += ADC1.analogRead(i);
    //}
    //sample_number += 1;
  }

  if (READING) {
//    for (byte i = 0; i < ADC1.channels(); i++) {
//      float a = analog[i] / sample_number;
//      biomass[i] = a;
//      analog[i] = 0;
//    }
//    sample_number = 0;

    esp_timer_start_once(timer_handle, sample_time);
    READING = false;
  }
  parseSerial();
  parseString(inputString);
  inputString = "";
}

void parseSerial(void){
  while(Serial2.available()){
    char inChar = char(Serial2.read());
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
        Serial2.println("115!");
      }
      else if (command == 2){
        Serial2.println("115!");
      }
      else if (command == 3){
        String sample;
        for(byte i = 0; i < 8; i++){
          sample = ADDRESS + " ," + (String) biomass[i];
        }
        sample = sample + ",115,!";
        Serial2.print(sample);
      }
    }

    inputString = "";
    newCommand = false;
  }
}
