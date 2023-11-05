#include <Arduino.h>
#include "MCP_ADC.h"
#include "QuickPID.h"

boolean         newCommand = false;
String          ADDRESS = "tmp";
String          inputString = "";

unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0};
float           temp[8];
float           a[8];
uint32_t        sample_per_second = 1;
uint32_t        sample_time = 1000000 / sample_per_second;
boolean         READING = false;
float           inputVoltage = 3.3;
float           refVoltage = 3.3;
int             resistorReference = 10000;

float           setpoint[] = {-100, -100, -100, -100, -100, -100, -100, -100};


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

MCP3208 ADC1(MCP_DOUT, MCP_DIN, MCP_CLK);

esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;

void read_temp(void *p) {
  READING = true;           // Change flag to enable print
}

void setup() {
  Serial.begin(230400);

  ADC1.begin(CS1);
  ADC1.setSPIspeed(1000000);

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

  ledcWrite(LEDC_CHANNEL_0, 0);
  ledcWrite(LEDC_CHANNEL_1, 0);
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
      a[i] = analog[i] / sample_number;
      float r = resistorReference/((4095*inputVoltage/(a[i]*refVoltage))-1);
      temp[i] = (1 / ( 8.294e-4 + 2.624e-4*log(r) + 1.369e-7*pow(log(r), 3) )) - 273.15;
      analog[i] = 0;
    }
    sample_number = 0;

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

      if(command == 1){
        int lastcomma = firstcomma;
        for (byte i = 0; i < 8; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          setpoint[i] = value;
          lastcomma = nextcomma;
        }
        ledcWrite(LEDC_CHANNEL_0, setpoint[0]);
        ledcWrite(LEDC_CHANNEL_1, setpoint[1]);
        ledcWrite(LEDC_CHANNEL_2, setpoint[2]);
        ledcWrite(LEDC_CHANNEL_3, setpoint[3]);
        ledcWrite(LEDC_CHANNEL_4, setpoint[4]);
        ledcWrite(LEDC_CHANNEL_5, setpoint[5]);
        ledcWrite(LEDC_CHANNEL_6, setpoint[6]);
        ledcWrite(LEDC_CHANNEL_7, setpoint[7]);
        Serial.println("115!");
      }
      else if (command == 2){
        Serial.println("115!");
      }
      else if (command == 3){
        Serial.println("115!");
        String sample = ADDRESS + " ";
        for(byte i = 0; i < 8; i++){
          sample = sample + "," + (String) temp[i];
        }
        sample = sample + ",115,!";
        Serial.println(sample);
      }
    }

    inputString = "";
    newCommand = false;
  }
}
