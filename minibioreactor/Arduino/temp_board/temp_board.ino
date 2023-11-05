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
float           output[8];
float           Kp = 140;
float           Ki = 0.004;
float           Kd = 1000;

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

QuickPID PID_0(&temp[0], &output[0], &setpoint[0]);
QuickPID PID_1(&temp[1], &output[1], &setpoint[1]);
QuickPID PID_2(&temp[2], &output[2], &setpoint[2]);
QuickPID PID_3(&temp[3], &output[3], &setpoint[3]);
QuickPID PID_4(&temp[4], &output[4], &setpoint[4]);
QuickPID PID_5(&temp[5], &output[5], &setpoint[5]);
QuickPID PID_6(&temp[6], &output[6], &setpoint[6]);
QuickPID PID_7(&temp[7], &output[7], &setpoint[7]);
QuickPID *allPIDS[8] = {&PID_0, &PID_1, &PID_2, &PID_3, &PID_4, &PID_5, &PID_6, &PID_7};

esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;

void read_temp(void *p) {
  READING = true;                                                            // Change flag to enable print
}

void setup() {
  //Serial.begin(9600);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);

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

  PID_0.SetMode(PID_0.Control::automatic);
  PID_1.SetMode(PID_1.Control::automatic);
  PID_2.SetMode(PID_2.Control::automatic);
  PID_3.SetMode(PID_3.Control::automatic);
  PID_4.SetMode(PID_4.Control::automatic);
  PID_5.SetMode(PID_5.Control::automatic);
  PID_6.SetMode(PID_6.Control::automatic);
  PID_7.SetMode(PID_7.Control::automatic);
  for (byte i = 0; i<8; i++){
    allPIDS[i]->SetOutputLimits(0, 255);
    allPIDS[i]->SetTunings(Kp, Ki, Kd);
    allPIDS[i]->SetSampleTimeUs(sample_time);
  }
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

      allPIDS[i]-> Compute();
    }
    sample_number = 0;

    ledcWrite(LEDC_CHANNEL_0, output[0]);
    ledcWrite(LEDC_CHANNEL_1, output[1]);
    ledcWrite(LEDC_CHANNEL_2, output[2]);
    ledcWrite(LEDC_CHANNEL_3, output[3]);
    ledcWrite(LEDC_CHANNEL_4, output[4]);
    ledcWrite(LEDC_CHANNEL_5, output[5]);
    ledcWrite(LEDC_CHANNEL_6, output[6]);
    ledcWrite(LEDC_CHANNEL_7, output[7]);

    //printf("%f %f %f %f %f %f %f %f\n", a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]);
    //printf("%f %f %f %f %f %f %f %f\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6], temp[7]);
    //printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7]);
    //printf("%f %f\n", temp[1], output[1]);

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
        int lastcomma = firstcomma;
        for (byte i = 0; i < 8; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          setpoint[i] = value;
          lastcomma = nextcomma;
        }
        Serial2.println("115!");
      }
      else if (command == 2){
        Serial2.println("115!");
      }
      else if (command == 3){
        Serial2.println("115!");
        String sample = ADDRESS + " ";
        for(byte i = 0; i < 8; i++){
          sample = sample + "," + (String) temp[i];
        }
        sample = sample + ",115,!";
        Serial2.println(sample);
      }
    }

    inputString = "";
    newCommand = false;
  }
}
