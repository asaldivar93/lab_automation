#include <Arduino.h>
#include "MCP_ADC.h"

String          inputString = "";
String          slaveString = "";
String          ADDRESS = "mas";
boolean         newCommand = false;
boolean         comm_confirmed = false;

float           rpm_setpoint[] = {0, 0, 0, 0, 0, 0, 0, 0};
float           rpm[8];
float           tmp_setpoint[] = {-100, -100, -100, -100, -100, -100, -100, -100};
float           tmp[8];
float           biomass[8];

float           sample_time = 1000000;
float           sample_number = 0;
unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0};
boolean         READING = false;

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
  READING = true; // Change flag to enable print
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);
  delay(500);
  Serial.println("\nInitializing Boards");
  check_board("str");
  update_rpm_sp(rpm_setpoint);
  check_board("tmp");
  update_tmp_sp(tmp_setpoint);

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
    for (byte i = 0; i < ADC1.channels(); i++) {
     analog[i] += ADC1.analogRead(i);
    }
    sample_number += 1;
  }

  if (READING) {
    for (byte i = 0; i < ADC1.channels(); i++) {
      float a = analog[i] / sample_number;
      biomass[i] = a;
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

void parseSerial2(void){
  while(Serial2.available()){
    char inChar = char(Serial2.read());
    slaveString += inChar;
    if(inChar == '!'){
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
        while(!comm_confirmed){
          Serial2.println("str 3,!");
          parseSerial2();
          int slave_ans = slaveString.toInt();
          if( slave_ans == 115){
            comm_confirmed = true;
          }
        }

        slaveString = "";
        comm_confirmed = false;

        while(!comm_confirmed){
          parseSerial2();
          String ADDR = slaveString.substring(0, inputString.indexOf(' '));

          int firstcomma = slaveString.indexOf(',');
          int lastcomma = firstcomma;
          for (byte i = 0; i < 8; i++){
            int nextcomma = slaveString.indexOf(',', lastcomma + 1);
            String val = slaveString.substring(lastcomma + 1, nextcomma);
            float value = val.toFloat();
            rpm[i] = value;
            lastcomma = nextcomma;
          }
          int nextcomma = slaveString.indexOf(',', lastcomma + 1);
          String val = slaveString.substring(lastcomma + 1, nextcomma);
          int value = val.toInt();

          if(value == 115){
            comm_confirmed=true;
            Serial2.println("115!");
          }
        }
        comm_confirmed = false;
        slaveString = "";

        while(!comm_confirmed){
          Serial2.println("tmp 3,!");
          parseSerial2();
          int slave_ans = slaveString.toInt();
          if( slave_ans == 115){
            comm_confirmed = true;
          }
        }

        slaveString = "";
        comm_confirmed = false;

        while(!comm_confirmed){
          parseSerial2();
          String ADDR = slaveString.substring(0, inputString.indexOf(' '));

          int firstcomma = slaveString.indexOf(',');
          int lastcomma = firstcomma;
          for (byte i = 0; i < 8; i++){
            int nextcomma = slaveString.indexOf(',', lastcomma + 1);
            String val = slaveString.substring(lastcomma + 1, nextcomma);
            float value = val.toFloat();
            tmp[i] = value;
            lastcomma = nextcomma;
          }
          int nextcomma = slaveString.indexOf(',', lastcomma + 1);
          String val = slaveString.substring(lastcomma + 1, nextcomma);
          int value = val.toInt();

          if(value == 115){
            comm_confirmed=true;
            Serial2.println("115!");
          }
        }
        comm_confirmed = false;
        slaveString = "";
        Serial.println(rpm[0]);
        Serial.println(tmp[0]);
      }
      else if(command == 2){
        int lastcomma = firstcomma;
        for (byte i = 0; i < 8; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          rpm_setpoint[i] = value;
          lastcomma = nextcomma;
        }
        update_rpm_sp(rpm_setpoint);
      }
      else if(command == 3){
        int lastcomma = firstcomma;
        for (byte i = 0; i < 8; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          tmp_setpoint[i] = value;
          lastcomma = nextcomma;
        }
        update_tmp_sp(tmp_setpoint);
      }
    }

    inputString = "";
    newCommand = false;
  }
}

void comm_verification(){
  while(!comm_confirmed){
    parseSerial2();
    int slave_ans = slaveString.toInt();
    if( slave_ans == 115){
      comm_confirmed = true;
    }
  }
  slaveString = "";
  comm_confirmed = false;
}

void check_board(String ADDRESS){
  String command = ADDRESS + " 2,!";
  Serial2.println(command);
  comm_verification();
  Serial.println(ADDRESS + " board : ok");
}

void update_rpm_sp(float rpm_setpoint[8]){
  String command = "str 1";
  for(byte i = 0; i < 8; i++){
    command = command + "," + (String) rpm_setpoint[i];
  }
  command = command + ",!";
  Serial2.println(command);
  comm_verification();
  Serial.println("rpm_sp updated");
}

void update_tmp_sp(float tmp_setpoint[8]){
  String command = "tmp 1";
  for(byte i = 0; i < 8; i++){
    command = command + "," + (String) tmp_setpoint[i];
  }
  command = command + ",!";
  Serial2.println(command);
  comm_verification();
  Serial.println("tmp_sp updated");
}
