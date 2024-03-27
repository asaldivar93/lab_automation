#include <Arduino.h>
#include <Wire.h>
#include "bioprocess.h"
#include "comms_handle.h"

#define N_INPUTS 1
#define N_OUTPUTS 1

String ADDRESS = "M0";
int transmit_pin = 4;
bool new_command = false;

boolean READING = false;
double sample_number;
unsigned long int samples_per_second = 1;
unsigned long int sample_time;

Output outputs[N_OUTPUTS] =
  {{ADDRESS, "pwm", 0, 13, MANUAL, 0}};

Input inputs[N_INPUTS] =
  {{ADDRESS, "i2c", 0, "pressure", 0}};

Sensors sensors(SPI_DOUT, SPI_DIN, SPI_CLK);
double pressure;
double delta_p = 0;
double p_last = 0;

esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;
unsigned long start_millis;
void read_data(void *p) {
  READING = true;
  start_millis = millis();
}

void setup() {
  Serial.begin(230400);
  Serial2.begin(9600, SERIAL_8N1, Rx, Tx);
  pinMode(transmit_pin, OUTPUT);
  digitalWrite(transmit_pin, LOW);
  
  Wire.begin();
  inputs[0].read = &Sensors::read_mprls;
  for(int i=0; i<N_INPUTS; i++){
    set_input_mssg_bp(&inputs[i]);
  }

  for(int i=0; i<N_OUTPUTS; i++){
    set_output_mssg_bp(&outputs[i]);
    if(outputs[i].type == "pwm"){
      ledcSetup(outputs[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
      ledcAttachPin(outputs[i].pin, outputs[i].channel);
      ledcWrite(outputs[i].channel, outputs[i].value);
    }
  }

  p_last = (sensors.*inputs[0].read)(0);
  samples_per_second = 1;
  sample_time = set_sample_time(samples_per_second);
  create_args.callback = read_data; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_start_once(timer_handle, 60000000);
}

void loop() {
  if(!READING){
  pressure = (sensors.*inputs[0].read)(0);
    if(pressure >= 11.4){
      delta_p += pressure - p_last;
      ledcWrite(outputs[0].channel, 255);
      delay(5);
      p_last = (sensors.*inputs[0].read)(0);
      ledcWrite(outputs[0].channel, 0);
    }
  }

  if(READING){
    
    Serial.println(delta_p, 6);
    esp_timer_create(&create_args, &timer_handle);
    esp_timer_start_once(timer_handle, 60000000);
    READING = false;
    delta_p = 0;
  }
  String input_string = parse_serial_master();
  parse_string(input_string);
}

String parse_serial_master(void){
  String input_string = "";
  while(Serial.available()){
    char inChar = char(Serial.read());
    input_string += inChar;
    if(inChar == '!'){
      new_command = true;
      break;
    }
  }
  return input_string;
}

void parse_string(String input_string){
  int firstcomma;
  int lastcomma;
  int nextcomma;
  int command;
  String ok_string = "{}115,!";

  if(new_command){
    String ADDR = input_string.substring(0, input_string.indexOf(' '));

    if(ADDR == ADDRESS){
      firstcomma = input_string.indexOf(',');
      command = input_string.substring(input_string.indexOf(' '), firstcomma).toInt();
    }
    else{
      write_to_slaves(input_string, transmit_pin);
      //Serial.println(input_string);
    }
    new_command = false;
  }
}
