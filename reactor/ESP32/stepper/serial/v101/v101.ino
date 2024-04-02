#include <Arduino.h>
#include <AccelStepper.h>
#include "bioprocess.h"
#include "comms_handle.h"

#define N_INPUTS 0
#define N_OUTPUTS 3
#define N_PULSES 0
#define N_SLAVES 0

// MAX485 PINS
int transmit_pin = 15;

// STEPPER PINS
#define MS1_0 13
#define MS2_0 12
#define MS3_0 14
#define STEP_0 27
#define DIR_0 26

#define MS1_1 2
#define MS2_1 4
#define MS3_1 5
#define STEP_1 18
#define DIR_1 19

#define MS1_2 21
#define MS2_2 3
#define MS3_2 1
#define STEP_2 22
#define DIR_2 23

#define MAX_SPEED 1000
#define ACCELERATION 100

// Serial info
String ADDRESS = "S2";
boolean new_command = false;
String input_string = "";

// Seriaf buffers
String outputs_buffer = "(0,0),(1,0),(2,0)";
String inputs_buffer = "";
String pulses_buffer = "";

// Config Data
String slaves[N_SLAVES];

Output outputs[N_OUTPUTS] =
  {{ADDRESS, "stp", 0, STEP_0, MANUAL, 0}, {ADDRESS, "stp", 1, STEP_1, MANUAL, 0},
   {ADDRESS, "stp", 2, STEP_2, MANUAL, 0}};

Input inputs[N_INPUTS];

Input pulses[N_PULSES];

A4988 stepper_pins[N_OUTPUTS] =
  {{MS1_0, MS2_0, MS3_0, STEP_0, DIR_0}, {MS1_1, MS2_1, MS3_1, STEP_1, DIR_1},
   {MS1_2, MS2_2, MS3_2, STEP_2, DIR_2}};

AccelStepper stepper_0(AccelStepper::DRIVER, STEP_0, DIR_0);
AccelStepper stepper_1(AccelStepper::DRIVER, STEP_1, DIR_1);
AccelStepper stepper_2(AccelStepper::DRIVER, STEP_2, DIR_2);
AccelStepper all_steppers[N_OUTPUTS] = {stepper_0, stepper_1, stepper_2};

TaskHandle_t serial_comms;

// Parallel task for communication
void serial_comms_code(void *parameters){
  for(;;){
    String input_string = parse_serial_slave(&new_command);
    parse_string(input_string);
  }
}

void setup() {
  //Serial.begin(57600);
  Serial2.begin(9600, SERIAL_8N1, Rx, Tx);
  pinMode(transmit_pin, OUTPUT);
  digitalWrite(transmit_pin, LOW);
  xTaskCreatePinnedToCore(serial_comms_code, "serial_comms", 10000, NULL, 0, &serial_comms, 0);

  // stepper Setup
  for(int i=0; i<N_OUTPUTS; i++){
    pinMode(stepper_pins[i].ms1, OUTPUT);
    pinMode(stepper_pins[i].ms2, OUTPUT);
    pinMode(stepper_pins[i].ms3, OUTPUT);

    digitalWrite(stepper_pins[i].ms1, LOW);
    digitalWrite(stepper_pins[i].ms2, LOW);
    digitalWrite(stepper_pins[i].ms3, LOW);

    all_steppers[i].setMaxSpeed(MAX_SPEED);
    all_steppers[i].setAcceleration(ACCELERATION);
    all_steppers[i].setSpeed(outputs[i].value);
  }
}

void loop() {
  set_output_vals();
  for(int i=0; i<N_OUTPUTS; i++){
    all_steppers[i].runSpeed();
  }
  //update_inputs_buffer(inputs, N_INPUTS, &inputs_buffer);
  //update_outputs_buffer(outputs, N_OUTPUTS, &outputs_buffer);
}


void set_output_vals(void){
  for(int i=0; i<N_OUTPUTS; i++){
    switch (outputs[i].control_mode){
      case MANUAL:
        outputs[i].value = outputs[i].manual_value;
        break;

      case TIMER:
        outputs[i].delta_time = outputs[i].delta_time + 1;
        if(outputs[i].delta_time > outputs[i].current_timer){
          if(outputs[i].is_on){
            outputs[i].is_on = false;
          }
          else{
            outputs[i].is_on = true;
          }
          outputs[i].delta_time = 0;
        }
        if(outputs[i].is_on){
          outputs[i].current_timer = outputs[i].time_on;
          outputs[i].value = outputs[i].manual_value;
        }
        else{
          outputs[i].current_timer = outputs[i].time_off;
          outputs[i].value = 0;
        }
        break;
    }
    all_steppers[i].setSpeed(outputs[i].value);
  }
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

      if (command == GET_OUTPUTS_INFO){
        // GET_OUTPUTS_INFO: "ADDR 2,!"
        String outputs_info = get_outputs_info(outputs, N_OUTPUTS) + "!";
        //Serial.println(outputs_info);
        write_to_master(outputs_info, transmit_pin);
      }

      if (command == GET_INPUTS_INFO){
        // GET_INPUTS_INFO: "ADDR 3,!"
        String inputs_info = get_inputs_info(inputs, N_INPUTS) + get_inputs_info(pulses, N_PULSES) + "!";
        //Serial.println(inputs_info);
        write_to_master(inputs_info, transmit_pin);
      }

      if (command == GET_OUTPUTS_DATA) {
        // GET_OUTPUTS_DATA: "ADDR 4,!"
        String data_string = outputs_buffer + "!";
        //Serial.println(data_string);
        write_to_master(data_string, transmit_pin);
      }

      if (command == GET_INPUTS_DATA) {
        // GET_INPUTS_DATA: "ADDR 5,!"
        String data_string = inputs_buffer + pulses_buffer + "!";
        //Serial.println(data_string);
        write_to_master(data_string, transmit_pin);
      }

      if(command == TOGGLE_CONTROL_MODE){
//      MANUAL: "ADDR, 1,0,OUT_CHANNEL,PWM,!"
//      TIMER:  "ADDR, 1,1,OUT_CHANNEL,TIME_ON,TIME_OFF,PWM,!"
//      PID:    "ADDR, 1,2,OUT_CHANNEL,IN_CHANNEL,SETPOINT,!"
//      ONOFF:  "ADDR, 1,3,OUT_CHANNEL,IN_CHANNEL,LOWER_BOUND,UPPER_BOUND,PWM,!"

        lastcomma = firstcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int control_mode = input_string.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int out_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();

        outputs[out_channel].control_mode = control_mode;

        switch(outputs[out_channel].control_mode){
          case MANUAL:{
            lastcomma = nextcomma;
            nextcomma = input_string.indexOf(',', lastcomma + 1);
            int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

            outputs[out_channel].manual_value = value;
            //Serial.println(ok_string);
            break;
          }

          case TIMER:{
            lastcomma = nextcomma;
            nextcomma = input_string.indexOf(',', lastcomma + 1);
            int time_on = input_string.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = input_string.indexOf(',', lastcomma + 1);
            int time_off = input_string.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = input_string.indexOf(',', lastcomma + 1);
            int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

            outputs[out_channel].manual_value = value;
            outputs[out_channel].time_on = time_on;
            outputs[out_channel].time_off = time_off;
            outputs[out_channel].delta_time = 0;
            outputs[out_channel].current_timer = outputs[out_channel].time_on;
            outputs[out_channel].is_on = true;
            //Serial.println(ok_string);
            break;
          }
        }
        update_outputs_buffer(outputs, N_OUTPUTS, &outputs_buffer);
      }
    input_string = "";
    new_command = false;
    }
  }
}
