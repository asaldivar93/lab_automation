#include <Arduino.h>
#include "comms_handle.h"
#include "bioprocess.h"


String request_outputs_info(String slave_address, int transmit_pin){
  String command = String(GET_OUTPUTS_INFO);
  String command_string = slave_address + " " + command + ",!";
  write_to_slaves(command_string, transmit_pin);

  String output_info = parse_serial2();
  return output_info;
}

String request_outputs_data(String slave_address, int transmit_pin){
  String command = String(GET_OUTPUTS_DATA);
  String command_string = slave_address + " " + command + ",!";
  write_to_slaves(command_string, transmit_pin);

  String output_data = parse_serial2();
  return output_data;
}

String request_inputs_info(String slave_address, int transmit_pin){
  String command = String(GET_INPUTS_INFO);
  String command_string = slave_address + " " + command + ",!";
  write_to_slaves(command_string, transmit_pin);

  String input_info = parse_serial2();
  return input_info;
}

String request_inputs_data(String slave_address, int transmit_pin){
  String command = String(GET_INPUTS_DATA);
  String command_string = slave_address + " " + command + ",!";
  write_to_slaves(command_string, transmit_pin);

  String input_data = parse_serial2();
  return input_data;
}

void write_to_master(String string, int transmit_pin){
  digitalWrite(transmit_pin, HIGH);
  delay(10);
  Serial2.print(string);
  delay(50);
  digitalWrite(transmit_pin, LOW);
}

void write_to_slaves(String string, int transmit_pin){
  digitalWrite(transmit_pin, HIGH);
  Serial2.print(string);
  delay(20);
  digitalWrite(transmit_pin, LOW);
}

String parse_serial2(void){
  unsigned long wait_for = 90;
  bool waiting = true;
  String slave_string = "";

  unsigned long started_waiting = millis();
  while(waiting && (millis() - started_waiting) <= wait_for){
    while(Serial2.available()){
      char inChar = char(Serial2.read());
      if(inChar != '!'){
        slave_string += inChar;
      }
      if(inChar == '!'){
        waiting = false;
        break;
      }
    }
  }
  return slave_string;
}

String get_output_info(Output output_channel){
  String output_string = "(";
  output_string = output_string + "'" + output_channel.type + "',";
  output_string = output_string + output_channel.channel + "),";
  return output_string;
}

String get_input_info(Input input_channel){
  String input_string = "(";
  input_string = input_string + "'" + input_channel.type + "',";
  input_string = input_string + input_channel.channel + ",";
  input_string = input_string + "'" + input_channel.variable + "'";
  input_string = input_string + "),";
  return input_string;
}

void set_output_mssg_bp(Output *output_channel){
  String output_string = "(";
  output_string = output_string + (*output_channel).channel + ",";
  (*output_channel).message_bp = output_string;
}

String get_output_data(Output output_channel){
  String output_string = output_channel.message_bp;
  output_string = output_string + output_channel.value;
  output_string = output_string + "),";
  return output_string;
}

void set_input_mssg_bp(Input *input_channel){
  String output_string = "(";
  output_string = output_string + (*input_channel).channel + ",";
  (*input_channel).message_bp = output_string;
}

String get_input_data(Input input_channel){
  String input_string = input_channel.message_bp;
  input_string = input_string + input_channel.value;
  input_string = input_string + "),";
  return input_string;
}
