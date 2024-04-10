#include <Arduino.h>
#include "comms_handle.h"
#include "bioreactify.h"

String parse_serial(char inChar[], bool *newCommand) {
  /*
  Reads Serial buffer
  */
  String inputString = "";
  int i = 0;
  while (i < 255) {
    inputString += inChar[i];
    if (inChar[i] == '!') {
      *newCommand = true;
      break;
    }
    i++;
  }
  return inputString;
}

String parse_serial_master(bool *new_command) {
  /*
  Reads Serial buffer
  */
  String input_string = "";
  while (Serial.available()) {
    char inChar = char(Serial.read());
    input_string += inChar;
    if (inChar == '!') {
      *new_command = true;
      break;
    }
  }
  return input_string;
}


String parse_serial_slave(bool *new_command){
  /*
  Reads Serial buffer
  */
  String input_string = "";
  while(Serial2.available()){
    char inChar = char(Serial2.read());
    input_string += inChar;
    if(inChar == '!'){
      *new_command = true;
      break;
    }
  }
  return input_string;
}

String request_outputs_info(String slaveAddress, int transmitPin){
  /*
  Sends a request for outputs info to slaves and waits for a response
  */
  String command = String(GET_OUTPUTS_INFO);
  String command_string = slaveAddress + " " + command + ",!";
  write_to_slaves(command_string, transmitPin);

  String output_info = parse_slave_to_master();
  return output_info;
}

String request_outputs_data(String slaveAddress, int transmitPin){
  /*
  Sends a request for outputs data to slaves and waits for a response
  */
  String command = String(GET_OUTPUTS_DATA);
  String command_string = slaveAddress + " " + command + ",!";
  write_to_slaves(command_string, transmitPin);

  String output_data = parse_slave_to_master();
  return output_data;
}

String request_inputs_info(String slaveAddress, int transmitPin){
  /*
  Sends a request for inputs info to slaves and waits for a response
  */
  String command = String(GET_INPUTS_INFO);
  String command_string = slaveAddress + " " + command + ",!";
  write_to_slaves(command_string, transmitPin);

  String input_info = parse_slave_to_master();
  return input_info;
}

String request_inputs_data(String slaveAddress, int transmitPin){
  /*
  Sends a request for inputs data to slaves and waits for a response
  */
  String command = String(GET_INPUTS_DATA);
  String command_string = slaveAddress + " " + command + ",!";
  write_to_slaves(command_string, transmitPin);

  String input_data = parse_slave_to_master();
  return input_data;
}


void write_to_master(String string, int transmitPin){
  digitalWrite(transmitPin, HIGH);
  delay(10);
  Serial2.print(string);
  delay(120);
  digitalWrite(transmitPin, LOW);
}


void write_to_slaves(String string, int transmitPin){
  digitalWrite(transmitPin, HIGH);
  Serial2.print(string);
  delay(20);
  digitalWrite(transmitPin, LOW);
}


String parse_slave_to_master(void){
  /*
  THIS IS A BLOCKING FUNCTION

  Waits for 90ms for an input from Slaves Serial Buffer (Serial2)
  Reads Serial2 Buffer if available
  */
  unsigned long wait_for = 500;
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

String get_outputs_info(Output outputs[], int numberOfOutputs){
  /* Gathers outputs config info and prints a string with struct:
   *  (type_0, channel_0), ... (type_N, channel_N),!
   */
  String outputs_string = "";

  for(int i=0; i < numberOfOutputs; i++){
    outputs_string = outputs_string + "(" + "'" + outputs[i].type + "'," + outputs[i].channel + "),";
  }
  return outputs_string;
}

String get_inputs_info(Input inputs[], int numberOfInputs){
  /* Gathers inputs config info and prints a string with struct:
   *  (type_0, channel_0, variable_0), ... (type_N, channel_N, variable_N)
   */
  String inputs_string = "";

  for (int i=0; i < numberOfInputs; i++){
    inputs_string = inputs_string + "(" + "'" + inputs[i].type + "'," + inputs[i].channel + "," + "'" + inputs[i].variable + "'" + "),";
  }
  return inputs_string;
}

void send_input_data(String address, String slaves[], int numberOfSlaves, int transmitPin, String *inputsBuffer, String *pulsesBuffer) {
  /*
    Gathers all sensor readings to Serial in a dictionary string with structure:
    {"master": [ins], "slave1": [ins], ... "slaveN": [ins]}115,!
  */
  String inputs_string = "{'" + address + "':[" + *inputsBuffer + *pulsesBuffer + "],";
  inputs_string = inputs_string + write_slaves_inputs(slaves, numberOfSlaves, transmitPin) + "}115,!";
  Serial.println(inputs_string);

  // Clear all buffers to keep most recent readings only
  *inputsBuffer = "";
  *pulsesBuffer = "";
}

String write_slaves_inputs(String slaves[], int numberOfSlaves, int transmitPin) {
  /*
    Requests inputs data from slaves
    Inputs:
     String slaves[]: an array with all slaves addressess
    Outputs:
    slaves_string: a dictionary entry string with structure:
     "slave1": [ins], ... "slaveN": [ins]
  */
  String slaves_input_data = "";

  for (int i = 0; i < numberOfSlaves; i++) {
    slaves_input_data = slaves_input_data + "'" + slaves[i] + "':[";
    slaves_input_data = slaves_input_data + request_inputs_data(slaves[i], transmitPin);
    slaves_input_data = slaves_input_data + "],";
  }
  return slaves_input_data;
}

void update_inputs_buffer(Input inputs[], int numberOfInputs, String *inputsBuffer){
  /*
  Takes inputs_buffer string from the global variables and updates
  the string every time new sensor readings are available

  Inputs:
    *inputsBuffer: a pointer to the inputs_buffer string with struct:
      (channel_0, value_0), (channel_1, value_1) ... (channel_N, value_N),
  */
  *inputsBuffer = ""; // Clear Buffer to keep only most recent readings
  for(int i=0; i < numberOfInputs; i++){
    *inputsBuffer = *inputsBuffer + "(" + inputs[i].channel + "," + inputs[i].value + "),";
  }
}

void send_output_data(String address, String slaves[], int numberOfSlaves, int transmitPin, String *outputsBuffer) {
  /*
    Gathers all outputs values and prints them to Serial in a dictionary string with structure:
    {"master": [outs], "slave1": [outs], ... "slaveN": [outs]}
  */
  String outputs_string = "{'" + address + "':[" + *outputsBuffer + "],";
  outputs_string = outputs_string + write_slaves_outputs(slaves, numberOfSlaves, transmitPin) + "}115,!";
  Serial.println(outputs_string);

  // Clear all buffers to keep most recent readings only
  *outputsBuffer = "";
}


String write_slaves_outputs(String slaves[], int numberOfSlaves, int transmitPin) {
  /*
    Requests inputs data from slaves
    Inputs:
     String slaves[]: an array with all slaves addressess
    Outputs:
    slaves_string: a dictionary entry string with structure:
     "slave1": [outs], ... "slaveN": [outs]
  */
  String slaves_output_data = "";

  for (int i = 0; i < numberOfSlaves; i++) {
    slaves_output_data = slaves_output_data + "'" + slaves[i] + "':[";
    slaves_output_data = slaves_output_data + request_outputs_data(slaves[i], transmitPin);
    slaves_output_data = slaves_output_data + "],";
  }
  return slaves_output_data;
}

void update_outputs_buffer(Output outputs[], int numberOfOutputs, String *outputsBuffer){
  /*
  Takes inputs_buffer string from the global variables and updates
  the string every time new sensor readings are available

  Inputs:
    String *outputsBuffer: a pointer to the outputs_buffer string with struct:
      (channel_0, value_0), (channel_1, value_1) ... (channel_N, value_N),
  */
  *outputsBuffer = ""; // Clear Buffer to keep only most recent readings
  for(int i=0; i < numberOfOutputs; i++){
    *outputsBuffer = *outputsBuffer + "(" + outputs[i].channel + "," + outputs[i].value + "),";
  }
}
