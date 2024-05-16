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
