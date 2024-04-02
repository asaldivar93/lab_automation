#include <Arduino.h>
#include "bioreactify.h"

// commands
#define GET_BOARD_INFO 0
#define TOGGLE_CONTROL_MODE 1
#define GET_OUTPUTS_INFO 2
#define GET_INPUTS_INFO 3
#define GET_OUTPUTS_DATA 4
#define GET_INPUTS_DATA 5
#define GET_ALL_OUTPUTS 6
#define GET_ALL_INPUTS 7

// MAX485 PINS
#define Rx 16
#define Tx 17

String parse_serial_master(bool *new_command);

String parse_serial_slave(bool *new_command);

String parse_slave_to_master(void);

String request_outputs_info(String slaveAddress, int transmitPin);

String request_outputs_data(String slaveAddress, int transmitPin);

String request_inputs_info(String slaveAddress, int transmitPin);

String request_inputs_data(String slaveAddress, int transmitPin);

void write_to_master(String string, int transmitPin);

void write_to_slaves(String string, int transmitPin);

String get_outputs_info(Output outputs[], int numberOfOutputs);

String get_inputs_info(Input inputs[], int numberOfInputs);

void send_input_data(String address, String slaves[], int numberOfSlaves, int transmitPin, String *inputsBuffer, String *pulsesBuffer);

String write_slaves_inputs(String slaves[], int numberOfSlaves, int transmitPin);

void update_inputs_buffer(Input inputs[], int numberOfInputs, String *inputsBuffer);

void send_output_data(String address, String slaves[], int numberOfSlaves, int transmitPin, String *outputsBuffer);

String write_slaves_outputs(String slaves[], int numberOfSlaves, int transmitPin);

void update_outputs_buffer(Output outputs[], int numberOfOutputs, String *outputsBuffer);
