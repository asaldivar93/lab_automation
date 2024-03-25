#include <Arduino.h>
#include "bioprocess.h"

// commands
#define GET_BOARD_INFO 0
#define TOGGLE_CONTROL_MODE 1
#define GET_OUTPUTS_INFO 2
#define GET_INPUTS_INFO 3
#define GET_OUTPUTS_DATA 4
#define GET_INPUTS_DATA 5

// MAX485 PINS
#define Rx 16
#define Tx 17

String request_outputs_info(String slave_address, int transmit_pin);

String request_outputs_data(String slave_address, int transmit_pin);

String request_inputs_info(String slave_address, int transmit_pin);

String request_inputs_data(String slave_address, int transmit_pin);

void write_to_master(String string, int transmit_pin);

void write_to_slaves(String string, int transmit_pin);

String parse_serial_master(void);

String parse_serial2(void);

String get_output_info(Output output_channel);

String get_input_info(Input input_channel);

String get_output_data(Output output_channel);

String get_input_data(Input input_channel);

void set_output_mssg_bp(Output *output_channel);

void set_input_mssg_bp(Input *input_channel);
