#include <Arduino.h>
#include "bioreactify.h"

// commands
#define GET_BOARD_INFO 0
#define TOGGLE_CONTROL_MODE 1
#define GET_ALL_DATA 2

// MAX485 PINS
#define Rx 16
#define Tx 17

typedef struct Slave{
  String name;
  uint8_t broadcastAddress[6];
  String inputsInfo;
  String ouputsInfo;
} Slave;

typedef struct SlaveMessage{
  int id;
  char name[2];
  char message[200];
} MessageSlaves;

typedef struct SimpleMessage{
  char message[200];
} SimpleMessage;

String parse_serial(char inChar[], bool *newCommand);

String parse_serial_master(bool *new_command);

String get_outputs_info(Output outputs[], int numberOfOutputs);

String get_inputs_info(Input inputs[], int numberOfInputs);

void update_inputs_buffer(Input inputs[], int numberOfInputs, String *inputsBuffer);

void update_outputs_buffer(Output outputs[], int numberOfOutputs, String *outputsBuffer);
