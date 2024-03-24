#include <Arduino.h>

// Output control_modes
#ifndef BIOPROCESS
#define BIOPROCESS

#define MANUAL 0
#define TIMER 1
#define PID 2
#define ONOFF 3

// Communication pins to SPI
#define SPI_CLK  5
#define SPI_DOUT 18
#define SPI_DIN  19
#define CS0      23

// PWM PARAMETERS
#define LEDC_BIT 8
#define LEDC_BASE_FREQ 800

// Inputs and Outputs structures
typedef struct{
  String address;
  String type;
  int channel;
  int pin;
  int control_mode;
  int manual_value;
  int value;
  double *input_val;
  double input_val_lb;
  double input_val_ub;
  int time_on;
  int time_off;
  int delta_time;
  int current_timer;
  bool is_on;
  String message_bp;
} Output;

typedef struct{
  String address;
  String type;
  int channel;
  String variable;
  double value;
  String message_bp;
} Input;

typedef struct {
  int ms1;
  int ms2;
  int ms3;
  int step_pin;
  int dir_pin;
} A4988;

unsigned long int set_sample_time(unsigned long int samples_per_second);

double get_current(double voltage);

double get_dissolved_oxygen(double voltage);

double get_ph(double voltage);

double get_temperature(double voltage);

#endif
