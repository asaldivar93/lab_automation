#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Output control_modes
#ifndef BIOPROCESS
#define BIOPROCESS

#define MANUAL 0
#define TIMER  1
#define PID    2
#define ONOFF  3

// Communication pins to SPI
#define SPI_CLK  5
#define SPI_DOUT 18
#define SPI_DIN  19
#define CS0      23

// PWM PARAMETERS
#define LEDC_BIT 8
#define LEDC_BASE_FREQ 800
#define PIN_CH0 13
#define PIN_CH1 12
#define PIN_CH2 14
#define PIN_CH3 27
#define PIN_CH4 26
#define PIN_CH5 25

// Inputs and Outputs structures

class Sensors
{
public:
  // MCP3208
  Sensors(uint8_t dataIn, uint8_t dataOut, uint8_t clock); //SOFTWARE SPI
  void begin(uint8_t select);
  void set_spi_speed(uint32_t speed); // speed in Hz
  void set_ref_voltage(float ref_voltage);
  float read_adc(uint8_t channel);

  // SEN0546 Temp and Humidity sensor
  float read_sen0546_temperature(uint8_t channel);
  float read_sen0546_humidity(uint8_t channel);

  // SEN0322 Oxygen Sensor

  float read_sen0322_address_0(uint8_t channel);
  float read_sen0322_address_1(uint8_t channel);
  float read_sen0322_address_2(uint8_t channel);
  float read_sen0322_default(uint8_t channel);

  // MPRLS Pressure sensor
  float read_mprls(uint8_t channel);
  void set_mprls_range(float p_min, float p_max);

  float read_sen0343_diffpressure(uint8_t channel);

private:
  // MCP3208
  uint8_t  _dataIn;
  uint8_t  _dataOut;
  uint8_t  _clock;
  uint8_t  _select;
  uint8_t  _channels;
  int16_t  _maxValue;
  uint32_t _SPIspeed = 1000000;
  SPISettings _spi_settings;
  float _ref_voltage = 3.3;
  uint8_t  _build_request_mcp3208(uint8_t channel, uint8_t * data);
  uint8_t  _swSPI_transfer(uint8_t d);

  // SEN0546 Temp and Humidity sensor
  uint8_t _SEN0546_ADDRESS=0x40;
  uint8_t _temperature_register = 0x00;
  uint8_t _humidity_register = 0x01;

  // SEN0322 Oxygen Sensor
  float _read_sen0322(uint8_t address);
  uint8_t _SEN0322_DEFAULT_ADDRESS=0x73;
  uint8_t _SEN0322_ADDRESS_0=0x70;
  uint8_t _SEN0322_ADDRESS_1=0x71;
  uint8_t _SEN0322_ADDRESS_2=0x72;
  uint8_t _o2_data_register = 0x03;
  float _cal = 20.9 / 100;

  // SEN0343 Differential pressure sensor
  uint8_t _SEN0343_ADDRESS=0x00;

  // MPRLS Pressure Sensor
  uint8_t _MPRLS_DEFAULT_ADDRESS=0x18;
  float _p_min = 0;
  float _p_max = 25;
  float _out_min=1677722;
  float _out_max=15099494;
  float _conversion = 1;
  float _press_counts = 0;
  float _pressure;
};

class Output
{
public:
  Output(int Channel, int Pin, String Type, int Control_mode, int Value);

  void get_info(void);
  void write_output(void);
  void set_manual_output(int value);
  void set_timer(int time_on, int time_off, int value);
  void set_pid(float *input_value, float setpoint);
  void set_onoff(float *input_value, int lb, int ub, int value);

  //----PID-----//
  float compute_pid(float input);
  void set_sample_time_us(uint32_t sample_time_us);
  void set_gh_filter(float alpha);
  void set_pid_tunings(float Kp, float Ki, float Kd);
  void set_output_limits(float min, float max);
  void initialize_pid(void);

  int channel;
  int pin;
  String type;
  int manual_value;
  int value;
  int control_mode;

private:
  int _time_on = 1;
  int _time_off = 2;
  int _delta_time = 0 ;
  int _current_timer = 2;
  bool _is_on = false;

  float *_input_value;
  float _input_value_lb = 0;
  float _input_value_ub = 1;

  //-----PID------//
  float _setpoint;
  float _alpha = 0.01;
  float _filtered_input;
  uint32_t _sample_time_us = 250000;
  float _samples_time_s = 0.25;
  float _kp, _ki, _kd;
  float _integral_sum;
  float _last_error, _last_time;
  float _output_min = 0;
  float _output_max = 255;
};


typedef float (Sensors::*Readfunc) (uint8_t channel);
typedef struct{
  String address;
  String type;
  int channel;
  String variable;
  int pin;
  float value;
  float delta_pressure;
  float last_pressure;
  String message_bp;
  Readfunc read;
  bool is_on;
  unsigned long start_millis;
} Input;

typedef struct {
  int ms1;
  int ms2;
  int ms3;
  int step_pin;
  int dir_pin;
} A4988;

float get_current(float voltage);

float get_dissolved_oxygen(float voltage);

float get_ph(float voltage);

float get_temperature(float voltage);
#endif
