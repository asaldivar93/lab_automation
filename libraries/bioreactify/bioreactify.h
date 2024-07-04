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
#define SPI_MISO 18
#define SPI_MOSI 19
#define CS0  23
#define CS1  15

// PWM PARAMETERS
#define LEDC_BIT 8
#define LEDC_BASE_FREQ 500

#define PIN_CH0 13
#define PIN_CH1 12
#define PIN_CH2 14
#define PIN_CH3 27
#define PIN_CH4 33
#define PIN_CH5 32
#define PIN_CH6 2
#define PIN_CH7 4

#define DAC1 25
#define DAC2 26

// Inputs and Outputs structures
class MCP3208 {
public:
  MCP3208(uint8_t MISO, uint8_t MOSI, uint8_t clock);
  void begin(uint8_t select);
  void set_ref_voltage(float ref_voltage);
  void set_spi_speed(uint32_t speed); // speed in Hz
  float read_adc(uint8_t channel);

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
};

class Sensors{
public:
  Sensors(uint8_t TCA954X_ADDRESS);
  void set_multiplexer_channel(uint8_t channel);

  // SEN0546 Temp and Humidity sensor
  float read_sen0546_temperature();
  float read_sen0546_humidity();

  // SEN0322 Oxygen Sensor
  float read_sen0322_address_0();
  float read_sen0322_address_1();
  float read_sen0322_address_2();
  float read_sen0322_default();

  // MPRLS Pressure sensor
  float read_mprls();
  void set_mprls_range(float p_min, float p_max);

  float read_sen0343_diffpressure();

private:
  uint8_t _TCA954X_ADDRESS;

  // SEN0546 Temp and Humidity sensor
  uint8_t _SEN0546_ADDRESS = 0x40;
  uint8_t _temperature_register = 0x00;
  uint8_t _humidity_register = 0x01;

  // SEN0322 Oxygen Sensor
  float _read_sen0322(uint8_t address);
  uint8_t _SEN0322_DEFAULT_ADDRESS = 0x73;
  uint8_t _SEN0322_ADDRESS_0 = 0x70;
  uint8_t _SEN0322_ADDRESS_1 = 0x71;
  uint8_t _SEN0322_ADDRESS_2 = 0x72;
  uint8_t _o2_data_register = 0x03;
  float _cal = 20.9 / 100;

  // SEN0343 Differential pressure sensor
  uint8_t _SEN0343_ADDRESS = 0x00;

  // MPRLS Pressure Sensor
  uint8_t _MPRLS_DEFAULT_ADDRESS = 0x18;
  float _p_min = 0;
  float _p_max = 25;
  float _out_min = 1677722;
  float _out_max = 15099494;
  float _conversion = 1;
  float _press_counts = 0;
  float _pressure;
};

class Output {
public:
  Output(int Channel, int Pin, String Type, int Control_mode, int Value);

  void get_info(void);
  void write_output(void);
  void write_dac(int value);
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

class Input{
public:
  Input(int channel, String type, String variable);

  void moving_average(float analog);
  void get_moving_average();
  void reset_moving_average();
  float get_number_of_samples();
  float get_analog_value();

  void set_ref_voltage(float vref);
  void set_current_cal(float m, float b);
  void set_dissolved_oxygen_cal(float m, float b);
  void set_ph_cal(float m, float b);
  void set_temperature_cal(float a, float b, float c);
  void set_temperature_resistor(float resistor_value);

  void set_blank();

  void get_ph();
  void get_dissolved_oxygen();
  void get_temperature();
  void get_current();
  void get_absorbance();

  int channel;
  String type;
  String variable;

  float value;

private:
  float _ref_voltage = 3.3;
  float _analog_value;
  uint32_t _number_of_samples;

  float _m_ph;
  float _b_ph;
  float _m_oxygen;
  float _b_oxygen;
  float _m_current;
  float _b_current;

  float _resistor_value;
  float _a_temp;
  float _b_temp;
  float _c_temp;

  float _blank;
};

typedef struct {
  int ms1;
  int ms2;
  int ms3;
  int step_pin;
  int dir_pin;
} A4988;

#endif
