#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

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

// I2C SENSORS
#define MPRLS_DEFAULT_ADDRESS 0x18

#define SEN0322_DEFAULT_ADDRESS 0x73
#define SEN0322_ADDRESS_0 0x70
#define SEN0322_ADDRESS_1 0x71
#define SEN0322_ADDRESS_2 0x72

#define SEN0546_ADDRESS 0x40

// Inputs and Outputs structures

class Sensors
{
public:
  // MCP3208
  Sensors(uint8_t dataIn, uint8_t dataOut, uint8_t clock); //SOFTWARE SPI
  void begin(uint8_t select);
  void set_spi_speed(uint32_t speed); // speed in Hz
  void set_ref_voltage(double ref_voltage);
  double read_adc(uint8_t channel);

  // SEN0546 Temp and Humidity sensor
  double read_sen0546_temperature(uint8_t channel);
  double read_sen0546_humidity(uint8_t channel);

  // SEN0322 Oxygen Sensor

  double read_sen0322_address_0(uint8_t channel);
  double read_sen0322_address_1(uint8_t channel);
  double read_sen0322_address_2(uint8_t channel);
  double read_sen0322_default(uint8_t channel);

  // MPRLS Pressure sensor
  double read_mprls(uint8_t channel);

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
  double _ref_voltage = 3.3;
  uint8_t  _build_request_mcp3208(uint8_t channel, uint8_t * data);
  uint8_t  _swSPI_transfer(uint8_t d);

  // SEN0546 Temp and Humidity sensor
  uint8_t _SEN0546_ADDRESS=0x40;
  uint8_t _temperature_register = 0x00;
  uint8_t _humidity_register = 0x01;

  // SEN0322 Oxygen Sensor
  double _read_sen0322(uint8_t address);
  uint8_t _SEN0322_DEFAULT_ADDRESS=0x73;
  uint8_t _SEN0322_ADDRESS_0=0x70;
  uint8_t _SEN0322_ADDRESS_1=0x71;
  uint8_t _SEN0322_ADDRESS_2=0x72;
  uint8_t _o2_data_register = 0x03;
  double _cal = 20.9 / 100;

  // MPRLS Pressure Sensor
  uint8_t _MPRLS_DEFAULT_ADDRESS=0x18;
  uint8_t _request[3] = {0xAA, 0x00, 0x00};
  double _p_min = 0;
  double _p_max = 25;
  double _out_min=1677722;
  double _out_max=15099494;
  double _conversion = 1;
  double _press_counts = 0;
  double _pressure;
};

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

typedef double (Sensors::*Readfunc) (uint8_t channel);
typedef struct{
  String address;
  String type;
  int channel;
  String variable;
  double value;
  String message_bp;
  Readfunc read;
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
