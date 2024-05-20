#include <Arduino.h>
#include "bioreactify.h"


Sensors::Sensors(uint8_t dataIn, uint8_t dataOut, uint8_t clock)
{
  _dataIn  = dataIn;
  _dataOut = dataOut;
  _clock   = clock;
  _select  = 255;
  _channels = 8;
  _maxValue = 4095;
}


void Sensors::begin(uint8_t select)
{
  // MCP3208
  _select = select;
  pinMode(_select, OUTPUT);
  digitalWrite(_select, HIGH);
  digitalWrite(_select, LOW);    //  force communication See datasheet)
  digitalWrite(_select, HIGH);

  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);

  pinMode(_dataIn,  INPUT);
  pinMode(_dataOut, OUTPUT);
  pinMode(_clock,   OUTPUT);
  digitalWrite(_dataOut, LOW);
  digitalWrite(_clock,   LOW);
}


void Sensors::set_ref_voltage(float ref_voltage){
  _ref_voltage = ref_voltage;
}

void Sensors::set_spi_speed(uint32_t speed){
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

void Sensors::set_mprls_range(float p_min, float p_max){
  _p_min = p_min;
  _p_max = p_max;
}


float Sensors::read_adc(uint8_t channel){
  if (channel >= _channels) return 0;

  uint8_t  data[3] = { 0,0,0 };
  uint8_t  bytes = _build_request_mcp3208(channel, data);

  digitalWrite(_select, LOW);
  for (uint8_t b = 0; b < bytes; b++){
    data[b] = _swSPI_transfer(data[b]);
  }
  digitalWrite(_select, HIGH);

  return ((256 * data[1] + data[2]) & _maxValue) * (_ref_voltage / _maxValue);
}


uint8_t Sensors::_swSPI_transfer(uint8_t val){
  uint8_t clk = _clock;
  uint8_t dao = _dataOut;
  uint8_t dai = _dataIn;

  uint8_t rv = 0;
  for (uint8_t mask = 0x80; mask; mask >>= 1)
  {
    digitalWrite(dao, (val & mask));
    digitalWrite(clk, HIGH);
    if (digitalRead(dai) == HIGH) rv |= mask;
    digitalWrite(clk, LOW);
  }
  return rv;
}


uint8_t Sensors::_build_request_mcp3208(uint8_t channel, uint8_t * data){
  //  P21  fig 6.1   MCP3204/3208
  data[0] = 0x04;  //  start bit
  data[0] |= 0x02; //  single read
  if (channel > 3) data[0] |= 0x01;        //  MSB channel (D2)
  if (channel) data[1] |= (channel << 6);  //  other 2 bits (D1 D0)
  return 3;
}


float Sensors::read_sen0322_address_0(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_0);
}


float Sensors::read_sen0322_address_1(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_1);
}


float Sensors::read_sen0322_address_2(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_2);
}


float Sensors::read_sen0322_default(uint8_t channel){
  return _read_sen0322(_SEN0322_DEFAULT_ADDRESS);
}


float Sensors::_read_sen0322(uint8_t address){
  uint8_t _data[10]={0};
  Wire.beginTransmission(address);
  int _status = Wire.write(_o2_data_register);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }

  delay(6);
  Wire.requestFrom(address, (uint8_t) 3);
  for(int i=0; i<3; i++){
    _data[i] = Wire.read();
  }
  return _cal * ((float)_data[0] + ( (float)_data[1] / 10.0) + ( (float)_data[2] / 100.0));
}


float Sensors::read_mprls(uint8_t channel){
  uint8_t _size = 7;
  uint8_t _data[_size];
  uint8_t _request[3] = {0xAA, 0x00, 0x00};

  Wire.beginTransmission(_MPRLS_DEFAULT_ADDRESS);
  int _status = Wire.write(_request, 3);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }

  delay(6);
  Wire.requestFrom(_MPRLS_DEFAULT_ADDRESS, _size);
  for(int i=0; i < 7; i++){
    _data[i] = Wire.read();
  }
  _press_counts = (float)_data[3] + (float)_data[2] * 256 + (float)_data[1] * 65536;
  _pressure = (( (_press_counts - _out_min) * (_p_max - _p_min) ) / (_out_max - _out_min)) + _p_min;
  return _pressure * _conversion;
}


float Sensors::read_sen0546_temperature(uint8_t channel){
  uint8_t _size = 2;
  uint8_t _buffer[_size];
  uint16_t _data;

  Wire.beginTransmission(_SEN0546_ADDRESS);
  int _status = Wire.write(_temperature_register);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }

  delay(10);
  Wire.requestFrom(_SEN0546_ADDRESS, _size);
  for(int i=0; i<2; i++){
    _buffer[i] = Wire.read();
  }
  _data = _buffer[0] << 8 | _buffer[1];
  return 165 * ( (float)_data / 65535.0) - 40;
}


float Sensors::read_sen0546_humidity(uint8_t channel){
  uint8_t _size = 2;
  uint8_t _buffer[_size];
  uint16_t _data;

  Wire.beginTransmission(_SEN0546_ADDRESS);
  int _status = Wire.write(_humidity_register);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }

  delay(10);
  Wire.requestFrom(_SEN0546_ADDRESS, _size);
  for(int i=0; i<2; i++){
    _buffer[i] = Wire.read();
  }
  _data = _buffer[0] << 8 | _buffer[1];
  return 100 * ( (float)_data / 65535.0);
}

float Sensors::read_sen0343_diffpressure(uint8_t channel){
  uint8_t _config = {0xaa,0x00,0x80};
  uint8_t _request=0x01;
  uint8_t _size = 7;
  uint8_t _data[_size];

  Wire.beginTransmission(_SEN0343_ADDRESS);
  int _status = Wire.write(_config, 3);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }
  Serial.println("1");
  delay(30);

  Wire.beginTransmission(_SEN0343_ADDRESS);
  int _status = Wire.write(_request);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }
  Serial.println("2");

  delay(10);
  Wire.requestFrom(_SEN0343_ADDRESS, _size);
  for(int i=0; i<_size; i++){
    _data[i] = Wire.read();
  }
  uint16_t _pressure_data = (data[1] << 8) + data[2];
  _pressure_data = _pressure_data >> 2;
  float _diff_pressure = ((500 - (-500)) / 16384.0) * (float) _pressure_data + (-500);
  return _diff_pressure;
}

Output::Output(int Channel, int Pin, String Type, int Control_mode, int Value){
  channel = Channel;
  pin = Pin;
  type = Type;
  control_mode = Control_mode;
  manual_value = Value;
}

void Output::get_info(void){
  Serial.println(type);
}

void Output::write_output(void) {

  switch (control_mode) {
    case MANUAL:
      value = manual_value;
      break;

    case TIMER:
      _delta_time = _delta_time + 1;
      if (_delta_time > _current_timer) {
        if (_is_on) {
          _is_on = false;
        }
        else {
          _is_on = true;
        }
        _delta_time = 0;
      }

      if (_is_on) {
        _current_timer = _time_on;
        value = manual_value;
      }
      else {
        _current_timer = _time_off;
        value = 0;
      }
      break;

    case PID:
      _filtered_input = _alpha * (*_input_value) + (1 - _alpha) * _filtered_input;
      value = compute_pid(_filtered_input);
      break;

    case ONOFF:
      if ((*_input_value) < _input_value_lb) {
        value = manual_value;
      }
      if ((*_input_value) > _input_value_ub) {
        value = 0;
      }
      break;
  }
  ledcWrite(channel, value);
}

void Output::set_manual_output(int value){
  control_mode = 0;
  manual_value = value;
}

void Output::set_timer(int time_on, int time_off, int value){
  control_mode = 1;
  _time_on = time_on;
  _time_off = time_off;
  manual_value = value;

  _delta_time = 0;
  _current_timer = _time_on;
  _is_on = true;
}

void Output::set_pid(float *input_value, float setpoint){
  control_mode = 2;
  _input_value = input_value;
  _setpoint = setpoint;
}

void Output::set_onoff(float *input_value, int lb, int ub, int value){
  control_mode = 3;
  _input_value = input_value;
  _input_value_lb = lb;
  _input_value_ub = ub;
  manual_value = value;
}

float Output::compute_pid(float input) {
  float output;
  uint32_t now = micros();
  uint32_t time_change = (now - _last_time);

  float error = _setpoint - input;
  float d_error = error - _last_error;

  float p_term = _kp * error;
  float i_term = _ki  * error;
  float d_term = _kd * d_error;

  _integral_sum += i_term;
  _integral_sum = constrain(_integral_sum, _output_min, _output_max);
  output = constrain(_integral_sum + p_term + d_term, _output_min, _output_max);

  _last_error = error;
  _last_time = now;

  return output;
}

void Output::set_sample_time_us(uint32_t sample_time_us){
  _sample_time_us = sample_time_us;
  float _sample_time_s = (float) _sample_time_us / 1000000;
}

void Output::set_gh_filter(float alpha){
  _alpha = alpha;
}

void Output::set_pid_tunings(float Kp, float Ki, float Kd){
  _kp = Kp;
  _ki = Ki * _sample_time_s;
  _kd = Kd / _sample_time_s;
}

void Output::set_output_limits(float min, float max){
  _output_min = min;
  _output_max = max;
}

void Output::initialize_pid(){
  _last_time = micros() - _sample_time_us;
  _last_error = 0;
  _integral_sum = 0;
  _filtered_input = (*_input_value);
}
