#include <Arduino.h>
#include <Wire.h>
#include "bioprocess.h"

MPRLS::MPRLS(double p_min, double p_max, double K){
  _p_min = p_min;
  _p_max = p_max;
  _K = K;
}

double MPRLS::read(void){
  uint8_t _data[7];
  Wire.beginTransmission(MPRLS_DEFAULT_ADDRESS);
  int _status = Wire.write(_command, 3);
  _status = Wire.endTransmission();

  delay(6);
  Wire.requestFrom(MPRLS_DEFAULT_ADDRESS, 7);
  for(int i=0; i < 7; i++){
    _data[i] = Wire.read();
  }
  _press_counts = _data[3] + _data[2] * 256 + _data[1] * 65536;
  _pressure = ((( (double)_press_counts - _out_min) * (_p_max - _p_min)) / (_out_max - _out_min)) + _p_min;
  _pressure = _pressure * _K;
  return _pressure;
}

SEN0322::SEN0322(uint8_t address){
  _address = address;
}

double SEN0322::read(void){
  uint8_t _data[10]={0};
  Wire.beginTransmission(_address);
  int _status = Wire.write(_o2_data_register);
  _status = Wire.endTransmission();

  delay(5);
  Wire.requestFrom(_address, (uint8_t) 3);
  for(int i=0; i<3; i++){
    _data[i] = Wire.read();
  }
  _oxygen = _cal * ((double)_data[0] + ( (double)_data[1] / 10.0) + ( (double)_data[2] / 100.0));
  return _oxygen;
}

SEN0546::SEN0546(void) {}

double SEN0546::read_temperature(void){
  uint8_t _buffer[2];
  uint16_t _data;

  Wire.beginTransmission(_address);
  int _status = Wire.write(_temperature_register);
  _status = Wire.endTransmission();

  delay(10);
  Wire.requestFrom(_address, (uint8_t) 2);
  for(int i=0; i<2; i++){
    _buffer[i] = Wire.read();
  }
  _data = _buffer[0] << 8 | _buffer[1];
  _temperature =  165 * ( (double)_data / 65535.0) - 40;
  return _temperature;
}

double SEN0546::read_humidity(void){
  uint8_t _buffer[2];
  uint16_t _data;

  Wire.beginTransmission(_address);
  int _status = Wire.write(_humidity_register);
  _status = Wire.endTransmission();
  if(_status != 0){
    return 0;
  }

  delay(10);
  Wire.requestFrom(_address, (uint8_t) 2);
  for(int i=0; i<2; i++){
    _buffer[i] = Wire.read();
  }
  _data = _buffer[0] << 8 | _buffer[1];
  _temperature =  100 * ( (double)_data / 65535.0);
  return _temperature;
}
