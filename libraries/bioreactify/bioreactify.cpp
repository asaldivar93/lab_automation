#include <Arduino.h>
#include "bioreactify.h"

unsigned long int set_sample_time(unsigned long int samples_per_second){
  unsigned long int sample_time = 1000000 / samples_per_second;
  return sample_time;
}


double get_current(double voltage){
  double current = (voltage - 2.5012) / -0.067;
  return current;
}


double get_dissolved_oxygen(double voltage){
  double dissolved_oxygen = 0.4558 * voltage;
  return dissolved_oxygen;
}


double get_ph(double voltage){
  double ph = 3.9811 * voltage - 3.5106;
  return ph;
}


double get_temperature(double voltage){
  double input_voltage = 3.3;
  double resistor_reference = 10000; // Vaulue of the termistor reference resistor in series
  double resistance = resistor_reference / ((input_voltage / voltage) - 1);
  double temperature = (1 / ( 8.7561e-4 + 2.5343e-4*log(resistance) + 1.84499e-7*pow(log(resistance), 3) )) - 273.15;
  return temperature;
}


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


void Sensors::set_ref_voltage(double ref_voltage){
  _ref_voltage = ref_voltage;
}

void Sensors::set_spi_speed(uint32_t speed){
  _SPIspeed = speed;
  _spi_settings = SPISettings(_SPIspeed, MSBFIRST, SPI_MODE0);
}

void Sensors::set_mprls_range(double p_min, double p_max){
  _p_min = p_min;
  _p_max = p_max;
}


double Sensors::read_adc(uint8_t channel){
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


double Sensors::read_sen0322_address_0(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_0);
}


double Sensors::read_sen0322_address_1(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_1);
}


double Sensors::read_sen0322_address_2(uint8_t channel){
  return _read_sen0322(_SEN0322_ADDRESS_2);
}


double Sensors::read_sen0322_default(uint8_t channel){
  return _read_sen0322(_SEN0322_DEFAULT_ADDRESS);
}


double Sensors::_read_sen0322(uint8_t address){
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
  return _cal * ((double)_data[0] + ( (double)_data[1] / 10.0) + ( (double)_data[2] / 100.0));
}


double Sensors::read_mprls(uint8_t channel){
  uint8_t _size = 7;
  uint8_t _data[_size];

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
  _press_counts = (double)_data[3] + (double)_data[2] * 256 + (double)_data[1] * 65536;
  _pressure = (( (_press_counts - _out_min) * (_p_max - _p_min) ) / (_out_max - _out_min)) + _p_min;
  return _pressure * _conversion;
}


double Sensors::read_sen0546_temperature(uint8_t channel){
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
  return 165 * ( (double)_data / 65535.0) - 40;
}


double Sensors::read_sen0546_humidity(uint8_t channel){
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
  return 100 * ( (double)_data / 65535.0);
}
