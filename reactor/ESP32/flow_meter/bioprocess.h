#include <Arduino.h>

#define PSI_TO_KPA 6.89476
#define MPRLS_DEFAULT_ADDRESS 0x18
#define SEN0322_DEFAULT_ADDRESS 0x73
#define SEN0322_ADDRESS_0 0x70
#define SEN0322_ADDRESS_1 0x71
#define SEN0322_ADDRESS_2 0x72

class MPRLS{
public:
  MPRLS(double p_min=0, double p_max=25, double K=1);
  double read(void);

private:
  uint8_t _command[3] = {0xAA, 0x00, 0x00};
  double _p_min;
  double _p_max;
  double _out_min=1677722;
  double _out_max=15099494;
  double _K;
  double _press_counts = 0;
  double _pressure;
};

class SEN0322{
public:
  SEN0322(uint8_t address = SEN0322_DEFAULT_ADDRESS);
  double read(void);

private:
  uint8_t _address;
  uint8_t _o2_data_register = 0x03;
  double _oxygen;
  double _cal = 20.9 / 100;
};

class SEN0546{
public:
  SEN0546(void);
  double read_temperature(void);
  double read_humidity(void);

private:
  uint8_t _address = 0x40;
  uint8_t _temperature_register = 0x00;
  uint8_t _humidity_register = 0x01;
  double _humidity;
  double _temperature;
};
