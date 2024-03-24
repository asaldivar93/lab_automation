#include <Arduino.h>
#include "bioprocess.h"

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
