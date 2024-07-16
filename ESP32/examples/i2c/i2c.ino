#include <Arduino.h>

#include "bioreactify.h"
#include "comms_handle.h"

#define N_I2C 3

uint8_t multiplexer_address = 0x70;
Sensors sensors(multiplexer_address);

Input DCH_0(0, "i2c", "oxygen");
Input DCH_1(1, "i2c", "temperature_0");
Input DCH_2(2, "i2c", "temperature_1");
Input digitals[N_I2C] = {DCH_0, DCH_1, DCH_2};

// Timer Configuration
uint32_t one_second = 1000000;

boolean TIMER_1 = false;
uint32_t samples_per_second_timer_1 = 4;
uint32_t cycle_time_timer_1 = one_second / samples_per_second_timer_1;

boolean TIMER_2 = false;
uint32_t samples_per_second_timer_2 = 1;
uint32_t cycle_time_timer_2 = one_second / samples_per_second_timer_2;

esp_timer_create_args_t timer_1_args;
esp_timer_handle_t timer_1_handle;
void timer_1_callback(void *p) {
  TIMER_1 = true;
}

esp_timer_create_args_t timer_2_args;
esp_timer_handle_t timer_2_handle;
void timer_2_callback(void *p) {
  TIMER_2 = true;
}

void setup(){
  Wire.begin();
  timer_1_args.callback = timer_1_callback;
  esp_timer_create(&timer_1_args, &timer_1_handle);
  esp_timer_start_once(timer_1_handle, cycle_time_timer_1);

  timer_2_args.callback = timer_2_callback;
  esp_timer_create(&timer_2_args, &timer_2_handle);
  esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
}

void loop(){
  // Sample analog channels as fast as possible
  for(int i=0; i<N_ADC; i++){
    uint8_t channel = analogs[i].channel;
    analogs[i].moving_average(adc_0.read_adc(channel));
  }

  if(TIMER_1){
    // Do something
    esp_timer_start_once(timer_1_handle, cycle_time_timer_1);
    TIMER_1 = false;
  }

  if(TIMER_2){
    // Do something else
    sensors.set_multiplexer_channel(0);
    digitals[0].value = sensors.read_sen0322_default();
    digitals[1].value = sensors.read_sen0546_temperature();

    sensors.set_multiplexer_channel(1);
    digitals[2].value = sensors.read_sen0546_temperature();

    esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
    TIMER_2 = false;
  }
}
