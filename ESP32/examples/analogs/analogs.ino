#include <Arduino.h>

#include "bioreactify.h"
#include "comms_handle.h"

#define N_ADC 6

// ADC Configuration
MCP3208 adc_0(SPI_MISO, SPI_MOSI, SPI_CLK);
Input ACH_0(1, "adc", "temperature_0");
Input ACH_1(2, "adc", "temperature_1");
Input ACH_2(3, "adc", "dissolved_oxygen");
Input ACH_3(4, "adc", "ph");
Input ACH_4(6, "adc", "biomass_1");
Input ACH_5(7, "adc", "biomass_2");
Input analogs[N_ADC] = {ACH_0, ACH_1, ACH_2, ACH_3, ACH_4, ACH_5};

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
  adc_0.begin(CS0);
  adc_0.set_spi_speed(1000000);
  adc_0.set_ref_voltage(3.3);

  float thermistor[3] = {8.7561e-4, 2.5343e-4, 1.84499e-7}; // 103JT-025 semitec
  float ref_resistance = 76800;
  float op_amp_resistors[2] = {1500, 4300};

  for (int i=0; i<2; i+=1){
    analogs[i].set_voltage_divider_resistor(ref_resistance);
    analogs[i].set_opamp_resistors(op_amp_resistors[0], op_amp_resistors[1]);
    analogs[i].set_steinhart_coeffs(thermistor[0], thermistor[1], thermistor[2]);
  }

  analogs[2].set_dissolved_oxygen_cal(85.1312926551, -49.7487023023);
  analogs[3].set_ph_cal(6.006, -3.5108);

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
    for(int i=0; i<N_ADC; i++){
      analogs[i].get_moving_average();
    }

    analogs[0].get_temp_opamp();
    analogs[1].get_temp_opamp();
    analogs[2].get_dissolved_oxygen();
    analogs[3].get_ph();
    analogs[4].value = analogs[4].get_analog_value();
    analogs[5].value = analogs[5].get_analog_value();

    for(int i=0; i<N_ADC; i++){
      analogs[i].reset_moving_average();
    }

    esp_timer_start_once(timer_1_handle, cycle_time_timer_1);
    TIMER_1 = false;
  }

  if(TIMER_2){
    // Do something else
    esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
    TIMER_2 = false;
  }
}
