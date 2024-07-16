#include <Arduino.h>

#include "bioreactify.h"
#include "comms_handle.h"

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
  timer_1_args.callback = timer_1_callback;
  esp_timer_create(&timer_1_args, &timer_1_handle);
  esp_timer_start_once(timer_1_handle, cycle_time_timer_1);

  timer_2_args.callback = timer_2_callback;
  esp_timer_create(&timer_2_args, &timer_2_handle);
  esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
}

void loop(){
  if(TIMER_1){
    // Do something
    esp_timer_start_once(timer_1_handle, cycle_time_timer_1);
    TIMER_1 = false;
  }

  if(TIMER_2){
    // Do something else
    esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
    TIMER_2 = false;
  }
}
