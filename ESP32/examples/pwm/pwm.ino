#include <Arduino.h>

#include "bioreactify.h"
#include "comms_handle.h"

#define N_PWM 8

float Kp = 100;
float Ki = 0.2;
float Kd = 0.;

Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output CH_5(5, PIN_CH5, "pwm", MANUAL, 0);
Output CH_6(6, PIN_CH6, "pwm", MANUAL, 0);
Output CH_7(7, PIN_CH7, "pwm", MANUAL, 0);
Output outputs[N_PWM] = {CH_0, CH_1, CH_2, CH_3, CH_4, CH_5, CH_6, CH_7};

void setup(){
  for (int i = 0; i < N_PWM; i++) {
    ledcSetup(outputs[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
    ledcAttachPin(outputs[i].pin, outputs[i].channel);
    ledcWrite(outputs[i].channel, outputs[i].value);

    outputs[i].set_output_limits(0, 255);
    outputs[i].set_pid_tunings(Kp, Ki, Kd);
    outputs[i].set_sample_time_us(250000);
    outputs[i].set_gh_filter(0.01);
  }
}

void loop(){
  for(int i=0; i<N_PWM; i++){
    outputs[i].write_output();
  }
  delay(250);
}
