#include <Arduino.h>
#include <esp_now.h>
#include <Wire.h>

#include "bioreactify.h"
#include "comms_handle.h"

#define N_OUTPUTS 5
#define N_INPUTS  0

// Serial info
String ADDRESS = "M0";
boolean new_command = false;
String input_string = "";

// Seriaf buffers
String outputs_buffer = "";
String inputs_buffer = "";

float Kp = 100;
float Ki = 0.2;
float Kd = 0.;

// Measurment Variables
float analog[N_INPUTS]; // This is an accumulator variable for analog inputs
boolean DATA_READY = false;
int sample_number;
uint32_t samples_per_second = 4;
uint32_t one_second = 1000000;
uint32_t sample_time = one_second / samples_per_second;

// Config Data
Input inputs[N_INPUTS];

Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output outputs[N_OUTPUTS] = {CH_0, CH_1, CH_2, CH_3, CH_4};

Sensors sensors(SPI_DOUT, SPI_DIN, SPI_CLK);

// Timer for sensors reading
esp_timer_create_args_t timer_sensor_args;
esp_timer_handle_t timer_sensors_handle;
void flag_data_ready(void *p) {
  DATA_READY = true;
}

// Parallel task for communication
TaskHandle_t serial_comms;
void serial_comms_code(void *parameters) {
  for (;;) {
    String input_string = parse_serial_master(&new_command);
    parse_string(input_string);
  }
}

// Inint functions
void init_comms(void){
  Wire.begin();
  Serial.begin(230400);
  xTaskCreatePinnedToCore(serial_comms_code, "serial_comms", 10000, NULL, 0, &serial_comms, 0);
}

void init_outputs(void){
  // True outputs
  for (int i = 0; i < N_OUTPUTS; i++) {
    ledcSetup(outputs[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
    ledcAttachPin(outputs[i].pin, outputs[i].channel);
    ledcWrite(outputs[i].channel, outputs[i].value);

    outputs[i].set_output_limits(0, 255);
    outputs[i].set_pid_tunings(Kp, Ki, Kd);
    outputs[i].set_sample_time_us(sample_time);
    outputs[i].set_gh_filter(0.01);
  }
}

void init_timers(void){
  // Initialize timers for sensors and volumes;
  timer_sensor_args.callback = flag_data_ready;
  esp_timer_create(&timer_sensor_args, &timer_sensors_handle);
  esp_timer_start_once(timer_sensors_handle, sample_time);
}

void setup() {
  // Initialize functions
  init_comms();
  init_outputs();
  init_timers();
  Wire.begin();
  sensors.begin(CS0);
  // ------------ Modify Configuration -------------- //
  
  // Initialize measurments

  
  // Initial Setup

  // ------------ Modify Configuration -------------- //
}

void loop() {
  
  if (DATA_READY) {
    // Transform data
    uint8_t bus = 0;
    Wire.beginTransmission(0x70);
    int _status = Wire.write(0x04 | bus);
    _status = Wire.endTransmission();
    Serial.print(sensors.read_sen0546_temperature(0));
    Serial.print(", ");
    bus = 1;
    Wire.beginTransmission(0x70);
    _status = Wire.write(0x04 | bus);
    _status = Wire.endTransmission();
    Serial.println(sensors.read_sen0546_temperature(0));
    
    
    // ------------ Modify Configuration -------------- //
    outputs[1].write_output();
    outputs[2].write_output();
    outputs[3].write_output();
    outputs[4].write_output();
    // ------------ Modify Configuration -------------- //
    
    // Update outputs

    // Reset timer
    esp_timer_start_once(timer_sensors_handle, sample_time);
    DATA_READY = false;
  }
}


float get_current(float voltage){
  float current = (voltage - 2.5012) / -0.067;
  return current;
}

float get_dissolved_oxygen(float voltage){
  float dissolved_oxygen = 85.1312926551 * voltage - 49.7487023023;
  return dissolved_oxygen;
}

float get_ph(float voltage){
  float ph = 6.006 * voltage - 3.5108;
  return ph;
}

float get_temperature(float voltage){
  float input_voltage = 3.3;
  float resistor_reference = 10000; // Vaulue of the termistor reference resistor in series
  float resistance = resistor_reference / ((input_voltage / voltage) - 1);
  float temperature = (1 / ( 8.7561e-4 + 2.5343e-4*log(resistance) + 1.84499e-7*pow(log(resistance), 3) )) - 273.15;
  return temperature;
}

void parse_string(String input_string) {
  int firstcomma;
  int lastcomma;
  int nextcomma;
  int command;
  String ok_string = "{}115,!";

  if (new_command) {
    String ADDR = input_string.substring(0, input_string.indexOf(' '));

    if (ADDR == ADDRESS) {
      firstcomma = input_string.indexOf(',');
      command = input_string.substring(input_string.indexOf(' '), firstcomma).toInt();

      if (command == TOGGLE_CONTROL_MODE) {
        // MANUAL: "ADDR 1,0,OUT_CHANNEL,PWM,!"
        // TIMER:  "ADDR 1,1,OUT_CHANNEL,TIME_ON,TIME_OFF,PWM,!"
        // PID:    "ADDR 1,2,OUT_CHANNEL,IN_CHANNEL,SETPOINT,!"
        // ONOFF:  "ADDR 1,3,OUT_CHANNEL,IN_CHANNEL,LOWER_BOUND,UPPER_BOUND,PWM,!"

        lastcomma = firstcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int control_mode = input_string.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int out_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();

        switch (control_mode) {
          case MANUAL: {
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

              outputs[out_channel].set_manual_output(value);
              Serial.println(ok_string);
              break;
            }

          case TIMER: {
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int time_on = input_string.substring(lastcomma + 1, nextcomma).toInt();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int time_off = input_string.substring(lastcomma + 1, nextcomma).toInt();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

              outputs[out_channel].set_timer(time_on, time_off, value);
              Serial.println(ok_string);
              break;
            }

          case PID: {
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int in_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              float setpoint = input_string.substring(lastcomma + 1, nextcomma).toFloat();

              outputs[out_channel].set_pid(&inputs[in_channel].value, setpoint);
              outputs[out_channel].initialize_pid();
              Serial.println(ok_string);
              break;
            }

          case ONOFF: {
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int in_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              float lower_bound = input_string.substring(lastcomma + 1, nextcomma).toFloat();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              float upper_bound = input_string.substring(lastcomma + 1, nextcomma).toFloat();
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

              outputs[out_channel].set_onoff(&inputs[in_channel].value, lower_bound, upper_bound, value);
              Serial.println(ok_string);
              break;
            }
        }
      }
    }
    new_command = false;
  }
}
