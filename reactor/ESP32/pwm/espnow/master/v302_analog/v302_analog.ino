#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "bioreactify.h"
#include "comms_handle.h"

#define N_OUTPUTS 5
#define N_INPUTS  5
#define N_VOLUMES 0
#define N_SLAVES  1

// Serial info
String ADDRESS = "M0";
int transmit_pin = 4;
boolean new_command = false;
String input_string = "";

// Seriaf buffers
String outputs_buffer = "";
String inputs_buffer = "";
String volumes_buffer = "";

// Config Data
Slave slaves[N_SLAVES] =
{ {"S1", {0x58, 0xBF, 0x25, 0x36, 0x78, 0x94}, "", "('stp',0),('stp',1),('stp',2)"} };

Input inputs[N_INPUTS] =
{ {ADDRESS, "adc", 0, "current"}, {ADDRESS, "adc", 1, "dissolved_oxygen"},
  {ADDRESS, "adc", 2, "ph"}, {ADDRESS, "adc", 3, "temperature_0"},
  {ADDRESS, "adc", 4, "temperature_1"}
};

Input volumes[N_VOLUMES]; // = { {ADDRESS, "vol", 8, "vol_ml_per_10s", PIN_CH5} };

Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output outputs[N_OUTPUTS] = {CH_0, CH_1, CH_2, CH_3, CH_4};

float Kp = 100;
float Ki = 0.2;
float Kd = 0.;

// Measurment Variables
float analog[N_INPUTS]; // This is an accumulator variable for analog inputs
boolean DATA_READY = false;
boolean VOLUMES_READY = false;
int sample_number;
uint32_t samples_per_second = 4;
uint32_t one_second = 1000000;
uint32_t sample_time = one_second / samples_per_second;
uint32_t volumes_time = 0.5 * one_second;

// SPI and I2C sensors
Sensors sensors(SPI_DOUT, SPI_DIN, SPI_CLK);

// Timer for sensors reading
esp_timer_create_args_t timer_sensor_args;
esp_timer_handle_t timer_sensors_handle;
void flag_data_ready(void *p) {
  DATA_READY = true;
}

// Timer for pulse counter
esp_timer_create_args_t timer_volumes_args;
esp_timer_handle_t timer_volumes_handle;
void flag_volumes_ready(void *p) {
  VOLUMES_READY = true;
}

// Parallel task for communication
TaskHandle_t serial_comms;
void serial_comms_code(void *parameters) {
  for (;;) {
    String input_string = parse_serial_master(&new_command);
    parse_string(input_string);
  }
}

// ESPnow
MessageSlaves incoming_message[N_SLAVES];
MessageSlaves buffer_message;
SimpleMessage outgoing_message;
bool waiting;
esp_now_peer_info_t esp_slaves[N_SLAVES];

void on_message_sent(const uint8_t *mac_address, esp_now_send_status_t status){
  if (status == 0){
    waiting = false;
  }
}

void on_message_recv(const uint8_t *mac_address, const uint8_t *inMessage, int size){
  waiting = true;
  memcpy(&buffer_message, inMessage, size);
  String inputString = parse_serial(buffer_message.message, &new_command);
  new_command = false;
}

// Inint functions
void init_comms(void){
  Wire.begin();
  Serial.begin(230400);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(on_message_recv);
  esp_now_register_send_cb(on_message_sent);
  for(int i=0; i<N_SLAVES; i++){
    memcpy(esp_slaves[i].peer_addr, slaves[i].broadcastAddress, 6);
    esp_slaves[i].channel = i;
    esp_slaves[i].encrypt = false;
    esp_now_add_peer(&esp_slaves[i]);
  }
  xTaskCreatePinnedToCore(serial_comms_code, "serial_comms", 10000, NULL, 0, &serial_comms, 0);
}

void init_outputs(void){
  // Solenoids for flow measurments
  for (int i = 0; i < N_VOLUMES; i++) {
    ledcSetup(volumes[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
    ledcAttachPin(volumes[i].pin, volumes[i].channel);
  }

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

  timer_volumes_args.callback = flag_volumes_ready;
  esp_timer_create(&timer_volumes_args, &timer_volumes_handle);
  esp_timer_start_once(timer_volumes_handle, volumes_time);
}

void setup() {
  // Initialize functions
  init_comms();
  init_outputs();
  init_timers();

  // ------------ Modify Configuration -------------- //
  sensors.begin(CS0);
  sensors.set_spi_speed(1000000);
  sensors.set_ref_voltage(3.3);

  // Set readings
  inputs[0].read = &Sensors::read_adc;
  inputs[1].read = &Sensors::read_adc;
  inputs[2].read = &Sensors::read_adc;
  inputs[3].read = &Sensors::read_adc;
  inputs[4].read = &Sensors::read_adc;

  // Initialize measurments
  moving_average();
  get_moving_average();
  inputs[0].value = get_current(analog[0]);
  inputs[1].value = get_dissolved_oxygen(analog[1]);
  inputs[2].value = get_ph(analog[2]);
  inputs[3].value = get_temperature(analog[3]);
  inputs[4].value = get_temperature(analog[4]);

  reset_moving_average();
  delay(100);

  for(int i=0; i<N_VOLUMES; i++){
    volumes[i].last_pressure = (sensors.*volumes[i].read)(i);
  }
  
  // Initial Setup
  //outputs[0].set_timer(15,60,255);
  //outputs[1].set_timer(15,60,125);
  outputs[4].set_pid(&inputs[3].value, 48);
  outputs[4].set_output_limits(0, 240);
  // ------------ Modify Configuration -------------- //
}

void loop() {
  moving_average();
  if (DATA_READY) {
    // Transform data
    get_moving_average();
    // ------------ Modify Configuration -------------- //
    inputs[0].value = analog[0];
    inputs[1].value = get_dissolved_oxygen(analog[1]);
    inputs[2].value = get_ph(analog[2]);
    inputs[3].value = get_temperature(analog[3]);
    inputs[4].value = get_temperature(analog[4]);
    outputs[1].write_output();
    outputs[2].write_output();
    outputs[3].write_output();
    outputs[4].write_output();
    // ------------ Modify Configuration -------------- //

    // Update outputs

    update_inputs_buffer(inputs, N_INPUTS, &inputs_buffer);
    update_outputs_buffer(outputs, N_OUTPUTS, &outputs_buffer);
    // Reset timer
    reset_moving_average();
    esp_timer_start_once(timer_sensors_handle, sample_time);
    DATA_READY = false;
  }

  pulse_counter();
  if (VOLUMES_READY) {
    outputs[0].write_output();
    for (int i = 0; i < N_VOLUMES; i++) {
      volumes[i].value = get_volume(volumes[i].delta_pressure);
    }
    reset_pulse_counters();
    update_inputs_buffer(volumes, N_VOLUMES, &volumes_buffer);
    esp_timer_start_once(timer_volumes_handle, volumes_time);
    VOLUMES_READY = false;
  }

}

void moving_average(void) {
  for (int i = 0; i < N_INPUTS; i++) {
    int channel = inputs[i].channel;
    analog[i] += (sensors.*inputs[i].read)(channel);
  }
  sample_number += 1;
}

void get_moving_average(void) {
  for (int i = 0; i < N_INPUTS; i++) {
    analog[i] = (analog[i] / sample_number);
  }
}

void reset_moving_average(void) {
  for (int i = 0; i < N_INPUTS; i++) {
    analog[i] = 0;
  }
  sample_number = 0;
}

void pulse_counter(void) {
  /*
    This pulses_counter is used to measure gas flow using the difference between p_final and p_initial.

    A pulse cycle goes from p_last (p_initial) to PULSES_PRESSURE_UB. When the pressure goes above the UB,
    the difference between the current pressure (p_final) and p_last is added to the delta pressure
    accumulator (pulse[i].delta_pressure), then a solenoid is activated to reset the pressure,
    and p_last is updated

    A measurment cycle last 60 seconds by default. The reported value is the accumulated delt_pressure
    after the 60s have elapsed
  */
  for (int i = 0; i < N_VOLUMES; i++) {
    double pressure = (sensors.*volumes[i].read)(i);
    if(!volumes[i].is_on){
      if (pressure >= 11) {
        volumes[i].delta_pressure += pressure - volumes[i].last_pressure;

        ledcWrite(volumes[i].channel, 150);
        volumes[i].is_on = true;
        volumes[i].start_millis = millis();
      }
    }
    if(volumes[i].is_on){
      if(millis() - volumes[i].start_millis > 500){
        volumes[i].last_pressure = (sensors.*volumes[i].read)(i);
        ledcWrite(volumes[i].channel, 0);
        volumes[i].is_on = false;
      }
    }
  }
}

void reset_pulse_counters(void) {
  for (int i = 0; i < N_VOLUMES; i++) {
    volumes[i].delta_pressure = 0;
  }
}

double get_volume(double delta_p){
  double volume_chamber = 61.1;
  double p_atm = 10.8;
  double volume = (delta_p * volume_chamber) / p_atm;
  return volume;
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

void send_board_info(void) {
  String slaves_output_info = "";
  String slaves_input_info = "";

  for (int i = 0; i < N_SLAVES; i++) {
    slaves_output_info = slaves_output_info + "'" + slaves[i].name + "':[" + slaves[i].ouputsInfo + "],";
    slaves_input_info = slaves_input_info + "'" + slaves[i].name + "':[" + slaves[i].inputsInfo + "],";
  }

  String this_outs = "";
  for(int i=0; i<N_OUTPUTS; i++){
    this_outs = this_outs + "(" + outputs[i].type + "," + outputs[i].channel + "),";
  }
  String outputs_string = "'outs':{'" + ADDRESS + "':[" + get_outputs_info(outputs, N_OUTPUTS) + "]," + slaves_output_info + "}";
  String inputs_string = "'ins':{'" + ADDRESS + "':[" + get_inputs_info(inputs, N_INPUTS) + get_inputs_info(volumes, N_VOLUMES) + "]," + slaves_input_info + "}";

  String all_data_json = "{'address': '" + ADDRESS + "', 'samples_per_second': " + samples_per_second + ", ";
  all_data_json = all_data_json + outputs_string + ", " + inputs_string;
  all_data_json = all_data_json + "}115,!";
  Serial.println(all_data_json);
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

      if (command == GET_BOARD_INFO) {
        // GET_BOARD_INFO: "ADDR 0,!"
        send_board_info();
      }

      if (command == GET_ALL_DATA) {
        // GET_ALL_INPUTS: "ADDR 2,!"
        String outputs_string = "'outs':[" + outputs_buffer + "],";
        String inputs_string = "'ins':[" + inputs_buffer + volumes_buffer + "]";
        String to_send_string = "{'" + ADDRESS + "':{" + outputs_string + inputs_string + "}}115,!";
        Serial.println(to_send_string);
        inputs_buffer = "";
        volumes_buffer = "";
      }

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
    else {
      strcpy(outgoing_message.message, input_string.c_str());
      for (int i = 0; i < N_SLAVES; i++){
        esp_now_send(slaves[i].broadcastAddress, (uint8_t *) &outgoing_message, sizeof(outgoing_message));
      }
      Serial.println(ok_string);
    }
    new_command = false;
  }
}
