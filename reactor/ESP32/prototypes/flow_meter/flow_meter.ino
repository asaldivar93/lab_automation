#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "QuickPID.h"
#include "bioreactify.h"
#include "comms_handle.h"

#define N_OUTPUTS 6
#define N_INPUTS  8
#define N_PULSES  1
#define N_SLAVES  1

// Serial info
String ADDRESS = "M0";
int transmit_pin = 4;
boolean new_command = false;
String input_string = "";

// Seriaf buffers
String outputs_buffer = "";
String inputs_buffer = "";
String pulses_buffer = "";

// Config Data
Slave slaves[N_SLAVES] =
{ {"S1", {0x58, 0xBF, 0x25, 0x36, 0x78, 0x94}} };

Output outputs[N_OUTPUTS] =
{ {ADDRESS, "pwm", 0, PIN_CH0, MANUAL, 0}, {ADDRESS, "pwm", 1, PIN_CH1, MANUAL, 0},
  {ADDRESS, "pwm", 2, PIN_CH2, MANUAL, 0}, {ADDRESS, "pwm", 3, PIN_CH3, MANUAL, 0},
  {ADDRESS, "pwm", 4, PIN_CH4, MANUAL, 0}
};

Input inputs[N_INPUTS] =
{ {ADDRESS, "adc", 0, "current"}, {ADDRESS, "adc", 1, "dissolved_oxygen"},
  {ADDRESS, "adc", 2, "ph"}, {ADDRESS, "adc", 3, "dump_0"},
  {ADDRESS, "adc", 4, "dump_1"}, {ADDRESS, "adc", 5, "dump_2"},
  {ADDRESS, "adc", 6, "dump_3"}, {ADDRESS, "adc", 7, "dump_4"},
};

Input pulses[N_PULSES] = {{ADDRESS, "pulse", 8, "delta_pressure", PIN_CH5}};

// Measurment Variables
double analog[N_INPUTS]; // This is an accumulator variable for analog inputs
boolean DATA_READY = false;
boolean PULSES_READY = false;
double sample_number;
unsigned long int samples_per_second = 4;
unsigned long int sample_time;
unsigned long int pulses_time = 60000000;

// PID Channels
float Kp = 100;
float Ki = 0.2;
float Kd = 0.;

float pid_filter[N_OUTPUTS], pid_output[N_OUTPUTS], pid_setpoint[N_OUTPUTS];
QuickPID PID_0(&pid_filter[0], &pid_output[0], &pid_setpoint[0]);
QuickPID PID_1(&pid_filter[1], &pid_output[1], &pid_setpoint[1]);
QuickPID PID_2(&pid_filter[2], &pid_output[2], &pid_setpoint[2]);
QuickPID PID_3(&pid_filter[3], &pid_output[3], &pid_setpoint[3]);
QuickPID PID_4(&pid_filter[4], &pid_output[4], &pid_setpoint[4]);
QuickPID PID_5(&pid_filter[5], &pid_output[5], &pid_setpoint[5]);
QuickPID all_pids[N_OUTPUTS] = {PID_0, PID_1, PID_2, PID_3, PID_4, PID_5};

// SPI and I2C sensors
Sensors sensors(SPI_DOUT, SPI_DIN, SPI_CLK);

// Timer for sensors reading
esp_timer_create_args_t timer_sensor_args;
esp_timer_handle_t timer_sensors_handle;
void flag_data_ready(void *p) {
  DATA_READY = true;
}

// Timer for pulse counter
esp_timer_create_args_t timer_pulses_args;
esp_timer_handle_t timer_pulses_handle;
void flag_pulses_ready(void *p) {
  PULSES_READY = true;
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
uint8_t broadcast_address[] = {0x7C, 0x87, 0xCE, 0x27, 0xEF, 0x44};
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

void setup() {
  // Communications Setup
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

  // Sensors instance - Inputs Setup
  Wire.begin();
  sensors.begin(CS0);
  sensors.set_spi_speed(1000000);
  sensors.set_ref_voltage(3.3);
  sensors.set_mprls_range(0, 25);

  inputs[0].read = &Sensors::read_adc;
  inputs[1].read = &Sensors::read_adc;
  inputs[2].read = &Sensors::read_adc;
  inputs[3].read = &Sensors::read_adc;
  inputs[4].read = &Sensors::read_adc;
  inputs[5].read = &Sensors::read_adc;
  inputs[6].read = &Sensors::read_adc;
  inputs[7].read = &Sensors::read_adc;
  pulses[0].read = &Sensors::read_mprls;
  pulses[0].is_on = false;

  for (int i = 0; i < N_PULSES; i++) {
    ledcSetup(pulses[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
    ledcAttachPin(pulses[i].pin, pulses[i].channel);
  }

  // Outputs Setup
  sample_time = set_sample_time(samples_per_second);

  for (int i = 0; i < N_OUTPUTS; i++) {
    if (outputs[i].type == "pwm") {
      ledcSetup(outputs[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
      ledcAttachPin(outputs[i].pin, outputs[i].channel);
      ledcWrite(outputs[i].channel, outputs[i].value);
    }
    all_pids[i].SetMode(all_pids[i].Control::manual);
    all_pids[i].SetOutputLimits(0, 255);
    all_pids[i].SetTunings(Kp, Ki, Kd);
    all_pids[i].SetSampleTimeUs(sample_time);
    all_pids[i].SetAntiWindupMode(all_pids[i].iAwMode::iAwClamp);
  }

  // Initialize measurments
  moving_average();
  get_moving_average();
  inputs[0].value = get_current(analog[0]);
  inputs[1].value = get_dissolved_oxygen(analog[1]);
  inputs[2].value = get_ph(analog[2]);
  inputs[3].value = get_temperature(analog[3]);
  inputs[4].value = get_temperature(analog[4]);
  inputs[5].value = get_temperature(analog[5]);
  inputs[6].value = get_temperature(analog[6]);
  inputs[7].value = get_temperature(analog[7]);
  reset_moving_average();
  delay(100);

  // Set default configuration

  // Initialize timers for sensors and pulses;
  timer_sensor_args.callback = flag_data_ready;
  esp_timer_create(&timer_sensor_args, &timer_sensors_handle);

  timer_pulses_args.callback = flag_pulses_ready;
  esp_timer_create(&timer_pulses_args, &timer_pulses_handle);

  esp_timer_start_once(timer_sensors_handle, sample_time);
  esp_timer_start_once(timer_pulses_handle, pulses_time);
}

uint16_t counter = 0;
void loop() {
  moving_average();
  if (DATA_READY) {
    // Transform data
    get_moving_average();
    inputs[0].value = get_current(analog[0]);
    inputs[1].value = get_dissolved_oxygen(analog[1]);
    inputs[2].value = get_ph(analog[2]);
    inputs[3].value = get_temperature(analog[3]);
    inputs[4].value = get_temperature(analog[4]);
    inputs[5].value = get_temperature(analog[5]);
    inputs[6].value = get_temperature(analog[6]);
    inputs[7].value = get_temperature(analog[7]);

    // Update outputs
    write_output_vals();
    update_inputs_buffer(inputs, N_INPUTS, &inputs_buffer);
    update_outputs_buffer(outputs, N_OUTPUTS, &outputs_buffer);

    // Reset timer
    reset_moving_average();
    esp_timer_start_once(timer_sensors_handle, sample_time);
    DATA_READY = false;
  }

  pulse_counter();
  if (PULSES_READY) {

    Serial.print(counter);
    Serial.print(", ");
    reset_pulse_counters();
    Serial.println(pulses[0].value, 4);
    update_inputs_buffer(pulses, N_PULSES, &pulses_buffer);
    esp_timer_start_once(timer_pulses_handle, pulses_time);
    PULSES_READY = false;
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
  for (int i = 0; i < N_PULSES; i++) {
    double pressure = (sensors.*pulses[i].read)(i);
    if(!pulses[i].is_on){
      if (pressure >= 11.40) {
        counter = counter + 1;
        pulses[i].delta_pressure += pressure - pulses[i].last_pressure;

        ledcWrite(pulses[i].channel, 150);
        pulses[i].is_on = true;
        pulses[i].start_millis = millis();
      }
    }
    if(pulses[i].is_on){
      if(millis() - pulses[i].start_millis > 500){
        pulses[i].last_pressure = (sensors.*pulses[i].read)(i);
        ledcWrite(pulses[i].channel, 0);
        pulses[i].is_on = false;
      }
    }
  }
}

void reset_pulse_counters(void) {
  counter = 0;
  for (int i = 0; i < N_PULSES; i++) {
    pulses[i].value = pulses[i].delta_pressure;
    pulses[i].delta_pressure = 0;
  }
}

void write_output_vals(void) {
  for (int i = 0; i < N_OUTPUTS; i++) {
    switch (outputs[i].control_mode) {
      case MANUAL:
        outputs[i].value = outputs[i].manual_value;
        break;

      case TIMER:
        outputs[i].delta_time = outputs[i].delta_time + 1;
        if (outputs[i].delta_time > outputs[i].current_timer) {
          if (outputs[i].is_on) {
            outputs[i].is_on = false;
          }
          else {
            outputs[i].is_on = true;
          }
          outputs[i].delta_time = 0;
        }
        if (outputs[i].is_on) {
          outputs[i].current_timer = outputs[i].time_on;
          outputs[i].value = outputs[i].manual_value;
        }
        else {
          outputs[i].current_timer = outputs[i].time_off;
          outputs[i].value = 0;
        }
        break;

      case PID:
        pid_filter[i] = 0.01 * (*outputs[i].input_val) + (1 - 0.01) * pid_filter[i];
        all_pids[i].Compute();
        outputs[i].value = pid_output[i];
        break;

      case ONOFF:
        if ((*outputs[i].input_val) < outputs[i].input_val_lb) {
          outputs[i].value = outputs[i].manual_value;
        }
        if ((*outputs[i].input_val) > outputs[i].input_val_ub) {
          outputs[i].value = 0;
        }
        break;
    }
    ledcWrite(outputs[i].channel, outputs[i].value);
  }
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

      // if (command == GET_BOARD_INFO) {
      //   // GET_BOARD_INFO: "ADDR 0,!"
      //   send_board_info();
      // }
      //
      // if (command == GET_OUTPUTS_INFO){
      //   // GET_OUTPUTS_INFO: "ADDR 2,!"
      //   String outputs_info = get_outputs_info(outputs, N_OUTPUTS) + "!";
      //   Serial.println(outputs_info);
      //   //write_to_master(outputs_info, transmit_pin);
      // }
      //
      // if (command == GET_INPUTS_INFO){
      //   // GET_INPUTS_INFO: "ADDR 3,!"
      //   String inputs_info = get_inputs_info(inputs, N_INPUTS) + get_inputs_info(pulses, N_PULSES) + "!";
      //   Serial.println(inputs_info);
      //   //write_to_master(inputs_info, transmit_pin);
      // }
      //
      // if (command == GET_OUTPUTS_DATA) {
      //   // GET_OUTPUTS_DATA: "ADDR 4,!"
      //   String data_string = outputs_buffer + "!";
      //   Serial.println(data_string);
      //   //write_to_master(data_string, transmit_pin);
      // }
      //
      // if (command == GET_INPUTS_DATA) {
      //   // GET_INPUTS_DATA: "ADDR 5,!"
      //   String data_string = inputs_buffer + pulses_buffer + "!";
      //   Serial.println(data_string);
      //   //write_to_master(data_string, transmit_pin);
      // }
      //
      // if (command == GET_ALL_INPUTS) {
      //   // GET_ALL_INPUTS: "ADDR 6,!"
      //   send_input_data(ADDRESS, slaves, N_SLAVES, &incoming_message, &inputs_buffer, &pulses_buffer);
      // }
      //
      // if (command == GET_ALL_OUTPUTS) {
      //   // GET_ALL_OUTPUTS: "ADDR 7,!"
      //   send_output_data(ADDRESS, slaves, N_SLAVES, &incoming_message, &outputs_buffer);
      //}

      if (command == TOGGLE_CONTROL_MODE) {
        // MANUAL: "ADDR, 1,0,OUT_CHANNEL,PWM,!"
        // TIMER:  "ADDR, 1,1,OUT_CHANNEL,TIME_ON,TIME_OFF,PWM,!"
        // PID:    "ADDR, 1,2,OUT_CHANNEL,IN_CHANNEL,SETPOINT,!"
        // ONOFF:  "ADDR, 1,3,OUT_CHANNEL,IN_CHANNEL,LOWER_BOUND,UPPER_BOUND,PWM,!"

        lastcomma = firstcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int control_mode = input_string.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int out_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();

        outputs[out_channel].control_mode = control_mode;

        switch (outputs[out_channel].control_mode) {
          case MANUAL: {
              lastcomma = nextcomma;
              nextcomma = input_string.indexOf(',', lastcomma + 1);
              int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

              outputs[out_channel].manual_value = value;
              all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);
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

              outputs[out_channel].manual_value = value;
              all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);
              outputs[out_channel].time_on = time_on;
              outputs[out_channel].time_off = time_off;
              outputs[out_channel].delta_time = 0;
              outputs[out_channel].current_timer = outputs[out_channel].time_on;
              outputs[out_channel].is_on = true;
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

              all_pids[out_channel].SetMode(all_pids[out_channel].Control::automatic);
              pid_setpoint[out_channel] = setpoint;

              outputs[out_channel].input_val = &inputs[in_channel].value;
              pid_filter[out_channel] = inputs[in_channel].value;
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

              outputs[out_channel].manual_value = value;
              outputs[out_channel].input_val_lb = lower_bound;
              outputs[out_channel].input_val_ub = upper_bound;
              all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);

              outputs[out_channel].input_val = &inputs[in_channel].value;
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
    }
    new_command = false;
  }
}
