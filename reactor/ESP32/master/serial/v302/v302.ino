#include <Arduino.h>
//#include "MCP_ADC.h"
#include "QuickPID.h"
#include "bioprocess.h"
#include "comms_handle.h"

#define N_OUTPUTS 6
#define N_INPUTS  9
#define N_SLAVES  1

// MAX485 PINS
int transmit_pin = 4;
String ADDRESS = "M0";
boolean new_command = false;

// Config Data
String slaves[N_SLAVES] = {"S2"};

Output outputs[N_OUTPUTS] =
  {{ADDRESS, "pwm", 0, 13, MANUAL, 0}, {ADDRESS, "pwm", 1, 12, MANUAL, 0},
   {ADDRESS, "pwm", 2, 14, MANUAL, 0}, {ADDRESS, "pwm", 3, 27, MANUAL, 0},
   {ADDRESS, "pwm", 4, 26, MANUAL, 0}, {ADDRESS, "pwm", 5, 15, MANUAL, 0}};

Input inputs[N_INPUTS] =
  {{ADDRESS, "adc", 0, "current", 0}, {ADDRESS, "adc", 1, "dissolved_oxygen", 0},
   {ADDRESS, "adc", 2, "ph", 0}, {ADDRESS, "adc", 3, "temperature_0", 0},
   {ADDRESS, "adc", 4, "temperature_1", 0}, {ADDRESS, "adc", 5, "temperature_2", 0},
   {ADDRESS, "adc", 6, "temperature_3", 0}, {ADDRESS, "adc", 7, "temperature_4", 0},
   {ADDRESS, "i2c", 8, "humidity", 0}};

// Measurment Variables
double analog[N_INPUTS]; // This is an accumulator variable for analog inputs
boolean READING = false;
double sample_number;
unsigned long int samples_per_second = 4;
unsigned long int sample_time;

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

// Timer for data reading
esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;
unsigned long start_millis;
void read_data(void *p) {
  READING = true;
  start_millis = millis();
}

// Parallel tasks for communication
TaskHandle_t comms_task;
void comms_task_code(void * pvParameters){
  for(;;){
    if(READING){
      delay(10);
      //send_data();
    }
    String input_string = parse_serial_master();
    parse_string(input_string);
  }
}

void setup() {
  // Communications Setup
  Serial.begin(230400);
  Serial2.begin(9600, SERIAL_8N1, Rx, Tx);
  pinMode(transmit_pin, OUTPUT);
  digitalWrite(transmit_pin, LOW);
  xTaskCreatePinnedToCore(comms_task_code, "comms_task", 10000, NULL, 0, &comms_task, 0);

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
  inputs[8].read = &Sensors::read_mprls;

  for(int i=0; i<N_INPUTS; i++){
    set_input_mssg_bp(&inputs[i]);
  }

  // Outputs Setup
  samples_per_second = 4;
  sample_time = set_sample_time(samples_per_second);

  for(int i=0; i<N_OUTPUTS; i++){
    set_output_mssg_bp(&outputs[i]);
    if(outputs[i].type == "pwm"){
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

  // Set initial configuration
  String input_string = "M0 1,2,3,4,51.5,!";
  new_command = true;
  parse_string(input_string);

  // Start Timer
  create_args.callback = read_data; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {
  if(!READING){
    moving_average();
  }

  if(READING){
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
    inputs[8].value = analog[8];

    // Update outputs 
    set_output_vals();

    // Reset timer
    reset_moving_average();
    esp_timer_start_once(timer_handle, sample_time);
    READING = false;
  }
}


void moving_average(void){
  for (int i = 0; i < N_INPUTS; i++) {
    analog[i] += (sensors.*inputs[i].read)(i);
  }
  sample_number += 1;
}


void get_moving_average(void){
  float ref_voltage = 3.3;
  for (int i = 0; i < N_INPUTS; i++) {
    analog[i] = (analog[i] / sample_number);
  }
}


void reset_moving_average(void){
  for (int i = 0; i < N_INPUTS; i++) {
    analog[i] = 0;
  }
  sample_number = 0;
}


void set_output_vals(void){
  for(int i=0; i<N_OUTPUTS; i++){
    switch (outputs[i].control_mode){
      case MANUAL:
        outputs[i].value = outputs[i].manual_value;
        break;

      case TIMER:
        outputs[i].delta_time = outputs[i].delta_time + 1;
        if(outputs[i].delta_time > outputs[i].current_timer){
          if(outputs[i].is_on){
            outputs[i].is_on = false;
          }
          else{
            outputs[i].is_on = true;
          }
          outputs[i].delta_time = 0;
        }
        if(outputs[i].is_on){
          outputs[i].current_timer = outputs[i].time_on;
          outputs[i].value = outputs[i].manual_value;
        }
        else{
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
        if((*outputs[i].input_val) < outputs[i].input_val_lb){
          outputs[i].value = outputs[i].manual_value;
        }
        if((*outputs[i].input_val) > outputs[i].input_val_ub){
          outputs[i].value = 0;
        }
        break;
    }
    ledcWrite(outputs[i].channel, outputs[i].value);
  }
}


void send_data(void){
  String outputs_json = write_outputs_string();
  String inputs_json = write_inputs_string();

  String all_data_json = "{";
  all_data_json = all_data_json + outputs_json + "," + inputs_json;
  all_data_json = all_data_json + "}100,!";
  Serial.println(all_data_json);
}


String write_outputs_string(void){

  String outputs_string = "'outs':{'" + ADDRESS + "':[";
  for(int i=0; i < N_OUTPUTS; i++){
    outputs_string = outputs_string + get_output_data(outputs[i]);
  }
  outputs_string = outputs_string + "],";

  String slaves_output_data = "";
  if(N_SLAVES>0){
    for(int i=0; i < N_SLAVES; i++){
      slaves_output_data = slaves_output_data + "'" + slaves[i] + "':[";
      slaves_output_data = slaves_output_data + request_outputs_data(slaves[i], transmit_pin);
      slaves_output_data = slaves_output_data + "],";
    }
  }

  outputs_string = outputs_string + slaves_output_data;
  outputs_string = outputs_string + "}";

  return outputs_string;
}

String write_inputs_string(void){

  String inputs_string = "'ins':{'" + ADDRESS + "':[";
  for(int i=0; i < N_INPUTS; i++){
    inputs_string = inputs_string + get_input_data(inputs[i]);
  }
  inputs_string = inputs_string + "],";

  String slaves_input_data = "";
  if(N_SLAVES>0){
    for(int i=0; i < N_SLAVES; i++){
      slaves_input_data = slaves_input_data + "'" + slaves[i] + "':[";
      slaves_input_data = slaves_input_data + request_inputs_data(slaves[i], transmit_pin);
      slaves_input_data = slaves_input_data + "],";
    }
  }
  inputs_string = inputs_string + slaves_input_data;
  inputs_string = inputs_string + "}";

  return inputs_string;
}


void send_board_info(void){
  String slaves_output_info = "";
  String slaves_input_info = "";

  for(int i=0; i<N_SLAVES; i++){    
    slaves_output_info = slaves_output_info + "'" + slaves[i] + "':[";
    slaves_output_info = slaves_output_info + request_outputs_info(slaves[i], transmit_pin);
    slaves_output_info = slaves_output_info + "],";
  }

  delay(10);
  for(int i=0; i<N_SLAVES; i++){
    slaves_input_info = slaves_input_info + "'" + slaves[i] + "':[";
    slaves_input_info = slaves_input_info + request_inputs_info(slaves[i], transmit_pin);
    slaves_input_info = slaves_input_info + "],";
  }

  String outputs_string = "'outs':{'" + ADDRESS + "':[";
  for(int i=0; i < N_OUTPUTS; i++){
    outputs_string = outputs_string + get_output_info(outputs[i]);
  }
  outputs_string = outputs_string + "],";
  outputs_string = outputs_string + slaves_output_info + "}";

  String inputs_string = "'ins':{'" + ADDRESS + "':[";
  for(int i=0; i < N_INPUTS; i++){
    inputs_string = inputs_string + get_input_info(inputs[i]);
  }
  inputs_string = inputs_string + "],";
  inputs_string = inputs_string + slaves_input_info + "}";

  String all_data_json = "{'address': '" + ADDRESS + "', 'samples_per_second': " + samples_per_second + ", ";
  all_data_json = all_data_json + outputs_string + ", " + inputs_string;
  all_data_json = all_data_json + "}115,!";
  Serial.println(all_data_json);
}


String parse_serial_master(void){
  String input_string = "";
  while(Serial.available()){
    char inChar = char(Serial.read());
    input_string += inChar;
    if(inChar == '!'){
      new_command = true;
      break;
    }
  }
  return input_string;
}


void parse_string(String input_string){
  int firstcomma;
  int lastcomma;
  int nextcomma;
  int command;
  String ok_string = "{}115,!";

  if(new_command){
    String ADDR = input_string.substring(0, input_string.indexOf(' '));

    if(ADDR == ADDRESS){
      firstcomma = input_string.indexOf(',');
      command = input_string.substring(input_string.indexOf(' '), firstcomma).toInt();

      if(command == GET_BOARD_INFO){
//      GET_BOARD_INFO: "ADDR 0,!"
        send_board_info();
      }

      if(command == TOGGLE_CONTROL_MODE){
//      MANUAL: "ADDR, 1,0,OUT_CHANNEL,PWM,!"
//      TIMER:  "ADDR, 1,1,OUT_CHANNEL,TIME_ON,TIME_OFF,PWM,!"
//      PID:    "ADDR, 1,2,OUT_CHANNEL,IN_CHANNEL,SETPOINT,!"
//      ONOFF:  "ADDR, 1,3,OUT_CHANNEL,IN_CHANNEL,LOWER_BOUND,UPPER_BOUND,PWM,!"

        lastcomma = firstcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int control_mode = input_string.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int out_channel = input_string.substring(lastcomma + 1, nextcomma).toInt();

        outputs[out_channel].control_mode = control_mode;

        switch(outputs[out_channel].control_mode){
          case MANUAL:{
            lastcomma = nextcomma;
            nextcomma = input_string.indexOf(',', lastcomma + 1);
            int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

            outputs[out_channel].manual_value = value;
            all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);
            Serial.println(ok_string);
            break;
          }

          case TIMER:{
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

          case PID:{
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

          case ONOFF:{
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
    else{
      write_to_slaves(input_string, transmit_pin);
      Serial.println(ok_string);
    }
    new_command = false;
  }
}
