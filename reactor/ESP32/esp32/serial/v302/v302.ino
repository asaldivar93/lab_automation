#include <Arduino.h>
#include "MCP_ADC.h"
#include "QuickPID.h"

#define N_OUTPUTS 6
#define N_INPUTS  8

// commands
#define GET_BOARD_INFO 0
#define TOGGLE_CONTROL_MODE 1
#define UPDATE_SAMPLES_PER_SECONDS 2

// Output control_modes
#define MANUAL 0
#define TIMER 1
#define PID 2
#define ONOFF 3

// Communication pins to MCP3208 ADC
#define MCP_CLK  5
#define MCP_DOUT 18
#define MCP_DIN  19
#define CS1      23

// PWM PARAMETERS
#define LEDC_BIT 8
#define LEDC_BASE_FREQ 800

typedef struct{
  String address;
  String type;
  int channel;
  int pin;
  int control_mode;
  int value;
  float *input_val;
  float input_val_lb;
  float input_val_ub;
  int time_on;
  int time_off;
  int delta_time;
  int current_timer;
  bool is_on;
} output;

typedef struct{
  String type;
  int channel;
  String variable;
  float value;
} input;

String ADDRESS = "M0";
boolean new_command = false;
String inputString = "";

// Measurment Variables
unsigned long analog[] = {0, 0, 0, 0, 0, 0, 0, 0}; // This is an accumulator variable for analog inputs
boolean READING = false;
float sample_number;
float ref_voltage = 3.3;
uint32_t samples_per_second = 4;
uint32_t sample_time;

// Config Data
output outputs[N_OUTPUTS] =
  {{ADDRESS, "pwm", 0, 13, MANUAL, 0}, {ADDRESS, "pwm", 1, 12, MANUAL, 0},
   {ADDRESS, "pwm", 2, 14, MANUAL, 0}, {ADDRESS, "pwm", 3, 27, MANUAL, 0},
   {ADDRESS, "pwm", 4, 26, MANUAL, 0}, {ADDRESS, "pwm", 5, 15, MANUAL, 0}};

input inputs[N_INPUTS] =
  {{"analog", 0, "current", 0}, {"analog", 1, "dissolved_oxygen", 0},
   {"analog", 2, "ph", 0}, {"analog", 3, "temperature_0", 0},
   {"analog", 4, "temperature_1", 0}, {"analog", 5, "temperature_2", 0},
   {"analog", 6, "temperature_3", 0}, {"analog", 7, "temperature_4", 0}};

// PID values
float           Kp = 100;
float           Ki = 0.2;
float           Kd = 0.;

float manual_outputs[6];
float pid_filter[6], pid_output[6], pid_setpoint[6];
QuickPID PID_0(&pid_filter[0], &pid_output[0], &pid_setpoint[0]);
QuickPID PID_1(&pid_filter[1], &pid_output[1], &pid_setpoint[1]);
QuickPID PID_2(&pid_filter[2], &pid_output[2], &pid_setpoint[2]);
QuickPID PID_3(&pid_filter[3], &pid_output[3], &pid_setpoint[3]);
QuickPID PID_4(&pid_filter[4], &pid_output[4], &pid_setpoint[4]);
QuickPID PID_5(&pid_filter[5], &pid_output[5], &pid_setpoint[5]);
QuickPID all_pids[6] = {PID_0, PID_1, PID_2, PID_3, PID_4, PID_5};

// MCP Object
MCP3208 ADC1(MCP_DOUT, MCP_DIN, MCP_CLK);

// Timer Callback function
esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;
void read_temp(void *p) {
  READING = true;
}

void setup() {
  Serial.begin(230400);
  Serial2.begin(230400, SERIAL_8N1, 16, 17);

  // MCP3208 instance
  ADC1.begin(CS1);
  ADC1.setSPIspeed(1000000);

  // PWM Setup
  for(int i=0; i<N_OUTPUTS; i++){
    if(outputs[i].type == "pwm"){
      ledcSetup(outputs[i].channel, LEDC_BASE_FREQ, LEDC_BIT);
      ledcAttachPin(outputs[i].pin, outputs[i].channel);
      ledcWrite(outputs[i].channel, outputs[i].value);
    }
  }

  samples_per_second = 4;
  sample_time = set_sample_time(samples_per_second);
  for (int i = 0; i<N_OUTPUTS; i++){
    all_pids[i].SetMode(all_pids[i].Control::manual);
    all_pids[i].SetOutputLimits(0, 255);
    all_pids[i].SetTunings(Kp, Ki, Kd);
    all_pids[i].SetSampleTimeUs(sample_time);
    all_pids[i].SetAntiWindupMode(all_pids[i].iAwMode::iAwClamp);
  }

  get_moving_average();
  inputs[0].value = get_current();
  inputs[1].value = get_dissolved_oxygen();
  inputs[2].value = get_ph();
  get_temperatures();
  delay(500);
  new_command = true;
  inputString = "r101 1,2,3,4,51.5,!";
  parseString(inputString);
  reset_moving_average();

  // Start Timer
  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {

  if (!READING){
    get_moving_average();
  }

  if (READING) {
    inputs[0].value = get_current();
    inputs[1].value = get_dissolved_oxygen();
    inputs[2].value = get_ph();
    get_temperatures();
    set_output_vals();
    send_data();
    reset_moving_average();

    // Restart Timer
    esp_timer_start_once(timer_handle, sample_time);
    READING = false;
  }
  parseSerial();
  parseString(inputString);
  inputString = "";
}


uint32_t set_sample_time(uint32_t samples_per_second){
  uint32_t sample_time = 1000000 / samples_per_second;
  for (int i = 0; i<N_OUTPUTS; i++){
    all_pids[i].SetSampleTimeUs(sample_time);
  }

  return sample_time;
}

void get_moving_average(void){
  for (byte i = 0; i < ADC1.channels(); i++) {
    analog[i] += ADC1.analogRead(i);
  }
  sample_number += 1;
}


void reset_moving_average(void){
  for (byte i = 0; i < ADC1.channels(); i++) {
    analog[i] = 0;
  }
  sample_number = 0;
}


float get_current(void){
  // Channel 0 has a ACS712 hall effect current sensor
  float voltage_acs712 = (analog[0] / sample_number) * (ref_voltage / 4095);
  float current = (voltage_acs712 - 2.5012) / -0.067;

  return current;
}


float get_dissolved_oxygen(void){
  // Channel 1 has a resistor connected to ground
  // to record ph and dissolved oxygen
  float voltage_oxygen = analog[1] / sample_number * (ref_voltage / 4095);
  float dissolved_oxygen = 0.4558 * voltage_oxygen;

  return dissolved_oxygen;
}


float get_ph(void){
  // Channel 2 has a resistor connected to ground
  // to record ph and dissolved oxygen
  float voltage_ph = analog[2] / sample_number * (ref_voltage / 4095);
  float ph = 3.9811 * voltage_ph - 3.5106;

  return ph;
}


void get_temperatures(void){
  float input_voltage = 3.3;
  int resistor_reference = 10000; // Vaulue of the termistor reference resistor in series
  float analog_dummy;
  float resistance;

  // Channels 3, 4, 5, 6, 7 have 10Kohm resistors connected to 3.3V
  // to record temperature from a 10Kohm NTC termistor
  for(byte i = 3; i < N_INPUTS; i++){
    analog_dummy = analog[i] / sample_number; // dummy variable
    // S-H equation for thin film resistor
    // sample_values[i] = (1 / ( 8.294e-4 + 2.624e-4*log(r) + 1.369e-7*pow(log(r), 3) )) - 273.15;
    // Calculate the resistance
    resistance = resistor_reference / ((4095 * input_voltage / (analog_dummy * ref_voltage))-1);
    // S-H equation for water resistant termistor
    inputs[i].value = (1 / ( 8.7561e-4 + 2.5343e-4*log(resistance) + 1.84499e-7*pow(log(resistance), 3) )) - 273.15;
  }
}


void set_output_vals(void){
  for(int i=0; i<N_OUTPUTS; i++){
    switch (outputs[i].control_mode){
      case MANUAL:
        outputs[i].value = manual_outputs[i];
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
          outputs[i].value = manual_outputs[i];
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
          outputs[i].value = manual_outputs[i];
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
  String outputs_json = write_outputs_json();
  String inputs_json = write_inputs_json();

  String all_data_json = "{'address': '" + ADDRESS + "', ";
  all_data_json = all_data_json + outputs_json + ", " + inputs_json;
  all_data_json = all_data_json + "}100,!";
  Serial.println(all_data_json);
}


String write_outputs_json(void){
  String outputs_json = "'outputs': [";

  for(byte i=0; i < N_OUTPUTS; i++){
    outputs_json = outputs_json + "(";
    outputs_json = outputs_json + "'" + ADDRESS + outputs[i].type + "',";
    outputs_json = outputs_json + outputs[i].channel + ",";
    outputs_json = outputs_json + outputs[i].value;
    outputs_json = outputs_json + "),";
  }
  outputs_json = outputs_json + "]";

  return outputs_json;
}


String write_inputs_json(void){
  String inputs_json = "'inputs': [";

  for(byte i=0; i < N_INPUTS; i++){
    inputs_json = inputs_json + "(";
    inputs_json = inputs_json + "'" + inputs[i].variable + "',";
    inputs_json = inputs_json + inputs[i].value;
    inputs_json = inputs_json + "),";
  }
  inputs_json = inputs_json + "]";

  return inputs_json;
}


void send_board_info(void){
  String outputs_json = "'outputs': [";

  for(byte i=0; i < N_OUTPUTS; i++){
    outputs_json = outputs_json + "(";
    outputs_json = outputs_json + "'" + outputs[i].address + "',";
    outputs_json = outputs_json + "'" + outputs[i].type + "',";
    outputs_json = outputs_json + outputs[i].channel + ",";
    outputs_json = outputs_json + outputs[i].control_mode;
    outputs_json = outputs_json + "),";
  }
  outputs_json = outputs_json + "]";

  String inputs_json = "'inputs': [";

  for(byte i=0; i < N_INPUTS; i++){
    inputs_json = inputs_json + "(";
    inputs_json = inputs_json + "'" + inputs[i].type + "',";
    inputs_json = inputs_json + inputs[i].channel + ",";
    inputs_json = inputs_json + "'" + inputs[i].variable + "'";
    inputs_json = inputs_json + "),";
  }
  inputs_json = inputs_json + "]";

  String all_data_json = "{'address': '" + ADDRESS + "', 'samples_per_second': " + samples_per_second + ", ";
  all_data_json = all_data_json + outputs_json + ", " + inputs_json;
  all_data_json = all_data_json + "}115,!";
  Serial.println(all_data_json);
}


void parseSerial(void){
  while(Serial.available()){
    char inChar = char(Serial.read());
    inputString += inChar;
    if(inChar == '!'){
      new_command = true;
      break;
    }
  }
}


void parseString(String inputString){
  int firstcomma;
  int lastcomma;
  int nextcomma;
  int command;
  String ok_string = "{}115,!";

  if(new_command){
    String ADDR = inputString.substring(0, inputString.indexOf(' '));

    if(ADDR == ADDRESS){
      firstcomma = inputString.indexOf(',');
      command = inputString.substring(inputString.indexOf(' '), firstcomma).toInt();

      if(command == GET_BOARD_INFO){
//      GET_BOARD_INFO: "ADDR 0,!"
        send_board_info();
      }

//      if(command == UPDATE_SAMPLES_PER_SECONDS){
////      GET_BOARD_INFO: "ADDR 2,SAMPLES_PER_SECOND,!"
//        lastcomma = firstcomma;
//        nextcomma = inputString.indexOf(',', lastcomma + 1);
//        uint32_t samples_per_second = inputString.substring(lastcomma + 1, nextcomma).toInt();
//        sample_time = set_sample_time(samples_per_second)
//      }

      if(command == TOGGLE_CONTROL_MODE){
//      MANUAL: "ADDR, 1,0,OUT_CHANNEL,PWM,!"
//      TIMER:  "ADDR, 1,1,OUT_CHANNEL,TIME_ON,TIME_OFF,PWM,!"
//      PID:    "ADDR, 1,2,OUT_CHANNEL,IN_CHANNEL,SETPOINT,!"
//      ONOFF:  "ADDR, 1,3,OUT_CHANNEL,IN_CHANNEL,LOWER_BOUND,UPPER_BOUND,PWM,!"

        lastcomma = firstcomma;
        nextcomma = inputString.indexOf(',', lastcomma + 1);
        int control_mode = inputString.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = inputString.indexOf(',', lastcomma + 1);
        int out_channel = inputString.substring(lastcomma + 1, nextcomma).toInt();

        outputs[out_channel].control_mode = control_mode;

        switch(outputs[out_channel].control_mode){
          case MANUAL:{
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int value = inputString.substring(lastcomma + 1, nextcomma).toInt();

            manual_outputs[out_channel] = value;
            all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);
            Serial.println(ok_string);
            break;
          }

          case TIMER:{
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int time_on = inputString.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int time_off = inputString.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int value = inputString.substring(lastcomma + 1, nextcomma).toInt();

            manual_outputs[out_channel] = value;
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
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int in_channel = inputString.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            float setpoint = inputString.substring(lastcomma + 1, nextcomma).toFloat();

            all_pids[out_channel].SetMode(all_pids[out_channel].Control::automatic);
            pid_setpoint[out_channel] = setpoint;

            outputs[out_channel].input_val = &inputs[in_channel].value;
            pid_filter[out_channel] = inputs[in_channel].value;
            Serial.println(ok_string);
            break;
          }

          case ONOFF:{
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int in_channel = inputString.substring(lastcomma + 1, nextcomma).toInt();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            float lower_bound = inputString.substring(lastcomma + 1, nextcomma).toFloat();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            float upper_bound = inputString.substring(lastcomma + 1, nextcomma).toFloat();
            lastcomma = nextcomma;
            nextcomma = inputString.indexOf(',', lastcomma + 1);
            int value = inputString.substring(lastcomma + 1, nextcomma).toInt();

            manual_outputs[out_channel] = value;
            outputs[out_channel].input_val_lb = lower_bound;
            outputs[out_channel].input_val_ub = upper_bound;
            all_pids[out_channel].SetMode(all_pids[out_channel].Control::manual);

            outputs[out_channel].input_val = &inputs[in_channel].value;
            Serial.println(ok_string);
            break;
          }
        }
      }
    else{
      Serial2.println(inputString);
    }
    inputString = "";
    new_command = false;
    }
  }
}
