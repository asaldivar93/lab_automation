#include <Arduino.h>
#include "MCP_ADC.h"
#include "QuickPID.h"

typedef struct{
  String type;
  int channel;
  int pin;
  int mode;
  int value;
} output;

typedef struct{
  String type;
  int channel;
  String variable;
  float value;
} input;

#define N_OUTPUTS 6
#define N_INPUTS  8

#define MANUAL 0
#define TIMER 1
#define PID 2
#define ONOFF 3

#define PWM 0
#define DIGITAL 1

//#define ANALOG 0
//#define I2C 1
//#define SPI 2
//#define FLOW 3

output outputs[N_OUTPUTS] =
  {{"pwm", 0, 13, MANUAL, 0}, {"pwm", 1, 12, MANUAL, 0},
   {"pwm", 2, 14, MANUAL, 0}, {"pwm", 3, 27, MANUAL, 0},
   {"pwm", 4, 26, MANUAL, 0}, {"pwm", 5, 15, MANUAL, 0}};

input inputs[N_INPUTS] =
  {{"analog", 0, "current", 0}, {"analog", 1, "dissolved_oxygen", 0},
   {"analog", 2, "ph", 0}, {"analog", 3, "temperature_0", 0},
   {"analog", 4, "temperature_1", 0}, {"analog", 5, "temperature_2", 0},
   {"analog", 6, "temperature_3", 0}, {"analog", 7, "temperature_4", 0}};

boolean         new_command = false;
String          ADDRESS = "r101";
String          inputString = "";

unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0}; // This is an accumulator variable for analog inputs
float           transformed_analogs[8]; // Variable to store analog values after transformation
float           sample_number;

uint32_t        sample_per_second = 4;
uint32_t        sample_time = 1000000 / sample_per_second;
float           sample_time_seconds = 1000000 / (float) sample_per_second / 1000000;

// Measurment Variables
boolean         READING = false;
float ref_voltage = 3.3;
float           total_current; // variable to store total current
float           dissolved_oxygen;
float           ph;

// Setpoints Variables
float           oxygen_bounds[2] = {0.01, 0.1};
float           temp_setpoint = 0;
float           temp_filter;
float           temp_reactor;
bool            FEED_ON = false;

// PID values
float           Kp = 100;
float           Ki = 0.2;
float           Kd = 0.;

// Communication pins to MCP3208 ADC
#define MCP_CLK  5
#define MCP_DOUT 18
#define MCP_DIN  19
#define CS1      23

// PWM CHANNELS
#define PWM_CHANNEL_0 0
#define PWM_CHANNEL_1 1
#define PWM_CHANNEL_2 2
#define PWM_CHANNEL_3 3
#define PWM_CHANNEL_4 4
#define PWM_CHANNEL_5 5

// PWM PARAMETERS
#define LEDC_BIT 8
#define LEDC_BASE_FREQ 800

// PWM
#define PWM_CHANNEL_0_PIN 13
#define PWM_CHANNEL_1_PIN 12
#define PWM_CHANNEL_2_PIN 14
#define PWM_CHANNEL_3_PIN 27
#define PWM_CHANNEL_4_PIN 26
#define PWM_CHANNEL_5_PIN 25

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

  // Start Timer
  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);
  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {

  if (!READING){
    // Between sampling times accumulate the analog values
    // Moving Average Filter
    for (byte i = 0; i < ADC1.channels(); i++) {
      analog[i] += ADC1.analogRead(i);
    }
    sample_number += 1;
  }

  if (READING) {
    // During sampling time averge the analog values and apply the necessary
    // transfomations to calculate temp, etc.

    // Channel 0 has a ACS712 hall effect current sensor
    inputs[0].value = get_current();

    // Channels 1 and 2 have a 220ohm connected to ground
    // to record ph and dissolved oxygen
    inputs[1].value = get_dissolved_oxygen();
    inputs[2].value = get_ph();

    // Channels 3, 4, 5, 6, 7 have 10Kohm resistors connected to 3.3V
    // to record temperature from a 10Kohm NTC termistor
    get_temperatures();

    for(int i=0; i<N_OUTPUTS; i++){
      
    }

    // print analog outputs to serial
    send_data();

    // reset accumulator variable for analog sampling
    for (byte i = 0; i < ADC1.channels(); i++) {
      analog[i] = 0;
    }
    sample_number = 0;

    // Restart Timer
    esp_timer_start_once(timer_handle, sample_time);
    READING = false;
  }
  parseSerial();
  parseString(inputString);
  inputString = "";
}

float get_current(void){
  float voltage_acs712;

  // Channel 0 has a ACS712 hall effect current sensor
  voltage_acs712 = (analog[0] / sample_number) * (ref_voltage / 4095);
  total_current = (voltage_acs712 - 2.5012) / -0.067;

  return total_current;
}

float get_dissolved_oxygen(void){
  float voltage_oxygen;

  // Channel 1 has a resistor connected to ground
  // to record ph and dissolved oxygen
  voltage_oxygen = analog[1] / sample_number * (ref_voltage / 4095);
  dissolved_oxygen = 0.4558 * voltage_oxygen;

  return dissolved_oxygen;
}

float get_ph(void){
  float voltage_ph;

  // Channel 2 has a resistor connected to ground
  // to record ph and dissolved oxygen
  voltage_ph = analog[2] / sample_number * (ref_voltage / 4095);
  ph = 3.9811 * voltage_ph - 3.5106;

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

void send_data(void){
  String outputs_json = write_outputs_json();
  String inputs_json = write_inputs_json();

  String all_data_json = "{'address': '" + ADDRESS + "', ";
  all_data_json = all_data_json + outputs_json + ", " + inputs_json;
  all_data_json = all_data_json + "}115,!";
  Serial.println(all_data_json);
}

String write_outputs_json(){
  String outputs_json = "'outputs': [";

  for(byte i=0; i < 6; i++){
    outputs_json = outputs_json + "(";
    outputs_json = outputs_json + "'" + outputs[i].type + "',";
    outputs_json = outputs_json + outputs[i].channel + ",";
    outputs_json = outputs_json + outputs[i].value;
    outputs_json = outputs_json + "),";
  }
  outputs_json = outputs_json + "]";

  return outputs_json;
}

String write_inputs_json(){
  String inputs_json = "'inputs': [";

  for(byte i=0; i < 8; i++){
    inputs_json = inputs_json + "(";
    inputs_json = inputs_json + "'" + inputs[i].variable + "',";
    inputs_json = inputs_json + inputs[i].value;
    inputs_json = inputs_json + "),";
  }
  inputs_json = inputs_json + "]";

  return inputs_json;
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
  if(new_command){
    String ADDR = inputString.substring(0, inputString.indexOf(' '));

    if(ADDR == ADDRESS){
      int firstcomma = inputString.indexOf(',');
      String cmd = inputString.substring(inputString.indexOf(' '), firstcomma);
      int command = cmd.toInt();

      if(command == 1){
        int lastcomma = firstcomma;
        for (byte i = 0; i < 8; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          manula_output[i] = value;
          lastcomma = nextcomma;
        }

      }
      else if (command == 2){
        int lastcomma = firstcomma;
        int nextcomma = inputString.indexOf(',', lastcomma + 1);
        String val = inputString.substring(lastcomma + 1, nextcomma);
        float value = val.toFloat();
        temp_setpoint = value;
      }
      else if (command == 3){
        int lastcomma = firstcomma;
        for (byte i = 0; i < 2; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          oxygen_bounds[i] = value;
          lastcomma = nextcomma;
        }
      }
    }

    inputString = "";
    new_command = false;
  }
}
