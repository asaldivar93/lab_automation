#include <Arduino.h>

#include "DacESP32.h"

#include "bioreactify.h"
#include "comms_handle.h"

#define N_PWM 8

#define N_ADC 8
#define N_I2C 0
#define N_SLAVES 1

// Serial Configuration
String ADDRESS = "M0";
boolean new_command = false;
String input_string = "";

String pwm_buffer = "";
String adc_buffer = "";
String i2c_buffer = "";

// Timers Configuration
uint32_t one_second = 1000000;

boolean TIMER_1 = false;
uint32_t samples_per_second_timer_1 = 4;
uint32_t cycle_time_timer_1 = one_second / samples_per_second_timer_1;

boolean TIMER_2 = false;
uint32_t samples_per_second_timer_2 = 1;
uint32_t cycle_time_timer_2 = one_second / samples_per_second_timer_2;

// PID Configuration
float Kp = 100;
float Ki = 0.2;
float Kd = 0.;

Slave slaves[N_SLAVES] =
{ {"S1", {0x58, 0xBF, 0x25, 0x36, 0x78, 0x94}, 
   "('adc',0,'temp_0'),('adc',1,'x_0'),('adc',2,'temp_1'),('adc',3,'x_1'),('adc',4,'temp_2'),('adc',5,'x_2'),('adc',6,'temp_3'),('adc',7,'x_3'),",
   "('pwm',0),('pwm',1),('pwm',2),('pwm',3),('pwm',4),('pwm',5),('pwm',6),('pwm',7)"} };

// PWM Configuration
Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output CH_5(5, PIN_CH5, "pwm", MANUAL, 0);
Output CH_6(6, PIN_CH6, "pwm", MANUAL, 0);
Output CH_7(7, PIN_CH7, "pwm", MANUAL, 0);
Output outputs[N_PWM] = {CH_0, CH_1, CH_2, CH_3, CH_4, CH_5, CH_6, CH_7};

// DAC Configuration
DacESP32 DAC_1(DAC_CHANNEL_1);
DacESP32 DAC_2(DAC_CHANNEL_2);

// ADC Configuration
MCP3208 adc_0(SPI_MISO, SPI_MOSI, SPI_CLK);
Input ACH_0(0, "adc", "temp_0");
Input ACH_1(1, "adc", "x_0");
Input ACH_2(2, "adc", "temp_1");
Input ACH_3(3, "adc", "x_1");
Input ACH_4(4, "adc", "temp_2");
Input ACH_5(5, "adc", "x_2");
Input ACH_6(6, "adc", "temp_3");
Input ACH_7(7, "adc", "x_3");
Input analogs[N_ADC] = {ACH_0, ACH_1, ACH_2, ACH_3, ACH_4, ACH_5, ACH_6, ACH_7};

// I2C Configuration
uint8_t multiplexer_address = 0x70;
Sensors sensors(multiplexer_address);
Input digitals[N_I2C];

// Timer Configuration
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

void init_timers(void){
  // Initialize timers for sensors and volumes;
  timer_1_args.callback = timer_1_callback;
  esp_timer_create(&timer_1_args, &timer_1_handle);
  esp_timer_start_once(timer_1_handle, cycle_time_timer_1);

  timer_2_args.callback = timer_2_callback;
  esp_timer_create(&timer_2_args, &timer_2_handle);
  esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
}

// Parallel task for communication
TaskHandle_t serial_comms;
void serial_comms_code(void *parameters) {
  for (;;) {
    String input_string = parse_serial_master(&new_command);
    parse_serial_slave(&new_command);
    parse_string(input_string);
    vTaskDelay(10);
  }
}

// Inint functions
void init_comms(void){
  Wire.begin();
  Serial.begin(230400);
  Serial2.begin(57600, SERIAL_8N1, Rx, Tx);
  xTaskCreatePinnedToCore(serial_comms_code, "serial_comms", 10000, NULL, 0, &serial_comms, 0);
}

void init_outputs(void){
  // Initialize PWM channels
  for (int i = 0; i < N_PWM; i++) {
    Serial.println(outputs[i].pin);
    ledcAttach(outputs[i].pin, LEDC_BASE_FREQ, LEDC_BIT);
    ledcWrite(outputs[i].pin, outputs[i].value);

    outputs[i].set_output_limits(0, 255);
    outputs[i].set_pid_tunings(Kp, Ki, Kd);
    outputs[i].set_sample_time_us(cycle_time_timer_1);
    outputs[i].set_gh_filter(0.7);
  }
  for(int i=0; i<N_PWM; i+=2){
    outputs[i].set_output_limits(0, 85);
  }

  // Initialize DAC for Biomass Sensor
  DAC_1.outputCW(500, DAC_CW_SCALE_2);
  DAC_1.setCwOffset(-45);

  DAC_2.outputCW(500, DAC_CW_SCALE_2);
  DAC_2.setCwOffset(-45);
}

void setup() {
  // Initialize functions
  init_comms();
  init_outputs();
  init_timers();

  // ------------ Modify Configuration -------------- //
  adc_0.begin(CS0);
  adc_0.set_spi_speed(1000000);
  adc_0.set_ref_voltage(3.3);

  float thermistor[3] = {8.1197e-4, 2.65207e-4, 1.272206e-7}; // 103JT-025 semitec
  float ref_resistance = 5100;
  float op_amp_resistors[2] = {1500, 2200};

  for (int i=0; i<N_ADC; i+=2){
    analogs[i].set_voltage_divider_resistor(ref_resistance);
    analogs[i].set_opamp_resistors(op_amp_resistors[0], op_amp_resistors[1]);
    analogs[i].set_steinhart_coeffs(thermistor[0], thermistor[1], thermistor[2]);
  }
  // ------------ Modify Configuration -------------- //
}

void loop() {

  // Sample analog channels as fast as possible
  for(int i=0; i<N_ADC; i++){
    uint8_t channel = analogs[i].channel;
    float voltage = adc_0.read_adc(channel);
    analogs[i].moving_average(voltage);
  }

  if (TIMER_1) {
    // Transform data
    for(int i=0; i<N_ADC; i++){
      analogs[i].get_moving_average();
    }
    // ------------ Modify Configuration -------------- //

    for(int i=0; i<N_ADC; i+=2){
      analogs[i].get_temp_opamp();
    }

    for(int i=1; i<N_ADC; i+=2){
      analogs[i].value = analogs[i].get_analog_value();;
    }

    for(int i=0; i<N_PWM; i++){
      outputs[i].write_output();
    }

    for(int i=0; i<N_ADC; i++){
      analogs[i].reset_moving_average();
    }
    // ------------ Modify Configuration -------------- //

    // Update buffers
    update_inputs_buffer(analogs, N_ADC, &adc_buffer);
    update_outputs_buffer(outputs, N_PWM, &pwm_buffer);

    // Reset timer
    esp_timer_start_once(timer_1_handle, cycle_time_timer_1);
    TIMER_1 = false;
  }
  
  if (TIMER_2) {
    update_inputs_buffer(digitals, N_I2C, &i2c_buffer);
    esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
    TIMER_2 = false;
  }

}

void send_board_info(void) {
  String slaves_output_info = "";
  String slaves_input_info = "";

  for (int i = 0; i < N_SLAVES; i++) {
    slaves_output_info = slaves_output_info + "'" + slaves[i].name + "':[" + slaves[i].ouputsInfo + "],";
    slaves_input_info = slaves_input_info + "'" + slaves[i].name + "':[" + slaves[i].inputsInfo + "],";
  }

  String outputs_string = "'outs':{'" + ADDRESS + "':[" + get_outputs_info(outputs, N_PWM) + "]," + slaves_output_info + "}";
  String inputs_string = "'ins':{'" + ADDRESS + "':[" + get_inputs_info(analogs, N_ADC) + get_inputs_info(digitals, N_I2C) + "]," + slaves_input_info + "}";

  String all_data_json = "{'address': '" + ADDRESS + "', ";
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
        String slave_outputs = "'outs':[" + write_to_slave("S1 4,!") + "],";
        String slave_inputs = "'ins':[" + write_to_slave("S1 5,!") + "],";
        String outputs_string = "'outs':[" + pwm_buffer + "],";
        String inputs_string = "'ins':[" + adc_buffer + i2c_buffer + "]";
        String to_send_string = "{'" + ADDRESS + "':{" + outputs_string + inputs_string + "},S1:{" + slave_outputs + slave_inputs + "}115,!";
        Serial.println(to_send_string);
        pwm_buffer = "";
        adc_buffer = "";
        i2c_buffer = "";
      }

      if (command == GET_OUTPUTS) {
        // GET_ALL_INPUTS: "ADDR 4,!"
        Serial.println(pwm_buffer);
        pwm_buffer = "";
      }

      if (command == GET_INPUTS) {
        // GET_ALL_INPUTS: "ADDR 5,!"
        Serial.println(adc_buffer);
        pwm_buffer = "";
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

              outputs[out_channel].set_pid(&analogs[in_channel].value, setpoint);
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

              outputs[out_channel].set_onoff(&analogs[in_channel].value, lower_bound, upper_bound, value);
              Serial.println(ok_string);
              break;
            }
        }
      }
    }
    else{
      write_to_slave(input_string);
    }
    new_command = false;
  }
}
