#include <Arduino.h>

#include "DacESP32.h"

#include "bioreactify.h"
#include "comms_handle.h"

#define N_PWM 5

#define N_ADC 6
#define N_I2C 1
#define N_SLAVES 0

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

// PWM Configuration
Output CH_0(0, PIN_CH0, "pwm", MANUAL, 0);
Output CH_1(1, PIN_CH1, "pwm", MANUAL, 0);
Output CH_2(2, PIN_CH2, "pwm", MANUAL, 0);
Output CH_3(3, PIN_CH3, "pwm", MANUAL, 0);
Output CH_4(4, PIN_CH4, "pwm", MANUAL, 0);
Output outputs[N_PWM] = {CH_0, CH_1, CH_2, CH_3, CH_4};

// DAC Configuration
DacESP32 DAC_1(DAC_CHANNEL_1);
DacESP32 DAC_2(DAC_CHANNEL_2);

// ADC Configuration
MCP3208 adc_0(SPI_MISO, SPI_MOSI, SPI_CLK);
Input ACH_0(1, "adc", "temperature_0");
Input ACH_1(2, "adc", "temperature_1");
Input ACH_2(3, "adc", "dissolved_oxygen");
Input ACH_3(4, "adc", "ph");
Input ACH_4(6, "adc", "biomass_1");
Input ACH_5(7, "adc", "biomass_2");
Input analogs[N_ADC] = {ACH_0, ACH_1, ACH_2, ACH_3, ACH_4, ACH_5};

// I2C Configuration
uint8_t multiplexer_address = 0x70;
Sensors sensors(multiplexer_address);

Input DCH_0(0, "i2c", "pressure_psi");
Input digitals[N_I2C] = {DCH_0};

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
    parse_string(input_string);
    vTaskDelay(10);
  }
}

// ESPnow
// Slaves Configuration
Slave slaves[N_SLAVES];


// Inint functions
void init_comms(void){
  Wire.begin();
  Serial.begin(230400);
  xTaskCreatePinnedToCore(serial_comms_code, "serial_comms", 10000, NULL, 0, &serial_comms, 0);
}

void init_outputs(void){
  // Initialize PWM channels
  for (int i = 0; i < N_PWM; i++) {
    ledcAttach(outputs[i].pin, LEDC_BASE_FREQ, LEDC_BIT);
    ledcWrite(outputs[i].channel, outputs[i].value);

    outputs[i].set_output_limits(0, 255);
    outputs[i].set_pid_tunings(Kp, Ki, Kd);
    outputs[i].set_sample_time_us(cycle_time_timer_1);
    outputs[i].set_gh_filter(0.01);
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

  float thermistor[3] = {8.7561e-4, 2.5343e-4, 1.84499e-7};
  float ref_resistance = 7680;
  float op_amp_resistors[2] = {1500, 4300};

  for(int i=0; i<2; i++){
    analogs[i].set_voltage_divider_resistor(ref_resistance);
    analogs[i].set_opamp_resistors(op_amp_resistors[0], op_amp_resistors[1]);
    analogs[i].set_steinhart_coeffs(thermistor[0], thermistor[1], thermistor[2]);
  }

  analogs[2].set_dissolved_oxygen_cal(85.1312926551, -49.7487023023);
  analogs[3].set_ph_cal(6.006, -3.5108);
  // ------------ Modify Configuration -------------- //
}

void loop() {

  // Sample analog channels as fast as possible
  for(int i=0; i<N_ADC; i++){
    uint8_t channel = analogs[i].channel;
    analogs[i].moving_average(adc_0.read_adc(channel));
  }

  if (TIMER_1) {
    // Transform data
    for(int i=0; i<N_ADC; i++){
      analogs[i].get_moving_average();
    }
    // ------------ Modify Configuration -------------- //
    analogs[0].get_temp_opamp();
    analogs[1].get_temp_opamp();
    analogs[2].get_dissolved_oxygen();
    analogs[3].get_ph();
    analogs[4].value = analogs[4].get_analog_value();
    analogs[5].value = analogs[5].get_analog_value();

    outputs[0].write_output();
    outputs[1].write_output();
    outputs[2].write_output();
    outputs[3].write_output();
    outputs[4].write_output();

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
    sensors.set_multiplexer_channel(0);
    digitals[0].value = sensors.read_mprls();
    digitals[1].value = 0;

    update_inputs_buffer(digitals, N_I2C, &i2c_buffer);
    esp_timer_start_once(timer_2_handle, cycle_time_timer_2);
    TIMER_2 = false;
    Serial.println(adc_buffer);
    Serial.println(i2c_buffer);
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
        String outputs_string = "'outs':[" + pwm_buffer + "],";
        String inputs_string = "'ins':[" + adc_buffer + i2c_buffer + "]";
        String to_send_string = "{'" + ADDRESS + "':{" + outputs_string + inputs_string + "}}115,!";
        Serial.println(to_send_string);
        pwm_buffer = "";
        adc_buffer = "";
        i2c_buffer = "";
      }

      if (command == WRITE_DAC) {
        // "ADDR 3,CHANNEL,VALUE,!"
        lastcomma = firstcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int channel = input_string.substring(lastcomma + 1, nextcomma).toInt();
        lastcomma = nextcomma;
        nextcomma = input_string.indexOf(',', lastcomma + 1);
        int value = input_string.substring(lastcomma + 1, nextcomma).toInt();

        dacs[channel].write_dac(value);
        Serial.println(ok_string);
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
