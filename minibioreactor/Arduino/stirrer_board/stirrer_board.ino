
#include <Arduino.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "QuickPID.h"

int16_t   pulses[8];
float     rpm[8];
uint32_t  sample_per_second = 1;
uint32_t  sample_time = 1000000 / sample_per_second;
float     sample_time_seconds = 1000000 / (float) sample_per_second / 1000000;
boolean   READING = true;

boolean   newCommand = false;
String    ADDRESS = "str";
String    inputString = "";
boolean   comm_confirmed;

float setpoint[] = {0, 0, 0, 0, 0, 0, 0, 0};
float output[8];
float Kp[8] = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
float Ki[8] = {0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06};
float Kd[8] = {0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009};

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2
#define LEDC_CHANNEL_3 3
#define LEDC_CHANNEL_4 4
#define LEDC_CHANNEL_5 5
#define LEDC_CHANNEL_6 6
#define LEDC_CHANNEL_7 7

// use 8 bit precission for LEDC timer
#define LEDC_BIT 8

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 4000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define MOTOR_PIN_0 13
#define MOTOR_PIN_1 12
#define MOTOR_PIN_2 14
#define MOTOR_PIN_3 27
#define MOTOR_PIN_4 26
#define MOTOR_PIN_5 25
#define MOTOR_PIN_6 33
#define MOTOR_PIN_7 32

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define TACHOMETER_PIN_0 35
#define TACHOMETER_PIN_1 34
#define TACHOMETER_PIN_2 39
#define TACHOMETER_PIN_3 36
#define TACHOMETER_PIN_4 15
#define TACHOMETER_PIN_5 4
#define TACHOMETER_PIN_6 18
#define TACHOMETER_PIN_7 19

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

esp_timer_create_args_t create_args;
esp_timer_handle_t timer_handle;

QuickPID PID_0(&rpm[0], &output[0], &setpoint[0]);
QuickPID PID_1(&rpm[1], &output[1], &setpoint[1]);
QuickPID PID_2(&rpm[2], &output[2], &setpoint[2]);
QuickPID PID_3(&rpm[3], &output[3], &setpoint[3]);
QuickPID PID_4(&rpm[4], &output[4], &setpoint[4]);
QuickPID PID_5(&rpm[5], &output[5], &setpoint[5]);
QuickPID PID_6(&rpm[6], &output[6], &setpoint[6]);
QuickPID PID_7(&rpm[7], &output[7], &setpoint[7]);
QuickPID *allPIDS[8] = {&PID_0, &PID_1, &PID_2, &PID_3, &PID_4, &PID_5, &PID_6, &PID_7};

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
    unsigned long timeStamp; // The time the event occured
} pcnt_evt_t;

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * and timestamp to the main program using a queue.
 */
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    unsigned long currentMillis = millis(); //Time at instant ISR was called
    uint32_t intr_status = PCNT.int_st.val;
    int i = 0;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;


    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            evt.timeStamp = currentMillis;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}


/* Initialize PCNT functions for one channel:
 */

void pcnt_init_channel(pcnt_unit_t PCNT_UNIT, int PCNT_INPUT_SIG_IO, pcnt_channel_t PCNT_CHANNEL = PCNT_CHANNEL_0, int PCNT_H_LIM_VAL = 20000) {
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config;
        // Set PCNT input signal and control GPIOs
        pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;
        pcnt_config.ctrl_gpio_num = 22;
        pcnt_config.channel = PCNT_CHANNEL;
        pcnt_config.unit = PCNT_UNIT;
        // What to do on the positive / negative edge of pulse input?
        pcnt_config.pos_mode = PCNT_COUNT_INC;   // Count up on the positive edge
        pcnt_config.neg_mode = PCNT_COUNT_DIS;   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        pcnt_config.lctrl_mode = PCNT_MODE_DISABLE; // Reverse counting direction if low
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;
        pcnt_config.counter_l_lim = -1;

    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_disable(PCNT_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_disable(PCNT_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT);
}

void read_PCNT(void *p) {
  READING = true;                                                            // Change flag to enable print
}

void setup() {
  Serial2.begin(230400, SERIAL_8N1, 16, 17);
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_3, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_4, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_5, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_6, LEDC_BASE_FREQ, LEDC_BIT);
  ledcSetup(LEDC_CHANNEL_7, LEDC_BASE_FREQ, LEDC_BIT);

  ledcAttachPin(MOTOR_PIN_0, LEDC_CHANNEL_0);
  ledcAttachPin(MOTOR_PIN_1, LEDC_CHANNEL_1);
  ledcAttachPin(MOTOR_PIN_2, LEDC_CHANNEL_2);
  ledcAttachPin(MOTOR_PIN_3, LEDC_CHANNEL_3);
  ledcAttachPin(MOTOR_PIN_4, LEDC_CHANNEL_4);
  ledcAttachPin(MOTOR_PIN_5, LEDC_CHANNEL_5);
  ledcAttachPin(MOTOR_PIN_6, LEDC_CHANNEL_6);
  ledcAttachPin(MOTOR_PIN_7, LEDC_CHANNEL_7);

  ledcWrite(LEDC_CHANNEL_0, 0);
  ledcWrite(LEDC_CHANNEL_1, 0);
  ledcWrite(LEDC_CHANNEL_2, 0);
  ledcWrite(LEDC_CHANNEL_3, 0);
  ledcWrite(LEDC_CHANNEL_4, 0);
  ledcWrite(LEDC_CHANNEL_5, 0);
  ledcWrite(LEDC_CHANNEL_6, 0);
  ledcWrite(LEDC_CHANNEL_7, 0);
  delay(500);

    /* Initialize PCNT event queue and PCNT functions */
  pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
  pcnt_init_channel(PCNT_UNIT_0, TACHOMETER_PIN_0);
  pcnt_init_channel(PCNT_UNIT_1, TACHOMETER_PIN_1);
  pcnt_init_channel(PCNT_UNIT_2, TACHOMETER_PIN_2);
  pcnt_init_channel(PCNT_UNIT_3, TACHOMETER_PIN_3);
  pcnt_init_channel(PCNT_UNIT_4, TACHOMETER_PIN_4);
  pcnt_init_channel(PCNT_UNIT_5, TACHOMETER_PIN_5);
  pcnt_init_channel(PCNT_UNIT_6, TACHOMETER_PIN_6);
  pcnt_init_channel(PCNT_UNIT_7, TACHOMETER_PIN_7);

  create_args.callback = read_PCNT; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);

  PID_0.SetMode(PID_0.Control::automatic);
  PID_1.SetMode(PID_1.Control::automatic);
  PID_2.SetMode(PID_2.Control::automatic);
  PID_3.SetMode(PID_3.Control::automatic);
  PID_4.SetMode(PID_4.Control::automatic);
  PID_5.SetMode(PID_5.Control::automatic);
  PID_6.SetMode(PID_6.Control::automatic);
  PID_7.SetMode(PID_7.Control::automatic);
  for (byte i = 0; i<8; i++){
    allPIDS[i]->SetOutputLimits(90, 130);
    allPIDS[i]->SetTunings(Kp[i], Ki[i], Kd[i]);
    allPIDS[i]->SetSampleTimeUs(sample_time);
  }
}

void loop()
{
  pcnt_evt_t evt;
  portBASE_TYPE res;

  if (READING == true){
    pcnt_get_counter_value(PCNT_UNIT_0, &pulses[0]);
    pcnt_get_counter_value(PCNT_UNIT_1, &pulses[1]);
    pcnt_get_counter_value(PCNT_UNIT_2, &pulses[2]);
    pcnt_get_counter_value(PCNT_UNIT_3, &pulses[3]);
    pcnt_get_counter_value(PCNT_UNIT_4, &pulses[4]);
    pcnt_get_counter_value(PCNT_UNIT_5, &pulses[5]);
    pcnt_get_counter_value(PCNT_UNIT_6, &pulses[6]);
    pcnt_get_counter_value(PCNT_UNIT_7, &pulses[7]);

    for (byte i = 0; i < 8; i = i + 1) {
      float p = (float) pulses[i];
      if (p > 0){
        rpm[i] = (p / 2) * (1 / sample_time_seconds) * 60;
      }
      else{
        rpm[i] = 0;
      }
      allPIDS[i]-> Compute();
    }

    ledcWrite(LEDC_CHANNEL_0, output[0]);
    ledcWrite(LEDC_CHANNEL_1, output[1]);
    ledcWrite(LEDC_CHANNEL_2, output[2]);
    ledcWrite(LEDC_CHANNEL_3, output[3]);
    ledcWrite(LEDC_CHANNEL_4, output[4]);
    ledcWrite(LEDC_CHANNEL_5, output[5]);
    ledcWrite(LEDC_CHANNEL_6, output[6]);
    ledcWrite(LEDC_CHANNEL_7, output[7]);

    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_2);
    pcnt_counter_clear(PCNT_UNIT_3);
    pcnt_counter_clear(PCNT_UNIT_4);
    pcnt_counter_clear(PCNT_UNIT_5);
    pcnt_counter_clear(PCNT_UNIT_6);
    pcnt_counter_clear(PCNT_UNIT_7);

    esp_timer_start_once(timer_handle, sample_time);
    READING = false;
  }

  parseSerial();
  parseString(inputString);
  inputString = "";

  if(user_isr_handle) {
    //Free the ISR service handle.
    esp_intr_free(user_isr_handle);
    user_isr_handle = NULL;
  }
}

void parseSerial(void){
  while(Serial2.available()){
    char inChar = char(Serial2.read());
    inputString += inChar;
    if(inChar == '!'){
      newCommand = true;
      break;
    }
  }
}

void parseString(String inputString){
  if(newCommand){
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
          setpoint[i] = value;
          lastcomma = nextcomma;
        }
        Serial2.println("115!");
      }
      else if (command == 2){
        Serial2.println("115!");
      }
      else if (command == 3){
        Serial2.println("115!");
        String sample = ADDRESS + " ";
        for(byte i = 0; i < 8; i++){
          sample = sample + "," + (String) rpm[i];
        }
        sample = sample + ",115,!";
        Serial2.println(sample);
      }
    }

    inputString = "";
    newCommand = false;
  }
}
