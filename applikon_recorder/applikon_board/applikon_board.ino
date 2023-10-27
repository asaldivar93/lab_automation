#include <Arduino.h>
#include "MCP_ADC.h"
#include <Wire.h>
#include "Adafruit_MPRLS.h"

#define sines  112
int s = 0;
static byte WaveFormTable[2][sines] = {
   // Sin wave
   { 
    0x80, 0x83, 0x87, 0x8A, 0x8E, 0x91, 0x95, 0x98, 0x9B, 0x9E, 0xA2, 0xA5, 0xA7, 0xAA, 0xAD, 0xAF,
    0xB2, 0xB4, 0xB6, 0xB8, 0xB9, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xBF, 0xBF, 0xC0, 0xBF, 0xBF, 0xBF,
    0xBE, 0xBD, 0xBC, 0xBB, 0xB9, 0xB8, 0xB6, 0xB4, 0xB2, 0xAF, 0xAD, 0xAA, 0xA7, 0xA5, 0xA2, 0x9E,
    0x9B, 0x98, 0x95, 0x91, 0x8E, 0x8A, 0x87, 0x83, 0x80, 0x7C, 0x78, 0x75, 0x71, 0x6E, 0x6A, 0x67,
    0x64, 0x61, 0x5D, 0x5A, 0x58, 0x55, 0x52, 0x50, 0x4D, 0x4B, 0x49, 0x47, 0x46, 0x44, 0x43, 0x42,
    0x41, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41, 0x42, 0x43, 0x44, 0x46, 0x47, 0x49, 0x4B,
    0x4D, 0x50, 0x52, 0x55, 0x58, 0x5A, 0x5D, 0x61, 0x64, 0x67, 0x6A, 0x6E, 0x71, 0x75, 0x78, 0x7C
   },
};


// Serial buffer
boolean         newCommand = false;
String          ADDRESS = "appl";
String          inputString = "";

// Analog read buffer
float           biom[17410];
int             n = 0;
unsigned long   analog[] = {0, 0, 0, 0, 0, 0, 0, 0};
float           reads[8];

// Analog read vars
float           refVoltage = 3.3;

// Analog read parameters
int             sample_number = 0;
uint32_t        samples_per_second = 10000;
uint32_t        sample_time = 1000000;
boolean         READING = false;

// Pressure read vars
uint16_t        PSI_min = 0; 
uint16_t        PSI_max = 25;
float           P_factor = 0.068046; //corversion factor psi to atm
float           p = 0;
float           pressure;

// pH and Dissolved Oxygen variables
float           Vph;
float           VDO;
uint16_t        PH_MCP_PIN = 1;
uint16_t        DO_MCP_PIN = 2;

// Biomass varibles
float           VBM;
uint16_t        BM_MCP_PIN = 5;

// Free MCP pins
uint16_t        AMP_MCP_PIN3 = 3;
uint16_t        MCP_PIN4 = 4;
uint16_t        MCP_PIN6 = 6;
uint16_t        MCP_PIN7 = 7;

float           setpoint[] = {127, 0, 0, 0, 0, 0, 0};

// PWM Channels
#define PWM_CHANNEL_0 0
#define PWM_CHANNEL_1 1
#define PWM_CHANNEL_2 2
#define PWM_CHANNEL_3 3
#define PWM_CHANNEL_4 4
#define PWM_CHANNEL_5 5
#define PWM_CHANNEL_6 6

#define PWM_BIT 8
#define PWM_BASE_FREQ 4000

// PWM pins
#define PWM_0 13
#define PWM_1 12
#define PWM_2 14
#define PWM_3 27
#define PWM_4 26
#define PWM_5 25
#define PWM_6 33

// Biomass Pin
#define CHN_BIOMASS_LEDS  7
#define BIOMASS_LEDS      32
#define BIOMASS_LEDS_BIT  8
#define BIOMASS_LEDS_FREQ 100

//Pins for MCP_3208
#define MCP_DOUT 18
#define MCP_DIN  19
#define MCP_CLK  5
#define CS1      23

//Pins for MPRLS sensor
#define RESET_PIN -1
#define EOC_PIN   -1
// MPRLS SDA_PIN  21
// MPRLS SCL_PIN  22

Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN, PSI_min, PSI_max, 10, 90, P_factor); 

MCP3208 ADC1(MCP_DOUT, MCP_DIN, MCP_CLK);

esp_timer_create_args_t create_args;                                  
esp_timer_handle_t timer_handle;

void read_temp(void *p) {
  READING = true;                                                            // Change flag to enable print
}

void setup() {
  Serial.begin(57600);
  mpr.begin();
//  if (! mpr.begin()) {
//    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
//    while (1) {
//      delay(10);
//    }
//  }
  //mpr.begin();
  ADC1.begin(CS1); 
  ADC1.setSPIspeed(1000000);
 
  ledcSetup(   PWM_CHANNEL_0,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_1,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_2,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_3,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_4,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_5,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(   PWM_CHANNEL_6,     PWM_BASE_FREQ,          PWM_BIT);
  ledcSetup(CHN_BIOMASS_LEDS, BIOMASS_LEDS_FREQ, BIOMASS_LEDS_BIT);
  
  ledcAttachPin(       PWM_0,    PWM_CHANNEL_0);
  ledcAttachPin(       PWM_1,    PWM_CHANNEL_1);
  ledcAttachPin(       PWM_2,    PWM_CHANNEL_2);
  ledcAttachPin(       PWM_3,    PWM_CHANNEL_3);
  ledcAttachPin(       PWM_4,    PWM_CHANNEL_4);
  ledcAttachPin(       PWM_5,    PWM_CHANNEL_5);
  ledcAttachPin(       PWM_6,    PWM_CHANNEL_6);
  ledcAttachPin(BIOMASS_LEDS, CHN_BIOMASS_LEDS);
  
  ledcWrite(   PWM_CHANNEL_0, setpoint[0]);
  ledcWrite(   PWM_CHANNEL_1, setpoint[1]);
  ledcWrite(   PWM_CHANNEL_2, setpoint[2]);
  ledcWrite(   PWM_CHANNEL_3, setpoint[3]);
  ledcWrite(   PWM_CHANNEL_4, setpoint[4]);
  ledcWrite(   PWM_CHANNEL_5, setpoint[5]);
  ledcWrite(   PWM_CHANNEL_6, setpoint[6]);
  ledcWrite(CHN_BIOMASS_LEDS, 127);
  delay(500);

  create_args.callback = read_temp; // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);


  esp_timer_start_once(timer_handle, sample_time);
}

void loop() {

  if (!READING){
    for (byte i = 0; i < ADC1.channels(); i++) {
      analog[i] += ADC1.analogRead(i);
    }
    sample_number += 1; 
  }

  if (READING) {
    for (byte i = 0; i < ADC1.channels(); i++) {
      float a = analog[i] / sample_number;
      reads[i] = a;
      analog[i] = 0;
    }
    pressure = mpr.readPressure();;
    Vph = reads[PH_MCP_PIN] * refVoltage / 4096;
    VDO = reads[DO_MCP_PIN] * refVoltage / 4096;
    VBM = reads[BM_MCP_PIN] * refVoltage / 4096;
    
    sample_number = 0;
    printf("%.4f %.4f %.4f %.4f\n", Vph, VDO, VBM, pressure);
    //printf("%.4f\n", VBM);
    esp_timer_start_once(timer_handle, sample_time);
    READING = false; 
  }
//  if (!READING){
//    biom[sample_number] = ADC1.analogRead(BM_MCP_PIN) * refVoltage / 4096;
//    VDO = ADC1.analogRead(DO_MCP_PIN) * refVoltage / 4096;
//    Vph = ADC1.analogRead(PH_MCP_PIN) * refVoltage / 4096;
//    //p = mpr.readPressure();
//    sample_number += 1; 
//  }
//
//  if (READING) {
//    if (n == 0){
//      for(int i = 0; i < sample_number; i++){
//        Serial.println(biom[i]);
//      }
//    }
//    n = n + 1;
//    sample_number = 0;
//    esp_timer_start_once(timer_handle, sample_time);
//    READING = false; 
//  }
  byte wave_type = 0; // Sine
  dacWrite(25, 150); 
  s++;
  if (s >= sines) s = 0; 
  parseSerial();
  parseString(inputString);
  inputString = "";
}

void parseSerial(void){
  while(Serial.available()){
    char inChar = char(Serial.read());
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
        for (byte i = 0; i < 7; i++){
          int nextcomma = inputString.indexOf(',', lastcomma + 1);
          String val = inputString.substring(lastcomma + 1, nextcomma);
          float value = val.toFloat();
          setpoint[i] = value;
          lastcomma = nextcomma;
        }
        ledcWrite(PWM_CHANNEL_0, setpoint[0]);
        ledcWrite(PWM_CHANNEL_1, setpoint[1]);
        ledcWrite(PWM_CHANNEL_2, setpoint[2]);
        ledcWrite(PWM_CHANNEL_3, setpoint[3]);
        ledcWrite(PWM_CHANNEL_4, setpoint[4]);
        ledcWrite(PWM_CHANNEL_5, setpoint[5]);
        ledcWrite(PWM_CHANNEL_6, setpoint[6]);
        Serial.println("115!");
        //Serial2.println("115!");
      }
      else if (command == 2){
//        String val = inputString.substring(inputString.indexOf(',')+1);
//        int value = val.toInt();
//        Serial.println(value);
//        ledcWrite(CHN_SOLENOID_GAS_SENSOR, value);
      }
      else if (command == 3){
//        String val = inputString.substring(inputString.indexOf(',')+1);
//        int value = val.toInt();
//        Serial.println(value);
//        ledcWrite(CHN_PUMP_GAS_SENSOR, value);
      }
    }
    inputString = "";
    newCommand = false;
  }
}
