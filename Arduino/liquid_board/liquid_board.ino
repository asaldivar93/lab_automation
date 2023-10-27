#include <Arduino.h>
#include <AccelStepper.h>

#define PUMP_STEP_PIN_0 13
#define PUMP_STEP_PIN_1 12

#define DIR_PIN 4

#define MAX_SPEED 50
#define MAX_ACCEL 50

AccelStepper PUMP_0(AccelStepper::DRIVER, PUMP_STEP_PIN_0, DIR_PIN);
AccelStepper PUMP_1(AccelStepper::DRIVER, PUMP_STEP_PIN_1, DIR_PIN);


void setup() {
  PUMP_0.setMaxSpeed(MAX_SPEED*5);
  PUMP_1.setMaxSpeed(MAX_SPEED*5);

  PUMP_0.setAcceleration(MAX_ACCEL*5);
  PUMP_1.setAcceleration(MAX_ACCEL*2);

}


void loop() {
  PUMP_0.moveTo(1000);
  PUMP_1.moveTo(1000);

  PUMP_0.run();
  PUMP_1.run();
}
