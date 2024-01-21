#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "conf.h"
#include "Aircraft.h"
Aircraft Drone;

void setup() {
  Serial.begin(115200);
  Drone.configureMotors(topl_pin, topr_pin, bottoml_pin, bottomr_pin);
  delay(5000);
  Drone.startArm(MIN_SIGNAL, MAX_SIGNAL, 4000); // uncomment block for class calibration, requires extra 10000 seconds cause of direct input sending
  //  delay(10000);
  Drone.configureMode(Aircraft::Debug);

  Drone.configureLights(green1pin, green2pin, red1pin, red2pin);
}

void loop() {
  Drone.updateLights(0);
  Drone.mrc(1050, 1050, 1050, 1050); // class function only available in debug mode
}
