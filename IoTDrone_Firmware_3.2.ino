#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "conf.h"
#include "Aircraft.h"
Aircraft Drone(21);

void GetDistance( void * pvParameters ) {
  Serial.print("UltraSonic running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (Drone.armed) {
      Serial.println(Drone.getGroundDistance());
      vTaskDelay(500);
    }
  }
}

void GetAngles( void * pvParameters) {
  Serial.print("UltraSonic running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (Drone.armed) {
      Serial.println(Drone.getAngleDegrees());
      vTaskDelay(500);
    }
  }
}

void debugCLI( void * pvParameters ) {
  Serial.print("debugCLI running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (Drone.armed) {
      Drone.debugCLIHandler();
      vTaskDelay(500);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Drone.configureMotors(topl_pin, topr_pin, bottoml_pin, bottomr_pin);
  Drone.configureLights(green1pin, green2pin, red1pin, red2pin);
  Drone::sensors sensors;
  Drone.configureDistanceSensor(trigPin, echoPin, SOUND_SPEED);
  Drone.configureMPU();
  Drone.configureMode(Aircraft::Debug);

  //  digitalWrite(21, 1);
  delay(5000);

  Drone.startArm(MIN_SIGNAL, MAX_SIGNAL, 5000); // uncomment block for class calibration, requires extra 10000 seconds cause of direct input sending
  Drone.showCLI();

  xTaskCreatePinnedToCore(
    debugCLI,//task function
    "CLI",//task name
    10000,//stack size
    NULL,
    1,//priority
    &CLI,//task handle
    0// core
  );
  xTaskCreatePinnedToCore(GetDistance, "UltraSonic", 10000, NULL, 1, &UltraSonic, 0);
  xTaskCreatePinnedToCore(GetAngles, "Mpu", 10000, NULL, 1, &Mpu, 0);
}

void loop() {
  Drone.updateLights();
}
