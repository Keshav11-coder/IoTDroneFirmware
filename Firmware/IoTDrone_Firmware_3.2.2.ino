#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include "conf.h"
#include "Aircraft.h"
#include <ArduinoJson.h>
Aircraft Drone(topl_pin, topr_pin, bottoml_pin, bottomr_pin, MIN_SIGNAL, MAX_SIGNAL);

int localMrc[4] = {1000, 1000, 1000, 1000};
int localThrottle = 1000;
int minM = 1000;
bool running = false;

void GetDistance( void * pvParameters ) {
  for (;;) {
    int index = Drone.get_fc_run("fc_alt_min");
    if (Drone.sensors[index].status) {
      if (index != -1) {
        Drone.sensors[index].fc_sensor->run();
        Serial.println(Drone.fc_alt_min.gdist);
      } // now this function is safely handled, because we have defaults in our virtual + class and this would then return whatever the last value was.
      vTaskDelay(5000);
    } else {
      vTaskDelay(2000);
      Serial.println("fc_warn : SENSE ALT_MIN OFF");
    }
  }
}

void GetAngles( void * pvParameters) {
  for (;;) {
    //    Serial.println(Drone.mpu.pitch);
    //    Drone.fc_pid.run(Drone.mpu.q);
    //    Drone.fc_mpu.run();

    // get index
    int index = Drone.get_fc_run("fc_mpu");
    if (Drone.sensors[index].status) {
      if (index != -1) {
        Drone.sensors[index].fc_sensor->run();
        Serial.println(Drone.fc_mpu.roll);
      } // now this function is safely handled, because we have defaults in our virtual + class and this would then return whatever the last value was.
    } else {
      vTaskDelay(2000);
      Serial.println("fc_warn : SENSE IMU OFF");
    }
    // Check stack high water mark
    //    UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    //    Serial.print("GetAngles Task Stack High Water Mark: ");
    //    Serial.println(stackHighWaterMark);
  }
}

void UpdateLights( void * pvParameters ) {
  for (;;) {
    Drone.updateLights();
  }
}

void setup() {
  Serial.begin(115200);
  Drone.configureLights(green1pin, green2pin, red1pin, red2pin);
  Drone.fc_alt_min.configure(trigPin, echoPin, SOUND_SPEED);
  Drone.fc_mpu.configure();
  Drone.configureMode(Aircraft::Debug);

  delay(5000);

  Drone.startArm(MIN_SIGNAL, MAX_SIGNAL, 5000); // uncomment block for class calibration, requires extra 10000 seconds cause of direct input sending
  Drone.showCLI();

  xTaskCreatePinnedToCore(GetDistance, "UltraSonic", 10000, NULL, 1, &UltraSonic, 0);
  xTaskCreatePinnedToCore(GetAngles, "Mpu", 10000, NULL, 2, &Mpu, 0);
}

void loop() {
  Drone.debugCLIHandler();
  Drone.updateLights();
  if (Drone.armed) {
    localThrottle = map(analogRead(35), 0, 4095, -2000, 2000);

    if (localThrottle >= 1050 && running == false) {
      minM = 1050;
      running = true;
    } else if (localThrottle == -2000) {
      Drone.FailSafeStop();
    }

    localMrc[0] = constrain(localThrottle, minM, 2000);
    localMrc[1] = constrain(localThrottle, minM, 2000);
    localMrc[2] = constrain(localThrottle, minM, 2000);
    localMrc[3] = constrain(localThrottle, minM, 2000);
    Drone.mrc(localMrc[0], localMrc[1], localMrc[2], localMrc[3]);
  }
}
