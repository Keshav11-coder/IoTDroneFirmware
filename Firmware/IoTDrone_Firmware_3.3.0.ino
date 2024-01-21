#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <EEPROM.h>
#include "conf.h"
#include "Aircraft.h"
#include <ArduinoJson.h>
#include "PIDSys.h"
#include <esp_now.h>
#include <WiFi.h>
Aircraft Drone(topl_pin, topr_pin, bottoml_pin, bottomr_pin, MIN_SIGNAL, MAX_SIGNAL);
PIDSys pitch(3.55, 0.005, 0.0001);
PIDSys roll(3.55, 0.005, 0.0001);
PIDSys yaw(3.55, 0.005, 0.0001);
int timerDone = false;

typedef struct struct_message {
  float j1x;
  float j1y;
  float j2x;
  float j2y;
} struct_message;
struct_message incomingReadings;
struct_message pidControlRates;

uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xF1, 0x50, 0x50};
//uint8_t broadcastAddress[] = {0x34, 0xB4, 0x72, 0x6A, 0x3E, 0xDA};
esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  pidControlRates.j1x = round(map_range(incomingReadings.j1x, 0, 1920, 4096, -20000, -177, 20000));
  pidControlRates.j1y = round(map_range(incomingReadings.j1y, 1875, 2985, 4096, 1060, 1500, 2000));
  pidControlRates.j2x = round(map_range(incomingReadings.j2x, 0, 1800, 4096, -20000, 558, 20000));
  pidControlRates.j2y = round(map_range(incomingReadings.j2y, 0, 1815, 4096, -20000, 348, 20000));
  if (incomingReadings.j1x == 4095 && incomingReadings.j1y == 0 && incomingReadings.j2x == 0 && incomingReadings.j2y == 0 && !Drone.started) {
    Drone.startMotors(1060);
  }
}

void GetDistance( void * pvParameters ) {
  for (;;) {
    int index = Drone.get_fc_run("fc_alt_min");
    if (Drone.sensors[index].status) {
      if (index != -1) {
        Drone.sensors[index].fc_sensor->run();
        Serial.println(Drone.fc_alt_min.gdist);
      }
      vTaskDelay(5000);
    } else {
      vTaskDelay(2000);
      //      Serial.println("fc_warn : SENSE ALT_MIN OFF");
    }
  }
}

void GetAngles( void * pvParameters) {
  for (;;) {
    int index = Drone.get_fc_run("fc_mpu");
    if (Drone.sensors[index].status) {
      if (index != -1) {
        Drone.sensors[index].fc_sensor->run();
      }
    } else {
      vTaskDelay(2000);
      //      Serial.println("fc_warn : SENSE IMU OFF");
    }
  }
}

void Correct() {
  Serial.println("working`");
  if (!Drone.FailSafe && Drone.armed) {
    int index = Drone.get_fc_run("fc_mpu");
    if (Drone.sensors[index].status) {
      if (index != -1) {
        Drone.sensors[index].fc_sensor->run();
        //      Serial.println(Drone.fc_mpu.gz);
        //        if ((abs(Drone.fc_mpu.roll) > 45) || (abs(Drone.fc_mpu.pitch) > 45)) {
        //          Drone.FailSafeStop();
        //        }
        if (Drone.fc_mpu.roll > 45 || Drone.fc_mpu.roll < -45 || Drone.fc_mpu.pitch > 45 || Drone.fc_mpu.pitch < -45) {
          Drone.FailSafeStop();
        }
        float pitched = 0.00f;
        float rolled = 0.00f;
        float yawed = 0.00f;
        pitch.compute(Drone.fc_mpu.gx, pidControlRates.j2y, &pitched);
        roll.compute(Drone.fc_mpu.gy, -pidControlRates.j2x, &rolled);
        yaw.compute(Drone.fc_mpu.gz, pidControlRates.j1x, &yawed);
        float pitchFinal = map(pitched, -30000.00f, 30000.00f, 300.00f, -300.00f);
        float rollFinal = map(rolled, -30000.00f, 30000.00f, -300.00f, 300.00f);
        float yawFinal = map(yawed, -30000.00f, 30000.00f, 300.00f, -300.00f);
        float throttle;
        if (incomingReadings.j1y <= 1870) {
          throttle = 1000;
        } else {
          throttle = pidControlRates.j1y;
        }
        Drone.mrc(
          constrain((throttle - yawFinal + pitchFinal - rollFinal), 1000, 2000),
          constrain((throttle + yawFinal + pitchFinal + rollFinal), 1000, 2000),
          constrain((throttle + yawFinal - pitchFinal - rollFinal), 1000, 2000),
          constrain((throttle - yawFinal - pitchFinal + rollFinal), 1000, 2000)
        );
      }
    } else {
      vTaskDelay(2000);
      Serial.println("(!) fc_imp_warn : SENSE CORRECTION OFF! PREDICT DRONE UNSTABLE!");
    }
  }
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(1);
  WiFi.mode(WIFI_STA);
  if (EEPROM.read(0) == 1) {
    Serial.println("RECOVERY MODE");
    Serial.println("use {\"fc_fc\":\"fc_eeprom\", \"fc_sub\":\"write\", \"args\":[0]} to enable flight mode and try again");
  } else if (EEPROM.read(0) == 0) {
    // your startup sequence here
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    Drone.fc_alt_min.configure(trigPin, echoPin, SOUND_SPEED);
    Drone.fc_mpu.configure();
    Drone.showCLI();
    xTaskCreatePinnedToCore(GetDistance, "UltraSonic", 4096, NULL, 1, &UltraSonic, 0);
    xTaskCreatePinnedToCore(GetAngles, "Mpu", 4096, NULL, 2, &Mpu, 0);

    delay(5000);
    Drone.startArm(MIN_SIGNAL, MAX_SIGNAL, 5000);
    delay(8000);

    EEPROM.write(0, 0);
    EEPROM.commit();
  }
}

void loop() {
  Drone.debugCLIHandler();
  if (Drone.fc_mpu.status) {
    if (Drone.started) {
      Correct();
    }
  }
}
