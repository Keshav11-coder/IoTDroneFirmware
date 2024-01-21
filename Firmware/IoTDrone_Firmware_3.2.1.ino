#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include "conf.h"
#include "Aircraft.h"
Aircraft Drone(23);

const char* ssid = "Team09";
const char* password =  "H@ckTe@m)(";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void configureWss() {
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false);
  });

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.begin();
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("Websocket client connection received");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("Client disconnected");
  } else if (type == WS_EVT_DATA) {
    char* received_data = (char*) data;

    // product string
    String data_string = (String)received_data;

    // request only my data, remove jargon
    String jds = data_string.substring(0, 12);

    if ((received_data[0]) == 'j') {
      String data_str = jds;
      if (data_str.substring(0, 3) == "jl_") {
        String temp_data_str = data_str;
        temp_data_str.remove(0, 3); // remove 'jl_'

        char arr[temp_data_str.length() + 1];
        temp_data_str.toCharArray(arr, sizeof(arr));

        char* xStr = strtok(arr, " ");
        char* yStr = strtok(NULL, " ");

        int x = atoi(xStr);
        int y = atoi(yStr);

        Serial.print(x / 100);
        Serial.print(" x--y ");
        Serial.println(y / 100);
      }
    }
  }
}

void GetDistance( void * pvParameters ) {
  Serial.print("UltraSonic running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (Drone.armed) {
      //      Serial.println(Drone.getGroundDistance());
      vTaskDelay(500);
    }
  }
}

void GetAngles( void * pvParameters) {
  Serial.print("UltraSonic running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    if (Drone.armed) {
      Drone.getAngleDegrees();
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
  Drone.configureDistanceSensor(trigPin, echoPin, SOUND_SPEED);
  Drone.configureMpu();
  //  Aircraft::mpu().configure();
  //  mpu.configure();
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
  //  mpu.ahrs();
  //  Aircraft::mpu().ahrs();
}
