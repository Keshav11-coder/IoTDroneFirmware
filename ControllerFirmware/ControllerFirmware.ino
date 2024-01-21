#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0xC8, 0xF0, 0x9E, 0xF2, 0x94, 0xAC};

// Define variables to store BME280 readings to be sent
float data;

String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
  float j1x;
  float j1y;
  float j2x;
  float j2y;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message Readings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

double map_range(double value, double in_min, double in_mid, double in_max, double out_min, double out_mid, double out_max) {
  // Ensure the input value is within the specified range
  value = min(max(value, in_min), in_max);

  // Map the input value to the output range
  if (value <= in_mid) {
    // Map to the left of the midpoint
    return out_min + (value - in_min) * (out_mid - out_min) / (in_mid - in_min);
  } else {
    // Map to the right of the midpoint
    return out_mid + (value - in_mid) * (out_max - out_mid) / (in_max - in_mid);
  }
}

void loop() {
  // Set values to send
  Readings.j1x = analogRead(34);
  Readings.j1y = analogRead(35);
  Readings.j2x = analogRead(33);
  Readings.j2y = analogRead(32);

  //  Serial.print("j1x: ");
  //  Serial.print(Readings.j1x);
  //  Serial.print(", j1y: ");
  //  Serial.print(Readings.j1y);
  //  Serial.print(", j2x: ");
  //  Serial.print(Readings.j2x);
  //  Serial.print(", j2y: ");
  //  Serial.println(Readings.j2y);

  // Add a small delay for stability
  delay(100);

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Readings, sizeof(Readings));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}
