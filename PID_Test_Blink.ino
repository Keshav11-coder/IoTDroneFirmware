/*########---------------------MOTOR CONFIG---------------------########*/
#define M_PI 3.14159265358979323846
#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32PWM.h>

Servo top_right_prop;
Servo top_left_prop;
Servo bottom_right_prop;
Servo bottom_left_prop;
/*########---------------------MOTOR CONFIG---------------------########*/

/*########---------------------NAVIGATION LIGHTS CONFIG---------------------########*/
// Define constants for LED pins and blinking periods
#define green1 14
#define green2 2
#define red1 13
#define red2 12
#define VOID 3
int PERIOD_BLINK = 100; // 150 ms blink
int BREAK_PERIOD = 1000; // 1 second break

int ledStates[] = {LOW, LOW, LOW, LOW};
unsigned long lastTimes[] = {0, 0, 0, 0};
unsigned long breakTime = 0;
bool inBreak = false;
int currentLED = 0;
int blinkCount = 0;
int flightState = 1; //0 for still, 1 for motion
int blinks = 2;
int xdirection = 0;
/*########---------------------NAVIGATION LIGHTS CONFIG---------------------########*/

/*########---------------------IMU CONFIG---------------------########*/
#include "MPU9250.h"
float gyroX, gyroY, gyroZ; // Gyroscope data in radians per second
float accelX, accelY, accelZ; // Accelerometer data in m/s^2
float dt; // Time interval between sensor updates
float pitch, roll; // Pitch and roll angles in degrees

MPU9250 IMU(Wire, 0x68);
int status;
/*########---------------------IMU CONFIG---------------------########*/

/*########---------------------PID CONFIG---------------------########*/
#include "PIDSys.h"
PIDSys pidY;
PIDSys pidX;

float mainElapsedTime, timePrev;

double kp = 8;
double ki = 0.001;//0.005
double kd = 0.1;

double throttle = 1300;
float desired_angle = 0;
/*########---------------------PID CONFIG---------------------########*/

/*SETUP*/
void setup() {
  flightState = 0;
  top_right_prop.attach(16);
  top_left_prop.attach(4);
  bottom_right_prop.attach(15);
  bottom_left_prop.attach(5);
  pinMode(green1, OUTPUT);
  pinMode(green2, OUTPUT);
  pinMode(red1, OUTPUT);
  pinMode(red2, OUTPUT);
  top_left_prop.writeMicroseconds(1000);
  top_right_prop.writeMicroseconds(1000);
  bottom_left_prop.writeMicroseconds(1000);
  bottom_right_prop.writeMicroseconds(1000);

  Serial.begin(115200);
  Serial.read();

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  flightState = 1;
  pidY.configure(kp, ki, kd, desired_angle, throttle);
  pidX.configure(kp, ki, kd, desired_angle, throttle);
}
/*SETUP*/

/*####---------------------GET X Y ANGLE---------------------####*/
void getAccelAngle(float* Xpaxis, float* Ypaxis) {
  // Assuming you have the accelerometer readings in m/s^2 for X and Y axes
  float accelX = IMU.getAccelX_mss();
  float accelY = IMU.getAccelY_mss();
  float accelZ = IMU.getAccelZ_mss();

  // Calculate pitch angle in radians
  float pitchRad = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

  // Calculate roll angle in radians
  float rollRad = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ));

  // Convert radians to degrees
  float pitchDeg = pitchRad * 180.0 / M_PI;
  float rollDeg = rollRad * 180.0 / M_PI;

  *Xpaxis = pitchDeg;
  *Ypaxis = rollDeg;
}
/*####---------------------GET X Y ANGLE---------------------####*/

void handleLights(float Ycords) {
  unsigned long now = millis();
  blinks = flightState == 0 ? 1 : 2;
  xdirection = Ycords > 16 ? 2 : Ycords < -16 ? 1 : 0;

  if (!inBreak) {
    // Handle the quick blink for the current LED
    if (now - lastTimes[currentLED] >= PERIOD_BLINK) {
      lastTimes[currentLED] = now;
      ledStates[currentLED] = !ledStates[currentLED]; // Toggle LED state
      if (xdirection == 0) {
        digitalWrite(currentLED == 0 ? green1 : currentLED == 1 ? green2 : currentLED == 2 ? red1 : red2, ledStates[currentLED]);
      } else if (xdirection == 1) {
        digitalWrite(currentLED == 1 ? green2 : currentLED == 3 ? red2 : VOID, HIGH);
      }
      else if (xdirection == 2) {
        digitalWrite(currentLED == 0 ? green1 : currentLED == 2 ? red1 : VOID, HIGH);
      }
    }

    // Check if we've done the quick blink 2 times for all LEDs
    if (currentLED == 3) {
      if (blinkCount < blinks) {
        // Blink twice, then reset timer
        if (millis() - breakTime >= PERIOD_BLINK * 2) {
          breakTime = millis();
          blinkCount++;
        }
      } else {
        // Stop and enter the break period
        inBreak = true;
        breakTime = millis();
        for (int i = 0; i < 4; i++) {
          digitalWrite(i == 0 ? green1 : i == 1 ? green2 : i == 2 ? red1 : red2, LOW); // Turn OFF all LEDs
        }
      }
    }
    currentLED = (currentLED + 1) % 4; // Move to the next LED
  } else {
    // Handle the break period
    if (millis() - breakTime >= BREAK_PERIOD) {
      inBreak = false;
      breakTime = millis();
      currentLED = 0; // Reset to the first LED
      blinkCount = 0; // Reset blink count
    }
  }
}

void loop() {
  IMU.readSensor();
  float Xangle, Yangle;
  getAccelAngle(&Xangle, &Yangle); // get new angle

  handleLights(Yangle);

  float time;  // actual time read
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();
  mainElapsedTime = (time - timePrev);

  float left, right; // also known as left (1) right(2) of Y axis
  float top, bottom; // also known as (left) 1 (right) 2 of Xaxis

  pidX.compute(IMU.getAccelX_mss(), 0, mainElapsedTime, &top, &bottom);
  pidY.compute(IMU.getAccelY_mss(), 0, mainElapsedTime, &left, &right);

    top_left_prop.writeMicroseconds((top + left) / 2);
    bottom_left_prop.writeMicroseconds((bottom + left) / 2);
    top_right_prop.writeMicroseconds((top + right) / 2);
    bottom_right_prop.writeMicroseconds((bottom + right) / 2);

  Serial.print(top);
  Serial.print(" ");
  Serial.println(bottom);
}
