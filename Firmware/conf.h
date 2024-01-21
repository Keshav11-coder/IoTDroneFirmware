// Drone Configuration
// Available variables to be changed anytime.

// mode RULES
//if both are off, the drone will listen to MODE
//if both are on, both will be turned off and the drone will listen to MODE
//if MODE is off, the drone will listen to flight sequence, SEQ
//if SEQ is off, the drone will listen to MODE
#ifndef conf_H
#define conf_H

enum drone_mode {
  Debug, // selection of debug options
  Sequence, // run a sequence of functionalities to test the drone (Work in progress, will not do anything as of 11-20-23
  OFF // does nothing
};

enum sequence { // tells the drone in which flying state it is. will be used for leds, interface and some more things
  Landed,
  Takeoff,
  Hover,
  Flight,
  Sportflight,
  Mission,
  Landing,
  SYSOFF
};

/* create some task handlers so the main code will stay clean and not interrupted by sensors and cli's*/
TaskHandle_t UltraSonic;
TaskHandle_t Mpu;
TaskHandle_t BLT;

drone_mode MODE = Debug;
sequence SEQ = SYSOFF;

int relayPowerSignal = 21; // edit relay IN here

int green1pin = 2;
int green2pin = 2;
int red1pin = 2;
int red2pin = 2;

unsigned long prevtime = 0;
int blinkCount = 0;
int blinkDelay = 100;
int breakDelay = 1000;
bool ledState = LOW;

Servo topl_prop;
Servo topr_prop;
Servo bottomr_prop;
Servo bottoml_prop;

int MAX_SIGNAL = 2000;
int MIN_SIGNAL = 1000;
int FMINS = 1000; //minimum the machine can go

int rc[4] = {1000, 1000, 1000, 1000};

int topl_pin = 12;
int topr_pin = 13;
int bottomr_pin = 14;
int bottoml_pin = 27;

float SOUND_SPEED = 0.034;
int trigPin = 18;
int echoPin = 19;
long duration;
float distanceCm;

bool armed = false;
bool FailSafe = false;

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
#endif
