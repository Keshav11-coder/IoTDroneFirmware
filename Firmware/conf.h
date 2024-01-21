// Drone Configuration
// Available variables to be changed anytime.

// mode RULES
//if both are off, the drone will listen to MODE
//if both are on, both will be turned off and the drone will listen to MODE
//if MODE is off, the drone will listen to flight sequence, SEQ
//if SEQ is off, the drone will listen to MODE

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

drone_mode MODE = Debug;
sequence SEQ = SYSOFF;

int green1pin = 14;
int green2pin = 17;
int red1pin = 13;
int red2pin = 12;

Servo topl_prop;
Servo topr_prop;
Servo bottomr_prop;
Servo bottoml_prop;

int MAX_SIGNAL = 2000;
int MIN_SIGNAL = 1000;

int topl_pin = 4;
int topr_pin = 16;
int bottomr_pin = 15;
int bottoml_pin = 5;

bool armed = false;
bool FailSafe = false;
