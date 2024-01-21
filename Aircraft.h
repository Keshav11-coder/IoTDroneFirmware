#include <ESP32Servo.h>
#include <ESP32PWM.h>

class Aircraft {
  private:
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

    int green1 = 14; // leds
    int green2 = 17;
    int red1 = 13;
    int red2 = 12;
    unsigned long prevtime = 0;
    int blinkCount = 0;
    int blinkDelay = 100;
    int breakDelay = 1000;
    bool ledState = LOW;
    bool lightsPattern = true; // true will enable the lights pattern. false will disable the lights pattern.
    bool green1state = LOW;
    bool green2state = LOW;
    bool red1state = LOW;
    bool red2state = LOW;
    int Fmode = 0;

    // ultra distance sensor

    float SOUND_SPEED = 0.034;
    int trigPin = 18;
    int echoPin = 19;
    long duration;

    int relaySwitch = 21;
  public:
    Aircraft(int relaySignal) {
      relaySwitch = relaySignal;
      pinMode(relaySwitch, OUTPUT);
    }

    enum drone_mode { // it takes a long time to come up with variable names
      Debug,
      Sequence,
      OFF
    };

    drone_mode MODE = OFF;
    int mrca[4] = {1000, 1000, 1000, 1000};
    bool armed = false;
    bool FailSafe = false;

    void configureMode(drone_mode mode) {
      MODE = mode;
    }

    void configureMotors(int topl, int topr, int botl, int botr) {
      if (!armed) {
        topl_pin = topl;
        topr_pin = topr;
        bottoml_pin = botl;
        bottomr_pin = botr;

        topl_prop.attach(topl_pin);
        topr_prop.attach(topr_pin);
        bottoml_prop.attach(bottoml_pin);
        bottomr_prop.attach(bottomr_pin);
      }
    }

    void configureLights(int green1, int green2, int red1, int red2) {
      green1 = green1;
      green2 = green2;
      red1 = red1;
      red2 = red2;

      pinMode(green1, OUTPUT);
      pinMode(green2, OUTPUT);
      pinMode(red1, OUTPUT);
      pinMode(red2, OUTPUT);
    }

    void configureDistanceSensor(int trig, int echo, float soundSpeed) {
      SOUND_SPEED = soundSpeed;
      trigPin = trig;
      echoPin = echo;

      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
    }



    void startArm(int MIN_S, int MAX_S, int Wait) {
      if (!armed) {
        armed = false;
        MIN_SIGNAL = MIN_S;
        MAX_SIGNAL = MAX_S;

        Serial.println("Sending MAX signal ...");
        topl_prop.writeMicroseconds(MAX_SIGNAL);
        topr_prop.writeMicroseconds(MAX_SIGNAL);
        bottomr_prop.writeMicroseconds(MAX_SIGNAL);
        bottoml_prop.writeMicroseconds(MAX_SIGNAL);

        //        Serial.println("Turn on drone now.");
        digitalWrite(relaySwitch, 1);
        delay(4000);

        Serial.println("Sending MIN signal ...");
        topl_prop.writeMicroseconds(MIN_SIGNAL);
        topr_prop.writeMicroseconds(MIN_SIGNAL);
        bottomr_prop.writeMicroseconds(MIN_SIGNAL);
        bottoml_prop.writeMicroseconds(MIN_SIGNAL);
        delay(Wait);
        armed = true;
        Serial.println("Calibrated. Now waiting for input.");
        Serial.println("");
        Serial.println("");
      }
    }

    float getGroundDistance() {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      float dist = duration * SOUND_SPEED / 2;
      return dist;
    }

    void getAngle(float* ax, float* ay, float* az) {

    }

    void mrc(int tlspd, int trspd, int blspd, int brspd) { // motor rc
      if (armed && FailSafe == false && MODE != OFF) {
        topl_prop.writeMicroseconds(constrain((int)tlspd, 1000, 1800));
        topr_prop.writeMicroseconds(constrain((int)trspd, 1000, 1800));
        bottomr_prop.writeMicroseconds(constrain((int)brspd, 1000, 1800));
        bottoml_prop.writeMicroseconds(constrain((int)blspd, 1000, 1800)); // could be the constraining
      }
    }

    void setLights(bool tl = -1, bool tr = -1, bool bl = -1, bool br = -1, int fmode = -1) {
      if (tl != -1 && tr != -1 && bl != -1 && br != -1 && fmode == -1) {
        lightsPattern = false;
        red1state = tl;
        red2state = tr;
        green1state = bl;
        green2state = br;
      }
      else if (fmode != -1) {
        lightsPattern = true;
        Fmode = fmode;
      }
    }

    void updateLights() { // fmode 0 - 2 hover - fly - sports
      unsigned long now = millis();

      if (lightsPattern) {
        blinkDelay = Fmode == 0 ? 100 : Fmode == 1 ? 100 : 80;
        breakDelay = Fmode == 0 ? 2500 : Fmode == 1 ? 2000 : 600;

        if (blinkCount < (Fmode == 0 ? 1 : Fmode == 1 ? 2 : 3)) {
          if (now - prevtime >= blinkDelay) {
            prevtime = now;

            if (ledState == LOW) {
              ledState = HIGH;
            } else {
              ledState = LOW;
              blinkCount++;
            }

            digitalWrite(green1, ledState);
            digitalWrite(green2, ledState);
          }
        }
        else if (now - prevtime >= breakDelay) {
          blinkCount = 0;
          ledState = LOW;
          digitalWrite(green1, ledState);
          digitalWrite(green2, ledState);
        }
        digitalWrite(red1, HIGH);
        digitalWrite(red2, HIGH);
      } else {
        digitalWrite(red1, red1state);
        digitalWrite(red2, red2state);
        digitalWrite(green1, green1state);
        digitalWrite(green2, green2state);
      }
    }

    void ems() {
      if (!FailSafe) {
        mrc(1000, 1000, 1000, 1000);
      }
    }

    void resetFailSafe() {
      FailSafe == false;
    }

    void FailSafeStop () {
      mrc(1000, 1000, 1000, 1000);
    }

    // cli's

    void showCLI() {
      if (MODE == Debug) { // debug cli
        Serial.println("       -- Debug --");
        Serial.println("available debug functions");
        Serial.println("   ---- ---- ---- ----");
        Serial.println("");
        Serial.println("[codename: motor]");
        Serial.println("  [description] Motor test - test each individual motor");
        Serial.println("  [syntax] motor> topleft topright bottomleft bottomright");
        Serial.println("  [limits] all motor values will be constrained to 1000-1800");
        Serial.println("");
        Serial.println("[codename: lights]");
        Serial.println("  [description] Lights test - test each individual light");
        Serial.println("  [syntax] lights> topleft topright bottomleft bottomright");
        Serial.println("  [limits] all light values are HIGH/LOW only in binary form, only available syntax is 1/0");
        Serial.println("");
        Serial.println("[codename: setlights]");
        Serial.println("  [description] Set Lights Pattern Preset - set any lights pattern preset");
        Serial.println("  [syntax] setlights> integer");
        Serial.println("  [limits] setlights is an integer case, so a value from 0 - 2 indicating hover, flight, sport flight");
        Serial.println("");
        Serial.println("[codename: pidtest]");
        Serial.println("  [description] PID test - test each pid axis effect on the motors, you can toggle them on/off");
        Serial.println("  [syntax] pidtest> x(pitch) y(roll) z(yaw)");
        Serial.println("  [limits] only binary true/false values allowed (1/0)");
        Serial.println("  [!] PROPS OFF. MOTORS WILL SPIN RAPIDLY IF NOT CONFIGURED.");
        Serial.println("");
        Serial.println("[codename: pidtune]");
        Serial.println("  [description] PID test - tune each axis kP kI kD values");
        Serial.println("  [syntax][0] pidtune pitch> kp ki kd");
        Serial.println("          [1] pidtune roll>  kp ki kd");
        Serial.println("          [2] pidtune yaw>   kp ki kd");
        Serial.println("  [limits] only floats allowed for all P I D values");
        Serial.println("");
        Serial.println("---- ---- ---- ----");
        Serial.println("");
        Serial.println("Choose the options by sending the codename with the syntax example provided");
        Serial.println("Call FailSafe stop at any time using the combination stop> or s> or <>. *you can change this in config");
      }
    }

    void debugCLIHandler() {
      String command = Serial.readStringUntil('>'); // "motor>1000 1000 1000 1000"
      if (command == "motor") {
        int mrca[4] = {Serial.parseFloat(), Serial.parseFloat(), Serial.parseFloat(), Serial.parseFloat()};
        mrc(mrca[0], mrca[1], mrca[2], mrca[3]);
      }
      else if (command == "lights") {
        int lightStates[4] = {Serial.parseFloat(), Serial.parseFloat(), Serial.parseFloat(), Serial.parseFloat()};
        lightsPattern = false;
        red1state = lightStates[0];
        red2state = lightStates[1];
        green1state = lightStates[2];
        green2state = lightStates[3];
      }
      if (command == "setlights") {
        lightsPattern = true;
        int pattern = Serial.parseFloat();
        setLights(-1, -1, -1, -1, pattern);
      }
      else if (command == "stop" || command == "s" || command == "<") {
        FailSafeStop();
      }
    }
};
