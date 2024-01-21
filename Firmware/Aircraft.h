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

    bool armed = false;
    bool FailSafe = false;
  public:
    Aircraft() {//enum drone_mode a
      //      Serial.println(a);
    }

    enum drone_mode { // it takes a long time to come up with variable names
      Debug,
      Sequence,
      OFF
    };

    drone_mode MODE = OFF;
    int mrca[4] = {1000, 1000, 1000, 1000};

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

        Serial.println("Turn on drone now.");
        delay((Wait + 2000));

        Serial.println("Sending MIN signal ...");
        topl_prop.writeMicroseconds(MIN_SIGNAL);
        topr_prop.writeMicroseconds(MIN_SIGNAL);
        bottomr_prop.writeMicroseconds(MIN_SIGNAL);
        bottoml_prop.writeMicroseconds(MIN_SIGNAL);
        armed = true;
        Serial.println("Calibrated. Now waiting for input.");
        Serial.println("");
        Serial.println("");
      }
    }

    void mrc(int tlspd, int trspd, int blspd, int brspd) { // motor rc
      if (armed && FailSafe == false && MODE==Debug) {
        topl_prop.writeMicroseconds(constrain((int)tlspd, 1000, 1800));
        topr_prop.writeMicroseconds(constrain((int)trspd, 1000, 1800));
        bottomr_prop.writeMicroseconds(constrain((int)brspd, 1000, 1800));
        bottoml_prop.writeMicroseconds(constrain((int)blspd, 1000, 1800)); // could be the constraining
      }
    }

    void writeMrca() {
      if (armed && FailSafe == false) {
        topl_prop.writeMicroseconds(constrain((int)mrca[0], 1000, 1800));
        topr_prop.writeMicroseconds(constrain((int)mrca[1], 1000, 1800));
        bottoml_prop.writeMicroseconds(constrain((int)mrca[2], 1000, 1800)); // could be the constraining
        bottomr_prop.writeMicroseconds(constrain((int)mrca[3], 1000, 1800));
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

    void updateLights(int fmode) { // fmode 0 - 2 hover - fly - sports
      unsigned long now = millis();

      blinkDelay = fmode == 0 ? 100 : fmode == 1 ? 100 : 80;
      breakDelay = fmode == 0 ? 2500 : fmode == 1 ? 2000 : 600;

      if (blinkCount < (fmode == 0 ? 1 : fmode == 1 ? 2 : 3)) {
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
    }
};
