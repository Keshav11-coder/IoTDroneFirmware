#include "Wire.h"

class MPU {
  private:
    int MPU_addr = 0x68;

    float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
    float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest
    float gscale ((250. / 32768.0) * (PI / 180.0)) //gyro default 250 LSB per d/s -> rad/s

    float q[4] = {1.0, 0.0, 0.0, 0.0};

    float Kp = 30.0;
    float Ki = 0.0;

    unsigned long now_ms, last_ms = 0; //millis() timers
    float yaw, pitch, roll; //Euler angle output

  public:
    void setup() {
      Wire.begin();
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
    }

    void ahrsReturnRaw() {

    }
}
