#ifndef PIDSys_h
#define PidSys_h

class PIDSys {
  private:
    float k1 = .5;
    float k2 = 55;
    float k3 = .00001;

    int milliOld;
    int milliNew;
    int dt;

    float Target = 0;
    float Actual;
    float ErrorNew = 0;
    float ErrorOld;
    float ErrorChange;
    float ErrorSlope = 0;
    float ErrorArea = 0;
    float ServoVal = 90;
  public:
    PIDSys(double kp, double ki, double kd) {
      float k1 = kp;
      float k2 = ki;
      float k3 = kd;
    }

    void compute(float actual, float target, float* output) {
      milliOld = milliNew;
      milliNew = millis();
      dt = milliNew - milliOld;

      ErrorOld = ErrorNew;
      ErrorNew = target - actual;
      ErrorChange = ErrorNew - ErrorOld;
      ErrorSlope = ErrorChange / dt;
      ErrorArea = ErrorArea + ErrorNew * dt;

      ServoVal = ServoVal + k1 * ErrorNew + k2 * ErrorSlope + k3 * ErrorArea;
      *output = ErrorNew;
    }

    void tune(float kp, float ki, float kd) {
      float k1 = kp;
      float k2 = ki;
      float k3 = kd;
    }
};

#endif
