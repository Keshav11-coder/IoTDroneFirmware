#ifndef PIDSys_h
#define PidSys_h

class PIDSys {
  private:
    double def_kp;
    double def_ki;
    double def_kd;

    float pid_p;
    float pid_i;
    float pid_d;

    double def_des_angle;
    double def_throttle;

    float PID, pwm_l, pwm_r, error, previous_error;
  public:
    PIDSys() {
      def_kp = 3.55; //3.55
      def_ki = 0.005; //0.003
      def_kd = 2.05; //2.05

      pid_p = 0;
      pid_i = 0;
      pid_d = 0;

      def_des_angle = 0;
      def_throttle = 1300;
    }

    void configure (double kp, double ki, double kd, float desired_angle, double throttle) {
      def_kp = kp;
      def_ki = ki;
      def_kd = kd;

      def_des_angle = desired_angle;
      def_throttle = throttle;
    }

    void compute(float angle, float desired_angle, float elapsedTime, float* spd_l, float* spd_r) {
      /*First calculate the error between the desired angle and
        the real measured angle*/

      error = angle - desired_angle;

      /*Next the proportional value of the PID is just a proportional constant
        multiplied by the error*/

      pid_p = def_kp * error;

      if (-3 < error < 3)
      {
        pid_i = pid_i + (def_ki * error);
      }

      pid_d = def_kd * ((error - previous_error) / elapsedTime);

      /*The final PID values is the sum of each of this 3 parts*/

      PID = pid_p + pid_i + pid_d;
      //PID=constrain(PID,1100,2000);
      
//      if (PID < -1100)
//      {
//        PID = -1100;
//      }
//      if (PID > 1100)
//      {
//        PID = 1100;
//      }

      /*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value*/

      pwm_l = def_throttle + PID;
      pwm_r = def_throttle - PID;

      *spd_l = pwm_l;
      *spd_r = pwm_r;

      previous_error = error; //Remember to store the previous error.
    }
};

#endif
