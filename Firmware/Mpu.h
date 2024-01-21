//
// MPU-6050 Mahony AHRS  S.J. Remington 3/2020
// 7/2020 added provision to recalibrate gyro upon startup. (variable cal_gyro)

#include "Wire.h"

// AD0 low = 0x68 (default for Sparkfun module)
// AD0 high = 0x69


class MPU {
  private:
    int MPU_addr = 0x68;

    // vvvvvvvvvvvvvvvvvv  VERY VERY IMPORTANT vvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    //These are the previously determined offsets and scale factors for accelerometer and gyro for
    // a particular example of an MPU-6050. They are not correct for other examples.
    //The AHRS will NOT work well or at all if these are not correct

    float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014}; // 0..2 offset xyz, 3..5 scale xyz
    // the code will work, but not as accurately, for an uncalibrated accelerometer. Use this line instead:
    // float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz

    float G_off[3] = { -499.5, -17.7, -82.0}; //raw offsets, determined for gyro at rest
    float gscale = ((250. / 32768.0) * (PI / 180.0)); //gyro default 250 LSB per d/s -> rad/s

    // ^^^^^^^^^^^^^^^^^^^ VERY VERY IMPORTANT ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    // GLOBALLY DECLARED, required for Mahony filter
    // vector to hold quaternion

    // Free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    float Kp = 30.0;
    float Ki = 0.0;

    // Notes: with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    // with MPU-6050, some instability observed at Kp=100, Kp=30 works well

    // char s[60]; //snprintf buffer, if needed for debug
  public:
    float yaw, pitch, roll; //Euler angle output
    float q[4] = {1.0, 0.0, 0.0, 0.0};
    int16_t gx, gy, gz;
    int cal_gyro = 1;  //set to zero to use gyro calibration offsets below.

    void configure() {
      Wire.begin();
      //      Serial.begin(9600);
      //      Serial.println("starting");

      // initialize sensor
      // defaults for gyro and accel sensitivity are 250 dps and +/- 2 g
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
    }

    void run() {
      static unsigned int i = 0; //loop counter
      static float deltat = 0;  //loop time in seconds
      static unsigned long now = 0, last = 0; //micros() timers
      static long gsum[3] = {0};
      //raw data
      int16_t ax, ay, az;
      int16_t Tmp; //temperature

      //scaled data as vector
      float Axyz[3];
      float Gxyz[3];


      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14); // request a total of 14 registers
      int t = Wire.read() << 8;
      ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      t = Wire.read() << 8;
      ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      t = Wire.read() << 8;
      az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      t = Wire.read() << 8;
      Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
      t = Wire.read() << 8;
      gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      t = Wire.read() << 8;
      gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      t = Wire.read() << 8;
      gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

      // calibrate gyro upon startup. SENSOR MUST BE HELD STILL (a few seconds)
      i++;
      if (cal_gyro) {

        gsum[0] += gx; gsum[1] += gy; gsum[2] += gz;
        if (i == 500) {
          cal_gyro = 0;  //turn off calibration and print results

          for (char k = 0; k < 3; k++) G_off[k] = ((float) gsum[k]) / 500.0;

          Serial.print("G_Off: ");
          Serial.print(G_off[0]);
          Serial.print(", ");
          Serial.print(G_off[1]);
          Serial.print(", ");
          Serial.print(G_off[2]);
          Serial.println();
        }
      }

      // normal AHRS calculations

      else {
        Axyz[0] = (float) ax;
        Axyz[1] = (float) ay;
        Axyz[2] = (float) az;

        //apply offsets and scale factors from Magneto
        for (i = 0; i < 3; i++) Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

        Gxyz[0] = ((float) gx - G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
        Gxyz[1] = ((float) gy - G_off[1]) * gscale;
        Gxyz[2] = ((float) gz - G_off[2]) * gscale;

        //  snprintf(s,sizeof(s),"mpu raw %d,%d,%d,%d,%d,%d",ax,ay,az,gx,gy,gz);
        //  Serial.println(s);

        now = micros();
        deltat = (now - last) * 1.0e-6; //seconds since last update
        last = now;

        Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

        // Compute Tait-Bryan angles.
        // In this coordinate system, the positive z-axis is down toward Earth.
        // Yaw is the angle between Sensor x-axis and Earth magnetic North
        // (or true North if corrected for local declination, looking down on the sensor
        // positive yaw is counterclockwise, which is not conventional for NED navigation.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        // Earth is positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
        // arise from the definition of the homogeneous rotation matrix constructed
        // from quaternions. Tait-Bryan angles as well as Euler angles are
        // non-commutative; that is, the get the correct orientation the rotations
        // must be applied in the correct order which for this configuration is yaw,
        // pitch, and then roll.
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.

        pitch  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
        roll = asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
        //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
        yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - ( q[2] * q[2] + q[3] * q[3]));
        // to degrees
        yaw   *= 180.0 / PI;
        if (yaw < 0) yaw += 360.0; //compass circle
        //ccrrect for local magnetic declination here
        pitch *= 180.0 / PI;
        roll *= 180.0 / PI;

        //        now_ms = millis(); //time to print?
        //        if (now_ms - last_ms >= print_ms) {
        //          last_ms = now_ms;
        //          // print angles for serial plotter...
        //          //  Serial.print("ypr ");
        //          Serial.print(yaw, 0);
        //          Serial.print(", ");
        //          Serial.print(pitch, 0);
        //          Serial.print(", ");
        //          Serial.println(roll, 0);
        //        }
      }
    }

    void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat) {
      float recipNorm;
      float vx, vy, vz;
      float ex, ey, ez;  //error terms
      float qa, qb, qc;
      static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
      float tmp;

      // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
      tmp = ax * ax + ay * ay + az * az;

      // ignore accelerometer if false (tested OK, SJR)
      if (tmp > 0.0)
      {

        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2];
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f) {
          ix += Ki * ex * deltat;  // integral error scaled by Ki
          iy += Ki * ey * deltat;
          iz += Ki * ez * deltat;
          gx += ix;  // apply integral feedback
          gy += iy;
          gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
      }

      // Integrate rate of change of quaternion, given by gyro term
      // rate of change = current orientation quaternion (qmult) gyro rate

      deltat = 0.5 * deltat;
      gx *= deltat;   // pre-multiply common factors
      gy *= deltat;
      gz *= deltat;
      qa = q[0];
      qb = q[1];
      qc = q[2];

      //add qmult*delta_t to current orientation
      q[0] += (-qb * gx - qc * gy - q[3] * gz);
      q[1] += (qa * gx + qc * gz - q[3] * gy);
      q[2] += (qa * gy - qb * gz + q[3] * gx);
      q[3] += (qa * gz + qb * gy - qc * gx);

      // Normalise quaternion
      recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
      q[0] = q[0] * recipNorm;
      q[1] = q[1] * recipNorm;
      q[2] = q[2] * recipNorm;
      q[3] = q[3] * recipNorm;
    }
};