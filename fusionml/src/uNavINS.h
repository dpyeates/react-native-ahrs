/*
uNavINS.h

Original Author:
Adhika Lie
2012-10-08
University of Minnesota
Aerospace Engineering and Mechanics
Copyright 2011 Regents of the University of Minnesota. All rights reserved.

Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to get gyro and accel bias. Added initialization to
estimated angles rather than assuming IMU is level. Added method to get psi,
rather than just heading, and ground track.
Brian R Taylor
brian.taylor@bolderflight.com
2017-12-20
Bolder Flight Systems
Copyright 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// XXX - add set methods for sensor characteristics.
// Confirmed: psi is heading (aircraft orientation), not ground track (velocity direction).
// 28/1/26: Added covariance std dev outputs for position, velocity, and attitude.
// 28/1/26: Magnetometer integration: Full 3D mag measurement with WMM expected field comparison


#ifndef UNAVINS_H
#define UNAVINS_H
#if defined(ARDUINO)
  #include "Arduino.h"
  #include "Eigen.h"
  #include <Eigen/Dense>
#else
  #include <sys/time.h>
  #include <stdint.h>
  #include <math.h>
  #include <Eigen/Core>
  #include <Eigen/Dense>

  static inline uint64_t micros() {
      struct timeval tv;
      gettimeofday(&tv,NULL);
      return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
  }

  class elapsedMicros
  {
  private:
  	unsigned long us;
  public:
  	elapsedMicros(void) { us = micros(); }
  	elapsedMicros(unsigned long val) { us = micros() - val; }
  	elapsedMicros(const elapsedMicros &orig) { us = orig.us; }
  	operator unsigned long () const { return micros() - us; }
  	elapsedMicros & operator = (const elapsedMicros &rhs) { us = rhs.us; return *this; }
  	elapsedMicros & operator = (unsigned long val) { us = micros() - val; return *this; }
  	elapsedMicros & operator -= (unsigned long val)      { us += val ; return *this; }
  	elapsedMicros & operator += (unsigned long val)      { us -= val ; return *this; }
  	elapsedMicros operator - (int val) const           { elapsedMicros r(*this); r.us += val; return r; }
  	elapsedMicros operator - (unsigned int val) const  { elapsedMicros r(*this); r.us += val; return r; }
  	elapsedMicros operator - (long val) const          { elapsedMicros r(*this); r.us += val; return r; }
  	elapsedMicros operator - (unsigned long val) const { elapsedMicros r(*this); r.us += val; return r; }
  	elapsedMicros operator + (int val) const           { elapsedMicros r(*this); r.us -= val; return r; }
  	elapsedMicros operator + (unsigned int val) const  { elapsedMicros r(*this); r.us -= val; return r; }
  	elapsedMicros operator + (long val) const          { elapsedMicros r(*this); r.us -= val; return r; }
  	elapsedMicros operator + (unsigned long val) const { elapsedMicros r(*this); r.us -= val; return r; }
  };
#endif

class uNavINS {
  public:
    // Update filter with all sensor inputs
    // @param dt Time step (seconds)
    // @param TOW GPS time of week (ms)
    // @param vn, ve, vd GPS velocity NED (m/s)
    // @param lat, lon, alt GPS position (rad, rad, m)
    // @param p, q, r Gyro rates (rad/s)
    // @param ax, ay, az Accelerometer (m/s^2)
    // @param hx, hy, hz Magnetometer body frame (uT)
    // @param bn, be, bd Expected magnetic field NED from WMM (nT), or NAN if unavailable
    void update(double dt, unsigned long TOW, double vn, double ve, double vd,
                double lat, double lon, double alt,
                float p, float q, float r,
                float ax, float ay, float az,
                float hx, float hy, float hz,
                float bn, float be, float bd);
    float getPitch_rad();
    float getRoll_rad();
    float getYaw_rad();
    float getHeading_rad();
    double getLatitude_rad();
    double getLongitude_rad();
    double getAltitude_m();
    double getVelNorth_ms();
    double getVelEast_ms();
    double getVelDown_ms();
    float getGroundTrack_rad();
    float getFlightPathAngle_rad();
    float getHorizontalFlightPathAngle_rad();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    float getAccelBiasX_mss();
    float getAccelBiasY_mss();
    float getAccelBiasZ_mss();
    float getMagBiasX_nT();
    float getMagBiasY_nT();
    float getMagBiasZ_nT();
    // Position uncertainty (1-sigma standard deviation)
    float getPositionNorthStd_m();
    float getPositionEastStd_m();
    float getPositionDownStd_m();
    // Velocity uncertainty (1-sigma standard deviation)
    float getVelocityNorthStd_ms();
    float getVelocityEastStd_ms();
    float getVelocityDownStd_ms();
    // Attitude uncertainty (1-sigma standard deviation)
    float getRollStd_rad();
    float getPitchStd_rad();
    float getYawStd_rad();
  private:
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // error characteristics of navigation parameters
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Std dev of Accelerometer Wide Band Noise (m/s^2)
    const float SIG_W_A = 1.0f;       // 1 m/s^2
    // Std dev of gyro output noise (rad/s)
    const float SIG_W_G = 0.00524f;   // 0.3 deg/s
    // Std dev of Accelerometer Markov Bias
    const float SIG_A_D = 0.1f;       // 5e-2*g
    // Correlation time or time constant
    const float TAU_A = 100.0f;
    // Std dev of correlated gyro bias
    const float SIG_G_D = 0.00873f;   // 0.1 deg/s
    // Correlation time or time constant
    const float TAU_G = 50.0f;
    // GPS measurement noise std dev (m)
    const float SIG_GPS_P_NE = 3.0f;
    const float SIG_GPS_P_D = 5.0f;
    // GPS measurement noise std dev (m/s)
    const float SIG_GPS_V = 0.5f;
    // Std dev of magnetometer measurement noise (nT)
    const float SIG_MAG = 200.0f;
    // Std dev of magnetometer bias random walk (nT)
    const float SIG_M_D = 50.0f;
    // Correlation time for mag bias (seconds) - slow drift
    const float TAU_M = 600.0f;
    // Initial set of covariance
    const float P_P_INIT = 10.0f;
    const float P_V_INIT = 1.0f;
    const float P_A_INIT = 0.34906f;     // 20 deg
    const float P_HDG_INIT = 3.14159f;   // 180 deg
    const float P_AB_INIT = 0.9810f;     // 0.5*g
    const float P_GB_INIT = 0.01745f;    // 5 deg/s
    const float P_MB_INIT = 5000.0f;     // Initial mag bias uncertainty (nT)
    // acceleration due to gravity
    const float G = 9.807f;
    // major eccentricity squared
    const double ECC2 = 0.0066943799901;
    // earth semi-major axis radius (m)
    const double EARTH_RADIUS = 6378137.0;
    // initialized
    bool initialized = false;
    // timing
    elapsedMicros _t;
    float _dt;
    unsigned long previousTOW;
    // estimated attitude
    float phi, theta, psi, heading;
    // initial heading angle
    float psi_initial;
    // estimated NED velocity
    double vn_ins, ve_ins, vd_ins;
    // estimated location
    double lat_ins, lon_ins, alt_ins;
    // magnetic heading corrected for roll and pitch angle
    float Bxc, Byc;
    // accelerometer bias
    float abx, aby, abz;
    // gyro bias
    float gbx, gby, gbz;
    // magnetometer bias (body frame, nT)
    float mbx, mby, mbz;
    // earth radius at location
    double Re, Rn, denom;
    // State matrix (18 states: 3 pos, 3 vel, 3 att, 3 accel bias, 3 gyro bias, 3 mag bias)
    Eigen::Matrix<float,18,18> Fs = Eigen::Matrix<float,18,18>::Identity();
    // State transition matrix
    Eigen::Matrix<float,18,18> PHI = Eigen::Matrix<float,18,18>::Zero();
    // Covariance matrix
    Eigen::Matrix<float,18,18> P = Eigen::Matrix<float,18,18>::Zero();
    // For process noise transformation (15 process noise sources: 3 accel, 3 gyro, 3 accel bias, 3 gyro bias, 3 mag bias)
    Eigen::Matrix<float,18,15> Gs = Eigen::Matrix<float,18,15>::Zero();
    Eigen::Matrix<float,15,15> Rw = Eigen::Matrix<float,15,15>::Zero();
    // Process noise matrix
    Eigen::Matrix<float,18,18> Q = Eigen::Matrix<float,18,18>::Zero();
    // Gravity model
    Eigen::Matrix<float,3,1> grav = Eigen::Matrix<float,3,1>::Zero();
    // Rotation rate
    Eigen::Matrix<float,3,1> om_ib = Eigen::Matrix<float,3,1>::Zero();
    // Specific force
    Eigen::Matrix<float,3,1> f_b = Eigen::Matrix<float,3,1>::Zero();
    // DCM
    Eigen::Matrix<float,3,3> C_N2B = Eigen::Matrix<float,3,3>::Zero();
    // DCM transpose
    Eigen::Matrix<float,3,3> C_B2N = Eigen::Matrix<float,3,3>::Zero();
    // Temporary to get dxdt
    Eigen::Matrix<float,3,1> dx = Eigen::Matrix<float,3,1>::Zero();
    Eigen::Matrix<double,3,1> dxd = Eigen::Matrix<double,3,1>::Zero();
    // NED velocity INS
    Eigen::Matrix<double,3,1> V_ins = Eigen::Matrix<double,3,1>::Zero();
    // LLA INS
    Eigen::Matrix<double,3,1> lla_ins = Eigen::Matrix<double,3,1>::Zero();
    // NED velocity GPS
    Eigen::Matrix<double,3,1> V_gps = Eigen::Matrix<double,3,1>::Zero();
    // LLA GPS
    Eigen::Matrix<double,3,1> lla_gps = Eigen::Matrix<double,3,1>::Zero();
    // Position ECEF INS
    Eigen::Matrix<double,3,1> pos_ecef_ins = Eigen::Matrix<double,3,1>::Zero();
    // Position NED INS
    Eigen::Matrix<double,3,1> pos_ned_ins = Eigen::Matrix<double,3,1>::Zero();
    // Position ECEF GPS
    Eigen::Matrix<double,3,1> pos_ecef_gps = Eigen::Matrix<double,3,1>::Zero();
    // Position NED GPS
    Eigen::Matrix<double,3,1> pos_ned_gps = Eigen::Matrix<double,3,1>::Zero();
    // Quat
    Eigen::Matrix<float,4,1> quat = Eigen::Matrix<float,4,1>::Zero();
    // dquat
    Eigen::Matrix<float,4,1> dq = Eigen::Matrix<float,4,1>::Zero();
    // Measurement innovation (9 measurements: 3 pos, 3 vel, 3 mag)
    Eigen::Matrix<float,9,1> y = Eigen::Matrix<float,9,1>::Zero();
    // Measurement noise covariance (9x9: GPS pos/vel + mag)
    Eigen::Matrix<float,9,9> R = Eigen::Matrix<float,9,9>::Zero();
    // State correction vector
    Eigen::Matrix<float,18,1> x = Eigen::Matrix<float,18,1>::Zero();
    // Kalman Gain (18 states x 9 measurements)
    Eigen::Matrix<float,18,9> K = Eigen::Matrix<float,18,9>::Zero();
    // Measurement matrix (9 measurements x 18 states)
    Eigen::Matrix<float,9,18> H = Eigen::Matrix<float,9,18>::Zero();
    // Expected mag field in body frame (computed from C_N2B * mag_ned)
    Eigen::Matrix<float,3,1> mag_expected_body = Eigen::Matrix<float,3,1>::Zero();
    // skew symmetric
    Eigen::Matrix<float,3,3> sk(Eigen::Matrix<float,3,1> w);
    // lla rate
    Eigen::Matrix<double,3,1> llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla);
    // lla to ecef
    Eigen::Matrix<double,3,1> lla2ecef(Eigen::Matrix<double,3,1> lla);
    // ecef to ned
    Eigen::Matrix<double,3,1> ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref);
    // quaternion to dcm
    Eigen::Matrix<float,3,3> quat2dcm(Eigen::Matrix<float,4,1> q);
    // quaternion multiplication
    Eigen::Matrix<float,4,1> qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q);
    // maps angle to +/- 180
    float constrainAngle180(float dta);
    // maps angle to 0-360
    float constrainAngle360(float dta);
};

#endif
