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
//
// Baseline:
// See https://forum.pjrc.com/index.php?threads/unav-ins.48856/page-31#759, post 759
// https://forum.pjrc.com/index.php?attachments/unavins-master-zip.27780/
//
// =============================================================================
// ENHANCEMENTS FROM BASELINE CHANGELOG
// =============================================================================
//
// Enhancement 1: Tilt-compensated magnetometer heading with WMM
// ------------------------------------------------------------------
//  Expected Earth field in NED from World Magnetic Model (XYZgeomag)
//  Measured field (after subtracting estimated mag bias) tilt-compensated to horizontal
//  Scalar yaw error drives heading update only (roll/pitch from IMU/GPS)
//
// Enhancement 2: GPS rate-aware measurement noise
// ------------------------------------------------------------------
// Scale GPS measurement noise from observed update interval.
// This is not delayed-state replay; it only reduces trust in slow/stale GPS.
//
// Enhancement 3: GPS Adaptive Noise
// ----------------------------------------------------------
// Adaptive GPS measurement noise based on reported accuracy:
//   - Uses GPS hacc (horizontal accuracy estimate)
//   - Uses GPS vacc (vertical accuracy estimate)
//   - Uses GPS sacc (speed accuracy estimate)
//
// Benefits:
//   - Better performance in varying GPS conditions
//   - Automatically trusts high-quality GPS more
//   - Reduces impact of poor GPS (urban canyons, etc.)
//   - Backward compatible (optional parameters)
//
// The filter now scales GPS trust based on BOTH delay and reported accuracy.
//
// Enhancement 4: Enhanced Magnetic Interference Rejection
// -------------------------------------------------------------------------------
// Multi-gate magnetic interference detection (VQF-inspired):
//   1. Magnitude gate: Field strength within 0.5-2.0x expected (existing)
//   2. Inclination gate: Field dip angle within ±20° of expected (NEW)
//   3. Temporal consistency: Rejects sudden >10µT changes (NEW)
//
// Benefits:
//   - Better rejection of power lines, metal structures
//   - Improved performance near vehicles, buildings
//   - More robust heading in magnetically-disturbed environments
//
// Reference: VQF (Versatile Quaternion Filter) magnetic disturbance rejection
//
// Enhancement 5: Covariance Health Monitoring
// ---------------------------------------------------------------------
// Real-time filter health monitoring (ArduPilot EKF-inspired):
//   - Detects NaN/Inf in covariance or states (CRITICAL)
//   - Monitors covariance bounds for divergence (ERROR/WARNING)
//   - Validates bias estimates are physically reasonable (WARNING)
//   - Separate thresholds for roll/pitch vs yaw (yaw can be large without mag)
//
// API:
//   bool isHealthy()       - Returns true if filter is healthy
//   int getHealthStatus()  - Returns 0=healthy, 1=warning, 2=error, 3=critical
//
// Benefits:
//   - Early divergence detection
//   - Enables automatic recovery/reset logic
//   - Provides user feedback on filter quality
//
// Enhancement 6: Stationary Detection and ZUPT
// ------------------------------------------------------
// Two complementary approaches to detect and handle stationary scenarios:
//
// 1. SPEED-BASED ZUPT (Primary - Zero-Velocity Update)
// -------------------------------------------------------
// Detects sustained low ground speed for ZUPT application:
//   - Trigger: ground speed < 1.0 m/s for 3+ seconds (hysteresis: exit at 1.5 m/s)
//   - Applies to ALL stationary scenarios:
//     * Person holding phone while standing still (hand tremors don't prevent ZUPT)
//     * Phone in car mount at stoplight (engine vibration okay)
//     * Phone on desk (truly motionless)
//   - When active: observe velocity = [0,0,0] at 5 Hz with R = (0.1 m/s)²
//   - Safety gate: INS speed < 3 m/s when GPS is stale; when GPS confirms a stop,
//     ZUPT may still pull back a diverged INS (up to ~80 m/s)
//   - Tighter noise (0.05 m/s) when variance-based rest also detected
//
// 2. VARIANCE-BASED REST DETECTION (Supplementary - VQF-inspired)
// -----------------------------------------------------------------
// Detects device is truly motionless (on desk, not handheld):
//   - Online variance estimation using Welford's algorithm
//   - Hysteresis: requires 1s sustained rest to enter, 0.17s motion to exit
//   - Thresholds: gyro <1°/s variance, accel <0.5m/s² variance
//   - During rest, bias estimation is enhanced:
//     * TAU_G reduced from 50s to 10s (5x faster convergence)
//     * Process noise increased 10x (allows faster adaptation)
//     * Result: 50x faster gyro bias convergence when stationary
//     * Tighter ZUPT noise (0.05 m/s instead of 0.1 m/s)
//
// API:
//   bool isAtRest()    - Returns true if variance-based rest detected (truly motionless)
//   bool isZuptActive() - Returns true if ZUPT is currently being applied (speed-based)
//
// Benefits:
//   - Prevents velocity runaway when stationary (no more CRITICAL health cycles)
//   - Stable groundspeed and vertical speed in all stationary scenarios
//   - Rapid gyro bias calibration after power-on (1-2s vs 30-60s) when on desk
//   - Works for handheld stationary use (not just "phone on desk")
//
// Enhancement 7: Barometric Altitude Fusion
// ------------------------------------------------------------------
// Fuse barometric altitude when GPS vertical accuracy is poor:
//   - Activates when GPS vacc >10m or unavailable
//   - Uses QNH (if provided) or QNE (standard atmosphere)
//   - Adaptive noise: 2m baseline, scales with altitude
//   - Innovation gate: 3-sigma rejection
//
// Benefits:
//   - Better altitude in GPS-degraded conditions (urban, trees)
//   - Smoother altitude estimates
//   - Backup when GPS vertical is poor
//
// Backward compatible: barometer data is optional parameter
//
// =============================================================================

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
    // @param hacc Horizontal accuracy (m), optional, use NAN or -1 if unavailable
    // @param vacc Vertical accuracy (m), optional, use NAN or -1 if unavailable
    // @param sacc Speed accuracy (m/s), optional, use NAN or -1 if unavailable
    // @param baro_pressure Barometric pressure (hPa), optional, use NAN or -1 if unavailable
    // @param   (hPa), optional, use 1013.25 for QNE or -1 if unavailable
    void update(double dt, unsigned long TOW, double vn, double ve, double vd,
                double lat, double lon, double alt,
                float p, float q, float r,
                float ax, float ay, float az,
                float hx, float hy, float hz,
                float bn, float be, float bd,
                float hacc = -1.0f,
                float vacc = -1.0f,
                float sacc = -1.0f,
                float baro_pressure = -1.0f,
                float baro_qnh = 1013.25f);
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
    // Filter health monitoring
    bool isInitialized();
    bool isHealthy();
    int getHealthStatus();  // Returns health code: 0=healthy, 1=warning, 2=error, 3=critical
    // Rest detection and ZUPT
    bool isAtRest();     // Returns true if device is detected to be stationary (variance-based)
    bool isZuptActive(); // Returns true if ZUPT is currently being applied (speed-based)
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
    const float SIG_MAG = 1500.0f;
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
    // GPS measurement innovation (6 measurements: 3 pos, 3 vel)
    Eigen::Matrix<float,6,1> y = Eigen::Matrix<float,6,1>::Zero();
    // GPS measurement noise covariance
    Eigen::Matrix<float,6,6> R = Eigen::Matrix<float,6,6>::Zero();
    // State correction vector
    Eigen::Matrix<float,18,1> x = Eigen::Matrix<float,18,1>::Zero();
    // Kalman Gain (18 states x 6 GPS measurements)
    Eigen::Matrix<float,18,6> K = Eigen::Matrix<float,18,6>::Zero();
    // GPS measurement matrix
    Eigen::Matrix<float,6,18> H = Eigen::Matrix<float,6,18>::Zero();
    // Expected mag field in body frame (computed from C_N2B * mag_ned)
    Eigen::Matrix<float,3,1> mag_expected_body = Eigen::Matrix<float,3,1>::Zero();

    double filter_time = 0.0; // Time since initialization (seconds)

    // GPS interval-based measurement-noise scale (not a delayed-state replay)
    static constexpr float GPS_DELAY_NOMINAL = 0.200f;
    static constexpr float GPS_DELAY_MAX = 0.500f;
    static constexpr float GPS_DELAY_MIN = 0.050f;
    float gps_delay_estimate = GPS_DELAY_NOMINAL;
    double last_gps_fusion_time = 0.0;
    bool gps_has_updated = false;
    float last_gps_ground_speed = 0.0f;
    bool have_gps_speed = false;

    float computeDelayScaleFactor(float delay_seconds);
    void applyNavigationCorrection(const Eigen::Matrix<float,18,1>& dx);

    // ==========================================================================
    // COVARIANCE HEALTH MONITORING
    // ==========================================================================
    int filter_health_status = 0;  // 0=healthy, 1=warning, 2=error, 3=critical (reported, with hysteresis)
    int health_failure_count = 0;   // Consecutive health check failures
    int health_pending_code = 0;    // Raw health code candidate for transition
    int health_pending_count = 0;   // Consecutive checks at health_pending_code
    bool checkCovarianceHealth();   // Internal health check function

    // ==========================================================================
    // REST DETECTION (VQF-inspired)
    // ==========================================================================
    // Detect when device is stationary for improved bias estimation.
    // Variance is computed over a sliding 1 s window so rest can exit after a long sit.
    struct RestDetector {
      static const int WINDOW_SIZE = 60; // 1 second at 60 Hz
      float gx_buf[WINDOW_SIZE] = {};
      float gy_buf[WINDOW_SIZE] = {};
      float gz_buf[WINDOW_SIZE] = {};
      float ax_buf[WINDOW_SIZE] = {};
      float ay_buf[WINDOW_SIZE] = {};
      float az_buf[WINDOW_SIZE] = {};
      int head = 0;
      int count = 0;
      int rest_samples = 0;
      int motion_samples = 0;
      bool is_at_rest = false;

      void reset() {
        head = 0;
        count = 0;
        rest_samples = 0;
        motion_samples = 0;
        is_at_rest = false;
      }

      bool update(float gx, float gy, float gz, float ax, float ay, float az);
    };
    RestDetector rest_detector;

    // Magnetometer update state (must live on the instance so resetAhrs() clears it)
    double last_mag_update_time = -1.0;
    Eigen::Matrix<float,3,1> mag_meas_filtered = Eigen::Matrix<float,3,1>::Zero();
    bool mag_filter_initialized = false;

    // Barometer update rate limit
    double last_baro_update_time = -1.0;
    // Accelerometer tilt aid (when specific force is ~1 g)
    double last_accel_tilt_time = -1.0;

    // ==========================================================================
    // SPEED-BASED ZUPT (Zero-Velocity Update)
    // ==========================================================================
    // Detect sustained low ground speed for stationary scenarios
    float low_speed_timer = 0.0f;   // seconds below speed threshold
    bool zupt_active = false;        // ZUPT currently enabled
    float last_zupt_time = -1.0f;    // for rate limiting (5 Hz)

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
