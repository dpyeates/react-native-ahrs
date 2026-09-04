/*
uNavINS.cpp

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

#include "uNavINS.h"
#include "AltitudeCalculator.h"
#include <cstdio>
#include <cmath>

void uNavINS::update(double dt_in, unsigned long TOW, double vn, double ve, double vd,
                     double lat, double lon, double alt,
                     float p, float q, float r,
                     float ax, float ay, float az,
                     float hx, float hy, float hz,
                     float bn, float be, float bd,
                     float hacc, float vacc, float sacc,
                     float baro_pressure, float baro_qnh) {
  if (!initialized) {
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
        !std::isfinite(p) || !std::isfinite(q) || !std::isfinite(r)) {
      return;
    }
    float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_norm < 0.5f * G || accel_norm > 1.5f * G) {
      return;
    }
    float ax_over_g = ax / G;
    if (fabsf(ax_over_g) > 1.0f) {
      return;
    }
    theta = asinf(ax_over_g);
    float cos_theta = cosf(theta);
    if (fabsf(cos_theta) < 1e-3f) {
      return;
    }
    float ay_term = -ay / (G * cos_theta);
    if (fabsf(ay_term) > 1.0f) {
      return;
    }
    phi = asinf(ay_term);

    float mag_norm_uT = sqrtf(hx * hx + hy * hy + hz * hz);
    bool mag_ok = std::isfinite(hx) && std::isfinite(hy) && std::isfinite(hz) && mag_norm_uT > 5.0f;
    if (mag_ok) {
      Bxc = hx * cosf(theta) + (hy * sinf(phi) + hz * cosf(phi)) * sinf(theta);
      Byc = hy * cosf(phi) - hz * sinf(phi);
      psi = constrainAngle180(atan2f(-Byc, Bxc));
    } else {
      psi = 0.0f;
    }
    psi_initial = psi;
    // euler to quaternion
    quat(0) = cosf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    quat(1) = cosf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f) - sinf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f);
    quat(2) = cosf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f);
    quat(3) = sinf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) - cosf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    if (!quat.allFinite()) {
      return;
    }
    // Assemble the matrices
    grav(2,0) = G;
    H.setZero();
    H.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();
    // Rw (15 process noise sources: 3 accel, 3 gyro, 3 accel bias, 3 gyro bias, 3 mag bias)
    // Mag bias is estimated by a slow LPF, not Kalman, so keep that process noise tiny.
    Rw(0,0) = SIG_W_A*SIG_W_A;          Rw(1,1) = SIG_W_A*SIG_W_A;            Rw(2,2) = SIG_W_A*SIG_W_A;
    Rw(3,3) = SIG_W_G*SIG_W_G;          Rw(4,4) = SIG_W_G*SIG_W_G;            Rw(5,5) = SIG_W_G*SIG_W_G;
    Rw(6,6) = 2.0f*SIG_A_D*SIG_A_D/TAU_A; Rw(7,7) = 2.0f*SIG_A_D*SIG_A_D/TAU_A;   Rw(8,8) = 2.0f*SIG_A_D*SIG_A_D/TAU_A;
    Rw(9,9) = 2.0f*SIG_G_D*SIG_G_D/TAU_G; Rw(10,10) = 2.0f*SIG_G_D*SIG_G_D/TAU_G; Rw(11,11) = 2.0f*SIG_G_D*SIG_G_D/TAU_G;
    Rw(12,12) = 1.0e-8f; Rw(13,13) = 1.0e-8f; Rw(14,14) = 1.0e-8f;
    // ... P
    P.setZero();
    P(0,0) = P_P_INIT*P_P_INIT;       P(1,1) = P_P_INIT*P_P_INIT;       P(2,2) = P_P_INIT*P_P_INIT;
    P(3,3) = P_V_INIT*P_V_INIT;       P(4,4) = P_V_INIT*P_V_INIT;       P(5,5) = P_V_INIT*P_V_INIT;
    P(6,6) = P_A_INIT*P_A_INIT;       P(7,7) = P_A_INIT*P_A_INIT;       P(8,8) = P_HDG_INIT*P_HDG_INIT;
    P(9,9) = P_AB_INIT*P_AB_INIT;     P(10,10) = P_AB_INIT*P_AB_INIT;   P(11,11) = P_AB_INIT*P_AB_INIT;
    P(12,12) = P_GB_INIT*P_GB_INIT;   P(13,13) = P_GB_INIT*P_GB_INIT;   P(14,14) = P_GB_INIT*P_GB_INIT;
    P(15,15) = P_MB_INIT*P_MB_INIT;   P(16,16) = P_MB_INIT*P_MB_INIT;   P(17,17) = P_MB_INIT*P_MB_INIT;
    if (!mag_ok) {
      P(8,8) = P_HDG_INIT * P_HDG_INIT;
    }
    // GPS R (6x6)
    R.setZero();
    R(0,0) = SIG_GPS_P_NE*SIG_GPS_P_NE; R(1,1) = SIG_GPS_P_NE*SIG_GPS_P_NE; R(2,2) = SIG_GPS_P_D*SIG_GPS_P_D;
    R(3,3) = SIG_GPS_V*SIG_GPS_V;       R(4,4) = SIG_GPS_V*SIG_GPS_V;       R(5,5) = SIG_GPS_V*SIG_GPS_V;

    abx = 0.0f; aby = 0.0f; abz = 0.0f;
    gbx = 0.0f; gby = 0.0f; gbz = 0.0f;
    mbx = 0.0f; mby = 0.0f; mbz = 0.0f;

    previousTOW = 0;
    gps_has_updated = false;
    have_gps_speed = false;
    last_gps_ground_speed = 0.0f;
    last_gps_fusion_time = 0.0;
    gps_delay_estimate = GPS_DELAY_NOMINAL;
    filter_time = 0.0;
    last_mag_update_time = -1.0;
    mag_filter_initialized = false;
    mag_meas_filtered.setZero();
    last_baro_update_time = -1.0;
    last_accel_tilt_time = -1.0;
    last_zupt_time = -1.0f;
    low_speed_timer = 0.0f;
    zupt_active = false;
    rest_detector.reset();

    lat_ins = lat;
    lon_ins = lon;
    alt_ins = alt;
    vn_ins = vn;
    ve_ins = ve;
    vd_ins = vd;
    f_b(0,0) = ax;
    f_b(1,0) = ay;
    f_b(2,0) = az;
    om_ib(0,0) = p;
    om_ib(1,0) = q;
    om_ib(2,0) = r;
    _t = 0;
    initialized = true;
    return;
  }

  if (dt_in <= 0.0) {
    return;
  }
  {
    // get the change in time
    _dt = (float)dt_in;
    if (_dt < 1e-4f) _dt = 1e-4f;
    if (_dt > 0.2f) _dt = 0.2f;
    
    filter_time += _dt;

    bool at_rest = rest_detector.update(p, q, r, ax, ay, az);
    
    lla_ins(0,0) = lat_ins;
    lla_ins(1,0) = lon_ins;
    lla_ins(2,0) = alt_ins;
    V_ins(0,0) = vn_ins;
    V_ins(1,0) = ve_ins;
    V_ins(2,0) = vd_ins;
    // AHRS Transformations
    C_N2B = quat2dcm(quat);
    C_B2N = C_N2B.transpose();
    // Attitude Update
    dq(0) = 1.0f;
    dq(1) = 0.5f*om_ib(0,0)*_dt;
    dq(2) = 0.5f*om_ib(1,0)*_dt;
    dq(3) = 0.5f*om_ib(2,0)*_dt;
    quat = qmult(quat,dq);
    quat.normalize();
    // Avoid quaternion flips sign
    if (quat(0) < 0) {
      quat = -1.0f*quat;
    }
    // obtain euler angles from quaternion
    theta = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
    phi = atan2f(2.0f*(quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0)),1.0f-2.0f*(quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0)));
    psi = atan2f(2.0f*(quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0)),1.0f-2.0f*(quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0)));
    // Velocity Update
    dx = C_B2N*f_b + grav;
    vn_ins += _dt*dx(0,0);
    ve_ins += _dt*dx(1,0);
    vd_ins += _dt*dx(2,0);
    // Position Update
    dxd = llarate(V_ins,lla_ins);
    lat_ins += _dt*dxd(0,0);
    lon_ins += _dt*dxd(1,0);
    alt_ins += _dt*dxd(2,0);
    // Jacobian (18x18 state matrix)
    Fs.setZero();
    // ... pos2vel
    Fs.block(0,3,3,3) = Eigen::Matrix<float,3,3>::Identity();
    // ... vel2pos (gravity gradient)
    Fs(5,2) = -2.0f*G/EARTH_RADIUS;
    // ... vel2att
    Fs.block(3,6,3,3) = -2.0f*C_B2N*sk(f_b);
    // ... vel2accel_bias
    Fs.block(3,9,3,3) = -C_B2N;
    // ... att2att
    Fs.block(6,6,3,3) = -sk(om_ib);
    // ... att2gyro_bias
    Fs.block(6,12,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();
    // ... Accel bias Markov process
    Fs.block(9,9,3,3) = -1.0f/TAU_A*Eigen::Matrix<float,3,3>::Identity();
    // During rest, bias can be estimated much more accurately
    // Reduce time constant (faster convergence) when stationary
    float tau_g_adaptive = at_rest ? 10.0f : TAU_G; // 5x faster convergence at rest (was 50s, now 10s)
    // ... Gyro bias Markov process (adaptive)
    Fs.block(12,12,3,3) = -1.0f/tau_g_adaptive*Eigen::Matrix<float,3,3>::Identity();
    // ... Mag bias Markov process (states 15-17)
    Fs.block(15,15,3,3) = -1.0f/TAU_M*Eigen::Matrix<float,3,3>::Identity();
    // State Transition Matrix (18x18)
    PHI = Eigen::Matrix<float,18,18>::Identity()+Fs*_dt;
    // Process Noise Transformation (18x15)
    Gs.setZero();
    Gs.block(3,0,3,3) = -C_B2N;                              // vel <- accel noise
    Gs.block(6,3,3,3) = -0.5f*Eigen::Matrix<float,3,3>::Identity();  // att <- gyro noise
    Gs.block(9,6,3,3) = Eigen::Matrix<float,3,3>::Identity();   // accel bias <- accel bias noise
    Gs.block(12,9,3,3) = Eigen::Matrix<float,3,3>::Identity();  // gyro bias <- gyro bias noise
    Gs.block(15,12,3,3) = Eigen::Matrix<float,3,3>::Identity(); // mag bias <- mag bias noise
    // During rest, increase gyro bias process noise to allow faster convergence
    // This combines with reduced TAU_G to significantly improve bias estimation
    Eigen::Matrix<float,15,15> Rw_adaptive = Rw;
    if (at_rest) {
      // Increase gyro bias process noise by 10x during rest
      // This allows the filter to "learn" gyro bias faster when stationary
      float rest_scale = 10.0f;
      Rw_adaptive(9,9) *= rest_scale;   // Gyro X bias process noise
      Rw_adaptive(10,10) *= rest_scale; // Gyro Y bias process noise
      Rw_adaptive(11,11) *= rest_scale; // Gyro Z bias process noise
    }
    // Discrete Process Noise (using adaptive Rw)
    Q = PHI*_dt*Gs*Rw_adaptive*Gs.transpose();
    Q = 0.5f*(Q+Q.transpose());
    // Covariance Time Update
    P = PHI*P*PHI.transpose()+Q;
    P = 0.5f*(P+P.transpose());
    // Health monitoring: check for filter divergence after covariance propagation
    checkCovarianceHealth();
    // SPEED-BASED ZUPT TRIGGER LOGIC
    // Track sustained low ground speed to enable ZUPT for stationary scenarios
    // (person holding phone while standing still, car at stoplight, etc.)
    // Trigger ZUPT from fresh GPS speed so a diverging INS cannot ZUPT itself,
    // and so a stale last-fix of 0 m/s cannot ZUPT a moving vehicle after GPS loss.
    const float GPS_SPEED_STALE_S = 2.0f;
    bool gps_speed_fresh = have_gps_speed &&
                           ((filter_time - last_gps_fusion_time) < GPS_SPEED_STALE_S);
    float zupt_speed = gps_speed_fresh
      ? last_gps_ground_speed
      : sqrtf(vn_ins * vn_ins + ve_ins * ve_ins);
    float ground_speed = zupt_speed;
    const float ZUPT_SPEED_ENTER = 1.0f;
    const float ZUPT_SPEED_EXIT = 1.5f;
    const float ZUPT_TIME_REQUIRED = 3.0f;
    // Update timer based on current ground speed
    if (ground_speed < ZUPT_SPEED_ENTER) {
      low_speed_timer += _dt;
      if (low_speed_timer >= ZUPT_TIME_REQUIRED) {
        zupt_active = true;
      }
    } else if (ground_speed > ZUPT_SPEED_EXIT) {
      low_speed_timer = 0.0f;
      zupt_active = false;
    }
    // Clamp timer to prevent unbounded growth
    if (low_speed_timer > ZUPT_TIME_REQUIRED * 2.0f) {
      low_speed_timer = ZUPT_TIME_REQUIRED * 2.0f;
    }
    bool new_gps = false;
    if (gps_has_updated) {
      new_gps = (TOW - previousTOW) > 0;
    } else {
      // First GPS: accept a non-zero TOW, or a non-origin lat/lon (X-Plane starts TOW at 0)
      new_gps = (TOW > 0) || (fabs(lat) > 1e-12 || fabs(lon) > 1e-12);
    }
    if (new_gps && std::isfinite(lat) && std::isfinite(lon) && std::isfinite(alt)) {
      double gps_interval = gps_has_updated ? ((double)(TOW - previousTOW) / 1000.0) : 1.0;
      previousTOW = TOW;
      gps_has_updated = true;
      last_gps_ground_speed = sqrtf((float)(vn * vn + ve * ve));
      have_gps_speed = true;
      if (gps_interval > 0.1 && gps_interval < 5.0) {
        float rate_based_delay = GPS_DELAY_NOMINAL;
        if (gps_interval < 0.5f) {
          rate_based_delay = GPS_DELAY_MIN + 0.05f;
        } else if (gps_interval >= 1.0f) {
          rate_based_delay = GPS_DELAY_NOMINAL + 0.05f;
        }
        const float DELAY_FILTER_ALPHA = 0.1f;
        gps_delay_estimate = (1.0f - DELAY_FILTER_ALPHA) * gps_delay_estimate +
                             DELAY_FILTER_ALPHA * rate_based_delay;
        if (gps_delay_estimate < GPS_DELAY_MIN) gps_delay_estimate = GPS_DELAY_MIN;
        if (gps_delay_estimate > GPS_DELAY_MAX) gps_delay_estimate = GPS_DELAY_MAX;
      }
      float delay_scale = computeDelayScaleFactor(gps_delay_estimate);
      last_gps_fusion_time = filter_time;
      lla_gps(0,0) = lat;
      lla_gps(1,0) = lon;
      lla_gps(2,0) = alt;
      V_gps(0,0) = vn;
      V_gps(1,0) = ve;
      V_gps(2,0) = vd;
      lla_ins(0,0) = lat_ins;
      lla_ins(1,0) = lon_ins;
      lla_ins(2,0) = alt_ins;
      V_ins(0,0) = vn_ins;
      V_ins(1,0) = ve_ins;
      V_ins(2,0) = vd_ins;
      pos_ecef_ins = lla2ecef(lla_ins);
      pos_ned_ins = ecef2ned(pos_ecef_ins,lla_ins);
      pos_ecef_gps = lla2ecef(lla_gps);
      pos_ned_gps = ecef2ned(pos_ecef_gps,lla_ins);
      y(0,0) = (float)(pos_ned_gps(0,0) - pos_ned_ins(0,0));
      y(1,0) = (float)(pos_ned_gps(1,0) - pos_ned_ins(1,0));
      y(2,0) = (float)(pos_ned_gps(2,0) - pos_ned_ins(2,0));
      y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
      y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
      y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
      H.setZero();
      H.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();
      Eigen::Matrix<float,6,6> R_scaled = R;
      float delay_scale_sq = delay_scale * delay_scale;
      bool has_valid_hacc = (hacc > 0.0f && std::isfinite(hacc));
      bool has_valid_vacc = (vacc > 0.0f && std::isfinite(vacc));
      bool has_valid_sacc = (sacc > 0.0f && std::isfinite(sacc));
      float gps_pos_noise_ne = SIG_GPS_P_NE;
      if (has_valid_hacc) {
        gps_pos_noise_ne = fmaxf(SIG_GPS_P_NE, hacc);
      }
      float gps_pos_noise_d = SIG_GPS_P_D;
      if (has_valid_vacc) {
        gps_pos_noise_d = fmaxf(SIG_GPS_P_D, vacc);
      }
      float gps_vel_noise = SIG_GPS_V;
      if (has_valid_sacc) {
        gps_vel_noise = fmaxf(SIG_GPS_V, sacc);
      }
      R_scaled(0,0) = gps_pos_noise_ne * gps_pos_noise_ne;
      R_scaled(1,1) = gps_pos_noise_ne * gps_pos_noise_ne;
      R_scaled(2,2) = gps_pos_noise_d * gps_pos_noise_d;
      R_scaled(3,3) = gps_vel_noise * gps_vel_noise;
      R_scaled(4,4) = gps_vel_noise * gps_vel_noise;
      R_scaled(5,5) = gps_vel_noise * gps_vel_noise;
      R_scaled *= delay_scale_sq;
      Eigen::Matrix<float,6,6> S_gps = H * P * H.transpose() + R_scaled;
      // Gross-outlier gate only. A tight 3-sigma gate rejects usable GPS when
      // the IMU model is slightly wrong (typical on a phone).
      bool gps_innovation_ok =
          (fabsf(y(0, 0)) < 100.0f) && (fabsf(y(1, 0)) < 100.0f) &&
          (fabsf(y(2, 0)) < 80.0f) && (fabsf(y(3, 0)) < 80.0f) &&
          (fabsf(y(4, 0)) < 80.0f) && (fabsf(y(5, 0)) < 80.0f);
      if (gps_innovation_ok) {
        K = P * H.transpose() * S_gps.inverse();
        Eigen::Matrix<float,18,18> I_KH = Eigen::Matrix<float,18,18>::Identity() - K * H;
        P = I_KH * P * I_KH.transpose() + K * R_scaled * K.transpose();
        P = 0.5f * (P + P.transpose());
        x = K * y;
        applyNavigationCorrection(x);
      }
    }
    // ZUPT MEASUREMENT UPDATE
    // Apply zero-velocity update when stationary (speed-based trigger)
    // Rate limit: 5 Hz (every 0.2 seconds)
    if (zupt_active && (filter_time - last_zupt_time) >= 0.2f) {
      last_zupt_time = filter_time;
      // Safety gate: only apply if current 3D velocity is reasonable
      // (prevents ZUPT during actual motion with stale GPS or filter divergence)
      float speed_3d = sqrtf(vn_ins*vn_ins + ve_ins*ve_ins + vd_ins*vd_ins);
      // When GPS confirms a stop, ZUPT must still run — that is how a
      // diverged INS recovers. The 3 m/s cap is only for INS-speed ZUPT
      // after GPS has gone stale.
      float zupt_speed_gate = gps_speed_fresh ? 80.0f : 3.0f;
      if (speed_3d < zupt_speed_gate) {
        // ZUPT: observe velocity = [0, 0, 0] in NED frame
        Eigen::Matrix<float,3,1> v_obs;
        v_obs << 0.0f, 0.0f, 0.0f;
        Eigen::Matrix<float,3,1> v_ins;
        v_ins << vn_ins, ve_ins, vd_ins;
        // Innovation (measurement - prediction)
        Eigen::Matrix<float,3,1> y_zupt = v_obs - v_ins;
        // Measurement matrix: observes velocity states (indices 3-5)
        Eigen::Matrix<float,3,18> H_zupt;
        H_zupt.setZero();
        H_zupt(0,3) = 1.0f;  // vn
        H_zupt(1,4) = 1.0f;  // ve
        H_zupt(2,5) = 1.0f;  // vd
        // Measurement noise: 0.1 m/s std per axis
        // Optionally tighter (0.05 m/s) if variance-based rest is also true
        float zupt_noise_std = 0.1f;  // m/s
        if (at_rest) {
          zupt_noise_std = 0.05f;  // tighter when device is truly still (on desk)
        }
        Eigen::Matrix<float,3,3> R_zupt;
        R_zupt.setZero();
        R_zupt(0,0) = zupt_noise_std * zupt_noise_std;
        R_zupt(1,1) = zupt_noise_std * zupt_noise_std;
        R_zupt(2,2) = zupt_noise_std * zupt_noise_std;
        // Kalman gain K = P * H^T * (H * P * H^T + R)^-1
        Eigen::Matrix<float,3,3> S_zupt = H_zupt * P * H_zupt.transpose() + R_zupt;
        Eigen::Matrix<float,18,3> K_zupt = P * H_zupt.transpose() * S_zupt.inverse();
        // State correction
        Eigen::Matrix<float,18,1> x_zupt = K_zupt * y_zupt;
        applyNavigationCorrection(x_zupt);
        Eigen::Matrix<float,18,18> I_KH_zupt =
          Eigen::Matrix<float,18,18>::Identity() - K_zupt * H_zupt;
        P = I_KH_zupt * P * I_KH_zupt.transpose() +
            K_zupt * R_zupt * K_zupt.transpose();
        P = 0.5f * (P + P.transpose());
      }
    }

    // Magnetometer: tilt-compensated heading at up to 10 Hz of filter time
    const float MAG_UPDATE_PERIOD_S = 0.1f;
    bool mag_update_ready = (last_mag_update_time < 0.0) ||
                            ((filter_time - last_mag_update_time) >= MAG_UPDATE_PERIOD_S);
    bool mag_field_valid = std::isfinite(bn) && std::isfinite(be) && std::isfinite(bd);
    bool mag_meas_valid = std::isfinite(hx) && std::isfinite(hy) && std::isfinite(hz);

    if (mag_update_ready && mag_field_valid && mag_meas_valid) {
      last_mag_update_time = filter_time;
      C_N2B = quat2dcm(quat);
      C_B2N = C_N2B.transpose();
      float hx_meas = hx * 1000.0f;
      float hy_meas = hy * 1000.0f;
      float hz_meas = hz * 1000.0f;
      Eigen::Matrix<float,3,1> mag_ned_exp;
      mag_ned_exp << bn, be, bd;
      mag_expected_body = C_N2B * mag_ned_exp;

      float measured_field = sqrtf(hx_meas*hx_meas + hy_meas*hy_meas + hz_meas*hz_meas);
      float expected_field = mag_ned_exp.norm();
      float field_ratio = (expected_field > 0.0f) ? (measured_field / expected_field) : 0.0f;
      bool mag_magnitude_ok = (field_ratio > 0.5f && field_ratio < 2.0f);

      // Attitude gate: horizontal mag is only a heading measurement near level
      const float MAG_ATT_LIMIT = 0.52f; // ~30 deg
      bool mag_attitude_ok = (fabsf(phi) < MAG_ATT_LIMIT && fabsf(theta) < MAG_ATT_LIMIT);

      // Inclination in NED for both measured and expected
      Eigen::Matrix<float,3,1> mag_body_corr;
      mag_body_corr << (hx_meas - mbx), (hy_meas - mby), (hz_meas - mbz);
      Eigen::Matrix<float,3,1> mag_ned_meas = C_B2N * mag_body_corr;
      float Bh_meas = sqrtf(mag_ned_meas(0)*mag_ned_meas(0) + mag_ned_meas(1)*mag_ned_meas(1));
      float Bh_exp = sqrtf(bn*bn + be*be);
      float inclination_meas = atan2f(mag_ned_meas(2), fmaxf(Bh_meas, 1.0f));
      float inclination_exp = atan2f(bd, fmaxf(Bh_exp, 1.0f));
      float inclination_error = fabsf(constrainAngle180(inclination_meas - inclination_exp));
      bool mag_inclination_ok = (inclination_error < 0.35f);

      Eigen::Matrix<float,3,1> mag_meas_current;
      mag_meas_current << hx_meas, hy_meas, hz_meas;
      if (!mag_filter_initialized) {
        mag_meas_filtered = mag_meas_current;
        mag_filter_initialized = true;
      }
      Eigen::Matrix<float,3,1> mag_innovation = mag_meas_current - mag_meas_filtered;
      float innovation_magnitude = mag_innovation.norm();
      const float MAG_LPF_ALPHA = 0.1f;
      mag_meas_filtered = MAG_LPF_ALPHA * mag_meas_current + (1.0f - MAG_LPF_ALPHA) * mag_meas_filtered;
      bool mag_temporally_stable = (innovation_magnitude < 10000.0f);

      bool mag_reasonable = mag_magnitude_ok && mag_attitude_ok && mag_inclination_ok && mag_temporally_stable;
      if (mag_reasonable) {
        float hx_c = hx_meas - mbx;
        float hy_c = hy_meas - mby;
        float hz_c = hz_meas - mbz;
        Bxc = hx_c * cosf(theta) + (hy_c * sinf(phi) + hz_c * cosf(phi)) * sinf(theta);
        Byc = hy_c * cosf(phi) - hz_c * sinf(phi);
        float mag_heading = constrainAngle180(atan2f(-Byc, Bxc));
        float yaw_error = constrainAngle180(mag_heading - psi);
        float sigma_yaw = (Bh_exp > 1.0f) ? (SIG_MAG / Bh_exp) : 0.0f;
        float innov_gate = 3.0f * sigma_yaw;
        if (sigma_yaw > 0.0f && fabsf(yaw_error) < innov_gate) {
          Eigen::Matrix<float,1,18> H_mag;
          H_mag.setZero();
          H_mag(0,8) = 1.0f;
          float R_mag = sigma_yaw * sigma_yaw;
          float S_mag = (H_mag * P * H_mag.transpose())(0,0) + R_mag;
          Eigen::Matrix<float,18,1> K_mag = (P * H_mag.transpose()) / S_mag;
          Eigen::Matrix<float,18,18> I_KH_mag = Eigen::Matrix<float,18,18>::Identity() - K_mag * H_mag;
          P = I_KH_mag * P * I_KH_mag.transpose() + K_mag * R_mag * K_mag.transpose();
          P = 0.5f*(P + P.transpose());
          x = K_mag * yaw_error;
          applyNavigationCorrection(x);
          const float MAG_BIAS_ALPHA = 0.01f;
          mbx += MAG_BIAS_ALPHA * (hx_meas - mag_expected_body(0) - mbx);
          mby += MAG_BIAS_ALPHA * (hy_meas - mag_expected_body(1) - mby);
        }
      }
    }

    // Weak accelerometer roll/pitch aid when specific force is ~1 g.
    // Coordinated turns measure nG along body z, so they are excluded.
    const float ACCEL_TILT_PERIOD_S = 0.1f;
    bool tilt_rate_ok = (last_accel_tilt_time < 0.0) ||
                        ((filter_time - last_accel_tilt_time) >= ACCEL_TILT_PERIOD_S);
    float accel_norm_tilt = sqrtf(ax * ax + ay * ay + az * az);
    bool near_1g = (fabsf(accel_norm_tilt - G) < 0.8f);
    bool gyro_slow = (fabsf(p) < 0.15f && fabsf(q) < 0.15f && fabsf(r) < 0.15f);
    if (tilt_rate_ok && near_1g && gyro_slow && fabsf(ax) <= G) {
      last_accel_tilt_time = filter_time;
      float theta_acc = asinf(ax / G);
      float cos_theta_acc = cosf(theta_acc);
      if (fabsf(cos_theta_acc) > 1e-3f) {
        float ay_term = -ay / (G * cos_theta_acc);
        if (fabsf(ay_term) <= 1.0f) {
          float phi_acc = asinf(ay_term);
          float roll_err = constrainAngle180(phi_acc - phi);
          float pitch_err = constrainAngle180(theta_acc - theta);
          const float SIG_TILT = 0.14f; // ~8 deg
          float tilt_gate = 3.0f * SIG_TILT;
          if (fabsf(roll_err) < tilt_gate && fabsf(pitch_err) < tilt_gate) {
            Eigen::Matrix<float,2,18> H_tilt;
            H_tilt.setZero();
            H_tilt(0, 6) = 1.0f;
            H_tilt(1, 7) = 1.0f;
            Eigen::Matrix<float,2,1> y_tilt;
            y_tilt(0, 0) = roll_err;
            y_tilt(1, 0) = pitch_err;
            Eigen::Matrix<float,2,2> R_tilt;
            R_tilt.setZero();
            R_tilt(0, 0) = SIG_TILT * SIG_TILT;
            R_tilt(1, 1) = SIG_TILT * SIG_TILT;
            Eigen::Matrix<float,2,2> S_tilt = H_tilt * P * H_tilt.transpose() + R_tilt;
            Eigen::Matrix<float,18,2> K_tilt = P * H_tilt.transpose() * S_tilt.inverse();
            Eigen::Matrix<float,18,18> I_KH_tilt =
                Eigen::Matrix<float,18,18>::Identity() - K_tilt * H_tilt;
            P = I_KH_tilt * P * I_KH_tilt.transpose() +
                K_tilt * R_tilt * K_tilt.transpose();
            P = 0.5f * (P + P.transpose());
            applyNavigationCorrection(K_tilt * y_tilt);
          }
        }
      }
    }

    bool has_valid_baro = (baro_pressure > 0.0f && std::isfinite(baro_pressure) &&
                           baro_pressure > 500.0f && baro_pressure < 1100.0f);
    bool vacc_available = (vacc > 0.0f && std::isfinite(vacc));
    bool gps_vert_poor = (!vacc_available || vacc > 10.0f);
    bool baro_rate_ok = (last_baro_update_time < 0.0) ||
                        ((filter_time - last_baro_update_time) >= 0.2);
    if (has_valid_baro && gps_vert_poor && baro_rate_ok) {
      last_baro_update_time = filter_time;
      float baro_alt_m;
      if (baro_qnh > 0.0f && std::isfinite(baro_qnh)) {
        baro_alt_m = AltitudeCalculator::calculateQNH_m(baro_pressure, baro_qnh);
      } else {
        baro_alt_m = AltitudeCalculator::calculateQNE_m(baro_pressure);
      }
      if (std::isfinite(baro_alt_m)) {
        float alt_innovation = baro_alt_m - (float)alt_ins;
        const float BARO_NOISE_BASE = 2.0f;
        float alt_scale = 1.0f + fabsf((float)alt_ins) / 10000.0f;
        float baro_noise = BARO_NOISE_BASE * alt_scale;
        float baro_innov_gate = 3.0f * baro_noise;
        if (fabsf(alt_innovation) < baro_innov_gate) {
          Eigen::Matrix<float,1,18> H_baro;
          H_baro.setZero();
          H_baro(0,2) = -1.0f;
          float R_baro = baro_noise * baro_noise;
          float S_baro = (H_baro * P * H_baro.transpose())(0,0) + R_baro;
          Eigen::Matrix<float,18,1> K_baro = (P * H_baro.transpose()) / S_baro;
          Eigen::Matrix<float,18,18> I_KH_baro = Eigen::Matrix<float,18,18>::Identity() - K_baro * H_baro;
          P = I_KH_baro * P * I_KH_baro.transpose() + K_baro * R_baro * K_baro.transpose();
          P = 0.5f*(P + P.transpose());
          x = K_baro * alt_innovation;
          alt_ins = alt_ins - x(2,0);
        }
      }
    }
    // Get the new Specific forces and Rotation Rate,
    // use in the next time update
    f_b(0,0) = ax - abx;
    f_b(1,0) = ay - aby;
    f_b(2,0) = az - abz;

    om_ib(0,0) = p - gbx;
    om_ib(1,0) = q - gby;
    om_ib(2,0) = r - gbz;
  }
}

// returns the pitch angle, rad
float uNavINS::getPitch_rad() {
  return theta;
}

// returns the roll angle, rad
float uNavINS::getRoll_rad() {
  return phi;
}

// returns the yaw angle, rad
float uNavINS::getYaw_rad() {
  return constrainAngle180(psi-psi_initial);
}

// returns the heading angle, rad
float uNavINS::getHeading_rad() {
  return constrainAngle360(psi);
}

// returns the INS latitude, rad
double uNavINS::getLatitude_rad() {
  return lat_ins;
}

// returns the INS longitude, rad
double uNavINS::getLongitude_rad() {
  return lon_ins;
}

// returns the INS altitude, m
double uNavINS::getAltitude_m() {
  return alt_ins;
}

// returns the INS north velocity, m/s
double uNavINS::getVelNorth_ms() {
  return vn_ins;
}

// returns the INS east velocity, m/s
double uNavINS::getVelEast_ms() {
  return ve_ins;
}

// returns the INS down velocity, m/s
double uNavINS::getVelDown_ms() {
  return vd_ins;
}

// returns the INS ground track, rad
float uNavINS::getGroundTrack_rad() {
  return atan2f((float)ve_ins,(float)vn_ins);
}

// returns the INS flight-path angle, rad (VERTICAL angle of velocity vector relative to horizontal)
// Positive = climbing, negative = descending
float uNavINS::getFlightPathAngle_rad() {
  float horizontal_speed = sqrtf((float)(vn_ins*vn_ins + ve_ins*ve_ins));
  if (horizontal_speed < 0.1f) {
    return 0.0f; // Avoid division by zero when stationary
  }
  return atan2f(-(float)vd_ins, horizontal_speed);
}

// returns the INS horizontal flight-path angle, rad (sideslip/crab angle in horizontal plane)
// Calculated from velocity components in body frame
// Positive = drifting right, negative = drifting left
float uNavINS::getHorizontalFlightPathAngle_rad() {
  // Transform NED velocity to body frame using DCM
  Eigen::Matrix<float,3,1> v_ned;
  v_ned << (float)vn_ins, (float)ve_ins, (float)vd_ins;
  Eigen::Matrix<float,3,1> v_body = C_N2B * v_ned;
  // Horizontal flight path angle = atan2(vy_body, vx_body)
  // where vx_body is forward velocity and vy_body is rightward velocity
  // This gives the angle between body forward axis and velocity vector in horizontal plane
  float vx_body = v_body(0); // Forward
  float vy_body = v_body(1); // Right
  float horizontal_speed_body = sqrtf(vx_body*vx_body + vy_body*vy_body);
  if (horizontal_speed_body < 0.1f) {
    return 0.0f; // Avoid division by zero when stationary
  }
  return atan2f(vy_body, vx_body);
}

// returns the gyro bias estimate in the x direction, rad/s
float uNavINS::getGyroBiasX_rads() {
  return gbx;
}

// returns the gyro bias estimate in the y direction, rad/s
float uNavINS::getGyroBiasY_rads() {
  return gby;
}

// returns the gyro bias estimate in the z direction, rad/s
float uNavINS::getGyroBiasZ_rads() {
  return gbz;
}

// returns the accel bias estimate in the x direction, m/s/s
float uNavINS::getAccelBiasX_mss() {
  return abx;
}

// returns the accel bias estimate in the y direction, m/s/s
float uNavINS::getAccelBiasY_mss() {
  return aby;
}

// returns the accel bias estimate in the z direction, m/s/s
float uNavINS::getAccelBiasZ_mss() {
  return abz;
}

// returns the mag bias estimate in the x direction (body frame), nT
float uNavINS::getMagBiasX_nT() {
  return mbx;
}

// returns the mag bias estimate in the y direction (body frame), nT
float uNavINS::getMagBiasY_nT() {
  return mby;
}

// returns the mag bias estimate in the z direction (body frame), nT
float uNavINS::getMagBiasZ_nT() {
  return mbz;
}

// returns the position North uncertainty (1-sigma standard deviation), m
float uNavINS::getPositionNorthStd_m() {
  return sqrtf(P(0,0));
}

// returns the position East uncertainty (1-sigma standard deviation), m
float uNavINS::getPositionEastStd_m() {
  return sqrtf(P(1,1));
}

// returns the position Down uncertainty (1-sigma standard deviation), m
float uNavINS::getPositionDownStd_m() {
  return sqrtf(P(2,2));
}

// returns the velocity North uncertainty (1-sigma standard deviation), m/s
float uNavINS::getVelocityNorthStd_ms() {
  return sqrtf(P(3,3));
}

// returns the velocity East uncertainty (1-sigma standard deviation), m/s
float uNavINS::getVelocityEastStd_ms() {
  return sqrtf(P(4,4));
}

// returns the velocity Down uncertainty (1-sigma standard deviation), m/s
float uNavINS::getVelocityDownStd_ms() {
  return sqrtf(P(5,5));
}

// returns the roll angle uncertainty (1-sigma standard deviation), rad
float uNavINS::getRollStd_rad() {
  return sqrtf(P(6,6));
}

// returns the pitch angle uncertainty (1-sigma standard deviation), rad
float uNavINS::getPitchStd_rad() {
  return sqrtf(P(7,7));
}

// returns the yaw angle uncertainty (1-sigma standard deviation), rad
float uNavINS::getYawStd_rad() {
  return sqrtf(P(8,8));
}

// This function gives a skew symmetric matrix from a given vector w
Eigen::Matrix<float,3,3> uNavINS::sk(Eigen::Matrix<float,3,1> w) {
  Eigen::Matrix<float,3,3> C;
  C(0,0) = 0.0f;    C(0,1) = -w(2,0); C(0,2) = w(1,0);
  C(1,0) = w(2,0);  C(1,1) = 0.0f;    C(1,2) = -w(0,0);
  C(2,0) = -w(1,0); C(2,1) = w(0,0);  C(2,2) = 0.0f;
  return C;
}

// This function calculates the rate of change of latitude, longitude, and altitude.
Eigen::Matrix<double,3,1> uNavINS::llarate(Eigen::Matrix<double,3,1> V,Eigen::Matrix<double,3,1> lla) {
  double Rew, Rns, denom;
  Eigen::Matrix<double,3,1> lla_dot;
  denom = (1.0 - (ECC2 * pow(sin(lla(0,0)),2.0)));
  denom = sqrt(denom*denom);
  Rew = EARTH_RADIUS / sqrt(denom);
  Rns = EARTH_RADIUS*(1.0-ECC2) / denom*sqrt(denom);
  lla_dot(0,0) = V(0,0)/(Rns + lla(2,0));
  lla_dot(1,0) = V(1,0)/((Rew + lla(2,0))*cos(lla(0,0)));
  lla_dot(2,0) = -V(2,0);
  return lla_dot;
}

// This function calculates the ECEF Coordinate given the Latitude, Longitude and Altitude.
Eigen::Matrix<double,3,1> uNavINS::lla2ecef(Eigen::Matrix<double,3,1> lla) {
  double Rew, denom;
  Eigen::Matrix<double,3,1> ecef;
  denom = (1.0 - (ECC2 * pow(sin(lla(0,0)),2.0)));
  denom = sqrt(denom*denom);
  Rew = EARTH_RADIUS / sqrt(denom);
  ecef(0,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * cos(lla(1,0));
  ecef(1,0) = (Rew + lla(2,0)) * cos(lla(0,0)) * sin(lla(1,0));
  ecef(2,0) = (Rew * (1.0 - ECC2) + lla(2,0)) * sin(lla(0,0));
  return ecef;
}

// This function converts a vector in ecef to ned coordinate centered at pos_ref.
Eigen::Matrix<double,3,1> uNavINS::ecef2ned(Eigen::Matrix<double,3,1> ecef,Eigen::Matrix<double,3,1> pos_ref) {
  Eigen::Matrix<double,3,1> ned;
  ned(2,0)=-cos(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-cos(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)-sin(pos_ref(0,0))*ecef(2,0);
  ned(1,0)=-sin(pos_ref(1,0))*ecef(0,0) + cos(pos_ref(1,0))*ecef(1,0);
  ned(0,0)=-sin(pos_ref(0,0))*cos(pos_ref(1,0))*ecef(0,0)-sin(pos_ref(0,0))*sin(pos_ref(1,0))*ecef(1,0)+cos(pos_ref(0,0))*ecef(2,0);
  return ned;
}

// quaternion to dcm
Eigen::Matrix<float,3,3> uNavINS::quat2dcm(Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,3,3> C_N2B;
  C_N2B(0,0) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(1,0),2.0f);
  C_N2B(1,1) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(2,0),2.0f);
  C_N2B(2,2) = 2.0f*powf(q(0,0),2.0f)-1.0f + 2.0f*powf(q(3,0),2.0f);
  C_N2B(0,1) = 2.0f*q(1,0)*q(2,0) + 2.0f*q(0,0)*q(3,0);
  C_N2B(0,2) = 2.0f*q(1,0)*q(3,0) - 2.0f*q(0,0)*q(2,0);
  C_N2B(1,0) = 2.0f*q(1,0)*q(2,0) - 2.0f*q(0,0)*q(3,0);
  C_N2B(1,2) = 2.0f*q(2,0)*q(3,0) + 2.0f*q(0,0)*q(1,0);
  C_N2B(2,0) = 2.0f*q(1,0)*q(3,0) + 2.0f*q(0,0)*q(2,0);
  C_N2B(2,1) = 2.0f*q(2,0)*q(3,0) - 2.0f*q(0,0)*q(1,0);
  return C_N2B;
}

// quaternion multiplication
Eigen::Matrix<float,4,1> uNavINS::qmult(Eigen::Matrix<float,4,1> p, Eigen::Matrix<float,4,1> q) {
  Eigen::Matrix<float,4,1> r;
  r(0,0) = p(0,0)*q(0,0) - (p(1,0)*q(1,0) + p(2,0)*q(2,0) + p(3,0)*q(3,0));
  r(1,0) = p(0,0)*q(1,0) + q(0,0)*p(1,0) + p(2,0)*q(3,0) - p(3,0)*q(2,0);
  r(2,0) = p(0,0)*q(2,0) + q(0,0)*p(2,0) + p(3,0)*q(1,0) - p(1,0)*q(3,0);
  r(3,0) = p(0,0)*q(3,0) + q(0,0)*p(3,0) + p(1,0)*q(2,0) - p(2,0)*q(1,0);
  return r;
}

// bound yaw angle between -180 and 180
float uNavINS::constrainAngle180(float dta) {
  while (dta >  M_PI) dta -= (M_PI * 2.0f);
  while (dta < -M_PI) dta += (M_PI * 2.0f);
  return dta;
}

// bound heading angle between 0 and 360
float uNavINS::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}

// Apply error-state correction to navigation states. Mag bias is LPF-only.
void uNavINS::applyNavigationCorrection(const Eigen::Matrix<float,18,1>& dx) {
  denom = (1.0 - (ECC2 * pow(sin(lat_ins), 2.0)));
  denom = sqrt(denom * denom);
  Re = EARTH_RADIUS / sqrt(denom);
  Rn = EARTH_RADIUS * (1.0 - ECC2) / denom * sqrt(denom);
  alt_ins = alt_ins - dx(2, 0);
  lat_ins = lat_ins + dx(0, 0) / (Re + alt_ins);
  lon_ins = lon_ins + dx(1, 0) / (Rn + alt_ins) / cos(lat_ins);
  vn_ins = vn_ins + dx(3, 0);
  ve_ins = ve_ins + dx(4, 0);
  vd_ins = vd_ins + dx(5, 0);
  dq(0, 0) = 1.0f;
  dq(1, 0) = dx(6, 0);
  dq(2, 0) = dx(7, 0);
  dq(3, 0) = dx(8, 0);
  quat = qmult(quat, dq);
  quat.normalize();
  if (quat(0) < 0) {
    quat = -1.0f * quat;
  }
  float sin_theta = -2.0f * (quat(1, 0) * quat(3, 0) - quat(0, 0) * quat(2, 0));
  if (sin_theta > 1.0f) sin_theta = 1.0f;
  if (sin_theta < -1.0f) sin_theta = -1.0f;
  theta = asinf(sin_theta);
  phi = atan2f(2.0f * (quat(0, 0) * quat(1, 0) + quat(2, 0) * quat(3, 0)),
               1.0f - 2.0f * (quat(1, 0) * quat(1, 0) + quat(2, 0) * quat(2, 0)));
  psi = atan2f(2.0f * (quat(1, 0) * quat(2, 0) + quat(0, 0) * quat(3, 0)),
               1.0f - 2.0f * (quat(2, 0) * quat(2, 0) + quat(3, 0) * quat(3, 0)));
  abx = abx + dx(9, 0);
  aby = aby + dx(10, 0);
  abz = abz + dx(11, 0);
  gbx = gbx + dx(12, 0);
  gby = gby + dx(13, 0);
  gbz = gbz + dx(14, 0);
}

// =============================================================================
// GPS interval noise scale
// =============================================================================

float uNavINS::computeDelayScaleFactor(float delay_seconds) {
  return 1.0f + sqrtf(delay_seconds / 0.2f);
}

// =============================================================================
// REST DETECTION (sliding window)
// =============================================================================

bool uNavINS::RestDetector::update(float gx, float gy, float gz, float ax, float ay, float az) {
  gx_buf[head] = gx;
  gy_buf[head] = gy;
  gz_buf[head] = gz;
  ax_buf[head] = ax;
  ay_buf[head] = ay;
  az_buf[head] = az;
  head = (head + 1) % WINDOW_SIZE;
  if (count < WINDOW_SIZE) {
    count++;
  }

  const int MIN_SAMPLES = 30;
  if (count < MIN_SAMPLES) {
    is_at_rest = false;
    return false;
  }

  float gx_mean = 0, gy_mean = 0, gz_mean = 0;
  float ax_mean = 0, ay_mean = 0, az_mean = 0;
  for (int i = 0; i < count; i++) {
    gx_mean += gx_buf[i];
    gy_mean += gy_buf[i];
    gz_mean += gz_buf[i];
    ax_mean += ax_buf[i];
    ay_mean += ay_buf[i];
    az_mean += az_buf[i];
  }
  float inv = 1.0f / (float)count;
  gx_mean *= inv; gy_mean *= inv; gz_mean *= inv;
  ax_mean *= inv; ay_mean *= inv; az_mean *= inv;

  float gx_var = 0, gy_var = 0, gz_var = 0;
  float ax_var = 0, ay_var = 0, az_var = 0;
  for (int i = 0; i < count; i++) {
    float dgx = gx_buf[i] - gx_mean;
    float dgy = gy_buf[i] - gy_mean;
    float dgz = gz_buf[i] - gz_mean;
    float dax = ax_buf[i] - ax_mean;
    float day = ay_buf[i] - ay_mean;
    float daz = az_buf[i] - az_mean;
    gx_var += dgx * dgx;
    gy_var += dgy * dgy;
    gz_var += dgz * dgz;
    ax_var += dax * dax;
    ay_var += day * day;
    az_var += daz * daz;
  }
  float inv_n1 = 1.0f / (float)(count - 1);
  gx_var *= inv_n1; gy_var *= inv_n1; gz_var *= inv_n1;
  ax_var *= inv_n1; ay_var *= inv_n1; az_var *= inv_n1;

  const float GYRO_THRESHOLD_SQ = 0.0004f;  // (0.02 rad/s)^2
  const float ACCEL_THRESHOLD_SQ = 0.25f;   // (0.5 m/s^2)^2
  bool gyro_still = (gx_var < GYRO_THRESHOLD_SQ) && (gy_var < GYRO_THRESHOLD_SQ) && (gz_var < GYRO_THRESHOLD_SQ);
  bool accel_still = (ax_var < ACCEL_THRESHOLD_SQ) && (ay_var < ACCEL_THRESHOLD_SQ) && (az_var < ACCEL_THRESHOLD_SQ);
  float gyro_mean_sq = gx_mean * gx_mean + gy_mean * gy_mean + gz_mean * gz_mean;
  const float GYRO_MEAN_SQ_LIMIT = 0.0025f; // (0.05 rad/s)^2 ≈ 3 deg/s
  bool gyro_quiet = (gyro_mean_sq < GYRO_MEAN_SQ_LIMIT);
  bool currently_still = gyro_still && accel_still && gyro_quiet;

  const int REST_SAMPLES_REQUIRED = 60;
  const int MOVING_SAMPLES_EXIT = 10;
  if (currently_still) {
    motion_samples = 0;
    rest_samples++;
    if (rest_samples > REST_SAMPLES_REQUIRED * 2) {
      rest_samples = REST_SAMPLES_REQUIRED * 2;
    }
    if (rest_samples >= REST_SAMPLES_REQUIRED) {
      is_at_rest = true;
    }
  } else {
    rest_samples = 0;
    motion_samples++;
    if (motion_samples > MOVING_SAMPLES_EXIT * 2) {
      motion_samples = MOVING_SAMPLES_EXIT * 2;
    }
    if (motion_samples >= MOVING_SAMPLES_EXIT) {
      is_at_rest = false;
    }
  }
  return is_at_rest;
}

// =============================================================================
// COVARIANCE HEALTH MONITORING IMPLEMENTATION
// =============================================================================

// Check filter health and detect divergence
bool uNavINS::checkCovarianceHealth() {
  if (!initialized) {
    filter_health_status = 0;
    health_pending_code = 0;
    health_pending_count = 0;
    return true;
  }
  // ==========================================================================
  // Health Check 1: NaN/Inf Detection (CRITICAL)
  // ==========================================================================
  // Check if covariance matrix contains invalid values
  if (!P.allFinite()) {
    int prev = filter_health_status;
    filter_health_status = 3;  // Critical - NaN/Inf detected
    std::printf("[AHRS health] %d -> 3 (CRITICAL) NaN/Inf in covariance P\n", prev);
    return false;
  }
  // Check if state values are finite
  if (!std::isfinite(lat_ins) || !std::isfinite(lon_ins) || !std::isfinite(alt_ins) ||
      !std::isfinite(vn_ins) || !std::isfinite(ve_ins) || !std::isfinite(vd_ins) ||
      !std::isfinite(phi) || !std::isfinite(theta) || !std::isfinite(psi)) {
    int prev = filter_health_status;
    filter_health_status = 3;  // Critical - NaN/Inf in state
    std::printf("[AHRS health] %d -> 3 (CRITICAL) NaN/Inf in state lat=%.6f lon=%.6f alt=%.1f vn=%.2f ve=%.2f vd=%.2f phi=%.2f theta=%.2f psi=%.2f\n",
                prev, lat_ins, lon_ins, alt_ins, vn_ins, ve_ins, vd_ins, phi, theta, psi);
    return false;
  }
  // ==========================================================================
  // Health Check 2: Covariance Bounds (ERROR)
  // ==========================================================================
  // Check if covariance diagonal elements exceed reasonable limits
  // Position variance limits (meters²) - relaxed WARNING to reduce boundary chatter (logs showed pos_std ~10m)
  float max_pos_var = P.block<3,3>(0,0).diagonal().maxCoeff();
  const float POS_VAR_WARNING = 200.0f;   // ~14m std dev (was 100)
  const float POS_VAR_ERROR = 1000.0f;    // 31m std dev
  const float POS_VAR_CRITICAL = 10000.0f; // 100m std dev
  // Velocity variance limits (m²/s²) - relaxed WARNING so vel_std ~3 m/s (between GPS updates) stays HEALTHY
  float max_vel_var = P.block<3,3>(3,3).diagonal().maxCoeff();
  const float VEL_VAR_WARNING = 25.0f;     // 5 m/s std dev (was 9 = 3 m/s; logs showed 3.02 m/s at boundary)
  const float VEL_VAR_ERROR = 100.0f;     // 10 m/s std dev
  const float VEL_VAR_CRITICAL = 400.0f;  // 20 m/s std dev
  // Attitude variance limits (rad²)
  // Note: Roll and pitch should be well-bounded by accelerometer
  // Yaw can be large (up to 180°) without magnetometer updates
  float roll_var = P(6,6);
  float pitch_var = P(7,7);
  float yaw_var = P(8,8);
  // Check roll/pitch separately (should be tight)
  // Note: With 1 Hz GPS and stationary, RP std of 18-20° is normal; relaxed WARNING threshold
  float max_rp_var = fmaxf(roll_var, pitch_var);
  const float RP_VAR_WARNING = 0.27f;      // ~30 deg std dev (was 0.1 = 18 deg; normal for 1Hz GPS stationary)
  const float RP_VAR_ERROR = 0.5f;         // ~40 deg std dev
  const float RP_VAR_CRITICAL = 1.0f;      // ~57 deg std dev
  // Yaw can be larger (especially without mag)
  const float YAW_VAR_WARNING = 3.14f;     // ~100 deg std dev
  const float YAW_VAR_ERROR = 12.0f;       // ~200 deg std dev (almost fully uncertain)
  const float YAW_VAR_CRITICAL = 20.0f;    // Truly diverged
  // Determine health status from worst case
  int health_code = 0;  // Healthy
  // Critical: Any state is severely diverged
  if (max_pos_var > POS_VAR_CRITICAL || max_vel_var > VEL_VAR_CRITICAL ||
      max_rp_var > RP_VAR_CRITICAL || yaw_var > YAW_VAR_CRITICAL) {
    health_code = 3;  // Critical
  }
  // Error: Any state is significantly diverged
  else if (max_pos_var > POS_VAR_ERROR || max_vel_var > VEL_VAR_ERROR ||
           max_rp_var > RP_VAR_ERROR || yaw_var > YAW_VAR_ERROR) {
    health_code = 2;  // Error
  }
  // Warning: Any state is approaching limits
  else if (max_pos_var > POS_VAR_WARNING || max_vel_var > VEL_VAR_WARNING ||
           max_rp_var > RP_VAR_WARNING || yaw_var > YAW_VAR_WARNING) {
    health_code = 1;  // Warning
  }
  // ==========================================================================
  // Health Check 3: Covariance Positive-Definite (ERROR)
  // ==========================================================================
  // Check diagonal elements are positive (necessary for positive-definite)
  for (int i = 0; i < 18; i++) {
    if (P(i,i) <= 0.0f) {
      health_code = (health_code < 2) ? 2 : health_code;  // At least Error level
      break;
    }
  }
  // ==========================================================================
  // Health Check 4: Bias Estimate Sanity (WARNING)
  // ==========================================================================
  // Check if bias estimates are within reasonable physical ranges
  // Gyro bias limits (rad/s) - typical MEMS gyros: ±1000 deg/s = ±17.5 rad/s
  const float GYRO_BIAS_LIMIT = 1.0f;  // 57 deg/s (conservative)
  if (fabsf(gbx) > GYRO_BIAS_LIMIT || fabsf(gby) > GYRO_BIAS_LIMIT || fabsf(gbz) > GYRO_BIAS_LIMIT) {
    health_code = (health_code < 1) ? 1 : health_code;  // At least Warning
  }
  // Accel bias limits (m/s²) - typical MEMS: ±16g = ±157 m/s²
  const float ACCEL_BIAS_LIMIT = 5.0f;  // ~0.5g (conservative)
  if (fabsf(abx) > ACCEL_BIAS_LIMIT || fabsf(aby) > ACCEL_BIAS_LIMIT || fabsf(abz) > ACCEL_BIAS_LIMIT) {
    health_code = (health_code < 1) ? 1 : health_code;  // At least Warning
  }
  // Hysteresis: require N consecutive same health_code before changing reported status (reduces chatter from brief spikes)
  const int HYSTERESIS_ESCALATION = 3;  // consecutive checks before worsening (0->1->2->3)
  const int HYSTERESIS_RECOVERY = 5;    // consecutive checks before improving (3->0)
  int prev_reported = filter_health_status;
  if (health_code == filter_health_status) {
    health_pending_code = health_code;
    health_pending_count = 0;
  } else {
    if (health_code == health_pending_code) {
      health_pending_count++;
    } else {
      health_pending_code = health_code;
      health_pending_count = 1;
    }
    int required = (health_code > filter_health_status) ? HYSTERESIS_ESCALATION : HYSTERESIS_RECOVERY;
    if (health_pending_count >= required) {
      filter_health_status = health_code;
      health_pending_count = 0;
    }
  }
  // Track consecutive failures for potential auto-reset logic (use raw health_code)
  if (health_code >= 2) {  // Error or Critical
    health_failure_count++;
  } else {
    health_failure_count = 0;
  }
  return (filter_health_status == 0);
}

bool uNavINS::isInitialized() {
  return initialized;
}

bool uNavINS::isHealthy() {
  return filter_health_status == 0;
}

int uNavINS::getHealthStatus() {
  return filter_health_status;
}

bool uNavINS::isAtRest() {
  return rest_detector.is_at_rest;
}

bool uNavINS::isZuptActive() {
  return zupt_active;
}
