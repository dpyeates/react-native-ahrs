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

void uNavINS::update(double dt_in, unsigned long TOW, double vn, double ve, double vd,
                     double lat, double lon, double alt,
                     float p, float q, float r,
                     float ax, float ay, float az,
                     float hx, float hy, float hz,
                     float bn, float be, float bd) {
  if (dt_in <= 0.0) {
    return;
  }
  if (!initialized) {
    // initial attitude and heading
    theta = asinf(ax/G);
    phi = asinf(-ay/(G*cosf(theta)));
    // magnetic heading correction due to roll and pitch angle
    Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
    Byc = hy*cosf(phi) - hz*sinf(phi);
    // finding initial heading
    if (-Byc > 0) {
      psi = M_PI/2.0f - atanf(Bxc/-Byc);
    } else {
      psi= 3.0f*M_PI/2.0f - atanf(Bxc/-Byc);
    }
    psi = constrainAngle180(psi);
    psi_initial = psi;
    // euler to quaternion
    quat(0) = cosf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    quat(1) = cosf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f) - sinf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f);
    quat(2) = cosf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f);
    quat(3) = sinf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) - cosf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    // Assemble the matrices
    // ... gravity
    grav(2,0) = G;
    // ... H
    H.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();
    // // ... Rw
    // Rw.block(0,0,3,3) = powf(SIG_W_A,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // Rw.block(3,3,3,3) = powf(SIG_W_G,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // Rw.block(6,6,3,3) = 2.0f*powf(SIG_A_D,2.0f)/TAU_A*Eigen::Matrix<float,3,3>::Identity();
    // Rw.block(9,9,3,3) = 2.0f*powf(SIG_G_D,2.0f)/TAU_G*Eigen::Matrix<float,3,3>::Identity();
    // // ... P
    // P.block(0,0,3,3) = powf(P_P_INIT,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // P.block(3,3,3,3) = powf(P_V_INIT,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // P.block(6,6,2,2) = powf(P_A_INIT,2.0f)*Eigen::Matrix<float,2,2>::Identity();
    // P(8,8) = powf(P_HDG_INIT,2.0f);
    // P.block(9,9,3,3) = powf(P_AB_INIT,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // P.block(12,12,3,3) = powf(P_GB_INIT,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // // ... R
    // R.block(0,0,2,2) = powf(SIG_GPS_P_NE,2.0f)*Eigen::Matrix<float,2,2>::Identity();
    // R(2,2) = powf(SIG_GPS_P_D,2.0f);
    // R.block(3,3,3,3) = powf(SIG_GPS_V,2.0f)*Eigen::Matrix<float,3,3>::Identity();
    // ... Rw (15 process noise sources: 3 accel, 3 gyro, 3 accel bias, 3 gyro bias, 3 mag bias)
    Rw(0,0) = SIG_W_A*SIG_W_A;          Rw(1,1) = SIG_W_A*SIG_W_A;            Rw(2,2) = SIG_W_A*SIG_W_A;
    Rw(3,3) = SIG_W_G*SIG_W_G;          Rw(4,4) = SIG_W_G*SIG_W_G;            Rw(5,5) = SIG_W_G*SIG_W_G;
    Rw(6,6) = 2.0f*SIG_A_D*SIG_A_D/TAU_A; Rw(7,7) = 2.0f*SIG_A_D*SIG_A_D/TAU_A;   Rw(8,8) = 2.0f*SIG_A_D*SIG_A_D/TAU_A;
    Rw(9,9) = 2.0f*SIG_G_D*SIG_G_D/TAU_G; Rw(10,10) = 2.0f*SIG_G_D*SIG_G_D/TAU_G; Rw(11,11) = 2.0f*SIG_G_D*SIG_G_D/TAU_G;
    Rw(12,12) = 2.0f*SIG_M_D*SIG_M_D/TAU_M; Rw(13,13) = 2.0f*SIG_M_D*SIG_M_D/TAU_M; Rw(14,14) = 2.0f*SIG_M_D*SIG_M_D/TAU_M;
    // ... P
    P(0,0) = P_P_INIT*P_P_INIT;       P(1,1) = P_P_INIT*P_P_INIT;       P(2,2) = P_P_INIT*P_P_INIT;
    P(3,3) = P_V_INIT*P_V_INIT;       P(4,4) = P_V_INIT*P_V_INIT;       P(5,5) = P_V_INIT*P_V_INIT;
    P(6,6) = P_A_INIT*P_A_INIT;       P(7,7) = P_A_INIT*P_A_INIT;       P(8,8) = P_HDG_INIT*P_HDG_INIT;
    P(9,9) = P_AB_INIT*P_AB_INIT;     P(10,10) = P_AB_INIT*P_AB_INIT;   P(11,11) = P_AB_INIT*P_AB_INIT;
    P(12,12) = P_GB_INIT*P_GB_INIT;   P(13,13) = P_GB_INIT*P_GB_INIT;   P(14,14) = P_GB_INIT*P_GB_INIT;
    P(15,15) = P_MB_INIT*P_MB_INIT;   P(16,16) = P_MB_INIT*P_MB_INIT;   P(17,17) = P_MB_INIT*P_MB_INIT;
    // ... R
    R(0,0) = SIG_GPS_P_NE*SIG_GPS_P_NE; R(1,1) = SIG_GPS_P_NE*SIG_GPS_P_NE; R(2,2) = SIG_GPS_P_D*SIG_GPS_P_D;
    R(3,3) = SIG_GPS_V*SIG_GPS_V;       R(4,4) = SIG_GPS_V*SIG_GPS_V;       R(5,5) = SIG_GPS_V*SIG_GPS_V;
    R(6,6) = SIG_MAG*SIG_MAG;           R(7,7) = SIG_MAG*SIG_MAG;           R(8,8) = SIG_MAG*SIG_MAG;
    
    // Initialize all biases to zero
    abx = 0.0f; aby = 0.0f; abz = 0.0f;  // accel bias
    gbx = 0.0f; gby = 0.0f; gbz = 0.0f;  // gyro bias
    mbx = 0.0f; mby = 0.0f; mbz = 0.0f;  // mag bias
    
    previousTOW = 0;  // GPS time of week
    
    // .. then initialize states with GPS Data
    lat_ins = lat;
    lon_ins = lon;
    alt_ins = alt;
    vn_ins = vn;
    ve_ins = ve;
    vd_ins = vd;
    // specific force
    f_b(0,0) = ax;
    f_b(1,0) = ay;
    f_b(2,0) = az;
    /* initialize the time */
    _t = 0;
    // initialized flag
    initialized = true;
  } else {
    // get the change in time
    _dt = (float)dt_in;
    if (_dt < 1e-4f) _dt = 1e-4f;
    if (_dt > 0.2f) _dt = 0.2f;
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
    // ... Gyro bias Markov process
    Fs.block(12,12,3,3) = -1.0f/TAU_G*Eigen::Matrix<float,3,3>::Identity();
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
    
    // Discrete Process Noise
    Q = PHI*_dt*Gs*Rw*Gs.transpose();
    Q = 0.5f*(Q+Q.transpose());
    
    // Covariance Time Update
    P = PHI*P*PHI.transpose()+Q;
    P = 0.5f*(P+P.transpose());

    // GPS measurement update
    if ((TOW - previousTOW) > 0) {
      previousTOW = TOW;
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
      // Position, converted to NED
      pos_ecef_ins = lla2ecef(lla_ins);
      pos_ned_ins = ecef2ned(pos_ecef_ins,lla_ins);
      pos_ecef_gps = lla2ecef(lla_gps);
      pos_ned_gps = ecef2ned(pos_ecef_gps,lla_ins);
      
      // Create GPS measurement innovation (y[0-5])
      y(0,0) = (float)(pos_ned_gps(0,0) - pos_ned_ins(0,0));
      y(1,0) = (float)(pos_ned_gps(1,0) - pos_ned_ins(1,0));
      y(2,0) = (float)(pos_ned_gps(2,0) - pos_ned_ins(2,0));
      y(3,0) = (float)(V_gps(0,0) - V_ins(0,0));
      y(4,0) = (float)(V_gps(1,0) - V_ins(1,0));
      y(5,0) = (float)(V_gps(2,0) - V_ins(2,0));
      
      // Build H matrix for GPS (first 6 rows, maps to pos/vel states 0-5)
      H.setZero();
      H.block(0,0,6,6) = Eigen::Matrix<float,6,6>::Identity();
      
      // GPS-only update: zero out magnetometer rows
      y(6,0) = 0.0f;
      y(7,0) = 0.0f;
      y(8,0) = 0.0f;
      H.block(6,0,3,18).setZero();
      
      // Kalman gain (18x9)
      K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
      
      // Covariance update (Joseph form for numerical stability)
      Eigen::Matrix<float,18,18> I_KH = Eigen::Matrix<float,18,18>::Identity() - K*H;
      P = I_KH*P*I_KH.transpose() + K*R*K.transpose();
      P = 0.5f*(P+P.transpose());
      
      // State update
      x = K*y;
      denom = (1.0 - (ECC2 * pow(sin(lla_ins(0,0)),2.0)));
      denom = sqrt(denom*denom);
      Re = EARTH_RADIUS / sqrt(denom);
      Rn = EARTH_RADIUS*(1.0-ECC2) / denom*sqrt(denom);
      alt_ins = alt_ins - x(2,0);
      lat_ins = lat_ins + x(0,0) / (Re + alt_ins);
      lon_ins = lon_ins + x(1,0) / (Rn + alt_ins) / cos(lat_ins);
      vn_ins = vn_ins + x(3,0);
      ve_ins = ve_ins + x(4,0);
      vd_ins = vd_ins + x(5,0);
      
      // Attitude correction
      dq(0,0) = 1.0f;
      dq(1,0) = x(6,0);
      dq(2,0) = x(7,0);
      dq(3,0) = x(8,0);
      quat = qmult(quat,dq);
      quat.normalize();
      
      // Obtain euler angles from quaternion
      theta = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
      phi = atan2f(2.0f*(quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0)),1.0f-2.0f*(quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0)));
      psi = atan2f(2.0f*(quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0)),1.0f-2.0f*(quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0)));
      
      // Update bias estimates
      abx = abx + x(9,0);
      aby = aby + x(10,0);
      abz = abz + x(11,0);
      gbx = gbx + x(12,0);
      gby = gby + x(13,0);
      gbz = gbz + x(14,0);
      mbx = mbx + x(15,0);
      mby = mby + x(16,0);
      mbz = mbz + x(17,0);
    }

    // Magnetometer measurement update (independent of GPS TOW)
    static elapsedMicros mag_update_timer;
    const float MAG_UPDATE_PERIOD_MS = 100.0f; // 10 Hz max
    bool mag_update_ready = (mag_update_timer / 1000.0f) >= MAG_UPDATE_PERIOD_MS;
    bool mag_field_valid = std::isfinite(bn) && std::isfinite(be) && std::isfinite(bd);

    if (mag_update_ready && mag_field_valid) {
      mag_update_timer = 0;

      // Convert measured mag from body frame to nT
      float hx_meas = hx * 1000.0f;
      float hy_meas = hy * 1000.0f;
      float hz_meas = hz * 1000.0f;

      // Expected mag field in NED frame (passed as parameters, already in nT)
      Eigen::Matrix<float,3,1> mag_ned;
      mag_ned << bn, be, bd;

      // Rotate expected NED field to body frame using current attitude DCM
      mag_expected_body = C_N2B * mag_ned;

      // Field magnitude gating to reject severe interference
      float measured_field = sqrtf(hx_meas*hx_meas + hy_meas*hy_meas + hz_meas*hz_meas);
      float expected_field = sqrtf(bn*bn + be*be + bd*bd);
      float field_ratio = (expected_field > 0.0f) ? (measured_field / expected_field) : 0.0f;
      bool mag_reasonable = (field_ratio > 0.5f && field_ratio < 2.0f);

      if (mag_reasonable) {
        // Yaw-only magnetometer update: compare horizontal mag direction
        float mx_meas = hx_meas - mbx;
        float my_meas = hy_meas - mby;
        float mx_exp = mag_expected_body(0);
        float my_exp = mag_expected_body(1);

        // Compute signed yaw error between expected and measured horizontal vectors
        float cross_z = mx_exp * my_meas - my_exp * mx_meas;
        float dot_xy = mx_exp * mx_meas + my_exp * my_meas;
        float yaw_error = constrainAngle180(atan2f(cross_z, dot_xy));

        // Innovation gate (3-sigma) using equivalent yaw noise
        float sigma_yaw = (expected_field > 0.0f) ? (SIG_MAG / expected_field) : 0.0f;
        float innov_gate = 3.0f * sigma_yaw;

        if (sigma_yaw > 0.0f && fabsf(yaw_error) < innov_gate) {
          // Measurement model: yaw error directly observes attitude yaw state (state 8)
          Eigen::Matrix<float,1,18> H_mag;
          H_mag.setZero();
          H_mag(0,8) = 1.0f;

          float R_mag = sigma_yaw * sigma_yaw;

          // Kalman gain for yaw-only magnetometer update (18x1)
          float S_mag = (H_mag * P * H_mag.transpose())(0,0) + R_mag;
          Eigen::Matrix<float,18,1> K_mag = (P * H_mag.transpose()) / S_mag;

          // Covariance update (Joseph form for numerical stability)
          Eigen::Matrix<float,18,18> I_KH_mag = Eigen::Matrix<float,18,18>::Identity() - K_mag * H_mag;
          P = I_KH_mag * P * I_KH_mag.transpose() + K_mag * R_mag * K_mag.transpose();
          P = 0.5f*(P + P.transpose());

          // State update with yaw-only measurement
          x = K_mag * yaw_error;

          // Apply attitude correction from yaw-only magnetometer update
          dq(0,0) = 1.0f;
          dq(1,0) = x(6,0);
          dq(2,0) = x(7,0);
          dq(3,0) = x(8,0);
          quat = qmult(quat,dq);
          quat.normalize();

          // Obtain euler angles from quaternion
          theta = asinf(-2.0f*(quat(1,0)*quat(3,0)-quat(0,0)*quat(2,0)));
          phi = atan2f(2.0f*(quat(0,0)*quat(1,0)+quat(2,0)*quat(3,0)),1.0f-2.0f*(quat(1,0)*quat(1,0)+quat(2,0)*quat(2,0)));
          psi = atan2f(2.0f*(quat(1,0)*quat(2,0)+quat(0,0)*quat(3,0)),1.0f-2.0f*(quat(2,0)*quat(2,0)+quat(3,0)*quat(3,0)));

          // Simple horizontal mag bias estimator (yaw-only)
          // Slowly adapt mbx/mby to reduce steady-state yaw error without affecting roll/pitch
          const float MAG_BIAS_ALPHA = 0.01f; // small adaptation rate
          mbx += MAG_BIAS_ALPHA * (hx_meas - mag_expected_body(0) - mbx);
          mby += MAG_BIAS_ALPHA * (hy_meas - mag_expected_body(1) - mby);
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
  if(dta >  M_PI) dta -= (M_PI*2.0f);
  if(dta < -M_PI) dta += (M_PI*2.0f);
  return dta;
}

// bound heading angle between 0 and 360
float uNavINS::constrainAngle360(float dta){
  dta = fmod(dta,2.0f*M_PI);
  if (dta < 0)
    dta += 2.0f*M_PI;
  return dta;
}
