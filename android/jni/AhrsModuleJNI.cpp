#include <jni.h>
#include <jsi/jsi.h>
#include <memory>
#include <mutex>
#include <chrono>
#include <android/log.h>
#include <android/location.h>
#include <cmath>

#include "../fusion/ekf.h"
#include "../fusion/ekf_core.h"
#include "../fusion/ekf_math.h"
#include "../fusion/ekf_flight_phase.h"

#define LOG_TAG "AhrsModuleJNI"
#ifdef DEBUG
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...) ((void)0)
#define LOGE(...) ((void)0)
#endif

using namespace facebook::jsi;

// Global EKF instance (using stack-allocated struct)
static ekf_t* g_ekf = nullptr;
static std::mutex g_ekf_mutex;

// Rotation enum matching iOS
enum AhrsRotation {
  AHR_ROTATION_VERTICAL = 0,
  AHR_ROTATION_LEFT = 1,
  AHR_ROTATION_RIGHT = 2
};

static AhrsRotation g_rotation = AHR_ROTATION_VERTICAL;
static bool g_ekf_attitude_initialized = false;
static uint64_t g_last_update_us = 0;

// Transform Android sensor readings to aviation body frame
static void transformToBodyFrame(float android_x, float android_y, float android_z,
                                 AhrsRotation rotation,
                                 float* out_x, float* out_y, float* out_z) {
  switch (rotation) {
    case AHR_ROTATION_LEFT:
      *out_x = -android_z;
      *out_y = android_y;
      *out_z = android_x;
      break;
    case AHR_ROTATION_RIGHT:
      *out_x = -android_z;
      *out_y = -android_y;
      *out_z = -android_x;
      break;
    case AHR_ROTATION_VERTICAL:
    default:
      *out_x = -android_z;
      *out_y = android_x;
      *out_z = -android_y;
      break;
  }
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_ahrs_AhrsModule_initAhrs(JNIEnv *env, jobject thiz) {
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  if (g_ekf) {
    delete g_ekf;
  }
  
  g_ekf = new ekf_t();
  if (!g_ekf) {
    LOGE("Failed to allocate EKF");
    return 0;
  }
  
  ekf_init(g_ekf);
  g_ekf_attitude_initialized = false;
  g_last_update_us = 0;
  
  LOGI("EKF initialized successfully");
  return reinterpret_cast<jlong>(g_ekf);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_destroyAhrs(JNIEnv *env, jobject thiz) {
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  if (g_ekf) {
    delete g_ekf;
    g_ekf = nullptr;
  }
  
  g_ekf_attitude_initialized = false;
  g_last_update_us = 0;
  LOGI("EKF destroyed");
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_updateAhrs(JNIEnv *env, jobject thiz,
                                    jlong ekfPtr,
                                    jlong timestampUs,
                                    jfloatArray accel,
                                    jfloatArray gyro,
                                    jfloatArray mag,
                                    jfloatArray gps,
                                    jfloatArray baro) {
  if (!ekfPtr) return;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  
  // Extract sensor data
  jfloat* accelData = env->GetFloatArrayElements(accel, nullptr);
  jfloat* gyroData = env->GetFloatArrayElements(gyro, nullptr);
  jfloat* magData = env->GetFloatArrayElements(mag, nullptr);
  
  // Transform accelerometer (Android gives m/s², convert to body frame)
  float acc_x, acc_y, acc_z;
  transformToBodyFrame(accelData[0], accelData[1], accelData[2],
                       g_rotation, &acc_x, &acc_y, &acc_z);
  
  vector3_t accel_vec;
  accel_vec.x = acc_x;
  accel_vec.y = acc_y;
  accel_vec.z = acc_z;
  accel_vec.valid = true;
  accel_vec.timestamp_us = timestampUs;
  
  // Transform gyroscope (Android gives rad/s, transform to body frame)
  float gyro_x, gyro_y, gyro_z;
  transformToBodyFrame(gyroData[0], gyroData[1], gyroData[2],
                       g_rotation, &gyro_x, &gyro_y, &gyro_z);
  
  vector3_t gyro_vec;
  gyro_vec.x = gyro_x;
  gyro_vec.y = gyro_y;
  gyro_vec.z = gyro_z;
  gyro_vec.valid = true;
  gyro_vec.timestamp_us = timestampUs;
  
  // Transform magnetometer (Android gives µT, normalize and transform)
  float mag_x, mag_y, mag_z;
  transformToBodyFrame(magData[0], magData[1], magData[2],
                       g_rotation, &mag_x, &mag_y, &mag_z);
  
  // Normalize magnetometer
  float mag_norm = sqrtf(mag_x*mag_x + mag_y*mag_y + mag_z*mag_z);
  vector3_t mag_vec;
  if (mag_norm > 0.01f) {
    mag_vec.x = mag_x / mag_norm;
    mag_vec.y = mag_y / mag_norm;
    mag_vec.z = mag_z / mag_norm;
    mag_vec.valid = true;
  } else {
    mag_vec.valid = false;
    mag_vec.x = 0;
    mag_vec.y = 0;
    mag_vec.z = 0;
  }
  mag_vec.timestamp_us = timestampUs;
  
  // Process GPS if available
  gps_position_t* gps_pos = nullptr;
  gps_position_t gps_data = {0};
  if (gps) {
    jsize gpsLen = env->GetArrayLength(gps);
    if (gpsLen >= 9) {
      jfloat* gpsData = env->GetFloatArrayElements(gps, nullptr);
      // GPS array format: [lat, lon, alt, vel_n, vel_e, vel_d, hdop, num_sats, valid]
      if (gpsData[8] > 0.5f) { // valid flag
        gps_data.lat_deg = gpsData[0];
        gps_data.lon_deg = gpsData[1];
        gps_data.alt_m = gpsData[2];
        // Note: New API computes velocity from track/speed, but we can set reference
        // For now, we'll set the reference and let the EKF compute velocity
        if (!ekf->gps_ref_init) {
          ekf_set_gps_reference(ekf, gpsData[0], gpsData[1], gpsData[2]);
        }
        // Compute track and speed from velocity components
        float vel_n = gpsData[3];
        float vel_e = gpsData[4];
        float speed = sqrtf(vel_n*vel_n + vel_e*vel_e);
        float track = atan2f(vel_e, vel_n) * 180.0f / M_PI;
        if (track < 0.0f) track += 360.0f;
        
        gps_data.track_deg = track;
        gps_data.speed_ms = speed;
        gps_data.vs_ms = -gpsData[5]; // Down is positive in NED, but vs_ms is positive up
        gps_data.valid = true;
        gps_data.timestamp_us = timestampUs;
        gps_pos = &gps_data;
      }
      env->ReleaseFloatArrayElements(gps, gpsData, JNI_ABORT);
    }
  }
  
  // Process barometer if available
  baro_pressure_t* baro_ptr = nullptr;
  baro_pressure_t baro_data = {0};
  if (baro) {
    jsize baroLen = env->GetArrayLength(baro);
    if (baroLen >= 3) {
      jfloat* baroData = env->GetFloatArrayElements(baro, nullptr);
      // Baro array format: [pressure_hpa, temperature_c, valid]
      if (baroData[2] > 0.5f) { // valid flag
        baro_data.pressure_hpa = baroData[0];
        baro_data.valid = true;
        baro_data.timestamp_us = timestampUs;
        baro_ptr = &baro_data;
      }
      env->ReleaseFloatArrayElements(baro, baroData, JNI_ABORT);
    }
  }
  
  // Calculate dt
  float dt = 0.016f; // Default 60Hz
  if (g_last_update_us > 0) {
    dt = (timestampUs - g_last_update_us) / 1000000.0f;
    if (dt <= 0.0f || dt > 1.0f) {
      dt = 0.016f; // Clamp to reasonable values
    }
  }
  g_last_update_us = timestampUs;
  
  // Update EKF
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  ekf_update(ekf, &gyro_vec, &accel_vec, &mag_vec, gps_pos, baro_ptr, dt);
  
  if (!g_ekf_attitude_initialized && ekf->initialized) {
    g_ekf_attitude_initialized = true;
    LOGI("EKF attitude initialized");
  }
  
  env->ReleaseFloatArrayElements(accel, accelData, JNI_ABORT);
  env->ReleaseFloatArrayElements(gyro, gyroData, JNI_ABORT);
  env->ReleaseFloatArrayElements(mag, magData, JNI_ABORT);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_resetAhrs(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  if (!ekfPtr) return;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  // Re-initialize the EKF
  ekf_init(ekf);
  g_ekf_attitude_initialized = false;
  g_last_update_us = 0;
  
  LOGI("AHRS reset");
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_zeroAhrs(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  if (!ekfPtr) return;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  ekf_zero_attitude(ekf);
  
  LOGI("AHRS leveled");
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setAhrsRotation(JNIEnv *env, jobject thiz, jint rotation) {
  g_rotation = static_cast<AhrsRotation>(rotation);
  g_ekf_attitude_initialized = false; // Force re-initialization
  LOGI("Rotation set to %d", rotation);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setMagneticDeclination(JNIEnv *env, jobject thiz, jlong ekfPtr, jfloat declination) {
  if (!ekfPtr) return;
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  ekf_set_magnetic_declination(ekf, declination);
  LOGI("Magnetic declination set to %.1f°", declination);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setQNH(JNIEnv *env, jobject thiz, jlong ekfPtr, jfloat qnh) {
  if (!ekfPtr) return;
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  ekf_set_local_pressure(ekf, qnh);
  LOGI("QNH set to %.2f hPa", qnh);
}


extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_ahrs_AhrsModule_getGpsPosition(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  jdoubleArray result = env->NewDoubleArray(2);
  if (!ekfPtr || !result) return nullptr;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  jdouble data[2] = {0.0, 0.0};
  
  if (ekf->gps_ref_init) {
    float pos_n, pos_e, pos_d;
    ekf_get_position(ekf, &pos_n, &pos_e, &pos_d);
    
    // Convert NED (meters) to lat/lon offset
    // 1 degree latitude ≈ 111,320 meters
    // 1 degree longitude ≈ 111,320 * cos(latitude) meters
    const double PI = 3.14159265358979323846;
    double lat_offset = pos_n / 111320.0;
    double lon_offset = pos_e / (111320.0 * std::cos(ekf->gps_ref_lat * PI / 180.0));
    data[0] = ekf->gps_ref_lat + lat_offset;
    data[1] = ekf->gps_ref_lon + lon_offset;
  }
  
  env->SetDoubleArrayRegion(result, 0, 2, data);
  return result;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_ahrs_AhrsModule_isPositionReliable(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  if (!ekfPtr) return JNI_FALSE;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  // Position is reliable if GPS reference is set and filter is initialized
  return (ekf->gps_ref_init && ekf->initialized) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_ahrs_AhrsModule_getAhrsOutput(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  jfloatArray result = env->NewFloatArray(22);
  if (!ekfPtr || !result) return nullptr;
  
  ekf_t* ekf = reinterpret_cast<ekf_t*>(ekfPtr);
  
  std::lock_guard<std::mutex> lock(g_ekf_mutex);
  
  // Get Euler angles
  float roll_deg, pitch_deg, yaw_deg;
  ekf_get_euler(ekf, &roll_deg, &pitch_deg, &yaw_deg);
  
  // Get velocity
  float vel_n, vel_e, vel_d;
  ekf_get_velocity(ekf, &vel_n, &vel_e, &vel_d);
  
  // Get position
  float pos_n, pos_e, pos_d;
  ekf_get_position(ekf, &pos_n, &pos_e, &pos_d);
  
  // Get barometer altitudes (QNE and QNH)
  float raw_pressure, filtered_pressure, alt_qne, alt_qnh, baro_vel_d;
  ekf_get_baro(ekf, &raw_pressure, &filtered_pressure, &alt_qne, &alt_qnh, &baro_vel_d);
  
  // Get all three altitudes (GPS, QNE, QNH)
  float gps_alt = 0.0f;
  ekf_get_altitudes(ekf, &gps_alt, &alt_qne, &alt_qnh);
  
  // Compute derived values
  float horizontal_speed = sqrtf(vel_n*vel_n + vel_e*vel_e);
  float total_speed = sqrtf(vel_n*vel_n + vel_e*vel_e + vel_d*vel_d);
  float track_angle = atan2f(vel_e, vel_n) * 180.0f / M_PI;
  if (track_angle < 0.0f) track_angle += 360.0f;
  
  // Calculate horizontal flight path angle (sideslip/crab angle) from velocity in body frame
  // Transform NED velocity to body frame using Euler angles
  float roll_rad = roll_deg * M_PI / 180.0f;
  float pitch_rad = pitch_deg * M_PI / 180.0f;
  float yaw_rad = yaw_deg * M_PI / 180.0f;
  
  // Build rotation matrix from NED to body frame (Euler ZYX convention)
  float cr = cosf(roll_rad);
  float sr = sinf(roll_rad);
  float cp = cosf(pitch_rad);
  float sp = sinf(pitch_rad);
  float cy = cosf(yaw_rad);
  float sy = sinf(yaw_rad);
  
  // DCM from NED to body: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  // Forward (x): cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr
  // Right (y): sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr
  // Down (z): -sp, cp*sr, cp*cr
  float vx_body = (cy*cp) * vel_n + (cy*sp*sr - sy*cr) * vel_e + (cy*sp*cr + sy*sr) * vel_d;
  float vy_body = (sy*cp) * vel_n + (sy*sp*sr + cy*cr) * vel_e + (sy*sp*cr - cy*sr) * vel_d;
  
  float horizontal_speed_body = sqrtf(vx_body*vx_body + vy_body*vy_body);
  
  // Flight path vector is only meaningful when moving
  // Threshold: 1.0 m/s - below this, FPA calculations are unreliable
  const float MIN_SPEED_FOR_FPV = 1.0f;
  
  // Calculate flight path angle (vertical) - only if moving faster than 1 m/s
  float flight_path_angle = 0.0f;
  if (horizontal_speed >= MIN_SPEED_FOR_FPV) {
    flight_path_angle = atan2f(-vel_d, horizontal_speed) * 180.0f / M_PI;
  }
  
  // Calculate horizontal flight path angle - only if moving faster than 1 m/s
  float horizontal_flight_path_angle = 0.0f;
  if (horizontal_speed_body >= 0.1f && horizontal_speed >= MIN_SPEED_FOR_FPV) {
    horizontal_flight_path_angle = atan2f(vy_body, vx_body) * 180.0f / M_PI;
  }
  
  // Get current timestamp for flight phase confidence/validity checks
  uint64_t timestamp_us = (uint64_t)(std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
  
  // Get flight phase (already updated in main filter loop)
  flight_phase_t flight_phase = ekf->flight_phase_state.current_phase;
  float flight_phase_confidence = ekf_flight_phase_get_confidence(&ekf->flight_phase_state, timestamp_us);
  bool flight_phase_valid = ekf_flight_phase_is_valid(&ekf->flight_phase_state, timestamp_us);
  
  // Output array: [roll, pitch, heading, flightPathAngle, horizontalFlightPathAngle, trackAngle,
  //                horizontalSpeed, totalSpeed, altitude, altitudeQNE, altitudeQNH,
  //                verticalSpeed,
  //                velocityNorth, velocityEast, velocityDown,
  //                flightPhase, flightPhaseConfidence,
  //                attitudeValid, altitudeValid, positionValid,
  //                flightPhaseValid]
  jfloat data[21] = {
    roll_deg,                    // 0: roll
    pitch_deg,                   // 1: pitch
    yaw_deg,                     // 2: heading (magnetic)
    flight_path_angle,           // 3: flightPathAngle (0 if speed < 1 m/s)
    horizontal_flight_path_angle, // 4: horizontalFlightPathAngle (0 if speed < 1 m/s)
    track_angle,                 // 5: trackAngle
    horizontal_speed,            // 6: horizontalSpeed
    total_speed,                 // 7: totalSpeed
    gps_alt,                     // 8: altitude (GPS MSL)
    alt_qne,                     // 9: altitudeQNE (barometric QNE, standard atmosphere)
    alt_qnh,                     // 10: altitudeQNH (barometric QNH, local pressure)
    -vel_d,                      // 11: verticalSpeed (positive up)
    vel_n,                       // 12: velocityNorth
    vel_e,                       // 13: velocityEast
    vel_d,                       // 14: velocityDown
    (float)(int)flight_phase,    // 15: flightPhase
    flight_phase_confidence,      // 16: flightPhaseConfidence
    ekf->initialized ? 1.0f : 0.0f, // 17: attitudeValid
    ekf->baro_filter_init ? 1.0f : 0.0f, // 18: altitudeValid
    ekf->gps_ref_init ? 1.0f : 0.0f, // 19: positionValid
    flight_phase_valid ? 1.0f : 0.0f  // 20: flightPhaseValid
  };
  
  env->SetFloatArrayRegion(result, 0, 21, data);
  return result;
}
