#include <jni.h>
#include <android/log.h>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <limits>
#include <mutex>
#include <new>

#include "AltitudeCalculator.h"
#include "FlightPhaseDetector.h"
#include "XYZgeomag.hpp"
#include "uNavINS.h"

#define LOG_TAG "AhrsModuleJNI"
#ifdef DEBUG
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#else
#define LOGI(...) ((void)0)
#endif
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double RAD2DEG = 180.0 / M_PI;
static const double DEG2RAD = M_PI / 180.0;
static const float MIN_GROUND_SPEED_FOR_TRACK = 0.514444f; // 1 kt
static const float MIN_SPEED_FOR_FPV = 2.57222f;           // 5 kt
static const double MIN_DECLINATION_UPDATE_M = 1852.0;

enum AhrsRotation {
  AHR_ROTATION_VERTICAL = 0,
  AHR_ROTATION_LEFT = 1,
  AHR_ROTATION_RIGHT = 2
};

struct FilterContext {
  uNavINS filter;
  FlightPhaseDetector phase;
  AhrsRotation rotation = AHR_ROTATION_VERTICAL;
  uint64_t last_update_us = 0;
  float qnh_hpa = 1013.25f;
  float roll_offset_deg = 0.0f;
  float pitch_offset_deg = 0.0f;
  bool has_declination = false;
  double last_decl_lat = 0.0;
  double last_decl_lon = 0.0;
  float declination_deg = 0.0f;
  float bn = NAN;
  float be = NAN;
  float bd = NAN;
  bool has_gps = false;
  double last_lat_rad = 0.0;
  double last_lon_rad = 0.0;
  double last_alt_m = 0.0;
  float last_track_deg = 0.0f;
  bool has_track = false;
  float last_body_ax = 0.0f;
  float last_baro_hpa = -1.0f;
};

static std::mutex g_mutex;

static FilterContext *ctxFrom(jlong ptr) {
  return reinterpret_cast<FilterContext *>(ptr);
}

static void transformToBodyFrame(float android_x, float android_y, float android_z,
                                 AhrsRotation rotation,
                                 float *out_x, float *out_y, float *out_z) {
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

static float wrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6378137.0;
  double dLat = lat2 - lat1;
  double dLon = lon2 - lon1;
  double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0) +
             std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
  return R * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

static float currentDecimalYear() {
  time_t now = time(nullptr);
  struct tm t{};
  gmtime_r(&now, &t);
  return (float)(t.tm_year + 1900) + ((float)t.tm_yday / 365.25f);
}

static void updateWmmIfNeeded(FilterContext *ctx, double lat_rad, double lon_rad, double alt_m) {
  if (!std::isfinite(lat_rad) || !std::isfinite(lon_rad) || !std::isfinite(alt_m)) {
    return;
  }
  if (std::fabs(lat_rad) < 1e-8 && std::fabs(lon_rad) < 1e-8) {
    return;
  }
  bool need = !ctx->has_declination;
  if (!need) {
    need = haversineMeters(ctx->last_decl_lat, ctx->last_decl_lon, lat_rad, lon_rad) >=
           MIN_DECLINATION_UPDATE_M;
  }
  if (!need) {
    return;
  }
  double lat_deg = lat_rad * RAD2DEG;
  double lon_deg = lon_rad * RAD2DEG;
  float year = currentDecimalYear();
  geomag::Vector position = geomag::geodetic2ecef((float)lat_deg, (float)lon_deg, (float)alt_m);
  geomag::Vector magField = geomag::GeoMag(year, position, geomag::WMM2025);
  geomag::Elements elements = geomag::magField2Elements(magField, (float)lat_deg, (float)lon_deg);
  ctx->declination_deg = elements.declination;
  ctx->bn = elements.north;
  ctx->be = elements.east;
  ctx->bd = elements.down;
  ctx->last_decl_lat = lat_rad;
  ctx->last_decl_lon = lon_rad;
  ctx->has_declination = true;
  LOGI("WMM declination %.1f deg  N=%.0f E=%.0f D=%.0f nT",
       ctx->declination_deg, ctx->bn, ctx->be, ctx->bd);
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_ahrs_AhrsModule_initAhrs(JNIEnv *env, jobject thiz) {
  std::lock_guard<std::mutex> lock(g_mutex);
  auto *ctx = new (std::nothrow) FilterContext();
  if (!ctx) {
    LOGE("Failed to allocate filter context");
    return 0;
  }
  LOGI("uNavINS initialized");
  return reinterpret_cast<jlong>(ctx);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_destroyAhrs(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  std::lock_guard<std::mutex> lock(g_mutex);
  delete ctxFrom(ekfPtr);
  LOGI("uNavINS destroyed");
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
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx || !accel || !gyro || !mag) {
    return;
  }

  jfloat *accelData = env->GetFloatArrayElements(accel, nullptr);
  jfloat *gyroData = env->GetFloatArrayElements(gyro, nullptr);
  jfloat *magData = env->GetFloatArrayElements(mag, nullptr);
  if (!accelData || !gyroData || !magData) {
    if (accelData) env->ReleaseFloatArrayElements(accel, accelData, JNI_ABORT);
    if (gyroData) env->ReleaseFloatArrayElements(gyro, gyroData, JNI_ABORT);
    if (magData) env->ReleaseFloatArrayElements(mag, magData, JNI_ABORT);
    return;
  }

  float ax, ay, az, p, q, r, hx, hy, hz;
  transformToBodyFrame(accelData[0], accelData[1], accelData[2], ctx->rotation, &ax, &ay, &az);
  transformToBodyFrame(gyroData[0], gyroData[1], gyroData[2], ctx->rotation, &p, &q, &r);
  transformToBodyFrame(magData[0], magData[1], magData[2], ctx->rotation, &hx, &hy, &hz);

  env->ReleaseFloatArrayElements(accel, accelData, JNI_ABORT);
  env->ReleaseFloatArrayElements(gyro, gyroData, JNI_ABORT);
  env->ReleaseFloatArrayElements(mag, magData, JNI_ABORT);

  double vn = 0.0, ve = 0.0, vd = 0.0;
  float hacc = -1.0f, vacc = -1.0f, sacc = -1.0f;
  unsigned long tow = 0;
  if (gps) {
    jsize gpsLen = env->GetArrayLength(gps);
    if (gpsLen >= 10) {
      jfloat *gpsData = env->GetFloatArrayElements(gps, nullptr);
      if (gpsData && gpsData[9] > 0.5f) {
        ctx->last_lat_rad = gpsData[0] * DEG2RAD;
        ctx->last_lon_rad = gpsData[1] * DEG2RAD;
        ctx->last_alt_m = gpsData[2];
        vn = gpsData[3];
        ve = gpsData[4];
        vd = gpsData[5];
        hacc = gpsData[6];
        vacc = gpsData[7];
        sacc = gpsData[8];
        ctx->has_gps = true;
        tow = (unsigned long)(timestampUs / 1000);
        updateWmmIfNeeded(ctx, ctx->last_lat_rad, ctx->last_lon_rad, ctx->last_alt_m);
      }
      if (gpsData) env->ReleaseFloatArrayElements(gps, gpsData, JNI_ABORT);
    }
  }

  float baro_hpa = -1.0f;
  if (baro) {
    jsize baroLen = env->GetArrayLength(baro);
    if (baroLen >= 3) {
      jfloat *baroData = env->GetFloatArrayElements(baro, nullptr);
      if (baroData && baroData[2] > 0.5f && baroData[0] > 0.0f) {
        baro_hpa = baroData[0];
        ctx->last_baro_hpa = baro_hpa;
      }
      if (baroData) env->ReleaseFloatArrayElements(baro, baroData, JNI_ABORT);
    }
  }

  double dt = 0.0;
  if (ctx->last_update_us > 0) {
    dt = (double)(timestampUs - (jlong)ctx->last_update_us) / 1e6;
    if (dt < 1e-4) dt = 1e-4;
    if (dt > 0.2) dt = 0.2;
  }
  ctx->last_update_us = (uint64_t)timestampUs;
  ctx->last_body_ax = ax;

  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->filter.update(dt, tow, vn, ve, vd, ctx->last_lat_rad, ctx->last_lon_rad,
                     ctx->last_alt_m, p, q, r, ax, ay, az, hx, hy, hz,
                     ctx->bn, ctx->be, ctx->bd, hacc, vacc, sacc,
                     baro_hpa, ctx->qnh_hpa);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_resetAhrs(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->filter = uNavINS();
  ctx->phase = FlightPhaseDetector();
  ctx->last_update_us = 0;
  ctx->roll_offset_deg = 0.0f;
  ctx->pitch_offset_deg = 0.0f;
  ctx->has_gps = false;
  ctx->has_track = false;
  LOGI("AHRS reset");
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_zeroAhrs(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->roll_offset_deg = ctx->filter.getRoll_rad() * (float)RAD2DEG;
  ctx->pitch_offset_deg = ctx->filter.getPitch_rad() * (float)RAD2DEG;
  LOGI("AHRS leveled (roll offset %.2f pitch offset %.2f)",
       ctx->roll_offset_deg, ctx->pitch_offset_deg);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setAhrsRotation(JNIEnv *env, jobject thiz, jlong ekfPtr, jint rotation) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->rotation = static_cast<AhrsRotation>(rotation);
  LOGI("Rotation set to %d", rotation);
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setMagneticDeclination(JNIEnv *env, jobject thiz,
                                                jlong ekfPtr, jfloat declination) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->declination_deg = declination;
  ctx->has_declination = true;
}

extern "C" JNIEXPORT void JNICALL
Java_com_ahrs_AhrsModule_setQNH(JNIEnv *env, jobject thiz, jlong ekfPtr, jfloat qnh) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return;
  std::lock_guard<std::mutex> lock(g_mutex);
  ctx->qnh_hpa = qnh;
  LOGI("QNH set to %.2f hPa", qnh);
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_ahrs_AhrsModule_getGpsPosition(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  jdoubleArray result = env->NewDoubleArray(2);
  if (!result) return nullptr;
  jdouble data[2] = {0.0, 0.0};
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (ctx) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (ctx->filter.isInitialized()) {
      data[0] = ctx->filter.getLatitude_rad() * RAD2DEG;
      data[1] = ctx->filter.getLongitude_rad() * RAD2DEG;
    }
  }
  env->SetDoubleArrayRegion(result, 0, 2, data);
  return result;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_ahrs_AhrsModule_isPositionReliable(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) return JNI_FALSE;
  std::lock_guard<std::mutex> lock(g_mutex);
  return (ctx->has_gps && ctx->filter.isInitialized()) ? JNI_TRUE : JNI_FALSE;
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_ahrs_AhrsModule_getAhrsOutput(JNIEnv *env, jobject thiz, jlong ekfPtr) {
  // [roll, pitch, heading, fpa, hfpa, groundTrack, groundSpeed,
  //  altitude, qne, qnh, vs, vn, ve, vd,
  //  phase, phaseConf, attitudeValid, altitudeValid, positionValid, phaseValid,
  //  health, atRest, zupt, declination]
  jfloatArray result = env->NewFloatArray(24);
  if (!result) return nullptr;
  jfloat data[24] = {0};
  FilterContext *ctx = ctxFrom(ekfPtr);
  if (!ctx) {
    env->SetFloatArrayRegion(result, 0, 24, data);
    return result;
  }

  std::lock_guard<std::mutex> lock(g_mutex);
  float roll = ctx->filter.getRoll_rad() * (float)RAD2DEG - ctx->roll_offset_deg;
  float pitch = ctx->filter.getPitch_rad() * (float)RAD2DEG - ctx->pitch_offset_deg;
  float heading = wrap360(ctx->filter.getHeading_rad() * (float)RAD2DEG);
  double vel_n = ctx->filter.getVelNorth_ms();
  double vel_e = ctx->filter.getVelEast_ms();
  double vel_d = ctx->filter.getVelDown_ms();
  float ground_speed = std::sqrt((float)(vel_n * vel_n + vel_e * vel_e));

  float fpa = 0.0f;
  float hfpa = 0.0f;
  if (ground_speed >= MIN_SPEED_FOR_FPV) {
    fpa = ctx->filter.getFlightPathAngle_rad() * (float)RAD2DEG;
    hfpa = ctx->filter.getHorizontalFlightPathAngle_rad() * (float)RAD2DEG;
  }

  float track;
  if (ground_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
    track = wrap360(ctx->filter.getGroundTrack_rad() * (float)RAD2DEG);
    ctx->last_track_deg = track;
    ctx->has_track = true;
  } else if (ctx->has_track) {
    track = ctx->last_track_deg;
  } else {
    track = heading;
    ctx->last_track_deg = track;
    ctx->has_track = true;
  }

  float altitude = (float)ctx->filter.getAltitude_m();
  float qne = altitude;
  float qnh_alt = altitude;
  if (ctx->last_baro_hpa > 0.0f) {
    float qne_m = AltitudeCalculator::calculateQNE_m(ctx->last_baro_hpa);
    if (std::isfinite(qne_m)) qne = qne_m;
    float qnh_m = AltitudeCalculator::calculateQNH_m(ctx->last_baro_hpa, ctx->qnh_hpa);
    if (std::isfinite(qnh_m)) qnh_alt = qnh_m;
  }

  float lat_deg = ctx->filter.getLatitude_rad() * (float)RAD2DEG;
  float lon_deg = ctx->filter.getLongitude_rad() * (float)RAD2DEG;
  int health = ctx->filter.getHealthStatus();
  bool initialized = ctx->filter.isInitialized();
  bool attitude_valid = initialized && health < 3 &&
                        std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(heading);

  if (ctx->has_gps) {
    ctx->phase.update(altitude, (float)(-vel_d), ground_speed, lat_deg, lon_deg,
                      ctx->last_update_us, std::numeric_limits<float>::quiet_NaN(),
                      ctx->last_body_ax);
  }

  data[0] = roll;
  data[1] = pitch;
  data[2] = heading;
  data[3] = fpa;
  data[4] = hfpa;
  data[5] = track;
  data[6] = ground_speed;
  data[7] = altitude;
  data[8] = qne;
  data[9] = qnh_alt;
  data[10] = (float)(-vel_d);
  data[11] = (float)vel_n;
  data[12] = (float)vel_e;
  data[13] = (float)vel_d;
  data[14] = (float)ctx->phase.getFlightPhase();
  data[15] = ctx->phase.getConfidence();
  data[16] = attitude_valid ? 1.0f : 0.0f;
  data[17] = (ctx->has_gps) ? 1.0f : 0.0f;
  data[18] = ctx->has_gps ? 1.0f : 0.0f;
  data[19] = ctx->phase.isValid() ? 1.0f : 0.0f;
  data[20] = (float)health;
  data[21] = ctx->filter.isAtRest() ? 1.0f : 0.0f;
  data[22] = ctx->filter.isZuptActive() ? 1.0f : 0.0f;
  data[23] = ctx->declination_deg;

  env->SetFloatArrayRegion(result, 0, 24, data);
  return result;
}
