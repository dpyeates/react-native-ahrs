#import "../fusionml/src/XYZgeomag.hpp"
#import "NativeAhrs+Emission.h"
#import "NativeAhrs+Location.h"
#import "NativeAhrs+Recording.h"
#import "NativeAhrs+Sensors.h"
#import "NativeAhrs+Transform.h"
#import <CoreMotion/CoreMotion.h>
#import <cmath>

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

/* =============================================================================
 * MAGNETIC DECLINATION CALCULATION HELPERS
 * =============================================================================
 */

/**
 * Calculate distance between two lat/lon points using haversine formula
 * @param lat1_rad Latitude 1 in radians
 * @param lon1_rad Longitude 1 in radians
 * @param lat2_rad Latitude 2 in radians
 * @param lon2_rad Longitude 2 in radians
 * @return Distance in meters
 */
static double calculateDistanceMeters(double lat1_rad, double lon1_rad,
                                      double lat2_rad, double lon2_rad) {
  const double EARTH_RADIUS_M = 6378137.0; // WGS84 Earth radius in meters
  double dLat = lat2_rad - lat1_rad;
  double dLon = lon2_rad - lon1_rad;
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
  cos(lat1_rad) * cos(lat2_rad) * sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return EARTH_RADIUS_M * c;
}

/**
 * Get current decimal year for WMM calculations
 * @return Decimal year (e.g., 2025.5 for mid-2025)
 */
static float getCurrentDecimalYear() {
  NSDate *now = [NSDate date];
  NSCalendar *calendar = [NSCalendar currentCalendar];
  NSDateComponents *components = [calendar components:NSCalendarUnitYear
                                             fromDate:now];
  NSInteger year = [components year];
  
  // Calculate day of year
  NSInteger dayOfYear = [calendar ordinalityOfUnit:NSCalendarUnitDay
                                            inUnit:NSCalendarUnitYear
                                           forDate:now];
  return (float)year + ((float)(dayOfYear - 1) / 365.25f);
}

/**
 * Calculate magnetic declination using XYZgeomag library
 * @param lat_deg Latitude in degrees
 * @param lon_deg Longitude in degrees
 * @param alt_m Altitude in meters
 * @return Magnetic declination in degrees (positive = east)
 */
static double calculateMagneticDeclination(double lat_deg, double lon_deg,
                                           double alt_m) {
  float decimalYear = getCurrentDecimalYear();
  
  // Convert geodetic coordinates to ECEF
  geomag::Vector position =
  geomag::geodetic2ecef((float)lat_deg, (float)lon_deg, (float)alt_m);
  
  // Calculate magnetic field at position
  geomag::Vector magField =
  geomag::GeoMag(decimalYear, position, geomag::WMM2025);
  
  // Convert to magnetic elements (includes declination)
  geomag::Elements elements =
  geomag::magField2Elements(magField, (float)lat_deg, (float)lon_deg);
  
  return (double)elements.declination;
}

/**
 * Calculate expected magnetic field in NED frame using XYZgeomag library
 * @param lat_deg Latitude in degrees
 * @param lon_deg Longitude in degrees
 * @param alt_m Altitude in meters
 * @param out_bn Output: North component (nT)
 * @param out_be Output: East component (nT)
 * @param out_bd Output: Down component (nT)
 */
static void calculateExpectedMagFieldNED(double lat_deg, double lon_deg, double alt_m,
                                         float *out_bn, float *out_be, float *out_bd) {
  float decimalYear = getCurrentDecimalYear();
  
  // Convert geodetic coordinates to ECEF
  geomag::Vector position =
  geomag::geodetic2ecef((float)lat_deg, (float)lon_deg, (float)alt_m);
  
  // Calculate magnetic field at position
  geomag::Vector magField =
  geomag::GeoMag(decimalYear, position, geomag::WMM2025);
  
  // Convert to magnetic elements (NED components in nT)
  geomag::Elements elements =
  geomag::magField2Elements(magField, (float)lat_deg, (float)lon_deg);
  
  *out_bn = elements.north;  // nT
  *out_be = elements.east;   // nT
  *out_bd = elements.down;   // nT
}

/* =============================================================================
 * SENSOR DATA PROCESSING
 * =============================================================================
 */

@implementation NativeAhrs (Sensors)

/**
 * Update magnetic declination using XYZgeomag when position changes
 * significantly.
 */
- (void)updateMagneticDeclinationIfNeededForLatRad:(double)latRad
                                            lonRad:(double)lonRad
                                              altM:(double)altM {
  if (!std::isfinite(latRad) || !std::isfinite(lonRad) ||
      !std::isfinite(altM)) {
    return;
  }
  
  // Recalculate if moved 10+ nautical miles (18,520 meters) since last
  // declination calc
  const double MIN_DISTANCE_FOR_DECLINATION_UPDATE_M = 18520.0;
  BOOL needDeclinationUpdate = NO;
  
  if (self.lastDeclinationLatRad == 0.0 && self.lastDeclinationLonRad == 0.0) {
    // First time - always calculate
    needDeclinationUpdate = YES;
  } else {
    double distanceM = calculateDistanceMeters(
                                               self.lastDeclinationLatRad, self.lastDeclinationLonRad, latRad, lonRad);
    if (distanceM >= MIN_DISTANCE_FOR_DECLINATION_UPDATE_M) {
      needDeclinationUpdate = YES;
    }
  }
  
  if (needDeclinationUpdate) {
    double latDeg = latRad * 180.0 / M_PI;
    double lonDeg = lonRad * 180.0 / M_PI;
    double declination = calculateMagneticDeclination(latDeg, lonDeg, altM);
    self.magneticDeclination = declination;
    self.lastDeclinationLatRad = latRad;
    self.lastDeclinationLonRad = lonRad;
    
    // Calculate expected magnetic field for EKF magnetometer measurement
    float bn_nT, be_nT, bd_nT;
    calculateExpectedMagFieldNED(latDeg, lonDeg, altM, &bn_nT, &be_nT, &bd_nT);
    self.expectedMagN_nT = bn_nT;
    self.expectedMagE_nT = be_nT;
    self.expectedMagD_nT = bd_nT;
    
    AHRS_LOG(@"🧭 Magnetic declination: %.1f°, expected field: N=%.1f, E=%.1f, D=%.1f nT at "
             @"lat=%.6f°, lon=%.6f°",
             declination, bn_nT, be_nT, bd_nT, latDeg, lonDeg);
  }
}

- (void)ProcessDeviceMotionData:(CMDeviceMotion *)motion {
  if (!self.running || !_filter || self.isPlaying || self.xplaneConnected) {
    return;
  }
  // Core Motion timestamp is NSTimeInterval (seconds since boot)
  // Convert to absolute time (microseconds since 1970)
  NSTimeInterval bootTime = [[NSDate date] timeIntervalSince1970] -
  [[NSProcessInfo processInfo] systemUptime];
  uint64_t timestamp_us = (uint64_t)((bootTime + motion.timestamp) * 1e6);
  
  const double g0 = 9.80665;
  double ios_accel_x = (motion.userAcceleration.x - motion.gravity.x) * g0;
  double ios_accel_y = (motion.userAcceleration.y - motion.gravity.y) * g0;
  double ios_accel_z = (motion.userAcceleration.z - motion.gravity.z) * g0;
  
  double body_gyro_x, body_gyro_y, body_gyro_z;
  double body_accel_x, body_accel_y, body_accel_z;
  double body_mag_x, body_mag_y, body_mag_z;
  
  transformToBodyFrame(motion.rotationRate.x, motion.rotationRate.y,
                       motion.rotationRate.z, self.rotation, &body_gyro_x,
                       &body_gyro_y, &body_gyro_z);
  transformToBodyFrame(ios_accel_x, ios_accel_y, ios_accel_z, self.rotation,
                       &body_accel_x, &body_accel_y, &body_accel_z);
  transformToBodyFrame(motion.magneticField.field.x,
                       motion.magneticField.field.y,
                       motion.magneticField.field.z, self.rotation, &body_mag_x,
                       &body_mag_y, &body_mag_z);
  
  if (self.isRecording) {
    float gyroData[3] = {(float)body_gyro_x, (float)body_gyro_y,
      (float)body_gyro_z};
    float accelData[3] = {(float)body_accel_x, (float)body_accel_y,
      (float)body_accel_z};
    float magData[3] = {(float)body_mag_x, (float)body_mag_y,
      (float)body_mag_z};
    [self writeRecordingPacket:1
                     timestamp:timestamp_us
                          data:gyroData
                        length:sizeof(gyroData)];
    [self writeRecordingPacket:2
                     timestamp:timestamp_us
                          data:accelData
                        length:sizeof(accelData)];
    [self writeRecordingPacket:3
                     timestamp:timestamp_us
                          data:magData
                        length:sizeof(magData)];
  }
  
  gps_position_t gps;
  [self.locationLock lock];
  gps = self.locationData;
  [self.locationLock unlock];
  
  double trackRad = gps.track_deg * M_PI / 180.0;
  double velN = gps.speed_ms * cos(trackRad);
  double velE = gps.speed_ms * sin(trackRad);
  double velD = -gps.vs_ms;
  
  // Check if we have new GPS data with acceptable accuracy
  BOOL hasNewGps = NO;
  if (gps.valid && gps.timestamp_us != self.lastGpsTimestampUs) {
    // Check GPS accuracy using shared helper method
    BOOL accuracyAcceptable = [self hasAcceptableGpsAccuracy:gps];
    
    if (accuracyAcceptable) {
      hasNewGps = YES;
      self.hasGpsFix = YES;
      double newLatRad = gps.lat_deg * M_PI / 180.0;
      double newLonRad = gps.lon_deg * M_PI / 180.0;
      self.lastLatRad = newLatRad;
      self.lastLonRad = newLonRad;
      self.lastAltM = gps.alt_m;
      self.lastGpsTimestampUs = gps.timestamp_us;
      
      [self updateMagneticDeclinationIfNeededForLatRad:self.lastLatRad
                                                lonRad:self.lastLonRad
                                                  altM:self.lastAltM];
      // Update TOW with new GPS data
      self.currentTow = (unsigned long)(gps.timestamp_us / 1000ULL);
    } else {
      // GPS is valid but accuracy is poor - update position state but don't
      // update TOW This allows the filter to continue with IMU-only updates
      // using the last good GPS TOW
      self.hasGpsFix = YES;
      self.lastLatRad = gps.lat_deg * M_PI / 180.0;
      self.lastLonRad = gps.lon_deg * M_PI / 180.0;
      self.lastAltM = gps.alt_m;
      // Don't update lastGpsTimestampUs or TOW - keep using the last good GPS
      // update
      [self updateMagneticDeclinationIfNeededForLatRad:self.lastLatRad
                                                lonRad:self.lastLonRad
                                                  altM:self.lastAltM];
    }
  } else if (gps.valid) {
    // GPS is valid but same timestamp - keep existing GPS state
    self.hasGpsFix = YES;
  }
  
  // If we still have no GPS fix, continue with the best available position
  // (may be stale or low-accuracy) to allow attitude updates to flow.
  if (!self.hasGpsFix) {
    self.lastLatRad = gps.lat_deg * M_PI / 180.0;
    self.lastLonRad = gps.lon_deg * M_PI / 180.0;
    self.lastAltM = gps.alt_m;
  }
  
  // Calculate dt from timestamp difference
  double dt = 0.0;
  if (self.lastTimestampUs > 0) {
    dt = (double)(timestamp_us - self.lastTimestampUs) /
    1e6; // Convert microseconds to seconds
    // Clamp dt to reasonable range (1e-4 to 0.2 seconds)
    if (dt < 1e-4)
      dt = 1e-4;
    if (dt > 0.2)
      dt = 0.2;
  }
  self.lastTimestampUs = timestamp_us;
  
  // Gate magnetometer fusion based on Core Motion calibration accuracy
  BOOL magAccuracyOk = (motion.magneticField.accuracy == CMMagneticFieldCalibrationAccuracyMedium ||
                        motion.magneticField.accuracy == CMMagneticFieldCalibrationAccuracyHigh);
  float expectedMagN_nT = magAccuracyOk ? self.expectedMagN_nT : NAN;
  float expectedMagE_nT = magAccuracyOk ? self.expectedMagE_nT: NAN;
  float expectedMagD_nT = magAccuracyOk ? self.expectedMagD_nT: NAN;

  // Update filter with sensor data and expected magnetic field from WMM
  _filter->update(dt, self.currentTow, velN, velE, velD, self.lastLatRad,
                  self.lastLonRad, self.lastAltM, (float)body_gyro_x,
                  (float)body_gyro_y, (float)body_gyro_z, (float)body_accel_x,
                  (float)body_accel_y, (float)body_accel_z, (float)body_mag_x,
                  (float)body_mag_y, (float)body_mag_z,
                  expectedMagN_nT, expectedMagE_nT, expectedMagD_nT);
  
  // Store latest forward acceleration for flight phase detection
  self.lastBodyAccelX = (float)body_accel_x;
  
  self.filterInitialized = YES;
  [self emitAhrsUpdateWithTimestamp:timestamp_us];
}

- (void)ProcessAltitudeData:(CMAltitudeData *)altitudeData {
  if (!self.running || self.isPlaying) {
    return;
  }
  
  // Core Motion timestamp is NSTimeInterval (seconds since boot)
  // Convert to absolute time (microseconds since 1970)
  NSTimeInterval bootTime = [[NSDate date] timeIntervalSince1970] -
  [[NSProcessInfo processInfo] systemUptime];
  uint64_t timestamp_us = (uint64_t)((bootTime + altitudeData.timestamp) * 1e6);
  
  // iOS provides relative pressure (kPa) - we need absolute pressure
  // CMAltitudeData.pressure is an NSNumber, so we need to extract the value
  float relative_pressure_kpa = [altitudeData.pressure floatValue];
  
  // Calibrate barometer against GPS if available
  [self.locationLock lock];
  gps_position_t gps = self.locationData;
  [self.locationLock unlock];
  
  if (gps.valid && !self.baroCalibrated) {
    // Calibrate barometer against GPS altitude
    // Use standard atmosphere model: P = P0 * (1 - L*h/T0)^(g*M/(R*L))
    // Simplified: P ≈ P0 * exp(-h*0.00012) where h is in meters
    // P0 = 101.325 kPa at sea level
    const float P0_KPA = 101.325f;
    const float LAPSE_RATE = 0.0065f; // K/m
    const float T0_K = 288.15f;       // K
    const float G = 9.80665f;         // m/s^2
    const float M = 0.0289644f;       // kg/mol
    const float R = 8.31447f;         // J/(mol*K)
    
    float h = gps.alt_m;
    float expected_pressure =
    P0_KPA * powf(1.0f - LAPSE_RATE * h / T0_K, G * M / (R * LAPSE_RATE));
    
    // Compute offset to convert relative to absolute
    self.baroPressureOffset = expected_pressure - relative_pressure_kpa;
    self.baroCalibrated = YES;
    
    AHRS_LOG(@"📊 Barometer calibrated: GPS alt=%.1fm, offset=%.3f kPa", h,
             self.baroPressureOffset);
  }
  
  // Convert to absolute pressure
  float absolute_pressure_kpa = relative_pressure_kpa + self.baroPressureOffset;
  
  // Convert kPa to hPa (hectopascals)
  float pressure_hpa = absolute_pressure_kpa * 10.0f;
  
  // Store barometer data
  baro_pressure_t baro = {.pressure_hpa = pressure_hpa,
      .valid = true,
    .timestamp_us = timestamp_us};
  
  [self.baroLock lock];
  self.baroData = baro;
  [self.baroLock unlock];
  
  // Record barometer data if recording
  if (self.isRecording) {
    float baroData[1] = {pressure_hpa};
    [self writeRecordingPacket:4
                     timestamp:timestamp_us
                          data:baroData
                        length:sizeof(baroData)];
  }
}

@end
