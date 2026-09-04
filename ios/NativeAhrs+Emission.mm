
#import "NativeAhrs+Emission.h"
#import "NativeAhrs+Transform.h"
#import "NativeAhrs.h"
#include "../fusionml/src/AltitudeCalculator.h"
#include "../fusionml/src/FlightPhaseDetector.h"
#include <cmath>

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

/* =============================================================================
 * REACT NATIVE EVENT EMISSION
 * =============================================================================
 */

@implementation NativeAhrs (Emission)

- (void)emitAhrsUpdateWithTimestamp:(uint64_t)timestamp {
  if (timestamp <= self.nextEmitTime) {
    return;
  }
  
  if (!_filter) {
    return;
  }
  
  const double rad2deg = 180.0 / M_PI;
  double roll = _filter->getRoll_rad() * rad2deg - self.rollOffsetDeg;
  double pitch = _filter->getPitch_rad() * rad2deg - self.pitchOffsetDeg;
  double filterHeading = normalizeHeadingDegrees(_filter->getHeading_rad() * rad2deg);
  
  // Heading: X-Plane ground truth when connected, otherwise filter output
  double heading;
  if (self.hasXPlaneHeading) {
    heading = normalizeHeadingDegrees(self.latestXPlaneHeadingDeg);
  } else {
    // Sensors are already transformed into aviation body axes before the filter.
    heading = filterHeading;
  }

  double vel_n = _filter->getVelNorth_ms();
  double vel_e = _filter->getVelEast_ms();
  double vel_d = _filter->getVelDown_ms();
  
  float horizontal_speed = sqrtf((float)(vel_n*vel_n + vel_e*vel_e));
  
  // Ground track is only meaningful when moving
  // Threshold: 0.514444 m/s (about 1 knot) - below this, ground track is noisy/unreliable
  const float MIN_GROUND_SPEED_FOR_TRACK = 0.514444f;
  
  // Flight path vector is only meaningful when moving
  // Threshold: 2.57222 m/s (about 5 knots) - below this, FPA calculations are unreliable
  const float MIN_SPEED_FOR_FPV = 2.57222f;
  
  // Calculate flight-path angle (vertical angle of velocity vector)
  float flightPathAngle = 0.0f;
  if (horizontal_speed >= MIN_SPEED_FOR_FPV) {
    flightPathAngle = (float)(_filter->getFlightPathAngle_rad() * rad2deg);
  }
  
  // Calculate ground track first (needed for both horizontal FPA methods)
  float track_angle;
  if (horizontal_speed >= MIN_GROUND_SPEED_FOR_TRACK) {
    track_angle = (float)normalizeHeadingDegrees(_filter->getGroundTrack_rad() * rad2deg);
    self.lastValidGroundTrack = track_angle;
    self.hasValidGroundTrack = YES;
  } else {
    // Stationary or very slow - use last valid ground track
    if (self.hasValidGroundTrack) {
      // We have a last valid track - freeze it
      track_angle = self.lastValidGroundTrack;
    } else {
      // No valid track yet - use heading as fallback
      track_angle = (float)heading;
      self.lastValidGroundTrack = track_angle;
      self.hasValidGroundTrack = YES;
    }
  }

  // Calculate horizontal flight path vector
  // Only output if moving faster than 1 m/s
  float horizontalFlightPathAngle = 0.0f;
  if (horizontal_speed >= MIN_SPEED_FOR_FPV) {
    horizontalFlightPathAngle = (float)(_filter->getHorizontalFlightPathAngle_rad() * rad2deg);
  }
  
  float lat_deg = self.hasGpsFix ? _filter->getLatitude_rad() * rad2deg : 0;
  float lon_deg = self.hasGpsFix ? _filter->getLongitude_rad() * rad2deg : 0;
  
  float altitude = (float)_filter->getAltitude_m();
  
  // Calculate barometric altitudes (QNE and QNH)
  float altitudeQNE = altitude; // Default to GPS altitude
  float altitudeQNH = altitude; // Default to GPS altitude
  float barometricPressure = 0.0f;
  
  // Read barometric pressure (thread-safe)
  [self.baroLock lock];
  baro_pressure_t baro = self.baroData;
  [self.baroLock unlock];
  
  if (baro.valid && baro.pressure_hpa > 0.0f) {
    barometricPressure = baro.pressure_hpa;
    
    // Calculate QNE altitude (standard atmosphere, 1013.25 hPa)
    float qne = AltitudeCalculator::calculateQNE_m(baro.pressure_hpa);
    if (std::isfinite(qne)) {
      altitudeQNE = qne;
    }
    
    // Calculate QNH altitude (user-provided QNH, or standard if not set)
    float qnh_hpa = self.qnh > 0.0 ? (float)self.qnh : 1013.25f;
    float qnh_alt = AltitudeCalculator::calculateQNH_m(baro.pressure_hpa, qnh_hpa);
    if (std::isfinite(qnh_alt)) {
      altitudeQNH = qnh_alt;
    }
  }
  
  // Update flight phase detector
  int flightPhase = 0;
  float flightPhaseConfidence = 0.0f;
  BOOL flightPhaseValid = NO;
  
  if (_flightPhaseDetector && self.hasGpsFix) {
    
    // Update flight phase detector (throttles to 1Hz internally)
    _flightPhaseDetector->update(
                                 altitude,                   // gps_alt_msl
                                 -vel_d,                     // vertical_speed (positive = climb)
                                 horizontal_speed,           // groundspeed_ms (will convert to knots internally)
                                 lat_deg,                    // lat_deg
                                 lon_deg,                    // lon_deg
                                 timestamp,                  // timestamp_us
                                 self.groundElevationM,      // ground_elev_msl (user-provided or NAN if not set)
                                 self.lastBodyAccelX         // forward acceleration (body frame, m/s²) for takeoff roll detection
                                 );
    
    // Get outputs
    flightPhase = _flightPhaseDetector->getFlightPhase();
    flightPhaseConfidence = _flightPhaseDetector->getConfidence();
    flightPhaseValid = _flightPhaseDetector->isValid();
  }
  
  int filterHealthStatus = _filter->getHealthStatus();
  BOOL attitudeValid = self.filterInitialized && _filter->isInitialized() &&
                       filterHealthStatus < 3 &&
                       std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(heading);
  [self emitOnAhrsUpdate:@{
    @"roll": @(roll),
    @"pitch": @(pitch),
    @"heading": @(heading),
    @"magneticDeclination": @(self.magneticDeclination),
    @"groundTrack": @(track_angle),
    @"groundSpeed": @(horizontal_speed),
    @"flightPathAngle": @(flightPathAngle),
    @"horizontalFlightPathAngle": @(horizontalFlightPathAngle),
    @"altitude": @(altitude),
    @"altitudeQNE": @(altitudeQNE),
    @"altitudeQNH": @(altitudeQNH),
    @"verticalSpeed": @(-vel_d),
    @"barometricPressure": @(barometricPressure),
    @"velocityNorth": @(vel_n),
    @"velocityEast": @(vel_e),
    @"velocityDown": @(vel_d),
    @"latitude": @(lat_deg),
    @"longitude": @(lon_deg),
    @"flightPhase": @(flightPhase),
    @"flightPhaseConfidence": @(flightPhaseConfidence),
    @"attitudeValid": @(attitudeValid),
    @"altitudeValid": @(self.hasGpsFix || self.baroCalibrated),
    @"positionValid": @(self.hasGpsFix),
    @"flightPhaseValid": @(flightPhaseValid),
    @"filterHealthStatus": @(filterHealthStatus),
    @"atRest": @(_filter->isAtRest()),
    @"zuptActive": @(_filter->isZuptActive()),
  }];
  
  self.nextEmitTime = timestamp + (uint64_t)(1000000.0 / self.emitRateHz);
}

- (void)emitPlaybackStateChangeWithStatus:(NSString *)status
                                 filename:(NSString *)filename
                                   reason:(NSString *)reason {
  NSMutableDictionary *payload = [@{ @"status": status } mutableCopy];
  if (filename) {
    payload[@"filename"] = filename;
  }
  if (reason) {
    payload[@"reason"] = reason;
  }
  [self emitOnPlaybackStateChanged:payload];
}

- (void)emitXPlaneConnectionChangedWithConnected:(BOOL)connected
                                            host:(NSString *)host {
  NSDictionary *payload = @{
    @"connected": @(connected),
    @"host": host ?: @""
  };
  [self emitOnXPlaneConnectionChanged:payload];
}

@end
