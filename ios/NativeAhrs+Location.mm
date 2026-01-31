
#import "NativeAhrs+Location.h"
#import "NativeAhrs+Transform.h"
#import "NativeAhrs+Recording.h"

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

/* =============================================================================
 * GPS PROCESSING
 * =============================================================================
 */

@implementation NativeAhrs (Location)

- (void)locationManager:(CLLocationManager *)manager
     didUpdateLocations:(NSArray<CLLocation *> *)locations {
  
  CLLocation *latestLocation = [locations lastObject];
  
  // ===== Validate CLLocation =====
  
  // Check coordinate validity
  if (!CLLocationCoordinate2DIsValid(latestLocation.coordinate)) {
    return;
  }
  
  // Check timestamp age (must be within 10 seconds, not in future)
  NSTimeInterval age = -[latestLocation.timestamp timeIntervalSinceNow];
  if (age > 10.0 || age < -1.0) {
    return;
  }
  
  gps_position_t data = {0};
  data.timestamp_us = (uint64_t)([latestLocation.timestamp timeIntervalSince1970] * 1e6);
  data.lat_deg = latestLocation.coordinate.latitude;
  data.lon_deg = latestLocation.coordinate.longitude;
  data.alt_m = latestLocation.altitude;
  data.horizontalAccuracy_m = latestLocation.horizontalAccuracy;
  data.verticalAccuracy_m = latestLocation.verticalAccuracy;
  data.speedAccuracy_ms = latestLocation.speedAccuracy;
  
  // Mark GPS as valid if coordinates are valid and timestamp is recent
  // Filter will handle accuracy-based trust via adaptive measurement noise
  data.valid = YES;
  
  // Compute velocity components and derive track/speed
  float vel_n = 0.0f;
  float vel_e = 0.0f;
  float vel_d = 0.0f;
  
  // Check speed and course accuracy before using GPS-provided values
  // Indoors, GPS speed/course can be very noisy even when position is somewhat accurate
  BOOL speedAccurate = YES;
  BOOL courseAccurate = YES;
  
  if (latestLocation.speedAccuracy >= 0 && latestLocation.speedAccuracy > 2.0f) {
    speedAccurate = NO;  // Speed accuracy > 2 m/s is too poor for reliable velocity
  }
  
  if (latestLocation.courseAccuracy >= 0 && latestLocation.courseAccuracy > 10.0f) {
    courseAccurate = NO;  // Course accuracy > 10° is too poor for reliable direction
  }
  
  if (latestLocation.speed >= 0 && latestLocation.speedAccuracy >= 0 && speedAccurate) {
    if (latestLocation.course >= 0 && latestLocation.courseAccuracy >= 0 && courseAccurate) {
      double course_rad = latestLocation.course * M_PI / 180.0;
      vel_n = (float)(latestLocation.speed * cos(course_rad));
      vel_e = (float)(latestLocation.speed * sin(course_rad));
      data.speed_ms = (float)latestLocation.speed;
      data.track_deg = (float)latestLocation.course;
    } else if (self.previousLocation) {
      [self computeVelocityFromPositions:latestLocation vel_n:&vel_n vel_e:&vel_e vel_d:&vel_d];
      data.speed_ms = sqrtf(vel_n*vel_n + vel_e*vel_e);
      data.track_deg = atan2f(vel_e, vel_n) * 180.0f / M_PI;
      if (data.track_deg < 0.0f) data.track_deg += 360.0f;
    } else {
      data.speed_ms = 0.0f;
      data.track_deg = 0.0f;
    }
    
    if (self.previousLocation) {
      NSTimeInterval dt = [latestLocation.timestamp
                           timeIntervalSinceDate:self.previousLocation.timestamp];
      if (dt > 0 && dt < 10.0) {
        vel_d = (float)((self.previousLocation.altitude - latestLocation.altitude) / dt);
        if (vel_d < -20.0f) vel_d = -20.0f;
        if (vel_d > 20.0f) vel_d = 20.0f;
      }
    }
    data.vs_ms = -vel_d; // vs_ms is positive up, vel_d is positive down
  } else if (self.previousLocation) {
    // GPS speed accuracy is poor - compute velocity from position difference
    [self computeVelocityFromPositions:latestLocation vel_n:&vel_n vel_e:&vel_e vel_d:&vel_d];
    data.speed_ms = sqrtf(vel_n*vel_n + vel_e*vel_e);
    data.track_deg = atan2f(vel_e, vel_n) * 180.0f / M_PI;
    if (data.track_deg < 0.0f) data.track_deg += 360.0f;
    data.vs_ms = -vel_d;
  } else {
    data.speed_ms = 0.0f;
    data.track_deg = 0.0f;
    data.vs_ms = 0.0f;
  }
  
  // Additional sanity checks
  if (data.speed_ms > 200.0f) {
    data.valid = false;
  }
  
  // If GPS accuracy is marginal (20-50m), be more cautious
  // Indoors, even "valid" GPS can have significant errors
  if (latestLocation.horizontalAccuracy > 20.0f && latestLocation.horizontalAccuracy <= 50.0f) {
    // GPS is marginal - reduce trust in velocity measurements
    // If speed is very low but accuracy is poor, likely stationary
    if (data.speed_ms < 0.2f) {
      // Likely stationary but GPS is noisy - set speed to zero
      data.speed_ms = 0.0f;
      vel_n = 0.0f;
      vel_e = 0.0f;
      vel_d = 0.0f;
      data.vs_ms = 0.0f;
    }
  }
  
  [self.locationLock lock];
  self.locationData = data;
  [self.locationLock unlock];
  
  // Record GPS data if recording
  if (self.isRecording && data.valid) {
    // Compute velocity components from track/speed for recording
    float vel_n = data.speed_ms * cosf(data.track_deg * M_PI / 180.0f);
    float vel_e = data.speed_ms * sinf(data.track_deg * M_PI / 180.0f);
    float vel_d = -data.vs_ms; // vs_ms is positive up, vel_d is positive down
    struct __attribute__((packed)) {
      double lat;
      double lon;
      float alt;
      float speed;
      float course;
      float vel_n;
      float vel_e;
      float vel_d;
    } gpsPayload = {
      .lat = data.lat_deg,
      .lon = data.lon_deg,
      .alt = data.alt_m,
      .speed = data.speed_ms,
      .course = data.track_deg,
      .vel_n = vel_n,
      .vel_e = vel_e,
      .vel_d = vel_d,
    };
    [self writeRecordingPacket:5 timestamp:data.timestamp_us data:&gpsPayload length:sizeof(gpsPayload)];
  }
  
  self.previousLocation = latestLocation;
}

- (void)computeVelocityFromPositions:(CLLocation *)current vel_n:(float *)vel_n vel_e:(float *)vel_e vel_d:(float *)vel_d {
  if (!self.previousLocation || !current || !vel_n || !vel_e || !vel_d) {
    if (vel_n) *vel_n = 0.0f;
    if (vel_e) *vel_e = 0.0f;
    if (vel_d) *vel_d = 0.0f;
    return;
  }
  
  NSTimeInterval dt = [current.timestamp timeIntervalSinceDate:self.previousLocation.timestamp];
  
  if (dt <= 0 || dt > 10.0) {
    *vel_n = 0.0f;
    *vel_e = 0.0f;
    *vel_d = 0.0f;
    return;
  }
  
  const double earthRadius = 6378137.0;
  
  double dLat = (current.coordinate.latitude - self.previousLocation.coordinate.latitude) * M_PI / 180.0;
  double dLon = (current.coordinate.longitude - self.previousLocation.coordinate.longitude) * M_PI / 180.0;
  double latAvg = ((current.coordinate.latitude + self.previousLocation.coordinate.latitude) / 2.0) * M_PI / 180.0;
  
  double northDisplacement = dLat * earthRadius;
  double eastDisplacement = dLon * earthRadius * cos(latAvg);
  
  *vel_n = (float)(northDisplacement / dt);
  *vel_e = (float)(eastDisplacement / dt);
  *vel_d = (float)((self.previousLocation.altitude - current.altitude) / dt);
  
  if (*vel_n < -100.0f) *vel_n = -100.0f;
  if (*vel_n > 100.0f) *vel_n = 100.0f;
  if (*vel_e < -100.0f) *vel_e = -100.0f;
  if (*vel_e > 100.0f) *vel_e = 100.0f;
  if (*vel_d < -20.0f) *vel_d = -20.0f;
  if (*vel_d > 20.0f) *vel_d = 20.0f;
}

- (void)locationManager:(CLLocationManager *)manager didFailWithError:(NSError *)error {
  AHRS_LOG(@"❌ Location manager failed: %@", error.localizedDescription);
}

// iOS 14+ authorization delegate method
// This replaces the deprecated didChangeAuthorizationStatus: method
// Note: We require iOS 14.0+, so we don't need to support the old method
- (void)locationManagerDidChangeAuthorization:(CLLocationManager *)manager {
  CLAuthorizationStatus status = manager.authorizationStatus;
  // Handle authorization status changes
  // This is called when user grants/denies permission or when app becomes active
  if (self.running) {
    if (status == kCLAuthorizationStatusAuthorizedWhenInUse ||
        status == kCLAuthorizationStatusAuthorizedAlways) {
      // Permission granted - start location updates if not already started
      if ([CLLocationManager locationServicesEnabled]) {
        [self.locationManager startUpdatingLocation];
        AHRS_LOG(@"✅ Location permission granted - GPS updates started");
      }
    } else if (status == kCLAuthorizationStatusDenied ||
               status == kCLAuthorizationStatusRestricted) {
      // Permission denied - stop location updates
      [self.locationManager stopUpdatingLocation];
      AHRS_LOG(@"⚠️ Location permission denied - GPS updates stopped");
    }
    // For kCLAuthorizationStatusNotDetermined, we wait for user response
  }
}

@end
