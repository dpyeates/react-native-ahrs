#import "NativeAhrs+XPlane.h"
#import "NativeAhrs+Emission.h"
#import "NativeAhrs+Sensors.h"
#import "NativeAhrs.h"
#import <Foundation/Foundation.h>
#include <cmath>

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

// Default X-Plane WebSocket port
static const NSInteger XPLANE_PORT = 8765;

@implementation NativeAhrs (XPlane)

- (void)connectToXPlane:(NSString *)host {
  if (self.xplaneConnected) {
    AHRS_LOG(@"⚠️ Already connected to X-Plane");
    return;
  }
  
  if (!self.running) {
    AHRS_LOG(@"⚠️ Cannot connect to X-Plane: AHRS not running. Call startAhrs first.");
    return;
  }
  
  self.xplaneHost = host;
  
  // Create WebSocket URL
  NSString *urlString = [NSString stringWithFormat:@"ws://%@:%ld", host, (long)XPLANE_PORT];
  NSURL *url = [NSURL URLWithString:urlString];
  
  if (!url) {
    AHRS_LOG(@"❌ Invalid X-Plane URL: %@", urlString);
    return;
  }
  
  AHRS_LOG(@"🔌 Connecting to X-Plane at %@", urlString);
  
  // Create URL session for WebSocket
  // WebSocket connections should stay open indefinitely - use very long timeouts
  NSURLSessionConfiguration *config = [NSURLSessionConfiguration defaultSessionConfiguration];
  config.timeoutIntervalForRequest = 60.0; // Longer timeout for individual requests
  config.timeoutIntervalForResource = DBL_MAX; // Effectively infinite - WebSocket should stay open as long as connected
  
  self.xplaneSession = [NSURLSession sessionWithConfiguration:config
                                                     delegate:self
                                                delegateQueue:[NSOperationQueue mainQueue]];
  
  // Create WebSocket task
  self.xplaneWebSocketTask = [self.xplaneSession webSocketTaskWithURL:url];
  
  // Start connection
  [self.xplaneWebSocketTask resume];
  
  // Start receiving messages
  [self receiveXPlaneMessage];
}

- (void)disconnectFromXPlane {
  if (!self.xplaneConnected && !self.xplaneWebSocketTask) {
    AHRS_LOG(@"⚠️ Not connected to X-Plane");
    return;
  }
  
  AHRS_LOG(@"🔌 Disconnecting from X-Plane");
  
  // Immediately stop processing X-Plane data to prevent filter updates during transition
  BOOL wasConnected = self.xplaneConnected;
  self.xplaneConnected = NO;
  self.hasXPlaneHeading = NO;
  self.xplaneWasPaused = NO;
  
  // Close WebSocket with normal closure
  [self.xplaneWebSocketTask cancelWithCloseCode:NSURLSessionWebSocketCloseCodeNormalClosure
                                         reason:[@"User disconnected" dataUsingEncoding:NSUTF8StringEncoding]];
  
  self.xplaneWebSocketTask = nil;
  [self.xplaneSession invalidateAndCancel];
  self.xplaneSession = nil;
  self.xplaneHost = nil;
  self.xplaneReconnectAttempts = 0; // Reset reconnection attempts on manual disconnect
  
  if (wasConnected) {
    [self emitXPlaneConnectionChanged:NO];
    AHRS_LOG(@"✅ Disconnected from X-Plane - resetting AHRS for real sensors");
    // Reset AHRS filter to prevent attitude divergence when switching back to real sensors
    // Use dispatch_async to ensure this happens after any pending X-Plane message processing
    dispatch_async(dispatch_get_main_queue(), ^{
      [self resetAhrs];
      AHRS_LOG(@"✅ AHRS reset complete - ready for real sensors");
    });
  }
}

- (void)receiveXPlaneMessage {
  if (!self.xplaneWebSocketTask) {
    return;
  }
  
  __weak NativeAhrs *weakSelf = self;
  [self.xplaneWebSocketTask receiveMessageWithCompletionHandler:^(NSURLSessionWebSocketMessage * _Nullable message, NSError * _Nullable error) {
    NativeAhrs *strongSelf = weakSelf;
    if (!strongSelf) return;
    
    if (error) {
      // Check if this is a timeout error that we should attempt to recover from
      BOOL isTimeoutError = (error.code == NSURLErrorTimedOut || error.code == -1001);
      
      // Log all errors including timeouts so we can debug filter issues
      AHRS_LOG(@"❌ X-Plane WebSocket error: %@ (code: %ld) - xplaneConnected=%d",
               error.localizedDescription, (long)error.code, strongSelf.xplaneConnected ? 1 : 0);
      
      if (strongSelf.xplaneConnected) {
        // For timeout errors, the connection is likely still alive - these are often false positives
        // Just continue receiving messages without canceling the connection
        if (isTimeoutError) {
          AHRS_LOG(@"⏱️ Timeout detected but continuing - connection may still be alive");
          // Timeout errors in WebSocket are often false positives - the connection is still alive
          // Just continue receiving messages without disrupting the filter
          [strongSelf receiveXPlaneMessage];
          return; // Don't process further - just continue receiving
        } else {
          // Non-timeout error - disconnect and fall back
          strongSelf.xplaneConnected = NO;
          strongSelf.hasXPlaneHeading = NO;
          
          [strongSelf emitXPlaneConnectionChanged:NO];
          AHRS_LOG(@"⚠️ X-Plane connection error - resetting AHRS for real sensors");
          strongSelf.xplaneReconnectAttempts = 0;
          
          // Clean up connection
          [strongSelf.xplaneWebSocketTask cancel];
          strongSelf.xplaneWebSocketTask = nil;
          [strongSelf.xplaneSession invalidateAndCancel];
          strongSelf.xplaneSession = nil;
          
          // Reset filter asynchronously to avoid issues during message processing
          dispatch_async(dispatch_get_main_queue(), ^{
            [strongSelf resetAhrs];
            AHRS_LOG(@"✅ AHRS reset complete - ready for real sensors");
          });
        }
      } else {
        // Not connected - just try to continue receiving if we have a task
        if (strongSelf.xplaneWebSocketTask && isTimeoutError) {
          [strongSelf receiveXPlaneMessage];
        }
      }
      return;
    }
    
    if (message) {
      if (message.type == NSURLSessionWebSocketMessageTypeString) {
        [strongSelf handleXPlaneMessage:message.string];
      } else if (message.type == NSURLSessionWebSocketMessageTypeData) {
        // Handle binary data - try to convert to string
        NSString *messageString = [[NSString alloc] initWithData:message.data encoding:NSUTF8StringEncoding];
        if (messageString) {
          [strongSelf handleXPlaneMessage:messageString];
        } else {
          AHRS_LOG(@"⚠️ Received binary X-Plane message that couldn't be decoded as UTF-8");
        }
      }
    }
    
    // Continue receiving messages
    [strongSelf receiveXPlaneMessage];
  }];
}

- (void)handleXPlaneMessage:(NSString *)json {
  // Immediately return if not connected (prevents processing stale messages during disconnect)
  if (!self.xplaneConnected) {
    return;
  }
  
  if (!json || json.length == 0) {
    return;
  }
  
  // Parse JSON
  NSData *jsonData = [json dataUsingEncoding:NSUTF8StringEncoding];
  NSError *error = nil;
  NSDictionary *data = [NSJSONSerialization JSONObjectWithData:jsonData options:0 error:&error];
  
  if (error || !data) {
    // Log the raw JSON string for debugging (truncate if too long)
    NSString *jsonPreview = json.length > 200 ? [[json substringToIndex:200] stringByAppendingString:@"..."] : json;
    AHRS_LOG(@"❌ Failed to parse X-Plane JSON: %@", error.localizedDescription);
    AHRS_LOG(@"   Raw JSON (first 200 chars): %@", jsonPreview);
    
    // Check if it's not a dictionary (might be an array or other type)
    id parsedObject = [NSJSONSerialization JSONObjectWithData:jsonData options:0 error:nil];
    if (parsedObject && ![parsedObject isKindOfClass:[NSDictionary class]]) {
      AHRS_LOG(@"   Parsed object is %@, expected NSDictionary", NSStringFromClass([parsedObject class]));
    }
    return;
  }
  
  // Extract timestamp (Unix seconds, convert to microseconds)
  NSNumber *timestampNum = data[@"timestamp"];
  uint64_t timestamp = timestampNum ? (uint64_t)([timestampNum doubleValue] * 1e6) :
  (uint64_t)([[NSDate date] timeIntervalSince1970] * 1e6);
  
  // Check if X-Plane is paused using metadata flag
  NSDictionary *metadata = data[@"metadata"];
  BOOL isPaused = NO;
  if (metadata) {
    NSNumber *simPaused = metadata[@"sim_paused"];
    if (simPaused && [simPaused boolValue]) {
      isPaused = YES;
    }
  }
  
  // Log pause state transitions
  if (isPaused != self.xplaneWasPaused) {
    if (isPaused) {
      AHRS_LOG(@"⏸️ X-Plane PAUSED - skipping filter updates");
    } else {
      AHRS_LOG(@"▶️ X-Plane RESUMED - resuming filter updates");
    }
    self.xplaneWasPaused = isPaused;
  }
  
  if (isPaused) {
    // X-Plane is paused, skip filter update to prevent processing stale data
    return;
  }
  
  double gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
  double accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
  double mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
  BOOL hasGyro = NO, hasAccel = NO, hasMag = NO, hasGps = NO;
  double latDeg = 0.0, lonDeg = 0.0, altM = 0.0;
  double velN = 0.0, velE = 0.0, velD = 0.0;
  
  // Extract sensors data
  NSDictionary *sensors = data[@"sensors"];
  
  // Extract ground truth data for comparison
  NSDictionary *groundTruth = data[@"ground_truth"];
  double gtRoll = 0.0, gtPitch = 0.0, gtYaw = 0.0;
  double gtMagneticHeading = 0.0;
  double gtVelN = 0.0, gtVelE = 0.0, gtVelD = 0.0;
  double gtPosN = 0.0, gtPosE = 0.0, gtPosD = 0.0;
  BOOL hasGroundTruth = NO;
  BOOL hasMagneticHeading = NO;
  
  // Clear X-Plane heading if no ground truth (will be set again below if valid)
  if (!groundTruth) {
    self.hasXPlaneHeading = NO;
  }
  
  if (groundTruth) {
    // Euler angles: [roll, pitch, yaw] in rad, NED frame (yaw is TRUE heading)
    NSArray *eulerArray = groundTruth[@"euler_angles"];
    if (eulerArray && [eulerArray count] >= 3) {
      id eulerRoll = eulerArray[0];
      id eulerPitch = eulerArray[1];
      id eulerYaw = eulerArray[2];
      if (eulerRoll != [NSNull null] && eulerPitch != [NSNull null] && eulerYaw != [NSNull null]) {
        gtRoll = [eulerRoll doubleValue];
        gtPitch = [eulerPitch doubleValue];
        gtYaw = [eulerYaw doubleValue];  // TRUE heading
        hasGroundTruth = YES;
      }
    }
    
    // Magnetic heading in rad (MAGNETIC heading, not true)
    NSNumber *magHeading = groundTruth[@"magnetic_heading"];
    if (magHeading && magHeading != [NSNull null]) {
      gtMagneticHeading = [magHeading doubleValue];
      hasMagneticHeading = YES;
    }
    
    // Velocity NED: [vn, ve, vd] in m/s
    NSArray *velNedArray = groundTruth[@"velocity_ned"];
    if (velNedArray && [velNedArray count] >= 3) {
      id velN_gt = velNedArray[0];
      id velE_gt = velNedArray[1];
      id velD_gt = velNedArray[2];
      if (velN_gt != [NSNull null] && velE_gt != [NSNull null] && velD_gt != [NSNull null]) {
        gtVelN = [velN_gt doubleValue];
        gtVelE = [velE_gt doubleValue];
        gtVelD = [velD_gt doubleValue];
      }
    }
    
    // Position NED: [n, e, d] in m (relative to first sample)
    NSArray *posNedArray = groundTruth[@"position_ned"];
    if (posNedArray && [posNedArray count] >= 3) {
      id posN_gt = posNedArray[0];
      id posE_gt = posNedArray[1];
      id posD_gt = posNedArray[2];
      if (posN_gt != [NSNull null] && posE_gt != [NSNull null] && posD_gt != [NSNull null]) {
        gtPosN = [posN_gt doubleValue];
        gtPosE = [posE_gt doubleValue];
        gtPosD = [posD_gt doubleValue];
      }
    }
    
  }
  
  if (sensors) {
    // Gyroscope: [x, y, z] in rad/s, body frame X_fwd_Y_right_Z_down
    // X = forward (roll rate about forward axis = p)
    // Y = right (pitch rate about right axis = q)
    // Z = down (yaw rate about down axis = r)
    NSArray *gyroArray = sensors[@"gyroscope"];
    if (gyroArray && [gyroArray count] >= 3) {
      id gyroX = gyroArray[0];
      id gyroY = gyroArray[1];
      id gyroZ = gyroArray[2];
      if (gyroX != [NSNull null] && gyroY != [NSNull null] && gyroZ != [NSNull null]) {
        gyro_x = [gyroX doubleValue];  // p = roll rate (rad/s)
        gyro_y = [gyroY doubleValue];  // q = pitch rate (rad/s)
        gyro_z = [gyroZ doubleValue];  // r = yaw rate (rad/s)
        hasGyro = YES;
      }
    }
    
    // Accelerometer: [x, y, z] in m/s², body frame X_fwd_Y_right_Z_down
    NSArray *accelArray = sensors[@"accelerometer"];
    if (accelArray && [accelArray count] >= 3) {
      id accelX = accelArray[0];
      id accelY = accelArray[1];
      id accelZ = accelArray[2];
      if (accelX != [NSNull null] && accelY != [NSNull null] && accelZ != [NSNull null]) {
        accel_x = [accelX doubleValue];  // forward acceleration (m/s²)
        accel_y = [accelY doubleValue];  // right acceleration (m/s²)
        accel_z = [accelZ doubleValue];  // down acceleration (m/s²)
        
        // Store forward acceleration for flight phase detection
        self.lastBodyAccelX = (float)accel_x;
        hasAccel = YES;
      }
    }
    
    // Magnetometer: [x, y, z] in µT, body frame X_fwd_Y_right_Z_down
    NSArray *magArray = sensors[@"magnetometer"];
    if (magArray && [magArray count] >= 3) {
      id magX = magArray[0];
      id magY = magArray[1];
      id magZ = magArray[2];
      if (magX != [NSNull null] && magY != [NSNull null] && magZ != [NSNull null]) {
        mag_x = [magX doubleValue];  // forward magnetic field (µT)
        mag_y = [magY doubleValue];  // right magnetic field (µT)
        mag_z = [magZ doubleValue];  // down magnetic field (µT)
        hasMag = YES;
      }
    }
    
    // GPS position: [lat_deg, lon_deg, alt_m_msl]
    NSArray *gpsPosArray = sensors[@"gps_position"];
    if (gpsPosArray && [gpsPosArray count] >= 3) {
      id gpsLat = gpsPosArray[0];
      id gpsLon = gpsPosArray[1];
      id gpsAlt = gpsPosArray[2];
      if (gpsLat != [NSNull null] && gpsLon != [NSNull null] && gpsAlt != [NSNull null]) {
        latDeg = [gpsLat doubleValue];
        lonDeg = [gpsLon doubleValue];
        altM = [gpsAlt doubleValue];
        hasGps = YES;
      }
    }
    
    // GPS velocity: [vn, ve, vd] in m/s, NED frame
    NSArray *gpsVelArray = sensors[@"gps_velocity"];
    if (gpsVelArray && [gpsVelArray count] >= 3) {
      id gpsVn = gpsVelArray[0];
      id gpsVe = gpsVelArray[1];
      id gpsVd = gpsVelArray[2];
      if (gpsVn != [NSNull null] && gpsVe != [NSNull null] && gpsVd != [NSNull null]) {
        velN = [gpsVn doubleValue];  // north velocity (m/s)
        velE = [gpsVe doubleValue];  // east velocity (m/s)
        velD = [gpsVd doubleValue];  // down velocity (m/s)
      }
    }
    
    // Barometric pressure: pressure in hPa (hectopascals) from sensors/barometric_pressure
    NSNumber *baroPressure = sensors[@"barometric_pressure"];
    if (baroPressure && baroPressure != [NSNull null]) {
      float pressure_hpa = [baroPressure floatValue];
      if (pressure_hpa > 0.0f) {
        // Update barometric data from X-Plane
        baro_pressure_t baro = {
          .pressure_hpa = pressure_hpa,
          .valid = true,
          .timestamp_us = timestamp
        };
        
        [self.baroLock lock];
        self.baroData = baro;
        self.baroCalibrated = YES; // X-Plane provides absolute pressure, no calibration needed
        [self.baroLock unlock];
      }
    }
  }
  
  
  if (hasGyro && hasAccel && _filter) {
    // Check if we have new GPS data (compare lat/lon/alt to detect changes)
    if (hasGps) {
      double newLatRad = latDeg * M_PI / 180.0;
      double newLonRad = lonDeg * M_PI / 180.0;
      // Check if GPS data has changed (using small epsilon for floating point comparison)
      if (!self.hasGpsFix ||
          fabs(newLatRad - self.lastLatRad) > 1e-8 ||
          fabs(newLonRad - self.lastLonRad) > 1e-8 ||
          fabs(altM - self.lastAltM) > 0.1) {
        self.hasGpsFix = YES;
        self.lastLatRad = newLatRad;
        self.lastLonRad = newLonRad;
        self.lastAltM = altM;
        self.lastGpsTimestampUs = timestamp;
        [self updateMagneticDeclinationIfNeededForLatRad:self.lastLatRad
                                                  lonRad:self.lastLonRad
                                                    altM:self.lastAltM];
        // For X-Plane, calculate TOW as milliseconds since first GPS fix
        // This gives a reasonable TOW value for simulation (starts at 0)
        if (self.xplaneFirstGpsTimestampUs == 0) {
          self.xplaneFirstGpsTimestampUs = timestamp;
        }
        // Update TOW with new GPS data
        self.currentTow = (unsigned long)((timestamp - self.xplaneFirstGpsTimestampUs) / 1000ULL) + 1;
      }
    }
    
    // If we still have no GPS reference, skip update
    if (!self.hasGpsFix) {
      return;
    }
    
    // Calculate dt from timestamp difference
    double dt = 0.0;
    if (self.lastTimestampUs > 0) {
      dt = (double)(timestamp - self.lastTimestampUs) / 1e6;
      
      // Detect timestamp issues that could cause filter problems
      if (timestamp < self.lastTimestampUs) {
        AHRS_LOG(@"🚨 TIMESTAMP WENT BACKWARDS: %llu -> %llu (dt would be negative!)",
                 self.lastTimestampUs, timestamp);
        // Skip this update if timestamp went backwards
        return;
      }
      
      // Skip updates with unreasonably large dt values
      // Large gaps indicate missing data or network issues
      // Clamping dt causes the filter to under-integrate motion, leading to divergence
      // It's safer to skip bad updates than feed incorrect time deltas
      if (dt > 0.2) {
        // Update lastTimestampUs so the next update can be evaluated correctly
        // Otherwise we'd keep skipping forever because dt would always be large
        self.lastTimestampUs = timestamp;
        return;
      }
      
      // Clamp very small dt to prevent numerical issues
      if (dt < 1e-4) dt = 1e-4;
    }
    self.lastTimestampUs = timestamp;
    
    // Use current TOW (only updates when GPS changes - TOW is just a flag for new GPS data)
    unsigned long tow = self.currentTow;
    
    // Store X-Plane ground truth magnetic heading for emission (like iOS CLHeading)
    if (hasMagneticHeading) {
      const double rad2deg = 180.0 / M_PI;
      self.latestXPlaneHeadingDeg = gtMagneticHeading * rad2deg;
      // Normalize to [0, 360)
      while (self.latestXPlaneHeadingDeg < 0) self.latestXPlaneHeadingDeg += 360.0;
      while (self.latestXPlaneHeadingDeg >= 360.0) self.latestXPlaneHeadingDeg -= 360.0;
      self.hasXPlaneHeading = YES;
    } else if (hasGroundTruth) {
      // Fallback: Calculate magnetic heading from true heading and declination
      const double rad2deg = 180.0 / M_PI;
      double trueHeadingDeg = gtYaw * rad2deg;
      self.latestXPlaneHeadingDeg = trueHeadingDeg - self.magneticDeclination;
      // Normalize to [0, 360)
      while (self.latestXPlaneHeadingDeg < 0) self.latestXPlaneHeadingDeg += 360.0;
      while (self.latestXPlaneHeadingDeg >= 360.0) self.latestXPlaneHeadingDeg -= 360.0;
      self.hasXPlaneHeading = YES;
    }
    
    // X-Plane data is in body frame X_fwd_Y_right_Z_down per plugin spec
    // Filter initialization shows:
    //   theta = asin(ax/G) - pitch from forward acceleration
    //   phi = asin(-ay/(G*denom)) - roll from right acceleration (negated)
    //   psi uses -Byc where Byc = hy*cos(phi) - hz*sin(phi)
    // Roll is excellent (~0.03° error), pitch is good (~1.2° error)
    // Yaw initialization now uses ground truth when available
    // Double-check connection status before updating filter (prevents updates during disconnect)
    if (!self.xplaneConnected) {
      AHRS_LOG(@"⚠️ Skipping filter update - not connected");
      return;
    }
    
    // Validate filter inputs to detect bad data that could cause filter divergence
    BOOL hasBadInput = NO;
    if (!std::isfinite(dt) || dt <= 0 || dt > 1.0) {
      hasBadInput = YES;
    }
    if (!std::isfinite(velN) || std::fabs(velN) > 1000) {
      hasBadInput = YES;
    }
    if (!std::isfinite(velE) || std::fabs(velE) > 1000) {
      hasBadInput = YES;
    }
    if (!std::isfinite(velD) || std::fabs(velD) > 1000) {
      hasBadInput = YES;
    }
    if (!std::isfinite(gyro_x) || std::fabs(gyro_x) > 100) {
      hasBadInput = YES;
    }
    if (!std::isfinite(gyro_y) || std::fabs(gyro_y) > 100) {
      hasBadInput = YES;
    }
    if (!std::isfinite(gyro_z) || std::fabs(gyro_z) > 100) {
      hasBadInput = YES;
    }
    if (!std::isfinite(accel_x) || std::fabs(accel_x) > 100) {
      hasBadInput = YES;
    }
    if (!std::isfinite(accel_y) || std::fabs(accel_y) > 100) {
      hasBadInput = YES;
    }
    if (!std::isfinite(accel_z) || std::fabs(accel_z) > 100) {
      hasBadInput = YES;
    }
    
    // Skip filter update if we have bad inputs
    if (hasBadInput) {
      return;
    }
    
    // Update filter with sensor data and expected magnetic field from WMM
    _filter->update(
                    dt,
                    tow,
                    velN,
                    velE,
                    velD,
                    self.lastLatRad,
                    self.lastLonRad,
                    self.lastAltM,
                    (float)gyro_x,   // p = roll rate (rad/s, rotation about forward/X axis)
                    (float)gyro_y,   // q = pitch rate (rad/s, rotation about right/Y axis)
                    (float)gyro_z,   // r = yaw rate (rad/s, rotation about down/Z axis)
                    (float)accel_x,  // ax = forward acceleration (m/s²)
                    (float)accel_y,  // ay = right acceleration (m/s², filter will negate internally for roll)
                    (float)accel_z,  // az = down acceleration (m/s²)
                    (float)mag_x,    // hx = forward magnetic field (µT)
                    (float)mag_y,    // hy = right magnetic field (µT)
                    (float)mag_z,    // hz = down magnetic field (µT)
                    self.expectedMagN_nT,  // Expected mag field N component (nT) from WMM
                    self.expectedMagE_nT,  // Expected mag field E component (nT) from WMM
                    self.expectedMagD_nT   // Expected mag field D component (nT) from WMM
                    );
    
    self.filterInitialized = _filter->isInitialized();
    self.xplaneConnected = YES;
    self.xplaneReconnectAttempts = 0; // Reset reconnection attempts on successful connection
    [self emitXPlaneConnectionChanged:YES];
    [self emitAhrsUpdateWithTimestamp:timestamp];
  }
}

- (void)emitXPlaneConnectionChanged:(BOOL)connected {
  [self emitXPlaneConnectionChangedWithConnected:connected host:self.xplaneHost];
}

#pragma mark - NSURLSessionWebSocketDelegate

- (void)URLSession:(NSURLSession *)session webSocketTask:(NSURLSessionWebSocketTask *)webSocketTask didOpenWithProtocol:(NSString *)protocol {
  self.xplaneConnected = YES;
  self.xplaneReconnectAttempts = 0; // Reset reconnection attempts on successful connection
  self.xplaneWasPaused = NO;
  [self emitXPlaneConnectionChanged:YES];
  AHRS_LOG(@"✅ Connected to X-Plane WebSocket");
  
  // Reset filter and all related state for clean start with X-Plane data
  // This ensures flight phase detector and filter are all reset
  [self resetAhrs];
  
  // Reset X-Plane-specific state
  self.xplaneFirstGpsTimestampUs = 0; // Reset first GPS timestamp
  
  AHRS_LOG(@"🎮 X-Plane mode active - real sensors bypassed");
}

- (void)URLSession:(NSURLSession *)session webSocketTask:(NSURLSessionWebSocketTask *)webSocketTask didCloseWithCode:(NSURLSessionWebSocketCloseCode)closeCode reason:(NSData *)reason {
  NSString *reasonStr = reason ? [[NSString alloc] initWithData:reason encoding:NSUTF8StringEncoding] : @"Unknown";
  AHRS_LOG(@"🔌 X-Plane WebSocket closed (code: %ld, reason: %@)", (long)closeCode, reasonStr);
  
  // Immediately stop processing X-Plane data
  BOOL wasConnected = self.xplaneConnected;
  self.xplaneConnected = NO;
  self.hasXPlaneHeading = NO;
  
  if (wasConnected) {
    // Check if we should attempt reconnection (for timeout/network errors)
    // Only attempt reconnection if we have a host and haven't exceeded attempts
    BOOL shouldReconnect = (self.xplaneHost && self.xplaneReconnectAttempts < 5 &&
                            (closeCode == NSURLSessionWebSocketCloseCodeAbnormalClosure ||
                             closeCode == NSURLSessionWebSocketCloseCodeGoingAway));
    
    if (shouldReconnect) {
      self.xplaneReconnectAttempts++;
      AHRS_LOG(@"🔄 X-Plane WebSocket closed - attempting reconnection (%d/5)", self.xplaneReconnectAttempts);
      
      // Clean up current connection
      self.xplaneWebSocketTask = nil;
      [self.xplaneSession invalidateAndCancel];
      self.xplaneSession = nil;
      
      // Attempt reconnection after a short delay
      dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(2.0 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
        if (self.xplaneHost && !self.xplaneConnected) {
          AHRS_LOG(@"🔄 Reconnecting to X-Plane...");
          [self connectToXPlane:self.xplaneHost];
        }
      });
    } else {
      [self emitXPlaneConnectionChanged:NO];
      AHRS_LOG(@"✅ X-Plane WebSocket closed - resetting AHRS for real sensors");
      self.xplaneReconnectAttempts = 0;
      // Reset AHRS filter to prevent attitude divergence when switching back to real sensors
      // Use dispatch_async to ensure this happens after any pending message processing
      dispatch_async(dispatch_get_main_queue(), ^{
        [self resetAhrs];
        AHRS_LOG(@"✅ AHRS reset complete - ready for real sensors");
      });
    }
  }
}

- (void)URLSession:(NSURLSession *)session task:(NSURLSessionTask *)task didCompleteWithError:(NSError *)error {
  if (error) {
    BOOL wasConnected = self.xplaneConnected;
    if (wasConnected) {
      // Immediately stop processing X-Plane data
      self.xplaneConnected = NO;
      self.hasXPlaneHeading = NO;
      
      // Check if this is a timeout error that we should attempt to recover from
      BOOL isTimeoutError = (error.code == NSURLErrorTimedOut || error.code == -1001);
      
      if (isTimeoutError && self.xplaneHost && self.xplaneReconnectAttempts < 5) {
        // Attempt to reconnect for timeout errors (up to 5 attempts)
        self.xplaneReconnectAttempts++;
        // Only log if we're getting multiple timeouts (3+) to reduce noise
        if (self.xplaneReconnectAttempts >= 3) {
          AHRS_LOG(@"⏱️ X-Plane connection timeout - attempting reconnection (%d/5)", self.xplaneReconnectAttempts);
        }
        
        // Clean up current connection
        self.xplaneWebSocketTask = nil;
        [self.xplaneSession invalidateAndCancel];
        self.xplaneSession = nil;
        
        // Attempt reconnection after a short delay
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(2.0 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
          if (self.xplaneHost && !self.xplaneConnected) {
            AHRS_LOG(@"🔄 Reconnecting to X-Plane...");
            [self connectToXPlane:self.xplaneHost];
          }
        });
      } else {
        // Non-timeout error or too many reconnection attempts - fall back to real sensors
        [self emitXPlaneConnectionChanged:NO];
        if (isTimeoutError) {
          AHRS_LOG(@"⚠️ X-Plane reconnection failed after %d attempts - resetting AHRS for real sensors", self.xplaneReconnectAttempts);
        } else {
          AHRS_LOG(@"✅ X-Plane connection error - resetting AHRS for real sensors");
        }
        self.xplaneReconnectAttempts = 0;
        // Reset AHRS filter to prevent attitude divergence when switching back to device sensors
        // Use dispatch_async to ensure this happens after any pending message processing
        dispatch_async(dispatch_get_main_queue(), ^{
          [self resetAhrs];
          AHRS_LOG(@"✅ AHRS reset complete - ready for real sensors");
        });
      }
    }
  }
}

@end

