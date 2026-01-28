
#import "NativeAhrs.h"
#include "../fusionml/src/FlightPhaseDetector.h"
#import "NativeAhrs+Emission.h"
#import "NativeAhrs+Location.h"
#import "NativeAhrs+Recording.h"
#import "NativeAhrs+Sensors.h"
#import "NativeAhrs+Transform.h"
#import "NativeAhrs+XPlane.h"

@implementation NativeAhrs

RCT_EXPORT_MODULE()

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

/* =============================================================================
 * INITIALIZATION
 * =============================================================================
 */

/**
 * @brief Initialize the AHRS module
 *
 * Creates and configures all sensor managers (Core Motion, Core Location,
 * Altimeter), initializes the EKF filter, and sets up thread synchronization
 * locks.
 *
 * Initialization order:
 * 1. Create location manager and configure settings
 * 2. Create motion manager and verify device motion availability
 * 3. Create altimeter (warns if not available, but continues)
 * 4. Configure sensor update intervals (60Hz for IMU)
 * 5. Create synchronization locks (locationLock, baroLock)
 * 6. Initialize all state variables to defaults
 * 7. Create EKF filter and flight phase detector
 *
 * @return Initialized NativeAhrs instance, or nil if device motion is
 * unavailable
 */
- (instancetype)init {
  self = [super init];
  if (self) {
    
    _locationManager = [[CLLocationManager alloc] init];
    _locationManager.delegate = self;
    
    _rotation = AhrsRotationVertical;
    
    // Configure location manager settings
    _locationManager.desiredAccuracy = kCLLocationAccuracyBest;
    _locationManager.distanceFilter = kCLDistanceFilterNone;
    _locationManager.allowsBackgroundLocationUpdates = NO;
    _locationManager.pausesLocationUpdatesAutomatically = NO;
    _locationManager.activityType = CLActivityTypeOtherNavigation;
    
    // Don't request permission in init - wait until startAhrs is called
    // This follows iOS best practices: request permissions when needed, not at
    // initialization
    
    _motionManager = [[CMMotionManager alloc] init];
    
    if (!_motionManager.deviceMotionAvailable) {
      AHRS_LOG(@"⚠️ Device Motion not available on this device");
      return nil;
    }
    
    _altimeter = [[CMAltimeter alloc] init];
    
    if (![CMAltimeter isRelativeAltitudeAvailable]) {
      AHRS_LOG(@"⚠️ Relative altitude not available on this device");
    }
    
    _motionManager.deviceMotionUpdateInterval = INTERVAL;
    _motionManager.accelerometerUpdateInterval = INTERVAL;
    _motionManager.gyroUpdateInterval = INTERVAL;
    _motionManager.magnetometerUpdateInterval = INTERVAL;
    
    _locationLock = [[NSLock alloc] init];
    _baroLock = [[NSLock alloc] init];
    
    _running = false;
    _emitRateHz = 5.0;
    _nextEmitTime = 0;
    _lastUsedLocationTimestamp = 0;
    _lastUsedBaroTimestamp = 0;
    _previousLocation = nil;
    
    _baroCalibrated = false;
    _baroPressureOffset = 0.0f;
    
    _waitingForInitialHeading = false;
    _initialHeadingFromCL = -1.0f; // -1 means not yet received
    _latestIosHeadingDeg = 0.0f;
    _hasIosHeading = NO;
    _hasXPlaneHeading = NO;
    _latestXPlaneHeadingDeg = 0.0;
    
    _isRecording = false;
    _recordingFileHandle = nil;
    _recordingStartTimestamp = 0;
    _recordingPacketCount = 0;
    _isPlaying = false;
    _playbackTimer = nil;
    _playbackPackets = nil;
    _playbackCurrentIndex = 0;
    
    // X-Plane WebSocket connection
    _xplaneSession = nil;
    _xplaneWebSocketTask = nil;
    _xplaneConnected = false;
    _xplaneHost = nil;
    _xplaneReconnectAttempts = 0;
    _xplaneWasPaused = NO;
    
    _filter = std::make_unique<uNavINS>();
    _flightPhaseDetector = new FlightPhaseDetector();
    _hasGpsFix = NO;
    _lastLatRad = 0.0;
    _lastLonRad = 0.0;
    _lastAltM = 0.0;
    _rollOffsetDeg = 0.0;
    _pitchOffsetDeg = 0.0;
    _lastTimestampUs = 0;
    _lastGpsTimestampUs = 0;
    _currentTow = 0;
    _filterInitialized = NO;
    _lastValidGroundTrack = 0.0f;
    _hasValidGroundTrack = NO;
    _lastDeclinationLatRad = 0.0;
    _lastDeclinationLonRad = 0.0;
    _expectedMagN_nT = std::numeric_limits<float>::quiet_NaN();
    _expectedMagE_nT = std::numeric_limits<float>::quiet_NaN();
    _expectedMagD_nT = std::numeric_limits<float>::quiet_NaN();
    _groundElevationM = std::numeric_limits<float>::quiet_NaN();
    
    AHRS_LOG(@"✅ AHRS initialized successfully");
  }
  return self;
}

/**
 * @brief Cleanup when object is deallocated
 *
 * Ensures all sensors are stopped and resources are released:
 * - Stops all sensor updates (via stopAhrs)
 * - Releases EKF filter (unique_ptr automatically handles deletion)
 * - Deletes flight phase detector
 *
 * Called automatically by ARC when object reference count reaches zero.
 */
- (void)dealloc {
  [self stopAhrs];
  _filter.reset();
  if (_flightPhaseDetector) {
    delete _flightPhaseDetector;
    _flightPhaseDetector = nullptr;
  }
}

/**
 * @brief Cleanup when React Native module is invalidated
 *
 * Called by React Native when the bridge is torn down or module is removed.
 * Ensures complete cleanup of all resources:
 * - Stops AHRS processing
 * - Stops any active recording
 * - Stops any active playback
 * - Disconnects from X-Plane (if connected)
 * - Releases EKF filter
 *
 * More comprehensive than dealloc as it handles React Native-specific cleanup.
 */
- (void)invalidate {
  [self stopAhrs];
  [self stopRecording];
  [self stopPlayback];
  [self disconnectFromXPlane];
  _filter.reset();
}

/* =============================================================================
 * START/STOP METHODS
 * =============================================================================
 */

/**
 * Starts AHRS sensor processing
 *
 * Begins collecting and fusing sensor data at 60Hz internally.
 * Data is emitted to React Native at the configured rate (default 5Hz).
 *
 * Initializes:
 * - Device Motion updates (gyro + accel + mag) at 60Hz
 * - Barometer updates at ~1-2Hz
 * - GPS location updates (if permissions granted)
 * - Heading updates (if available, for initial attitude)
 *
 * Requests location permissions if not yet determined.
 * Continues without GPS if permissions denied (IMU-only mode).
 *
 * Threading: All sensor callbacks run on main queue (iOS requirement).
 */
RCT_EXPORT_METHOD(startAhrs) {
  // Early return if already running to avoid duplicate sensor callbacks
  if (self.running) {
    AHRS_LOG(@"⚠️ AHRS already running");
    return;
  }
  
  // ===== GPS Permission Handling =====
  // Check and request location permissions following iOS best practices
  CLAuthorizationStatus authStatus = [self.locationManager authorizationStatus];
  
  if (authStatus == kCLAuthorizationStatusNotDetermined) {
    // Path 1: Permission not yet requested
    // Request permission now - delegate method will start location updates when
    // granted
    [self.locationManager requestWhenInUseAuthorization];
    // Note: We'll start location updates in
    // locationManagerDidChangeAuthorization: delegate method
    
  } else if (authStatus == kCLAuthorizationStatusDenied ||
             authStatus == kCLAuthorizationStatusRestricted) {
    // Path 2: Permission denied or restricted by system/parental controls
    AHRS_LOG(@"⚠️ Location permission denied or restricted");
    // Continue without GPS - AHRS can still work with IMU only (attitude-only
    // mode)
    
  } else if (authStatus == kCLAuthorizationStatusAuthorizedWhenInUse ||
             authStatus == kCLAuthorizationStatusAuthorizedAlways) {
    // Path 3: Permission already granted
    // Start location updates immediately
    if ([CLLocationManager locationServicesEnabled]) {
      [self.locationManager startUpdatingLocation];
      
      // Start heading updates continuously (for heading output to React Native)
      if ([CLLocationManager headingAvailable]) {
        [self.locationManager startUpdatingHeading];
        AHRS_LOG(@"📍 Starting heading updates for CLHeading output");
      } else {
        // Heading not available on this device (e.g., iPad without
        // magnetometer)
        AHRS_LOG(@"⚠️ Heading not available - will use filter yaw for heading");
      }
    } else {
      // Location services disabled system-wide (Settings > Privacy > Location
      // Services)
      AHRS_LOG(@"⚠️ Location services not enabled");
    }
  }
  
  // ===== Start IMU Sensor Updates =====
  // Device Motion provides fused gyro + accel + mag at 60Hz
  // All callbacks run on main queue (iOS requirement for Core Motion)
  [_motionManager
   startDeviceMotionUpdatesUsingReferenceFrame:
     CMAttitudeReferenceFrameXMagneticNorthZVertical
   toQueue:[NSOperationQueue mainQueue]
   withHandler:^(CMDeviceMotion *motionData,
                 NSError *error) {
    if (error) {
      // Error path: Log and skip this
      // update
      AHRS_LOG(
               @"❌ Device motion error: %@",
               error.localizedDescription);
      return;
    }
    if (motionData) {
      // Success path: Process sensor data
      // and update EKF
      [self ProcessDeviceMotionData:
       motionData];
    }
  }];
  
  // ===== Start Barometer Updates =====
  // Barometer provides relative pressure at ~1-2Hz
  // Must be calibrated against GPS altitude to get absolute pressure
  [self.altimeter
   startRelativeAltitudeUpdatesToQueue:[NSOperationQueue mainQueue]
   withHandler:^(
                 CMAltitudeData *_Nullable altitudeData,
                 NSError *_Nullable error) {
                   if (error) {
                     // Error path: Log and skip this update
                     AHRS_LOG(@"❌ Altimeter error: %@",
                              error.localizedDescription);
                     return;
                   }
                   if (altitudeData) {
                     // Success path: Process barometer data and
                     // calibrate if GPS available
                     [self ProcessAltitudeData:altitudeData];
                   }
                 }];
  
  // Mark as running - sensor callbacks will now process data
  self.running = YES;
  AHRS_LOG(@"✅ AHRS started");
}

/**
 * Stops AHRS sensor processing
 *
 * Halts all sensor updates to save battery:
 * - Stops Device Motion updates
 * - Stops barometer updates
 * - Stops GPS location updates
 * - Stops heading updates
 *
 * Resets state flags (filterInitialized, waitingForInitialHeading).
 * Can be restarted later with startAhrs().
 */
RCT_EXPORT_METHOD(stopAhrs) {
  if (!self.running) {
    AHRS_LOG(@"⚠️ AHRS not running");
    return;
  }
  
  [self.locationManager stopUpdatingLocation];
  [self.locationManager stopUpdatingHeading];
  [self.motionManager stopDeviceMotionUpdates];
  [self.altimeter stopRelativeAltitudeUpdates];
  
  self.nextEmitTime = 0;
  self.running = NO;
  
  self.filterInitialized = NO;
  self.waitingForInitialHeading = false;
  self.initialHeadingFromCL = -1.0f;
  
  AHRS_LOG(@"✅ AHRS stopped");
}

/* =============================================================================
 * CORE EXPORTED METHODS
 * =============================================================================
 */

/**
 * Resets the EKF filter to initial state
 *
 * Clears all state estimates (attitude, velocity, position, biases, wind)
 * and resets covariances. The filter will re-initialize from current
 * sensor readings on the next update. Requires 2-3 seconds for reconvergence.
 *
 * Thread-safe: Uses ekfLock to protect EKF state during reset.
 *
 * Also resets filterInitialized flag to force attitude re-initialization.
 */
RCT_EXPORT_METHOD(resetAhrs) {
  // Early return if filter not available
  if (!_filter) {
    return;
  }
  
  AHRS_LOG(@"🔄 resetAhrs called");
  
  // ===== Reset EKF Filter =====
  // Create new filter instance - this clears all state (attitude, velocity,
  // position, biases, covariances)
  _filter = std::make_unique<uNavINS>();
  
  // ===== Reset Flight Phase Detector =====
  // Clear flight phase state (takeoff, cruise, landing, etc.)
  if (_flightPhaseDetector) {
    _flightPhaseDetector->reset();
  }
  
  // ===== Reset GPS State =====
  // Clear GPS fix flag - filter will need new GPS data to reinitialize position
  _hasGpsFix = NO;
  _lastGpsTimestampUs = 0;
  _currentTow = 0;
  
  // ===== Reset Timestamp Tracking =====
  // Clear last timestamp to force recalculation of dt on next update
  _lastTimestampUs = 0;
  
  // ===== Reset Initialization Flags =====
  // Force filter to reinitialize from next sensor update
  _filterInitialized = NO;
  
  AHRS_LOG(@"✅ AHRS reset - waiting for new sensor updates");
}

/**
 * Levels the attitude reference
 *
 * Captures the current attitude as the "zero" reference.
 * Sets roll=0° and pitch=0° based on current orientation.
 * Does not reset heading or other states.
 *
 * Thread-safe: Uses ekfLock to protect EKF state.
 *
 * Call this when the device is level to establish the reference attitude.
 */
RCT_EXPORT_METHOD(levelAhrs) {
  // Early return if filter not available
  if (!_filter) {
    return;
  }
  
  AHRS_LOG(@"📏 levelAhrs called");
  
  // ===== Capture Current Attitude as Zero Reference =====
  // Get current roll and pitch from filter (in radians)
  const double rad2deg = 180.0 / M_PI;
  double roll = _filter->getRoll_rad() * rad2deg;
  double pitch = _filter->getPitch_rad() * rad2deg;
  
  // Store as offsets - these will be subtracted from filter output in emission
  // This establishes the current orientation as "level" (roll=0°, pitch=0°)
  _rollOffsetDeg = roll;
  _pitchOffsetDeg = pitch;
  
  AHRS_LOG(@"✅ AHRS leveled");
}

/**
 * Sets the output emission rate
 *
 * Controls how often data is sent to React Native JavaScript.
 * Independent of internal sensor rate (60Hz).
 * Lower rates save battery and reduce JavaScript load.
 *
 * @param newRate - Rate in Hz, range [1, 60]. Values outside range are ignored.
 *                  Default: 5 Hz
 */
RCT_EXPORT_METHOD(setAhrsRate : (double)newRate) {
  // Validate rate is within acceptable range [1, 60] Hz
  // Values outside range are silently ignored (no error logged)
  if (newRate >= 1 && newRate <= 60) {
    self.emitRateHz = newRate;
  }
  // Note: Rate change takes effect on next emission cycle
}

/**
 * Sets device rotation/mounting orientation
 *
 * Configures coordinate frame transformations from iOS device sensors
 * to aviation body frame. Must match physical device mounting.
 *
 * Supported values:
 * - "left", "landscape_left", "landscapeleft" -> Landscape Left (90° CCW)
 * - "right", "landscape_right", "landscaperight" -> Landscape Right (90° CW)
 * - "none", "vertical", "portrait" -> Portrait/Vertical (default)
 *
 * When rotation changes, automatically:
 * - Resets the AHRS filter (calls resetAhrs)
 * - Restarts heading updates for reinitialization (if running)
 *
 * @param newRotation - Rotation string (case-insensitive)
 */
RCT_EXPORT_METHOD(setAhrsRotation : (NSString *)newRotation) {
  // Normalize input to lowercase for case-insensitive comparison
  NSString *rotation = [newRotation lowercaseString];
  AhrsRotation oldRotation = self.rotation;
  
  // ===== Parse Rotation String =====
  // Path 1: Landscape Left (90° CCW from portrait)
  if ([rotation isEqualToString:@"left"] ||
      [rotation isEqualToString:@"landscape_left"] ||
      [rotation isEqualToString:@"landscapeleft"]) {
    self.rotation = AhrsRotationLeft;
    AHRS_LOG(@"✅ AHRS rotation set to: Landscape Left");
    
    // Path 2: Landscape Right (90° CW from portrait)
  } else if ([rotation isEqualToString:@"right"] ||
             [rotation isEqualToString:@"landscape_right"] ||
             [rotation isEqualToString:@"landscaperight"]) {
    self.rotation = AhrsRotationRight;
    AHRS_LOG(@"✅ AHRS rotation set to: Landscape Right");
    
    // Path 3: Default to Vertical/Portrait (any unrecognized value)
  } else {
    self.rotation = AhrsRotationVertical;
    AHRS_LOG(@"✅ AHRS rotation set to: Vertical/Portrait");
  }
  
  // ===== Reset Filter if Rotation Changed =====
  // Coordinate frame transformation changed - must reset filter to prevent
  // attitude divergence from incorrect frame transformations
  if (oldRotation != self.rotation) {
    AHRS_LOG(@"🔄 setAhrsRotation changed (%ld -> %ld) -> calling resetAhrs",
             (long)oldRotation, (long)self.rotation);
    [self resetAhrs];
    
    // Restart heading updates if AHRS is running to reinitialize attitude
    if (self.running && [CLLocationManager locationServicesEnabled]) {
      if ([CLLocationManager headingAvailable]) {
        [self.locationManager startUpdatingHeading];
        AHRS_LOG(@"🔄 Rotation changed - restarting heading updates");
      }
    }
  }
}

/**
 * Checks if AHRS is supported on this device
 *
 * Verifies that all required sensors are available:
 * - Device Motion (fused IMU)
 * - Accelerometer
 * - Gyroscope
 * - Magnetometer
 *
 * @param resolve - Promise resolver, called with @YES if supported, @NO
 * otherwise
 * @param reject - Promise rejecter (unused, never rejects)
 */
RCT_EXPORT_METHOD(isSupported : (RCTPromiseResolveBlock)
                  resolve reject : (RCTPromiseRejectBlock)reject) {
  BOOL supported = self.motionManager.isDeviceMotionAvailable &&
  self.motionManager.isAccelerometerAvailable &&
  self.motionManager.isGyroAvailable &&
  self.motionManager.isMagnetometerAvailable;
  resolve(@(supported));
}

/**
 * Starts recording sensor data to a file using an auto-generated filename.
 */
RCT_EXPORT_METHOD(startRecording) { [self startRecording]; }

/**
 * Stops recording and closes the file
 */
RCT_EXPORT_METHOD(stopRecording) { [self stopRecording]; }

/**
 * Starts playback of a recorded file
 *
 * @param filename - Name of the file to play back
 */
RCT_EXPORT_METHOD(playbackRecording : (NSString *)filename) {
  [self playbackRecording:filename];
}

/**
 * Stops playback
 */
RCT_EXPORT_METHOD(stopPlayback) { [self stopPlayback]; }

/**
 * Gets list of recording files
 *
 * @param resolve - Promise resolver with array of file info
 * @param reject - Promise rejecter
 */
RCT_EXPORT_METHOD(getRecordingFiles : (RCTPromiseResolveBlock)
                  resolve reject : (RCTPromiseRejectBlock)reject) {
  [self getRecordingFiles:resolve reject:reject];
}

/**
 * Deletes a recording file
 *
 * @param filename - Name of the file to delete
 */
RCT_EXPORT_METHOD(deleteRecording : (NSString *)filename) {
  [self deleteRecording:filename];
}

/**
 * Sets QNH pressure for altitude correction
 *
 * QNH is local sea-level pressure used by aviation.
 * Required for accurate altitude calculations from barometric pressure.
 *
 * @param qnh - Pressure in hectopascals (hPa = mbar)
 *              Range: [900, 1100] hPa
 *              Standard: 1013.25 hPa
 *
 * Thread-safe: Uses ekfLock to protect EKF state.
 */
RCT_EXPORT_METHOD(setQNH : (double)qnh) {
  _qnh = qnh;
  AHRS_LOG(@"✅ QNH set to %.2f hPa", qnh);
}

/**
 * Sets ground elevation above sea level
 *
 * Provides an estimate of current ground elevation (MSL) to the
 * FlightPhaseDetector. Used to calculate accurate altitude above ground (AGL)
 * for flight phase detection.
 *
 * If not set (default NAN), FlightPhaseDetector will use GPS altitude only.
 * Setting this improves accuracy of takeoff/landing detection and ground
 * proximity calculations.
 *
 * @param elevationM - Ground elevation in meters above sea level (MSL)
 *                      Typical range: [-100, 5000] meters
 *                      Pass NAN to clear/reset (use GPS altitude only)
 *
 * Thread-safe: Simple assignment, no locks needed.
 */
RCT_EXPORT_METHOD(setGroundElevation : (double)elevationM) {
  _groundElevationM = (float)elevationM;
  if (std::isnan(_groundElevationM)) {
    AHRS_LOG(@"✅ Ground elevation cleared (using GPS altitude only)");
  } else {
    AHRS_LOG(@"✅ Ground elevation set to %.1f m MSL", _groundElevationM);
  }
}

/**
 * Checks if position estimate is reliable
 *
 * Considers:
 * - GPS fix availability (hasGpsFix)
 * - GPS data freshness (not stale - within last 10 seconds)
 * - Filter initialization state
 * - GPS accuracy (if available from locationData)
 *
 * Position is considered reliable if:
 * - GPS fix is available
 * - GPS data is recent (within 10 seconds)
 * - Filter has been initialized
 * - GPS accuracy is acceptable (if available)
 *
 * @param resolve - Promise resolver, called with @YES if reliable, @NO
 * otherwise
 * @param reject - Promise rejecter (unused, never rejects)
 *
 * Thread-safe: Uses locationLock to protect locationData during read.
 */
RCT_EXPORT_METHOD(isPositionReliable : (RCTPromiseResolveBlock)
                  resolve reject : (RCTPromiseRejectBlock)reject) {
  // ===== Check 1: GPS Fix Availability =====
  // Must have received at least one valid GPS fix
  if (!self.hasGpsFix) {
    resolve(@(NO));
    return;
  }
  
  // ===== Check 2: Filter Initialization =====
  // Filter must have processed at least one sensor update
  // Uninitialized filter has invalid position estimates
  if (!self.filterInitialized) {
    resolve(@(NO));
    return;
  }
  
  // ===== Check 3: GPS Data Freshness =====
  // GPS data must be recent (within last 10 seconds)
  // Stale GPS data indicates GPS outage or poor signal
  uint64_t currentTimeUs =
  (uint64_t)([[NSDate date] timeIntervalSince1970] * 1e6);
  const uint64_t MAX_GPS_AGE_US =
  10ULL * 1000000ULL; // 10 seconds in microseconds
  
  if (self.lastGpsTimestampUs > 0) {
    uint64_t gpsAgeUs = currentTimeUs - self.lastGpsTimestampUs;
    if (gpsAgeUs > MAX_GPS_AGE_US) {
      // Path: GPS data is stale (older than 10 seconds)
      resolve(@(NO));
      return;
    }
  } else {
    // Path: No GPS timestamp available (shouldn't happen if hasGpsFix is YES,
    // but check anyway)
    resolve(@(NO));
    return;
  }
  
  // ===== Check 4: GPS Accuracy =====
  // GPS accuracy must meet thresholds (5m horizontal, 10m vertical)
  // Uses shared helper method to avoid code duplication
  [self.locationLock lock];
  gps_position_t gps = self.locationData;
  [self.locationLock unlock];
  
  if (gps.valid && ![self hasAcceptableGpsAccuracy:gps]) {
    // Path: GPS accuracy is too poor (exceeds thresholds)
    resolve(@(NO));
    return;
  }
  
  // ===== All Checks Passed =====
  // Position is reliable: GPS fix available, filter initialized, data fresh,
  // accuracy acceptable
  resolve(@(YES));
}

/* =============================================================================
 * X-PLANE WEBSOCKET CONNECTION METHODS
 * =============================================================================
 */

/**
 * Connects to X-Plane plugin via WebSocket
 *
 * When connected, real device sensors are bypassed and X-Plane data feeds the
 * EKF. Requires AHRS to be running (call startAhrs first).
 *
 * @param host - Hostname or IP address of X-Plane computer (e.g.,
 * "192.168.1.100")
 */
RCT_EXPORT_METHOD(connectToXPlane : (NSString *)host) {
  [self connectToXPlane:host];
}

/**
 * Disconnects from X-Plane plugin
 *
 * Closes WebSocket connection and returns to using real device sensors.
 */
RCT_EXPORT_METHOD(disconnectFromXPlane) { [self disconnectFromXPlane]; }

/* =============================================================================
 * TURBO MODULE PROTOCOL
 * =============================================================================
 */

/**
 * @brief Returns the Turbo Module implementation for React Native
 *
 * Required by React Native's Turbo Module system (New Architecture).
 * Creates and returns the JSI (JavaScript Interface) implementation
 * that bridges JavaScript calls to native Objective-C methods.
 *
 * This method is called automatically by React Native's module system.
 * Do not call directly.
 *
 * @param params - Turbo Module initialization parameters from React Native
 * @return Shared pointer to the Turbo Module JSI implementation
 */
- (std::shared_ptr<facebook::react::TurboModule>)getTurboModule:
(const facebook::react::ObjCTurboModule::InitParams &)params {
  return std::make_shared<facebook::react::NativeAhrsSpecJSI>(params);
}

@end
