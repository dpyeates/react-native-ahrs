#import "uNavINS.h"
#import <CoreLocation/CoreLocation.h>
#import <CoreMotion/CoreMotion.h>
#import <NativeAhrsSpec/NativeAhrsSpec.h>
#import <React/RCTComponent.h>
#include <memory>

// Forward declarations
class FlightPhaseDetector;

typedef struct {
  bool valid;
  uint64_t timestamp_us;
  double lat_deg;
  double lon_deg;
  float alt_m;
  float speed_ms;
  float track_deg;
  float vs_ms;
  float horizontalAccuracy_m; // Horizontal accuracy in meters (negative = invalid)
  float verticalAccuracy_m; // Vertical accuracy in meters (negative = invalid)
} gps_position_t;

typedef struct {
  bool valid;
  uint64_t timestamp_us;
  float pressure_hpa;
} baro_pressure_t;

// Constants defining update rate
static const NSInteger FRAME_RATE = 60; // Hz - IMU sensor update rate
static const NSTimeInterval INTERVAL =
(1.0 / (double)FRAME_RATE); // Seconds between updates

NS_ASSUME_NONNULL_BEGIN

/**
 * @brief Device rotation/mounting orientation
 *
 * Defines how the device is physically mounted relative to aircraft.
 * Affects coordinate frame transformations from iOS sensors to aviation body
 * frame.
 */
typedef NS_ENUM(NSInteger, AhrsRotation) {
  AhrsRotationVertical = 0, // Portrait: top edge up (default, VR goggle style)
  AhrsRotationLeft = 1,     // Landscape left: rotated 90° CCW from portrait
  AhrsRotationRight = 2     // Landscape right: rotated 90° CW from portrait
};

/**
 * @interface NativeAhrs
 * @brief React Native Turbo Module providing AHRS functionality on iOS
 *
 * This module manages:
 * - iOS sensor data collection (IMU, GPS, barometer)
 * - Coordinate frame transformations
 * - EKF sensor fusion
 * - Data emission to React Native at configurable rate
 *
 * Threading:
 * - All sensor callbacks run on main queue (iOS requirement)
 * - EKF operations protected by locks for thread safety
 * - Location and barometer data cached and accessed via locks
 *
 * Lifecycle:
 * - init: Creates managers, initializes EKF
 * - startAhrs: Begins sensor data collection
 * - stopAhrs: Stops sensors
 * - dealloc/invalidate: Cleanup and destroy EKF
 */
@interface NativeAhrs
: NativeAhrsSpecBase <NativeAhrsSpec, CLLocationManagerDelegate> {
  std::unique_ptr<uNavINS> _filter;
  FlightPhaseDetector *_flightPhaseDetector;
}

/* ===== Core Motion Managers ===== */

/**
 * @property motionManager
 * @brief Core Motion manager for accessing IMU sensors
 *
 * Provides:
 * - Device Motion (fused gyro + accel + mag from iOS)
 * - Raw gyroscope data (rad/s)
 * - Raw accelerometer data (g's)
 * - Raw magnetometer data (µT)
 *
 * Update rate: 60Hz (configured in init)
 */
@property(nonatomic, strong) CMMotionManager *motionManager;

/**
 * @property locationManager
 * @brief Core Location manager for GPS data
 *
 * Provides:
 * - Position (latitude, longitude, altitude)
 * - Velocity (speed, course)
 * - Quality metrics (horizontal/vertical accuracy)
 *
 * Update rate: Variable (1-10 Hz depending on motion)
 * Accuracy: kCLLocationAccuracyBest (typically 5-10m)
 */
@property(strong, nonatomic) CLLocationManager *locationManager;

/**
 * @property altimeter
 * @brief CMAltimeter for barometric pressure/altitude
 *
 * Provides:
 * - Relative pressure (kPa)
 * - Relative altitude changes (meters)
 *
 * Note: iOS provides RELATIVE altitude, not absolute
 * Requires calibration against GPS altitude
 *
 * Update rate: ~1-2 Hz
 */
@property(nonatomic, strong) CMAltimeter *altimeter;

/* ===== State and Configuration ===== */

/**
 * @property nextEmitTime
 * @brief Timestamp (microseconds) when next data should be emitted to React
 * Native
 *
 * Used to throttle output rate independently of sensor rate.
 * Default: 5 Hz (200ms between emissions)
 */
@property(nonatomic, assign) uint64_t nextEmitTime;

/**
 * @property previousLocation
 * @brief Last GPS location, used to compute velocity from position changes
 *
 * iOS doesn't always provide velocity directly. When unavailable, we compute
 * velocity from consecutive position measurements using finite differences.
 *
 * Stored to enable: velocity = (current_pos - prev_pos) / dt
 */
@property(nonatomic, strong) CLLocation *previousLocation;

/**
 * @property filterInitialized
 * @brief Flag indicating whether EKF filter has been initialized with sensor
 * data
 *
 * Set to YES after first successful filter update with valid sensor data.
 * Used to track filter convergence state.
 */
@property(nonatomic, assign) BOOL filterInitialized;

/**
 * @property hasGpsFix
 * @brief Flag indicating whether valid GPS position data is available
 *
 * Set to YES when GPS data with acceptable accuracy is received.
 * Required for EKF updates (filter needs GPS reference for earth frame
 * calculations).
 */
@property(nonatomic, assign) BOOL hasGpsFix;

/**
 * @property hasValidGroundTrack
 * @brief Flag indicating whether a valid ground track angle has been computed
 *
 * Ground track is only meaningful when moving (> 0.5 m/s).
 * This flag tracks whether we have a valid track to use when stationary.
 */
@property(nonatomic, assign) BOOL hasValidGroundTrack;

/**
 * @property currentTow
 * @brief Current GPS Time of Week (TOW) in milliseconds
 *
 * Updated with each new GPS fix. Used by EKF for time synchronization.
 * For X-Plane, calculated as milliseconds since first GPS fix.
 */
@property(nonatomic, assign) unsigned long currentTow;

/**
 * @property lastValidGroundTrack
 * @brief Last valid ground track angle in degrees [0, 360)
 *
 * Frozen when speed drops below threshold (0.5 m/s) to prevent noisy track
 * values. Used as fallback when stationary or moving very slowly.
 */
@property(nonatomic, assign) float lastValidGroundTrack;

/**
 * @property lastBodyAccelX
 * @brief Last forward (X-axis) acceleration in body frame (m/s²)
 *
 * Used by flight phase detector for takeoff roll detection.
 * Updated with each IMU measurement.
 */
@property(nonatomic, assign) float lastBodyAccelX;

/**
 * @property qnh
 * @brief QNH pressure setting in hectopascals (hPa)
 *
 * QNH is local sea-level pressure used for altitude calculations.
 * Used to compute QNH altitude from barometric pressure.
 * Default: 1013.25 hPa (standard atmosphere)
 */
@property(nonatomic, assign) double qnh;

/**
 * @property groundElevationM
 * @brief Ground elevation above sea level in meters (MSL)
 *
 * User-provided estimate of current ground elevation.
 * Used by FlightPhaseDetector for accurate altitude above ground (AGL)
 * calculations. Default: NAN (not set) - FlightPhaseDetector will use GPS
 * altitude only if not set.
 *
 * Set via setGroundElevation: method.
 */
@property(nonatomic, assign) float groundElevationM;

/**
 * @property lastLatRad
 * @brief Last GPS latitude in radians
 *
 * Cached GPS position used by EKF for earth frame calculations.
 * Updated when new GPS data with acceptable accuracy is received.
 */
@property(nonatomic, assign) double lastLatRad;

/**
 * @property lastLonRad
 * @brief Last GPS longitude in radians
 *
 * Cached GPS position used by EKF for earth frame calculations.
 * Updated when new GPS data with acceptable accuracy is received.
 */
@property(nonatomic, assign) double lastLonRad;

/**
 * @property lastAltM
 * @brief Last GPS altitude in meters (MSL)
 *
 * Cached GPS altitude used by EKF for earth frame calculations.
 * Updated when new GPS data with acceptable accuracy is received.
 */
@property(nonatomic, assign) double lastAltM;

/**
 * @property rollOffsetDeg
 * @brief Roll angle offset in degrees applied to filter output
 *
 * Set by levelAhrs() method to establish zero reference.
 * Applied as: displayed_roll = filter_roll - rollOffsetDeg
 */
@property(nonatomic, assign) double rollOffsetDeg;

/**
 * @property pitchOffsetDeg
 * @brief Pitch angle offset in degrees applied to filter output
 *
 * Set by levelAhrs() method to establish zero reference.
 * Applied as: displayed_pitch = filter_pitch - pitchOffsetDeg
 */
@property(nonatomic, assign) double pitchOffsetDeg;
/**
 * @property magneticDeclination
 * @brief Magnetic declination in degrees (positive = east)
 *
 * Used to convert between magnetic and true heading.
 * Updated from WMM (World Magnetic Model) when GPS position changes
 * significantly.
 */
@property(nonatomic, assign) double magneticDeclination;

/**
 * @property lastDeclinationLatRad
 * @brief Latitude (radians) where last magnetic declination was calculated
 *
 * Used to determine when to recalculate declination (when moved 10+ nautical
 * miles).
 */
@property(nonatomic, assign) double lastDeclinationLatRad;

/**
 * @property lastDeclinationLonRad
 * @brief Longitude (radians) where last magnetic declination was calculated
 *
 * Used to determine when to recalculate declination (when moved 10+ nautical
 * miles).
 */
@property(nonatomic, assign) double lastDeclinationLonRad;

/**
 * @property expectedMagN_nT
 * @brief Expected magnetic field North component from WMM (nanoTesla)
 *
 * Updated when position changes significantly (10+ nautical miles).
 * Passed to EKF filter for magnetometer measurement updates.
 * NAN when not yet calculated.
 */
@property(nonatomic, assign) float expectedMagN_nT;

/**
 * @property expectedMagE_nT
 * @brief Expected magnetic field East component from WMM (nanoTesla)
 *
 * Updated when position changes significantly (10+ nautical miles).
 * Passed to EKF filter for magnetometer measurement updates.
 * NAN when not yet calculated.
 */
@property(nonatomic, assign) float expectedMagE_nT;

/**
 * @property expectedMagD_nT
 * @brief Expected magnetic field Down component from WMM (nanoTesla)
 *
 * Updated when position changes significantly (10+ nautical miles).
 * Passed to EKF filter for magnetometer measurement updates.
 * NAN when not yet calculated.
 */
@property(nonatomic, assign) float expectedMagD_nT;

/**
 * @property lastTimestampUs
 * @brief Timestamp (microseconds) of last sensor data processed by EKF
 *
 * Used to calculate dt (time delta) between filter updates.
 * Prevents processing duplicate timestamps.
 */
@property(nonatomic, assign) uint64_t lastTimestampUs;

/**
 * @property lastGpsTimestampUs
 * @brief Timestamp (microseconds) of last GPS data used by EKF
 *
 * Used to detect new GPS data and prevent duplicate GPS fusion.
 * Updated when GPS data with acceptable accuracy is received.
 */
@property(nonatomic, assign) uint64_t lastGpsTimestampUs;

/**
 * @property xplaneFirstGpsTimestampUs
 * @brief Timestamp (microseconds) of first GPS fix received from X-Plane
 *
 * Used to calculate TOW for X-Plane simulation (TOW = milliseconds since first
 * fix). Reset when X-Plane connection is established.
 */
@property(nonatomic, assign) uint64_t xplaneFirstGpsTimestampUs;

/**
 * @property running
 * @brief Flag indicating whether AHRS is currently active
 *
 * true: Sensors running, data being processed
 * false: Sensors stopped, no processing
 *
 * Set by startAhrs/stopAhrs methods
 */
@property(nonatomic, assign) bool running;

/**
 * @property emitRateHz
 * @brief Rate at which data is emitted to React Native (Hz)
 *
 * Independent of sensor rate (60Hz internally).
 * Lower rates save battery and reduce JavaScript load.
 *
 * Range: [1, 60] Hz
 * Default: 5 Hz
 */
@property(nonatomic, assign) float emitRateHz;

/* ===== Barometer Calibration ===== */

/**
 * @property baroCalibrated
 * @brief Flag indicating whether barometer has been calibrated against GPS
 *
 * iOS CMAltimeter provides RELATIVE pressure/altitude.
 * Must calibrate against GPS altitude to get absolute pressure.
 *
 * false: Barometer data not yet used (waiting for GPS + baro)
 * true: Calibration complete, barometer data valid
 */
@property(nonatomic, assign) bool baroCalibrated;

/**
 * @property baroPressureOffset
 * @brief Pressure offset (kPa) to convert relative to absolute pressure
 *
 * Computed as: offset = sea_level_pressure - measured_pressure
 * Applied as: absolute_pressure = measured_pressure + offset
 *
 * Calculated once when both GPS altitude and barometer data available.
 */
@property(nonatomic, assign) float baroPressureOffset;

/* ===== Cached Sensor Data ===== */

/**
 * @property locationData
 * @brief Latest GPS data in EKF-compatible format
 *
 * Updated by locationManager:didUpdateLocations: delegate method.
 * Read by ProcessDeviceMotionData when fusing into EKF.
 *
 * Thread safety: Access protected by locationLock
 */
@property(nonatomic, assign) gps_position_t locationData;

/**
 * @property baroData
 * @brief Latest barometer data in EKF-compatible format
 *
 * Updated by CMAltimeter callback in ProcessAltitudeData.
 * Read by ProcessDeviceMotionData when fusing into EKF.
 *
 * Thread safety: Access protected by baroLock
 */
@property(nonatomic, assign) baro_pressure_t baroData;

/* ===== Thread Synchronization ===== */

/**
 * @property locationLock
 * @brief NSLock protecting locationData structure
 *
 * Prevents race conditions between:
 * - Write: locationManager delegate (GPS callback thread)
 * - Read: ProcessDeviceMotionData (main queue)
 */
@property(nonatomic, strong) NSLock *locationLock;

/**
 * @property baroLock
 * @brief NSLock protecting baroData structure
 *
 * Prevents race conditions between:
 * - Write: CMAltimeter callback (main queue)
 * - Read: ProcessDeviceMotionData (main queue)
 *
 * Note: Even though both on main queue, lock provides memory barrier
 */
@property(nonatomic, strong) NSLock *baroLock;

/**
 * @property waitingForInitialHeading
 * @brief Flag indicating whether we're waiting for initial heading from
 * CoreLocation
 *
 * Set to YES when AHRS starts to wait for CLHeading for attitude
 * initialization. Reset to NO once heading is received or filter initializes.
 */
@property(nonatomic, assign) bool waitingForInitialHeading;

/**
 * @property initialHeadingFromCL
 * @brief Initial heading value received from CoreLocation (degrees)
 *
 * Used to seed EKF attitude on startup.
 * Value of -1.0 indicates no heading received yet.
 */
@property(nonatomic, assign) float initialHeadingFromCL;

/**
 * @property latestIosHeadingDeg
 * @brief Latest magnetic heading from iOS CoreLocation (degrees [0, 360))
 *
 * Updated from CLHeading delegate callbacks.
 * Used for heading output priority (iOS heading preferred over filter yaw when
 * available).
 */
@property(nonatomic, assign) float latestIosHeadingDeg;

/**
 * @property hasIosHeading
 * @brief Flag indicating whether valid iOS CoreLocation heading is available
 *
 * Set to YES when CLHeading with acceptable accuracy is received.
 * Used to determine heading output priority.
 */
@property(nonatomic, assign) BOOL hasIosHeading;

/**
 * @property latestXPlaneHeadingDeg
 * @brief Latest magnetic heading from X-Plane ground truth (degrees [0, 360))
 *
 * Updated from X-Plane WebSocket messages.
 * Used for heading output priority (X-Plane heading preferred when connected).
 */
@property(nonatomic, assign) double latestXPlaneHeadingDeg;

/**
 * @property hasXPlaneHeading
 * @brief Flag indicating whether valid X-Plane heading is available
 *
 * Set to YES when X-Plane provides ground truth magnetic heading.
 * Used to determine heading output priority.
 */
@property(nonatomic, assign) BOOL hasXPlaneHeading;

/* ===== Recording and Playback ===== */

/**
 * @property isRecording
 * @brief Flag indicating whether recording is active
 */
@property(nonatomic, assign) bool isRecording;

/**
 * @property recordingFileHandle
 * @brief File handle for the current recording file
 */
@property(nonatomic, strong, nullable) NSFileHandle *recordingFileHandle;

/**
 * @property recordingStartTimestamp
 * @brief Timestamp when recording started (microseconds)
 */
@property(nonatomic, assign) uint64_t recordingStartTimestamp;

/**
 * @property recordingPacketCount
 * @brief Number of packets written to current recording
 */
@property(nonatomic, assign) uint32_t recordingPacketCount;

/**
 * @property isPlaying
 * @brief Flag indicating whether playback is active
 */
@property(nonatomic, assign) bool isPlaying;

/**
 * @property playbackTimer
 * @brief Timer used for playback timing
 */
@property(nonatomic, strong, nullable) NSTimer *playbackTimer;

/**
 * @property playbackPackets
 * @brief Array of packet arrays grouped by timestamp for playback
 */
@property(nonatomic, strong, nullable) NSMutableArray *playbackPackets;

/**
 * @property playbackCurrentIndex
 * @brief Current index in playbackPackets array
 */
@property(nonatomic, assign) NSUInteger playbackCurrentIndex;

/**
 * @property currentPlaybackFilename
 * @brief Filename of the currently playing recording
 */
@property(nonatomic, strong, nullable) NSString *currentPlaybackFilename;

/**
 * @property rotation
 * @brief Device rotation/mounting orientation
 *
 * Defines how the device is physically mounted relative to aircraft.
 * Affects coordinate frame transformations from iOS sensors to aviation body
 * frame. See AhrsRotation enum for valid values.
 */
@property(nonatomic, assign) AhrsRotation rotation;

/* ===== X-Plane WebSocket Connection ===== */

/**
 * @property xplaneSession
 * @brief NSURLSession for WebSocket connection to X-Plane plugin
 */
@property(nonatomic, strong, nullable) NSURLSession *xplaneSession;

/**
 * @property xplaneWebSocketTask
 * @brief WebSocket task for receiving X-Plane data
 */
@property(nonatomic, strong, nullable)
NSURLSessionWebSocketTask *xplaneWebSocketTask;

/**
 * @property xplaneConnected
 * @brief Flag indicating active X-Plane connection
 *
 * When true, real device sensors are bypassed and X-Plane data feeds the EKF.
 */
@property(nonatomic, assign) bool xplaneConnected;

/**
 * @property xplaneHost
 * @brief Hostname/IP of X-Plane computer (e.g., "192.168.1.100")
 */
@property(nonatomic, copy, nullable) NSString *xplaneHost;

/**
 * @property xplaneReconnectAttempts
 * @brief Number of reconnection attempts made for X-Plane WebSocket
 *
 * Incremented on timeout/network errors. Reset on successful connection or
 * manual disconnect. Used to limit reconnection attempts (max 5) before falling
 * back to real sensors.
 */
@property(nonatomic, assign) int xplaneReconnectAttempts;

/**
 * @property xplaneWasPaused
 * @brief Previous pause state of X-Plane simulation
 *
 * Used to detect pause state transitions (paused -> running or running ->
 * paused). When X-Plane is paused, filter updates are skipped to prevent
 * processing stale data.
 */
@property(nonatomic, assign) BOOL xplaneWasPaused;

/**
 * @property recordingDataPoints
 * @brief Dictionary of recording data points keyed by timestamp (used during
 * recording)
 */
@property(nonatomic, strong, nullable) NSMutableDictionary *recordingDataPoints;

@end

NS_ASSUME_NONNULL_END
