import { CodegenTypes, type TurboModule } from 'react-native';
import { TurboModuleRegistry } from 'react-native';

/**
 * Device rotation/mounting orientation
 */
export type AhrsRotation = 'none' | 'left' | 'right';

/**
 * AHRS output data structure
 * Emitted via onAhrsUpdate event
 */
export interface AhrsData {
  // Attitude (degrees)
  roll: number; // [-180, 180], positive = right wing down
  pitch: number; // [-90, 90], positive = nose up
  heading: number; // [0, 360), magnetic heading from iOS CLHeading (tilt-compensated, calibrated)
  yaw: number; // [0, 360), true heading from filter (absolute heading, not Earth relative)
  magneticDeclination: number; // [degrees], magnetic declination (positive = east)
  groundTrack: number; // [0, 360), horizontal direction of travel
  groundSpeed: number; // m/s, speed in horizontal plane
  flightPathAngle: number; // [degrees], VERTICAL angle of velocity vector relative to horizontal (positive = climbing, negative = descending)
  horizontalFlightPathAngle: number; // [degrees], HORIZONTAL angle from body frame velocity (sideslip/crab angle, [-180, 180], positive = drifting right)

  // Altitude (meters)
  altitude: number; // GPS altitude (MSL)
  altitudeQNE: number; // Barometric altitude QNE (standard atmosphere, 1013.25 hPa)
  altitudeQNH: number; // Barometric altitude QNH (local pressure)
  verticalSpeed: number; // m/s, positive = climbing
  barometricPressure: number; // Measured barometric pressure in hPa (hectopascals)

  // Velocity (NED frame)
  velocityNorth: number; // m/s, velocity component in North direction
  velocityEast: number; // m/s, velocity component in East direction
  velocityDown: number; // m/s, velocity component in Down direction

  // Position
  latitude?: number; // latitude in degrees
  longitude?: number; // longitude in degrees

  // Flight phase
  flightPhase: number; // 0=GROUND, 1=TAKEOFF, 2=CLIMB, 3=CRUISE, 4=DESCENT, 5=APPROACH, 6=LANDING
  flightPhaseConfidence: number; // 0.0-1.0 confidence in current phase

  // Validity flags - indicate if outputs are reliable and should be used
  flightPhaseValid: boolean; // Flight phase detection is reliable
  attitudeValid: boolean; // Roll, pitch, heading are reliable
  altitudeValid: boolean; // Altitude estimates are reliable
  positionValid: boolean; // Position (lat/lon) is reliable
}

/**
 * Recording file information
 */
export interface RecordingFile {
  filename: string; // Name of the file
  size: number; // File size in bytes
  date: number; // Modification date as timestamp (milliseconds)
}

export type PlaybackStateStatus = 'started' | 'stopped' | 'completed';

export interface PlaybackStateEvent {
  status: PlaybackStateStatus;
  filename?: string;
  reason?: string;
}

/**
 * X-Plane connection state event
 */
export interface XPlaneConnectionEvent {
  connected: boolean;
  host: string;
}

export interface Spec extends TurboModule {
  /**
   * Start AHRS sensor processing
   * Begins collecting and fusing sensor data at 60Hz
   */
  startAhrs(): void;

  /**
   * Stop AHRS sensor processing
   * Halts all sensor updates (saves battery)
   */
  stopAhrs(): void;

  /**
   * Reset filter to initial state
   * Clears all state estimates and covariances
   * Requires reconvergence (2-3 seconds)
   */
  resetAhrs(): void;

  /**
   * Level attitude reference
   * Captures current attitude as "zero" reference
   * Call when device is level
   */
  levelAhrs(): void;

  /**
   * Set output emission rate
   * Controls how often data is sent to JavaScript
   *
   * @param rate - Rate in Hz, range [1, 60]
   *               Lower rates save battery
   *               Default: 5 Hz
   */
  setAhrsRate(rate: number): void;

  /**
   * Set device rotation/mounting orientation
   * Configures coordinate frame transformations
   *
   * @param rotation - Device orientation:
   *                   "none" - Portrait/vertical (top edge up, default)
   *                   "left" - Landscape left (rotated 90° CCW from portrait)
   *                   "right" - Landscape right (rotated 90° CW from portrait)
   */
  setAhrsRotation(rotation: AhrsRotation): void;

  /**
   * Check if AHRS is supported on this device
   * Verifies all required sensors are available
   *
   * @returns Promise<boolean> - true if supported
   */
  isSupported(): Promise<boolean>;

  /**
   * Set QNH pressure for altitude correction
   * QNH is local sea-level pressure used by aviation
   *
   * @param qnh - Pressure in hectopascals (hPa = mbar)
   *              Range: [900, 1100] hPa
   *              Standard: 1013.25 hPa
   *              Get from METAR, ATIS, or weather service
   */
  setQNH(qnh: number): void;

  /**
   * Check if position estimate is reliable
   * Considers GPS reference initialization and filter state
   *
   * @returns Promise<boolean> - true if position can be trusted for navigation
   */
  isPositionReliable(): Promise<boolean>;

  /**
   * Start recording sensor data to a file
   * Records all sensor inputs (gyro, accel, mag, GPS, baro) to a gzipped JSON file
   * Files use auto-generated names (YYMMDDHHmmss.json.gz) in the documents directory
   */
  startRecording(): void;

  /**
   * Stop recording and close the file
   * Updates file header with final metadata
   */
  stopRecording(): void;

  /**
   * Start playback of a recorded file
   * Feeds recorded sensor data to EKF in real-time
   * Requires AHRS to be running (call startAhrs first)
   *
   * @param filename - Name of the file to play back
   */
  playbackRecording(filename: string): void;

  /**
   * Stop playback
   * Stops feeding recorded data to EKF
   */
  stopPlayback(): void;

  /**
   * Get list of recording files
   * Returns all .json and .json.gz files in app documents directory
   *
   * @returns Promise<RecordingFile[]> - Array of file info, sorted by date (newest first)
   */
  getRecordingFiles(): Promise<RecordingFile[]>;

  /**
   * Delete a recording file
   *
   * @param filename - Name of the file to delete
   */
  deleteRecording(filename: string): void;

  /**
   * Playback state events (start/stop/completed)
   */
  readonly onPlaybackStateChanged: CodegenTypes.EventEmitter<PlaybackStateEvent>;

  readonly onAhrsUpdate: CodegenTypes.EventEmitter<AhrsData>;

  /**
   * X-Plane connection state events (connected/disconnected)
   */
  readonly onXPlaneConnectionChanged: CodegenTypes.EventEmitter<XPlaneConnectionEvent>;

  /**
   * Connect to X-Plane plugin via WebSocket
   *
   * When connected, real device sensors are bypassed and X-Plane data feeds the EKF.
   * Requires AHRS to be running (call startAhrs first).
   *
   * @param host - Hostname or IP address of X-Plane computer (e.g., "192.168.1.100")
   */
  connectToXPlane(host: string): void;

  /**
   * Disconnect from X-Plane plugin
   *
   * Closes WebSocket connection and returns to using real device sensors.
   */
  disconnectFromXPlane(): void;
}

export default TurboModuleRegistry.getEnforcing<Spec>('NativeAhrs');
