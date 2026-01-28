import NativeAhrs, {
  type AhrsData,
  type AhrsRotation,
  type RecordingFile,
  type PlaybackStateEvent,
  type XPlaneConnectionEvent,
} from './NativeAhrs';
import { logger } from './AhrsLogger';
import { AhrsCallbackManager, type AhrsCallback } from './AhrsCallbackManager';
import { AhrsLifecycleManager } from './AhrsLifecycleManager';
import { AhrsConfiguration } from './AhrsConfiguration';

// Re-export types for consumers of the library
export type {
  AhrsData,
  AhrsRotation,
  RecordingFile,
  PlaybackStateEvent,
  XPlaneConnectionEvent,
} from './NativeAhrs';
export type { AhrsCallback } from './AhrsCallbackManager';

/**
 * Main AHRS sensor class providing high-level interface to React Native AHRS module
 *
 * This class orchestrates:
 * - Sensor lifecycle (start/stop) via AhrsLifecycleManager
 * - Listener registration and callback management via AhrsCallbackManager
 * - Configuration (rate, rotation, magnetic declination, QNH) via AhrsConfiguration
 *
 * Follows SOLID principles by delegating responsibilities to specialized managers.
 *
 * @example
 * ```ts
 * import { Ahrs } from 'react-native-ahrs';
 *
 * const unsubscribe = Ahrs.addListener((data) => {
 *   console.log('Roll:', data.roll, 'Pitch:', data.pitch);
 * });
 *
 * Ahrs.start();
 * // ... later
 * Ahrs.stop();
 * unsubscribe();
 * ```
 */
class AhrsSensor {
  private callbackManager: AhrsCallbackManager;
  private lifecycleManager: AhrsLifecycleManager;
  private configuration: AhrsConfiguration;
  private playbackListeners: Set<(event: PlaybackStateEvent) => void>;
  private playbackActive = false;
  private xplaneListeners: Set<(event: XPlaneConnectionEvent) => void>;
  private xplaneConnected = false;
  private xplaneHost: string | null = null;

  private nativeSubscriptionInitialized = false;

  /**
   * Creates a new AhrsSensor instance
   * Native module subscription is deferred until first use
   */
  constructor() {
    this.callbackManager = new AhrsCallbackManager();
    this.lifecycleManager = new AhrsLifecycleManager();
    this.configuration = new AhrsConfiguration();
    this.playbackListeners = new Set();
    this.xplaneListeners = new Set();
  }

  /**
   * Ensures native module subscription is initialized
   * Called lazily on first use to avoid circular dependency issues
   */
  private ensureNativeSubscription(): void {
    if (this.nativeSubscriptionInitialized) {
      return;
    }
    if (NativeAhrs) {
      NativeAhrs.onAhrsUpdate((data: AhrsData) => {
        this.callbackManager.distributeData(data);
      });
      NativeAhrs.onPlaybackStateChanged((event) => {
        this.playbackActive = event.status === 'started';
        this.playbackListeners.forEach((listener) => {
          try {
            listener(event);
          } catch (error) {
            logger.warn('Playback listener threw an error', error);
          }
        });
      });
      NativeAhrs.onXPlaneConnectionChanged((event) => {
        this.xplaneConnected = event.connected;
        this.xplaneHost = event.connected ? event.host : null;
        this.xplaneListeners.forEach((listener) => {
          try {
            listener(event);
          } catch (error) {
            logger.warn('X-Plane connection listener threw an error', error);
          }
        });
      });
      this.nativeSubscriptionInitialized = true;
    }
  }

  /**
   * Checks if AHRS is supported on this device
   *
   * Verifies that all required sensors are available:
   * - Gyroscope
   * - Accelerometer
   * - Magnetometer
   *
   * @returns Promise resolving to true if supported, false otherwise
   * @throws Never throws, returns false on error
   *
   * @example
   * ```ts
   * const supported = await Ahrs.isSupported();
   * if (!supported) {
   *   console.log('AHRS not available on this device');
   * }
   * ```
   */
  public async isSupported(): Promise<boolean> {
    try {
      const supported = await NativeAhrs.isSupported();
      if (supported) {
        logger.log('✅ AHRS supported on this device');
      } else {
        logger.warn('❌ AHRS not supported on this device');
      }
      return supported;
    } catch (e) {
      logger.warn('Failed to query AHRS support', e);
      return false;
    }
  }

  /**
   * Registers a callback to receive AHRS data updates
   *
   * The callback will be invoked at the configured rate (default 5 Hz)
   * with the latest sensor fusion data.
   *
   * @param callback - Function to call with AHRS data updates
   * @returns Unsubscribe function to remove this listener
   *
   * @example
   * ```ts
   * const unsubscribe = Ahrs.addListener((data) => {
   *   console.log('Heading:', data.heading);
   * });
   *
   * // Later, to remove:
   * unsubscribe();
   * ```
   */
  public addListener(callback: AhrsCallback) {
    this.ensureNativeSubscription();
    const unsubscribe = this.callbackManager.addListener(callback);
    return () => {
      unsubscribe();
      if (this.callbackManager.getListenerCount() === 0) {
        this.stop();
      }
    };
  }

  /**
   * Registers a callback for playback state changes
   *
   * @param listener - Receives events when playback starts/stops/completes
   * @returns Unsubscribe function
   */
  public addPlaybackListener(listener: (event: PlaybackStateEvent) => void) {
    this.ensureNativeSubscription();
    this.playbackListeners.add(listener);
    return () => {
      this.playbackListeners.delete(listener);
    };
  }

  /**
   * Indicates whether playback is currently active
   */
  public isPlaybackActive(): boolean {
    return this.playbackActive;
  }

  /**
   * Starts AHRS sensor processing
   *
   * Begins collecting and fusing sensor data at 60Hz internally.
   * Data is emitted to registered listeners at the configured rate (default 5Hz).
   *
   * Requires at least one listener to be registered via `addListener()`.
   *
   * @throws {Error} If AHRS is not supported or initialization fails
   *
   * @example
   * ```ts
   * Ahrs.addListener((data) => console.log(data));
   * Ahrs.start();
   * ```
   */
  public start(): void {
    this.ensureNativeSubscription();
    if (!this.callbackManager.hasListeners()) {
      logger.warn(
        '❌ Attempt to start Ahrs without any callbacks registered. Call addListener before starting.'
      );
      return;
    }
    this.lifecycleManager.start();
  }

  /**
   * Stops AHRS sensor processing
   *
   * Halts all sensor updates to save battery.
   * Can be restarted later with `start()`.
   *
   * @example
   * ```ts
   * Ahrs.stop();
   * ```
   */
  public stop(): void {
    this.lifecycleManager.stop();
  }

  /**
   * Resets the EKF filter to initial state
   *
   * Clears all state estimates and covariances.
   * The filter will re-initialize from current sensor readings.
   * Requires 2-3 seconds for reconvergence.
   *
   * Useful when:
   * - Device orientation changes significantly
   * - Filter has diverged
   * - Starting a new flight/session
   *
   * @example
   * ```ts
   * Ahrs.reset();
   * ```
   */
  public reset(): void {
    this.lifecycleManager.reset();
  }

  /**
   * Levels the attitude reference
   *
   * Captures the current attitude as the "zero" reference.
   * Call this when the device is level to set roll=0° and pitch=0°.
   *
   * Does not reset heading or other states.
   *
   * @example
   * ```ts
   * // Place device level, then:
   * Ahrs.level();
   * ```
   */
  public level(): void {
    this.lifecycleManager.level();
  }

  /**
   * Sets QNH pressure for altitude correction
   *
   * QNH is local sea-level pressure used by aviation.
   * Required for accurate altitude calculations from barometric pressure.
   *
   * @param qnh - Pressure in hectopascals (hPa = mbar)
   *             Range: [900, 1100] hPa
   *             Standard: 1013.25 hPa
   *
   * @example
   * ```ts
   * // Get from METAR, ATIS, or weather service
   * Ahrs.setQNH(1013.25);
   * ```
   */
  public setQNH(qnh: number): void {
    this.configuration.setQNH(qnh);
  }

  /**
   * Sets the output emission rate
   *
   * Controls how often data is sent to JavaScript listeners.
   * Independent of internal sensor rate (60Hz).
   * Lower rates save battery and reduce JavaScript load.
   *
   * @param newRate - Rate in Hz, range [1, 60]
   *                  Default: 5 Hz
   *
   * @example
   * ```ts
   * // Emit at 10 Hz for higher update rate
   * Ahrs.setRate(10);
   *
   * // Emit at 1 Hz to save battery
   * Ahrs.setRate(1);
   * ```
   */
  public setRate(newRate: number): void {
    this.configuration.setRate(newRate);
  }

  /**
   * Sets device rotation/mounting orientation
   *
   * Configures coordinate frame transformations from device sensors
   * to aviation body frame. Must match physical device mounting.
   *
   * Changing rotation resets the AHRS filter to re-initialize with new orientation.
   *
   * @param rotation - Device orientation:
   *                   "none" - Portrait/vertical (top edge up, default)
   *                   "left" - Landscape left (rotated 90° CCW from portrait)
   *                   "right" - Landscape right (rotated 90° CW from portrait)
   *
   * @example
   * ```ts
   * // Device mounted in landscape left orientation
   * Ahrs.setRotation('left');
   * ```
   */
  public setRotation(rotation: AhrsRotation): void {
    this.configuration.setRotation(rotation);
  }

  /**
   * Removes all registered listeners and stops AHRS
   *
   * Convenience method to clean up all callbacks and stop processing.
   *
   * @example
   * ```ts
   * Ahrs.removeAllListeners();
   * ```
   */
  public removeAllListeners() {
    this.callbackManager.removeAllListeners();
    this.playbackListeners.clear();
    this.playbackActive = false;
    this.xplaneListeners.clear();
    this.xplaneConnected = false;
    this.xplaneHost = null;
    this.disconnectFromXPlane();
    this.stop();
    // Keep native subscriptions intact to avoid duplicate event handlers.
  }

  /**
   * Starts recording sensor data to a file
   *
   * Records all sensor inputs (gyro, accel, mag, GPS, baro) to a gzipped JSON file.
   * Files are saved with auto-generated timestamp names (YYMMDDHHmmss.json.gz).
   *
   * @example
   * ```ts
   * Ahrs.startRecording();
   * // ... later
   * Ahrs.stopRecording();
   * ```
   */
  public startRecording(): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.startRecording();
  }

  /**
   * Stops recording and closes the file
   *
   * Updates file header with final metadata (end timestamp, packet count).
   *
   * @example
   * ```ts
   * Ahrs.startRecording();
   * // ... record data ...
   * Ahrs.stopRecording();
   * ```
   */
  public stopRecording(): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.stopRecording();
  }

  /**
   * Starts playback of a recorded file
   *
   * Feeds recorded sensor data to EKF in real-time, preserving original timestamps.
   * Requires AHRS to be running (call start() first).
   *
   * @param filename - Name of the file to play back
   *
   * @example
   * ```ts
   * Ahrs.start();
   * Ahrs.playbackRecording('test-flight.json.gz');
   * // ... later
   * Ahrs.stopPlayback();
   * ```
   */
  public playbackRecording(filename: string): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.playbackRecording(filename);
  }

  /**
   * Stops playback
   *
   * Stops feeding recorded data to EKF. Normal sensor processing continues if AHRS is running.
   *
   * @example
   * ```ts
   * Ahrs.playbackRecording('test-flight.json.gz');
   * // ... later
   * Ahrs.stopPlayback();
   * ```
   */
  public stopPlayback(): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.stopPlayback();
  }

  /**
   * Gets list of recording files
   *
   * Returns all .json and .json.gz files in app documents directory, sorted by date (newest first).
   *
   * @returns Promise resolving to array of file info
   *
   * @example
   * ```ts
   * const files = await Ahrs.getRecordingFiles();
   * files.forEach(file => {
   *   console.log(file.filename, file.size, 'bytes');
   * });
   * ```
   */
  public async getRecordingFiles(): Promise<RecordingFile[]> {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return [];
    }
    return await NativeAhrs.getRecordingFiles();
  }

  /**
   * Deletes a recording file
   *
   * @param filename - Name of the file to delete
   *
   * @example
   * ```ts
   * await Ahrs.deleteRecording('old-test.json.gz');
   * ```
   */
  public deleteRecording(filename: string): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.deleteRecording(filename);
  }

  // ============================================================================
  // X-PLANE CONNECTION
  // ============================================================================

  /**
   * Connects to X-Plane plugin via WebSocket
   *
   * When connected, real device sensors are bypassed and X-Plane data feeds the EKF.
   * Requires AHRS to be running (call start() first).
   *
   * @param host - Hostname or IP address of X-Plane computer (e.g., "192.168.1.100")
   *
   * @example
   * ```ts
   * Ahrs.start();
   * Ahrs.connectToXPlane('192.168.1.100');
   * ```
   */
  public connectToXPlane(host: string): void {
    this.ensureNativeSubscription();
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.connectToXPlane(host);
  }

  /**
   * Disconnects from X-Plane plugin
   *
   * Closes WebSocket connection and returns to using real device sensors.
   *
   * @example
   * ```ts
   * Ahrs.disconnectFromXPlane();
   * ```
   */
  public disconnectFromXPlane(): void {
    if (!NativeAhrs) {
      logger.warn('NativeAhrs module not available');
      return;
    }
    NativeAhrs.disconnectFromXPlane();
  }

  /**
   * Registers a callback for X-Plane connection state changes
   *
   * @param listener - Receives events when X-Plane connects/disconnects
   * @returns Unsubscribe function
   *
   * @example
   * ```ts
   * const unsubscribe = Ahrs.addXPlaneConnectionListener((event) => {
   *   if (event.connected) {
   *     console.log('Connected to X-Plane at', event.host);
   *   } else {
   *     console.log('Disconnected from X-Plane');
   *   }
   * });
   * ```
   */
  public addXPlaneConnectionListener(
    listener: (event: XPlaneConnectionEvent) => void
  ) {
    this.ensureNativeSubscription();
    this.xplaneListeners.add(listener);
    return () => {
      this.xplaneListeners.delete(listener);
    };
  }

  /**
   * Indicates whether X-Plane is currently connected
   *
   * @returns true if connected to X-Plane, false otherwise
   */
  public isXPlaneConnected(): boolean {
    return this.xplaneConnected;
  }

  /**
   * Gets the hostname/IP of the connected X-Plane computer
   *
   * @returns Host string if connected, null otherwise
   */
  public getXPlaneHost(): string | null {
    return this.xplaneHost;
  }

  /**
   * Gets current status information
   *
   * @returns Object with:
   *          - isRunning: Whether AHRS is currently active
   *          - listenerCount: Number of registered listeners
   *
   * @example
   * ```ts
   * const status = Ahrs.getStatus();
   * console.log('Running:', status.isRunning);
   * console.log('Listeners:', status.listenerCount);
   * ```
   */
  public getStatus() {
    return {
      isRunning: this.lifecycleManager.getRunning(),
      listenerCount: this.callbackManager.getListenerCount(),
    };
  }
}

/**
 * Singleton instance of AhrsSensor
 *
 * This is the main entry point for using the AHRS library.
 * All methods are available directly on this instance.
 *
 * @example
 * ```ts
 * import { Ahrs } from 'react-native-ahrs';
 *
 * Ahrs.addListener((data) => {
 *   console.log('Roll:', data.roll);
 * });
 * Ahrs.start();
 * ```
 */
export const Ahrs = new AhrsSensor();
