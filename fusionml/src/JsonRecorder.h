/**
 * JsonRecorder.h
 *
 * JSON-based sensor data recorder for AHRS flight data collection.
 *
 * Records sensor data (IMU, GPS, barometer) in a compact JSON format with gzip
 * compression. Designed for ML training datasets - human-readable JSON that's
 * easy to parse and process.
 *
 * Usage:
 *   1. Create JsonRecorder instance
 *   2. Call startSession() to begin recording
 *   3. Call appendReading() for each sensor sample
 *   4. Call save() to write compressed JSON file
 *
 * Output format: gzipped JSON file (.json.gz) containing:
 *   - Session metadata (device ID, timestamps, sample count)
 *   - Device metadata (model, OS version, app version)
 *   - Custom metadata (key-value pairs)
 *   - Array of sensor readings with timestamps
 *
 * Thread safety: Not thread-safe. Use external synchronization if calling from
 * multiple threads.
 */

#ifndef JSON_RECORDER_H
#define JSON_RECORDER_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>

/**
 * @struct SensorReading
 * @brief Single sensor reading containing all sensor data at one timestamp
 *
 * Represents one time-synchronized sample of all sensors. All fields are
 * populated from sensor hardware or computed values.
 */
struct SensorReading {
  double timestamp;      ///< Timestamp in seconds (Unix epoch or relative)
  float acc[3];          ///< Accelerometer [x, y, z] in m/s² (body frame)
  float gyro[3];         ///< Gyroscope [x, y, z] in rad/s (body frame)
  float mag[3];          ///< Magnetometer [x, y, z] in µT (body frame)
  double gpsLat;         ///< GPS latitude in degrees
  double gpsLon;         ///< GPS longitude in degrees
  double gpsAlt;         ///< GPS altitude in meters (MSL)
  float gpsAccuracy;    ///< GPS accuracy estimate in meters (horizontal)
  float gpsVel[3];       ///< GPS velocity [north, east, down] in m/s (NED frame)
  float baroAlt;         ///< Barometric altitude in meters (MSL)
  float pressure;        ///< Barometric pressure in hPa (hectopascals)
};

/**
 * @class JsonRecorder
 * @brief Records sensor data to compressed JSON format for ML training datasets
 *
 * Collects sensor readings in memory and writes them to a gzipped JSON file
 * when save() is called. Designed for flight data collection where sensor data
 * is recorded continuously and then saved at the end of a flight session.
 *
 * Memory usage: All readings are stored in memory until save() is called.
 * For long recordings, consider periodic saves or streaming to disk.
 */
class JsonRecorder {
public:
  /**
   * @brief Constructs a new JsonRecorder instance
   *
   * Initializes recorder in non-recording state. Call startSession() to begin.
   */
  JsonRecorder();
  
  /**
   * @brief Destructor - automatically saves if recording is active
   *
   * If recording is in progress, calls save() to write data to disk before
   * destruction. Ensures no data loss if recorder goes out of scope.
   */
  ~JsonRecorder();
  
  /**
   * @brief Starts a new recording session
   *
   * Clears any previous data and begins a new recording session. Must be
   * called before appendReading().
   *
   * @param filepath Full path to output file (will be created as .json.gz)
   * @param deviceId Unique device identifier (e.g., UIDevice identifierForVendor)
   * @param sessionId Unique session identifier (e.g., UUID)
   * @return true if session started successfully, false if already recording
   */
  bool startSession(const std::string& filepath,
                    const std::string& deviceId,
                    const std::string& sessionId);
  
  /**
   * @brief Appends a sensor reading to the recording
   *
   * Adds one time-synchronized sensor sample to the in-memory buffer.
   * Data is not written to disk until save() is called.
   *
   * @param timestamp Timestamp in seconds (Unix epoch or relative)
   * @param acc Accelerometer [x, y, z] in m/s² (body frame)
   * @param gyro Gyroscope [x, y, z] in rad/s (body frame)
   * @param mag Magnetometer [x, y, z] in µT (body frame)
   * @param gpsLat GPS latitude in degrees
   * @param gpsLon GPS longitude in degrees
   * @param gpsAlt GPS altitude in meters (MSL)
   * @param gpsAccuracy GPS accuracy estimate in meters (horizontal)
   * @param gpsVel GPS velocity [north, east, down] in m/s (NED frame)
   * @param baroAlt Barometric altitude in meters (MSL)
   * @param pressure Barometric pressure in hPa
   *
   * @note Does nothing if not recording (startSession() not called or save() already called)
   */
  void appendReading(double timestamp,
                     const float acc[3],
                     const float gyro[3],
                     const float mag[3],
                     double gpsLat, double gpsLon, double gpsAlt, float gpsAccuracy,
                     const float gpsVel[3],
                     float baroAlt,
                     float pressure);
  
  /**
   * @brief Saves all recorded data to compressed JSON file
   *
   * Writes all accumulated readings to disk as gzipped JSON (compression level 9).
   * After successful save, recording is stopped and cannot be resumed.
   *
   * Output JSON structure:
   *   {
   *     "device_id": "...",
   *     "session_id": "...",
   *     "start_time": 1234567890,
   *     "end_time": 1234567891,
   *     "sample_count": 1000,
   *     "device_model": "...",
   *     "os_version": "...",
   *     "app_version": "...",
   *     "sample_rate": 100.0,
   *     "metadata": { "key": "value", ... },
   *     "readings": [
   *       {
   *         "timestamp": 1234567890.123,
   *         "accelerometer": [0.1, 0.2, 0.3],
   *         "gyroscope": [0.01, 0.02, 0.03],
   *         "magnetometer": [1.1, 1.2, 1.3],
   *         "gps": {"lat": 51.5074, "lon": -0.1278, "alt": 35.0, "acc": 3.0},
   *         "gps_velocity": [5.0, 0.0, -0.5],
   *         "baro_alt": 42.0,
   *         "pressure": 1010.0
   *       },
   *       ...
   *     ]
   *   }
   *
   * @return true if save succeeded, false on error (disk full, permission denied, etc.)
   */
  bool save();
  
  /**
   * @brief Returns the number of readings currently buffered
   *
   * @return Number of readings appended since startSession()
   */
  int getReadingCount() const { return readings_.size(); }
  
  /**
   * @brief Sets device metadata for the recording session
   *
   * Device metadata is written to JSON output. Optional but recommended for
   * dataset provenance.
   *
   * @param deviceModel Device model name (e.g., "iPhone 14 Pro")
   * @param osVersion Operating system version (e.g., "iOS 18.0")
   * @param appVersion Application version (e.g., "1.2.3")
   */
  void setDeviceMetadata(const std::string& deviceModel,
                         const std::string& osVersion,
                         const std::string& appVersion);
  
  /**
   * @brief Sets the nominal sample rate for the recording
   *
   * Sample rate is written to JSON output for reference. Does not affect
   * recording behavior - readings are appended as received.
   *
   * @param sampleRateHz Nominal sample rate in Hz (e.g., 100.0 for 100 Hz)
   */
  void setSampleRate(float sampleRateHz) { sampleRate_ = sampleRateHz; }
  
  /**
   * @brief Adds custom metadata key-value pair
   *
   * Custom metadata is written to JSON "metadata" object. Useful for adding
   * flight-specific information (pilot, aircraft, weather, etc.).
   *
   * @param key Metadata key (will be JSON-escaped)
   * @param value Metadata value (will be JSON-escaped)
   */
  void addMetadata(const std::string& key, const std::string& value);
  
  /**
   * @brief Checks if recording is currently active
   *
   * @return true if recording (startSession() called, save() not yet called)
   */
  bool isRecording() const { return recording_; }
  
private:
  /**
   * @brief Escapes special characters in JSON strings
   *
   * Handles quotes, backslashes, control characters, etc. according to JSON spec.
   *
   * @param str Input string to escape
   * @return JSON-escaped string safe for use in JSON values
   */
  std::string escapeJson(const std::string& str);
  
  /**
   * @brief Converts 3-element float array to JSON array string
   *
   * Formats as "[x, y, z]" with 6 decimal places precision.
   *
   * @param v 3-element float array
   * @return JSON array string (e.g., "[0.123456, -0.456789, 9.810000]")
   */
  std::string vectorToJson(const float v[3]);
  
  // Session configuration
  std::string filepath_;      ///< Output file path (.json.gz)
  std::string deviceId_;      ///< Unique device identifier
  std::string sessionId_;     ///< Unique session identifier
  std::string deviceModel_;   ///< Device model name
  std::string osVersion_;     ///< Operating system version
  std::string appVersion_;    ///< Application version
  float sampleRate_;          ///< Nominal sample rate in Hz
  int64_t startTimeMs_;       ///< Session start time (Unix epoch milliseconds)
  
  // Recording data
  std::vector<SensorReading> readings_;              ///< Buffered sensor readings
  std::map<std::string, std::string> metadata_;     ///< Custom metadata key-value pairs
  bool recording_;                                  ///< Recording state flag
};

#endif // JSON_RECORDER_H
