/**
 * JsonRecorder.cpp
 *
 * Implementation of JSON-based sensor data recorder with gzip compression.
 *
 * Records sensor data to memory and writes compressed JSON on save(). Uses
 * zlib (gzip) compression level 9 for maximum compression ratio. JSON format
 * is human-readable and easy to parse for ML training pipelines.
 *
 * Dependencies:
 *   - zlib (for gzip compression)
 *   - C++11 standard library (chrono, iostream, etc.)
 */

#include "JsonRecorder.h"
#include <chrono>
#include <sstream>
#include <iomanip>
#include <zlib.h>

JsonRecorder::JsonRecorder()
: sampleRate_(0.0f)
, startTimeMs_(0)
, recording_(false) {
  // Initialize recorder in non-recording state
  // Call startSession() to begin recording
}

JsonRecorder::~JsonRecorder() {
  // Auto-save on destruction to prevent data loss
  // If recording is active, write data to disk before destroying object
  if (recording_) {
    save();
  }
}

bool JsonRecorder::startSession(const std::string& filepath,
                                const std::string& deviceId,
                                const std::string& sessionId) {
  // Prevent starting new session while already recording
  if (recording_) {
    return false; // Already recording
  }
  
  // Store session configuration
  filepath_ = filepath;
  deviceId_ = deviceId;
  sessionId_ = sessionId;
  
  // Clear any previous data
  readings_.clear();
  metadata_.clear();
  
  // Record session start time (Unix epoch milliseconds)
  auto now = std::chrono::system_clock::now();
  startTimeMs_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                                       now.time_since_epoch()).count();
  
  recording_ = true;
  return true;
}

void JsonRecorder::appendReading(double timestamp,
                                 const float acc[3],
                                 const float gyro[3],
                                 const float mag[3],
                                 double gpsLat, double gpsLon, double gpsAlt, float gpsAccuracy,
                                 const float gpsVel[3],
                                 float baroAlt,
                                 float pressure) {
  // Silently ignore if not recording (allows safe calls after save())
  if (!recording_) {
    return;
  }
  
  // Copy sensor data into reading structure
  SensorReading reading;
  reading.timestamp = timestamp;
  
  // IMU sensors (body frame)
  reading.acc[0] = acc[0];
  reading.acc[1] = acc[1];
  reading.acc[2] = acc[2];
  
  reading.gyro[0] = gyro[0];
  reading.gyro[1] = gyro[1];
  reading.gyro[2] = gyro[2];
  
  reading.mag[0] = mag[0];
  reading.mag[1] = mag[1];
  reading.mag[2] = mag[2];
  
  // GPS position (geodetic coordinates)
  reading.gpsLat = gpsLat;
  reading.gpsLon = gpsLon;
  reading.gpsAlt = gpsAlt;
  reading.gpsAccuracy = gpsAccuracy;
  
  // GPS velocity (NED frame)
  reading.gpsVel[0] = gpsVel[0];
  reading.gpsVel[1] = gpsVel[1];
  reading.gpsVel[2] = gpsVel[2];
  
  // Barometric data
  reading.baroAlt = baroAlt;
  reading.pressure = pressure;
  
  // Append to in-memory buffer (written to disk on save())
  readings_.push_back(reading);
}

bool JsonRecorder::save() {
  // Cannot save if not recording
  if (!recording_) {
    return false;
  }
  
  // Calculate session end time
  auto now = std::chrono::system_clock::now();
  int64_t endTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                                            now.time_since_epoch()).count();
  
  // Build JSON string in memory first (more efficient than streaming to compressed file)
  std::ostringstream jsonStream;
  jsonStream << std::fixed << std::setprecision(6); // 6 decimal places for floats
  
  // JSON root object
  jsonStream << "{\n";
  
  // Session metadata (required fields)
  jsonStream << "  \"device_id\": \"" << escapeJson(deviceId_) << "\",\n";
  jsonStream << "  \"session_id\": \"" << escapeJson(sessionId_) << "\",\n";
  jsonStream << "  \"start_time\": " << startTimeMs_ << ",\n";
  jsonStream << "  \"end_time\": " << endTimeMs << ",\n";
  jsonStream << "  \"sample_count\": " << readings_.size() << ",\n";
  
  // Device metadata (optional fields - only write if set)
  if (!deviceModel_.empty()) {
    jsonStream << "  \"device_model\": \"" << escapeJson(deviceModel_) << "\",\n";
  }
  if (!osVersion_.empty()) {
    jsonStream << "  \"os_version\": \"" << escapeJson(osVersion_) << "\",\n";
  }
  if (!appVersion_.empty()) {
    jsonStream << "  \"app_version\": \"" << escapeJson(appVersion_) << "\",\n";
  }
  if (sampleRate_ > 0.0f) {
    jsonStream << "  \"sample_rate\": " << sampleRate_ << ",\n";
  }
  
  // Custom metadata object (optional)
  if (!metadata_.empty()) {
    jsonStream << "  \"metadata\": {\n";
    bool first = true;
    for (const auto& kv : metadata_) {
      if (!first) jsonStream << ",\n";
      jsonStream << "    \"" << escapeJson(kv.first) << "\": \"" << escapeJson(kv.second) << "\"";
      first = false;
    }
    jsonStream << "\n  },\n";
  }
  
  // Sensor readings array
  jsonStream << "  \"readings\": [\n";
  for (size_t i = 0; i < readings_.size(); ++i) {
    const SensorReading& r = readings_[i];
    jsonStream << "    {\n";
    jsonStream << "      \"timestamp\": " << r.timestamp << ",\n";
    jsonStream << "      \"accelerometer\": " << vectorToJson(r.acc) << ",\n";
    jsonStream << "      \"gyroscope\": " << vectorToJson(r.gyro) << ",\n";
    jsonStream << "      \"magnetometer\": " << vectorToJson(r.mag) << ",\n";
    jsonStream << "      \"gps\": {\"lat\": " << r.gpsLat << ", \"lon\": " << r.gpsLon
    << ", \"alt\": " << r.gpsAlt << ", \"acc\": " << r.gpsAccuracy << "},\n";
    jsonStream << "      \"gps_velocity\": " << vectorToJson(r.gpsVel) << ",\n";
    jsonStream << "      \"baro_alt\": " << r.baroAlt << ",\n";
    jsonStream << "      \"pressure\": " << r.pressure << "\n";
    jsonStream << "    }";
    if (i < readings_.size() - 1) jsonStream << ","; // Comma between array elements
    jsonStream << "\n";
  }
  jsonStream << "  ]\n";
  jsonStream << "}\n";
  
  // Convert JSON to string
  std::string jsonStr = jsonStream.str();
  
  // Write compressed JSON to file using gzip (compression level 9 = maximum)
  // Mode "wb9" = write binary, compression level 9
  gzFile gzf = gzopen(filepath_.c_str(), "wb9");
  if (!gzf) {
    // Failed to open file (permission denied, disk full, invalid path, etc.)
    return false;
  }
  
  // Write entire JSON string to compressed file
  int written = gzwrite(gzf, jsonStr.c_str(), jsonStr.size());
  gzclose(gzf);
  
  // Verify all data was written
  if (written != (int)jsonStr.size()) {
    return false;
  }
  
  // Stop recording (cannot append more readings after save)
  recording_ = false;
  return true;
}

void JsonRecorder::setDeviceMetadata(const std::string& deviceModel,
                                     const std::string& osVersion,
                                     const std::string& appVersion) {
  // Store device metadata (written to JSON on save())
  deviceModel_ = deviceModel;
  osVersion_ = osVersion;
  appVersion_ = appVersion;
}

void JsonRecorder::addMetadata(const std::string& key, const std::string& value) {
  // Add custom metadata key-value pair (written to JSON "metadata" object on save())
  metadata_[key] = value;
}

std::string JsonRecorder::escapeJson(const std::string& str) {
  // Escape special characters according to JSON specification (RFC 7159)
  // Handles quotes, backslashes, control characters, etc.
  std::ostringstream oss;
  for (char c : str) {
    switch (c) {
      case '"':  oss << "\\\""; break;  // Quote
      case '\\': oss << "\\\\"; break;  // Backslash
      case '\b': oss << "\\b"; break;   // Backspace
      case '\f': oss << "\\f"; break;   // Form feed
      case '\n': oss << "\\n"; break;   // Newline
      case '\r': oss << "\\r"; break;   // Carriage return
      case '\t': oss << "\\t"; break;   // Tab
      default:
        // Control characters (< 32) encoded as \uXXXX
        if (c < 32) {
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') << (int)c;
        } else {
          // Regular character - output as-is
          oss << c;
        }
    }
  }
  return oss.str();
}

std::string JsonRecorder::vectorToJson(const float v[3]) {
  // Convert 3-element float array to JSON array string format: "[x, y, z]"
  // Uses 6 decimal places precision for sensor data
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);
  oss << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
  return oss.str();
}
