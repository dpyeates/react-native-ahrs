/*
 * Unit tests for JsonRecorder
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <unistd.h>
#include <zlib.h>

#include "JsonRecorder.h"

static int g_failed = 0;

static void expectTrue(bool condition, const char *message) {
  if (!condition) {
    g_failed++;
    std::printf("✗ %s\n", message);
  } else {
    std::printf("✓ %s\n", message);
  }
}

static std::string readGzipFile(const std::string &path) {
  gzFile file = gzopen(path.c_str(), "rb");
  if (!file) {
    return "";
  }

  std::string output;
  char buffer[4096];
  int bytesRead = 0;
  while ((bytesRead = gzread(file, buffer, sizeof(buffer))) > 0) {
    output.append(buffer, bytesRead);
  }
  gzclose(file);
  return output;
}

int main() {
  std::printf("JsonRecorder Unit Tests\n");

  char tmpTemplate[] = "/tmp/ahrs_json_recorder_XXXXXX";
  int fd = mkstemp(tmpTemplate);
  if (fd < 0) {
    std::printf("✗ Failed to create temp file\n");
    return 1;
  }
  close(fd);

  std::string gzipPath = std::string(tmpTemplate) + ".json.gz";
  unlink(tmpTemplate); // remove the placeholder file

  JsonRecorder recorder;

  expectTrue(recorder.startSession(gzipPath, "device-123", "session-abc"),
             "startSession succeeds");
  expectTrue(recorder.isRecording(), "isRecording true after startSession");

  recorder.setDeviceMetadata("iPhone", "iOS 18", "1.0.0");
  recorder.setSampleRate(100.0f);
  recorder.addMetadata("test_key", "test_value");

  float acc[3] = {0.1f, 0.2f, 0.3f};
  float gyro[3] = {0.01f, 0.02f, 0.03f};
  float mag[3] = {1.1f, 1.2f, 1.3f};
  float gpsVel[3] = {5.0f, 0.0f, -0.5f};

  recorder.appendReading(1700000000.0, acc, gyro, mag,
                         51.5074, -0.1278, 35.0, 3.0f,
                         gpsVel, 42.0f, 1010.0f);

  expectTrue(recorder.getReadingCount() == 1, "appendReading increases count");
  expectTrue(recorder.save(), "save() returns true");
  expectTrue(!recorder.isRecording(), "isRecording false after save");

  std::string json = readGzipFile(gzipPath);
  expectTrue(!json.empty(), "gzip file can be read");
  expectTrue(json.find("\"sample_count\": 1") != std::string::npos,
             "JSON contains sample_count");
  expectTrue(json.find("\"device_id\": \"device-123\"") != std::string::npos,
             "JSON contains device_id");
  expectTrue(json.find("\"session_id\": \"session-abc\"") != std::string::npos,
             "JSON contains session_id");
  expectTrue(json.find("\"metadata\"") != std::string::npos,
             "JSON contains metadata");

  unlink(gzipPath.c_str());

  std::printf("\nJsonRecorder: %s\n", g_failed == 0 ? "PASS ✓" : "FAIL ✗");
  return g_failed == 0 ? 0 : 1;
}
