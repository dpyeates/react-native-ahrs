#import "NativeAhrs+Recording.h"
#import "NativeAhrs+Emission.h"
#import "NativeAhrs+Sensors.h"
#import "NativeAhrs.h"
#import <Foundation/Foundation.h>
#import <CoreLocation/CoreLocation.h>
#include <cmath>
#include <zlib.h>
#include "JsonRecorder.h"

// Packet types
#define PACKET_TYPE_GYRO 1
#define PACKET_TYPE_ACCEL 2
#define PACKET_TYPE_MAG 3
#define PACKET_TYPE_BARO 4
#define PACKET_TYPE_GPS 5

#ifdef DEBUG
#define AHRS_LOG(fmt, ...) NSLog(fmt, ##__VA_ARGS__)
#else
#define AHRS_LOG(fmt, ...) ((void)0)
#endif

// Data point structure for JSON recording
@interface RecordingDataPoint : NSObject
@property (nonatomic, assign) uint64_t timestamp;
@property (nonatomic, assign) float mag_x;
@property (nonatomic, assign) float mag_y;
@property (nonatomic, assign) float mag_z;
@property (nonatomic, assign) BOOL hasMag;
@property (nonatomic, assign) float acc_x;
@property (nonatomic, assign) float acc_y;
@property (nonatomic, assign) float acc_z;
@property (nonatomic, assign) BOOL hasAcc;
@property (nonatomic, assign) float gyro_x;
@property (nonatomic, assign) float gyro_y;
@property (nonatomic, assign) float gyro_z;
@property (nonatomic, assign) BOOL hasGyro;
@property (nonatomic, assign) float press;
@property (nonatomic, assign) BOOL hasPress;
@property (nonatomic, assign) double gps_lat;
@property (nonatomic, assign) double gps_lon;
@property (nonatomic, assign) float gps_alt;
@property (nonatomic, assign) float gps_trk;
@property (nonatomic, assign) float gps_spd;
@property (nonatomic, assign) float gps_vel_n;
@property (nonatomic, assign) float gps_vel_e;
@property (nonatomic, assign) float gps_vel_d;
@property (nonatomic, assign) float gps_accuracy;
@property (nonatomic, assign) BOOL hasGps;
@end

@implementation RecordingDataPoint
@end

@implementation NativeAhrs (Recording)

- (NSString *)documentsDirectory {
  NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
  return [paths firstObject];
}

- (NSString *)defaultRecordingFilename {
  NSDateFormatter *formatter = [[NSDateFormatter alloc] init];
  formatter.locale = [NSLocale localeWithLocaleIdentifier:@"en_US_POSIX"];
  formatter.timeZone = [NSTimeZone localTimeZone];
  formatter.dateFormat = @"yyMMddHHmmss";
  
  NSString *timestamp = [formatter stringFromDate:[NSDate date]];
  return [NSString stringWithFormat:@"%@.json.gz", timestamp];
}

- (void)startRecording {
  if (self.isRecording) {
    AHRS_LOG(@"⚠️ Already recording, stopping previous recording");
    [self stopRecording];
  }
  
  if (self.isPlaying) {
    AHRS_LOG(@"⚠️ Cannot record while playing back");
    return;
  }
  
  // Initialize data points dictionary (still used to merge packets with same timestamp)
  self.recordingDataPoints = [NSMutableDictionary dictionary];
  
  self.isRecording = YES;
  self.recordingStartTimestamp = (uint64_t)([[NSDate date] timeIntervalSince1970] * 1e6);
  self.recordingPacketCount = 0;
  
  AHRS_LOG(@"✅ Started recording");
}

- (void)stopRecording {
  if (!self.isRecording) {
    return;
  }
  
  // Generate filename
  NSString *finalFilename = [self defaultRecordingFilename];
  NSString *documentsDir = [self documentsDirectory];
  NSString *filePath = [documentsDir stringByAppendingPathComponent:finalFilename];
  
  // Ensure Documents directory exists
  NSFileManager *fileManager = [NSFileManager defaultManager];
  NSError *dirError = nil;
  if (![fileManager createDirectoryAtPath:documentsDir withIntermediateDirectories:YES attributes:nil error:&dirError]) {
    AHRS_LOG(@"⚠️ Failed to create Documents directory: %@", dirError.localizedDescription);
  }
  
  // Convert file path to std::string
  std::string jsonPath = [filePath UTF8String];
  
  // Create JSON recorder
  JsonRecorder recorder;
  if (!recorder.startSession(jsonPath, 
                             [[UIDevice currentDevice].identifierForVendor.UUIDString UTF8String],
                             [[NSUUID UUID].UUIDString UTF8String])) {
    AHRS_LOG(@"❌ Failed to start JSON recording session");
    self.recordingDataPoints = nil;
    self.isRecording = NO;
    return;
  }
  
  // Set device metadata
  recorder.setDeviceMetadata(
                             [[UIDevice currentDevice].model UTF8String],
                             [[UIDevice currentDevice].systemVersion UTF8String],
                             [[[NSBundle mainBundle] objectForInfoDictionaryKey:@"CFBundleShortVersionString"] UTF8String] ?: "unknown"
                             );
  recorder.setSampleRate(100.0f); // Nominal sample rate
  
  // Sort timestamps
  NSArray *sortedTimestamps = [[self.recordingDataPoints allKeys] sortedArrayUsingSelector:@selector(compare:)];
  
  // Compute barometer altitude from pressure using standard atmosphere formula
  auto computeBaroAltitude = [](float pressure_hpa) -> float {
    if (pressure_hpa <= 0.0f) return 0.0f;
    float ratio = pressure_hpa / 1013.25f;
    return 44330.0f * (1.0f - powf(ratio, 0.1903f));
  };
  
  // Write all data points to JSON in sorted order
  for (NSNumber *timestampKey in sortedTimestamps) {
    RecordingDataPoint *point = self.recordingDataPoints[timestampKey];
    
    // Convert timestamp from microseconds to seconds (Unix timestamp)
    double timestamp_seconds = (double)point.timestamp / 1e6;
    
    // Prepare sensor data (use zeros if not available)
    float acc[3] = {
      point.hasAcc ? point.acc_x : 0.0f,
      point.hasAcc ? point.acc_y : 0.0f,
      point.hasAcc ? point.acc_z : 0.0f
    };
    float gyro[3] = {
      point.hasGyro ? point.gyro_x : 0.0f,
      point.hasGyro ? point.gyro_y : 0.0f,
      point.hasGyro ? point.gyro_z : 0.0f
    };
    float mag[3] = {
      point.hasMag ? point.mag_x : 0.0f,
      point.hasMag ? point.mag_y : 0.0f,
      point.hasMag ? point.mag_z : 0.0f
    };
    float gpsVel[3] = {
      point.hasGps ? point.gps_vel_n : 0.0f,
      point.hasGps ? point.gps_vel_e : 0.0f,
      point.hasGps ? point.gps_vel_d : 0.0f
    };
    
    double gpsLat = point.hasGps ? point.gps_lat : 0.0;
    double gpsLon = point.hasGps ? point.gps_lon : 0.0;
    double gpsAlt = point.hasGps ? point.gps_alt : 0.0f;
    float gpsAccuracy = point.hasGps ? point.gps_accuracy : 0.0f;
    float baroAlt = point.hasPress ? computeBaroAltitude(point.press) : 0.0f;
    float pressure = point.hasPress ? point.press : 0.0f;
    
    recorder.appendReading(
                           timestamp_seconds,
                           acc, gyro, mag,
                           gpsLat, gpsLon, gpsAlt, gpsAccuracy,
                           gpsVel,
                           baroAlt,
                           pressure
                           );
  }
  
  // Save JSON file
  if (!recorder.save()) {
    AHRS_LOG(@"❌ Failed to save JSON file: %@", filePath);
    self.recordingDataPoints = nil;
    self.isRecording = NO;
    return;
  }
  
  // Verify file was written
  NSFileManager *fm = [NSFileManager defaultManager];
  if ([fm fileExistsAtPath:filePath]) {
    NSDictionary *attrs = [fm attributesOfItemAtPath:filePath error:nil];
    NSNumber *fileSize = attrs[NSFileSize];
    AHRS_LOG(@"✅ Stopped recording. Wrote %d data points to: %@ (size: %@ bytes)", 
             recorder.getReadingCount(), finalFilename, fileSize);
  } else {
    AHRS_LOG(@"⚠️ File save reported success but file not found at: %@", filePath);
  }
  
  self.recordingDataPoints = nil;
  self.isRecording = NO;
}

- (void)writeRecordingPacket:(uint8_t)packetType timestamp:(uint64_t)timestamp data:(const void *)data length:(size_t)dataLength {
  if (!self.isRecording || !self.recordingDataPoints) {
    return;
  }
  
  NSNumber *timestampKey = @(timestamp);
  RecordingDataPoint *point = self.recordingDataPoints[timestampKey];
  
  if (!point) {
    point = [[RecordingDataPoint alloc] init];
    point.timestamp = timestamp;
    self.recordingDataPoints[timestampKey] = point;
  }
  
  // Parse data based on packet type
  switch (packetType) {
    case PACKET_TYPE_GYRO: {
      if (dataLength >= sizeof(float) * 3) {
        const float *values = (const float *)data;
        point.gyro_x = values[0];
        point.gyro_y = values[1];
        point.gyro_z = values[2];
        point.hasGyro = YES;
      }
      break;
    }
    case PACKET_TYPE_ACCEL: {
      if (dataLength >= sizeof(float) * 3) {
        const float *values = (const float *)data;
        point.acc_x = values[0];
        point.acc_y = values[1];
        point.acc_z = values[2];
        point.hasAcc = YES;
      }
      break;
    }
    case PACKET_TYPE_MAG: {
      if (dataLength >= sizeof(float) * 3) {
        const float *values = (const float *)data;
        point.mag_x = values[0];
        point.mag_y = values[1];
        point.mag_z = values[2];
        point.hasMag = YES;
      }
      break;
    }
    case PACKET_TYPE_BARO: {
      if (dataLength >= sizeof(float)) {
        const float *values = (const float *)data;
        point.press = values[0];
        point.hasPress = YES;
      }
      break;
    }
    case PACKET_TYPE_GPS: {
      // GPS payload: lat (double), lon (double), alt (float), speed (float), course (float), vel_n (float), vel_e (float), vel_d (float)
      if (dataLength >= sizeof(double) + sizeof(double) + sizeof(float) * 6) {
        const uint8_t *bytes = (const uint8_t *)data;
        size_t offset = 0;
        
        double lat, lon;
        float alt, speed, course, vel_n, vel_e, vel_d;
        
        memcpy(&lat, bytes + offset, sizeof(double));
        offset += sizeof(double);
        memcpy(&lon, bytes + offset, sizeof(double));
        offset += sizeof(double);
        memcpy(&alt, bytes + offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&speed, bytes + offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&course, bytes + offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&vel_n, bytes + offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&vel_e, bytes + offset, sizeof(float));
        offset += sizeof(float);
        memcpy(&vel_d, bytes + offset, sizeof(float));
        
        point.gps_lat = lat;
        point.gps_lon = lon;
        point.gps_alt = alt;
        point.gps_spd = speed;
        point.gps_trk = course;
        point.gps_vel_n = vel_n;
        point.gps_vel_e = vel_e;
        point.gps_vel_d = vel_d;
        point.hasGps = YES;
      }
      break;
    }
  }
  
  self.recordingPacketCount++;
}

- (void)playbackRecording:(NSString *)filename {
  if (self.isPlaying) {
    [self stopPlaybackWithStatus:@"stopped" reason:@"interrupted"];
  }
  
  if (self.isRecording) {
    AHRS_LOG(@"⚠️ Cannot play back while recording");
    return;
  }
  
  if (!self.running) {
    AHRS_LOG(@"⚠️ Cannot play back when AHRS is not running");
    return;
  }
  
  self.currentPlaybackFilename = filename;
  NSString *filePath = [[self documentsDirectory] stringByAppendingPathComponent:filename];
  
  // Read file (gzipped or plain)
  NSError *error = nil;
  NSData *fileData = [NSData dataWithContentsOfFile:filePath options:0 error:&error];
  if (!fileData) {
    AHRS_LOG(@"❌ Failed to read file: %@ - %@", filePath, error.localizedDescription);
    self.currentPlaybackFilename = nil;
    return;
  }
  
  NSData *jsonData = fileData;
  
  // Decompress if it's a gzipped file (check magic bytes 0x1f 0x8b)
  if (fileData.length >= 2) {
    const unsigned char *bytes = (const unsigned char *)fileData.bytes;
    if (bytes[0] == 0x1f && bytes[1] == 0x8b) {
      // File is gzipped, decompress it
      jsonData = [self decompressGzip:fileData];
      if (!jsonData) {
        AHRS_LOG(@"❌ Failed to decompress gzipped file: %@", filePath);
        self.currentPlaybackFilename = nil;
        return;
      }
      AHRS_LOG(@"✅ Decompressed %lu bytes -> %lu bytes", (unsigned long)fileData.length, (unsigned long)jsonData.length);
    }
  }
  
  // Parse JSON
  NSDictionary *jsonDict = [NSJSONSerialization JSONObjectWithData:jsonData options:0 error:&error];
  if (!jsonDict) {
    AHRS_LOG(@"❌ Failed to parse JSON: %@", error.localizedDescription);
    self.currentPlaybackFilename = nil;
    return;
  }
  
  // Extract readings array
  NSArray *readings = jsonDict[@"readings"];
  if (!readings || readings.count == 0) {
    AHRS_LOG(@"❌ No readings found in JSON file");
    self.currentPlaybackFilename = nil;
    return;
  }
  
  AHRS_LOG(@"📖 Loading %lu readings from JSON file", (unsigned long)readings.count);
  
  // Convert to playback format (array of RecordingDataPoint objects)
  self.playbackPackets = [NSMutableArray array];
  for (NSDictionary *reading in readings) {
    RecordingDataPoint *point = [[RecordingDataPoint alloc] init];
    
    // Get timestamp (convert from seconds to microseconds)
    double timestamp_seconds = [reading[@"timestamp"] doubleValue];
    point.timestamp = (uint64_t)(timestamp_seconds * 1e6);
    
    // Get accelerometer
    NSArray *acc = reading[@"accelerometer"];
    if (acc && acc.count == 3) {
      point.acc_x = [acc[0] floatValue];
      point.acc_y = [acc[1] floatValue];
      point.acc_z = [acc[2] floatValue];
      point.hasAcc = YES; // If data exists in JSON, we have it
    }
    
    // Get gyroscope
    NSArray *gyro = reading[@"gyroscope"];
    if (gyro && gyro.count == 3) {
      point.gyro_x = [gyro[0] floatValue];
      point.gyro_y = [gyro[1] floatValue];
      point.gyro_z = [gyro[2] floatValue];
      point.hasGyro = YES; // If data exists in JSON, we have it
    }
    
    // Get magnetometer
    NSArray *mag = reading[@"magnetometer"];
    if (mag && mag.count == 3) {
      point.mag_x = [mag[0] floatValue];
      point.mag_y = [mag[1] floatValue];
      point.mag_z = [mag[2] floatValue];
      point.hasMag = YES; // If data exists in JSON, we have it
    }
    
    // Get GPS data
    NSDictionary *gps = reading[@"gps"];
    if (gps) {
      point.gps_lat = [gps[@"lat"] doubleValue];
      point.gps_lon = [gps[@"lon"] doubleValue];
      point.gps_alt = [gps[@"alt"] floatValue];
      point.gps_accuracy = [gps[@"acc"] floatValue];
      point.hasGps = (point.gps_lat != 0.0 || point.gps_lon != 0.0 || point.gps_alt != 0.0f);
    }
    
    // Get GPS velocity
    NSArray *gpsVel = reading[@"gps_velocity"];
    if (gpsVel && gpsVel.count == 3) {
      point.gps_vel_n = [gpsVel[0] floatValue];
      point.gps_vel_e = [gpsVel[1] floatValue];
      point.gps_vel_d = [gpsVel[2] floatValue];
      
      // Compute speed and track from velocity
      point.gps_spd = sqrtf(point.gps_vel_n * point.gps_vel_n + point.gps_vel_e * point.gps_vel_e);
      point.gps_trk = atan2f(point.gps_vel_e, point.gps_vel_n) * 180.0f / M_PI;
      if (point.gps_trk < 0.0f) point.gps_trk += 360.0f;
    }
    
    // Get barometric pressure
    point.press = [reading[@"pressure"] floatValue];
    point.hasPress = (point.press > 0.0f);
    
    [self.playbackPackets addObject:point];
  }
  
  AHRS_LOG(@"✅ Loaded %lu data points from JSON", (unsigned long)[self.playbackPackets count]);
  
  // Check if recording has any valid GPS data
  BOOL hasAnyGps = NO;
  for (RecordingDataPoint *point in self.playbackPackets) {
    if (point.hasGps && (point.gps_lat != 0.0 || point.gps_lon != 0.0)) {
      hasAnyGps = YES;
      break;
    }
  }
  
  // Start playback
  self.isPlaying = YES;
  self.nextEmitTime = 0; // allow playback packets to emit immediately
  
  // Reset filter to initial state before starting playback
  // This ensures playback starts from the recording's initial angles, not current device state
  AHRS_LOG(@"🔄 Starting playback - resetting EKF to initial state");
  [self resetAhrs];
  
  // If no GPS data in recording, use a default location to initialize the filter
  // This allows playback to work for indoor recordings without GPS
  // MUST be set AFTER resetAhrs, which resets hasGpsFix to NO
  if (!hasAnyGps) {
    AHRS_LOG(@"⚠️ No GPS data in recording - using default location for playback");
    // Use London coordinates as default (51.5074° N, 0.1278° W, 0m altitude)
    self.hasGpsFix = YES;
    self.lastLatRad = 51.5074 * M_PI / 180.0;
    self.lastLonRad = -0.1278 * M_PI / 180.0;
    self.lastAltM = 0.0;
    self.lastGpsTimestampUs = 0;
    self.currentTow = 0;
    [self updateMagneticDeclinationIfNeededForLatRad:self.lastLatRad
                                              lonRad:self.lastLonRad
                                                altM:self.lastAltM];
  }
  
  [self emitPlaybackStateChangeWithStatus:@"started"
                                 filename:self.currentPlaybackFilename
                                   reason:nil];
  
  // Process first data point immediately, then set index to 1
  // This must happen BEFORE timer starts to avoid race condition
  if ([self.playbackPackets count] > 0) {
    [self processPlaybackDataPoint:self.playbackPackets[0]];
  }
  
  // Set index to 1 BEFORE creating timer to avoid race condition
  self.playbackCurrentIndex = 1;
  
  // Schedule timer for remaining packets (60Hz = ~16.67ms)
  if ([NSThread isMainThread]) {
    self.playbackTimer = [NSTimer scheduledTimerWithTimeInterval:1.0/60.0
                                                          target:self
                                                        selector:@selector(playbackTimerTick:)
                                                        userInfo:nil
                                                         repeats:YES];
    [[NSRunLoop currentRunLoop] addTimer:self.playbackTimer forMode:NSRunLoopCommonModes];
  } else {
    dispatch_async(dispatch_get_main_queue(), ^{
      self.playbackTimer = [NSTimer scheduledTimerWithTimeInterval:1.0/60.0
                                                            target:self
                                                          selector:@selector(playbackTimerTick:)
                                                          userInfo:nil
                                                           repeats:YES];
      [[NSRunLoop currentRunLoop] addTimer:self.playbackTimer forMode:NSRunLoopCommonModes];
    });
  }
}

- (void)playbackTimerTick:(NSTimer *)timer {
  if (!self.isPlaying) {
    return;
  }
  
  if (self.playbackCurrentIndex >= [self.playbackPackets count]) {
    [self stopPlaybackWithStatus:@"completed" reason:@"end_of_file"];
    return;
  }
  
  // Process next data point
  RecordingDataPoint *point = self.playbackPackets[self.playbackCurrentIndex];
  [self processPlaybackDataPoint:point];
  self.playbackCurrentIndex++;
}

- (void)processPlaybackDataPoint:(RecordingDataPoint *)point {
  if (!point) {
    return;
  }
  
  uint64_t timestamp = point.timestamp;
  
  double gyro_x = point.hasGyro ? point.gyro_x : 0.0;
  double gyro_y = point.hasGyro ? point.gyro_y : 0.0;
  double gyro_z = point.hasGyro ? point.gyro_z : 0.0;
  double accel_x = point.hasAcc ? point.acc_x : 0.0;
  double accel_y = point.hasAcc ? point.acc_y : 0.0;
  double accel_z = point.hasAcc ? point.acc_z : 0.0;
  double mag_x = point.hasMag ? point.mag_x : 0.0;
  double mag_y = point.hasMag ? point.mag_y : 0.0;
  double mag_z = point.hasMag ? point.mag_z : 0.0;
  
  BOOL hasGps = point.hasGps;
  double latDeg = hasGps ? point.gps_lat : 0.0;
  double lonDeg = hasGps ? point.gps_lon : 0.0;
  double altM = hasGps ? point.gps_alt : 0.0;
  // Use GPS velocity from point (already in NED frame)
  double velN = hasGps ? point.gps_vel_n : 0.0;
  double velE = hasGps ? point.gps_vel_e : 0.0;
  double velD = hasGps ? point.gps_vel_d : 0.0;
  
  if (_filter && point.hasGyro && point.hasAcc) {
    // Check if we have new GPS data
    BOOL hasNewGps = NO;
    if (hasGps) {
      double newLatRad = latDeg * M_PI / 180.0;
      double newLonRad = lonDeg * M_PI / 180.0;
      // Check if GPS data has changed (using small epsilon for floating point comparison)
      if (!self.hasGpsFix || 
          fabs(newLatRad - self.lastLatRad) > 1e-8 ||
          fabs(newLonRad - self.lastLonRad) > 1e-8 ||
          fabs(altM - self.lastAltM) > 0.1 ||
          timestamp != self.lastGpsTimestampUs) {
        hasNewGps = YES;
        self.hasGpsFix = YES;
        self.lastLatRad = newLatRad;
        self.lastLonRad = newLonRad;
        self.lastAltM = altM;
        self.lastGpsTimestampUs = timestamp;
        // Update TOW with new GPS data
        self.currentTow = (unsigned long)(timestamp / 1000ULL);
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
      if (dt < 1e-4) dt = 1e-4;
      if (dt > 0.2) dt = 0.2;
    }
    self.lastTimestampUs = timestamp;
    
    // Use current TOW (only updates when GPS changes)
    unsigned long tow = self.currentTow;
    
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
                    (float)gyro_x,
                    (float)gyro_y,
                    (float)gyro_z,
                    (float)accel_x,
                    (float)accel_y,
                    (float)accel_z,
                    (float)mag_x,
                    (float)mag_y,
                    (float)mag_z,
                    self.expectedMagN_nT,
                    self.expectedMagE_nT,
                    self.expectedMagD_nT
                    );
    
    self.filterInitialized = YES;
  }
  
  [self emitAhrsUpdateWithTimestamp:timestamp];
}

- (void)stopPlaybackWithStatus:(NSString *)status reason:(NSString *)reason {
  BOOL wasPlaying = self.isPlaying || self.playbackTimer != nil;
  NSString *filename = self.currentPlaybackFilename;
  if (self.playbackTimer) {
    [self.playbackTimer invalidate];
    self.playbackTimer = nil;
  }
  self.isPlaying = NO;
  self.playbackPackets = nil;
  self.playbackCurrentIndex = 0;
  self.currentPlaybackFilename = nil;
  
  // Restart heading updates for reinitialization
  if ([CLLocationManager headingAvailable]) {
    [self.locationManager startUpdatingHeading];
    AHRS_LOG(@"🔄 Playback ended - EKF reset, restarting heading updates");
  }
  
  // Reset ALL EKF state - it reflects playback data, not real device state
  // resetAhrs resets: quaternion, velocity, position, biases, wind, covariance, etc.
  // and sets filterInitialized = NO
  [self resetAhrs];
  
  
  AHRS_LOG(@"✅ Stopped playback");
  
  if (wasPlaying) {
    [self emitPlaybackStateChangeWithStatus:status filename:filename reason:reason];
  }
}

- (void)stopPlayback {
  if (!self.isPlaying && !self.playbackTimer) {
    return;
  }
  [self stopPlaybackWithStatus:@"stopped" reason:@"user"];
}

- (void)getRecordingFiles:(RCTPromiseResolveBlock)resolve reject:(RCTPromiseRejectBlock)reject {
  NSString *documentsDir = [self documentsDirectory];
  NSFileManager *fileManager = [NSFileManager defaultManager];
  
  NSError *error = nil;
  NSArray *files = [fileManager contentsOfDirectoryAtPath:documentsDir error:&error];
  
  if (error) {
    reject(@"FILE_ERROR", @"Failed to list files", error);
    return;
  }
  
  NSMutableArray *recordingFiles = [NSMutableArray array];
  
  for (NSString *filename in files) {
    // Support .json (uncompressed) and .json.gz (compressed)
    if ([filename hasSuffix:@".json"] || [filename hasSuffix:@".json.gz"]) {
      NSString *filePath = [documentsDir stringByAppendingPathComponent:filename];
      NSDictionary *attributes = [fileManager attributesOfItemAtPath:filePath error:nil];
      
      if (attributes) {
        NSNumber *fileSize = attributes[NSFileSize];
        NSDate *modificationDate = attributes[NSFileModificationDate];
        
        [recordingFiles addObject:@{
          @"filename": filename,
          @"size": fileSize ?: @0,
          @"date": @([modificationDate timeIntervalSince1970] * 1000) // Convert to milliseconds
        }];
      }
    }
  }
  
  // Sort by date (newest first)
  [recordingFiles sortUsingComparator:^NSComparisonResult(NSDictionary *obj1, NSDictionary *obj2) {
    NSNumber *date1 = obj1[@"date"];
    NSNumber *date2 = obj2[@"date"];
    return [date2 compare:date1];
  }];
  
  resolve(recordingFiles);
}

- (void)deleteRecording:(NSString *)filename {
  NSString *filePath = [[self documentsDirectory] stringByAppendingPathComponent:filename];
  NSFileManager *fileManager = [NSFileManager defaultManager];
  
  NSError *error = nil;
  if ([fileManager removeItemAtPath:filePath error:&error]) {
    AHRS_LOG(@"✅ Deleted recording: %@", filename);
  } else {
    AHRS_LOG(@"❌ Failed to delete recording: %@ - %@", filename, error.localizedDescription);
  }
}

// Helper method to decompress gzipped data
- (NSData *)decompressGzip:(NSData *)gzippedData {
  if (!gzippedData || gzippedData.length == 0) {
    return nil;
  }
  
  // Check gzip magic bytes
  const unsigned char *bytes = (const unsigned char *)gzippedData.bytes;
  if (gzippedData.length < 2 || bytes[0] != 0x1f || bytes[1] != 0x8b) {
    return gzippedData; // Not gzipped, return as-is
  }
  
  // Use zlib to decompress
  z_stream stream;
  stream.zalloc = Z_NULL;
  stream.zfree = Z_NULL;
  stream.opaque = Z_NULL;
  stream.avail_in = (uInt)gzippedData.length;
  stream.next_in = (Bytef *)gzippedData.bytes;
  
  // Initialize with windowBits = 15 + 16 for gzip
  if (inflateInit2(&stream, 15 + 16) != Z_OK) {
    return nil;
  }
  
  NSMutableData *decompressed = [NSMutableData dataWithLength:gzippedData.length * 4]; // Initial guess: 4x compression
  
  do {
    if (stream.total_out >= decompressed.length) {
      [decompressed increaseLengthBy:gzippedData.length];
    }
    
    stream.next_out = (Bytef *)((unsigned char *)decompressed.mutableBytes + stream.total_out);
    stream.avail_out = (uInt)(decompressed.length - stream.total_out);
    
    int status = inflate(&stream, Z_SYNC_FLUSH);
    if (status == Z_STREAM_END) {
      break;
    }
    if (status != Z_OK) {
      inflateEnd(&stream);
      return nil;
    }
  } while (stream.avail_out == 0);
  
  inflateEnd(&stream);
  
  [decompressed setLength:stream.total_out];
  return decompressed;
}

@end
