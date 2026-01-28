
#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * Recording and Playback category for NativeAhrs
 * 
 * Handles recording sensor data to binary files and playback of recorded sessions.
 */
@interface NativeAhrs (Recording)

/**
 * Starts recording sensor data to a file with an auto-generated name
 */
- (void)startRecording;

/**
 * Stops recording and closes the file
 */
- (void)stopRecording;

/**
 * Starts playback of a recorded file
 * 
 * @param filename - Name of the file to play back
 */
- (void)playbackRecording:(NSString *)filename;

/**
 * Stops playback
 */
- (void)stopPlayback;

/**
 * Gets list of recording files
 * 
 * @param resolve - Promise resolver with array of file info
 * @param reject - Promise rejecter
 */
- (void)getRecordingFiles:(RCTPromiseResolveBlock)resolve reject:(RCTPromiseRejectBlock)reject;

/**
 * Deletes a recording file
 * 
 * @param filename - Name of the file to delete
 */
- (void)deleteRecording:(NSString *)filename;

/**
 * Writes a sensor data packet to the recording file
 * 
 * @param packetType - Type of packet (1=gyro, 2=accel, 3=mag, 4=baro, 5=gps)
 * @param timestamp - Timestamp in microseconds
 * @param data - Pointer to data bytes
 * @param dataLength - Length of data in bytes
 */
- (void)writeRecordingPacket:(uint8_t)packetType timestamp:(uint64_t)timestamp data:(const void *)data length:(size_t)dataLength;

@end

NS_ASSUME_NONNULL_END
