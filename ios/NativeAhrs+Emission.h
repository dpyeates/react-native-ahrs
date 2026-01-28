
#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * @category NativeAhrs (Emission)
 * @brief React Native event emission and data formatting
 *
 * Handles formatting EKF output data and emitting events to React Native.
 */
@interface NativeAhrs (Emission)

/**
 * @brief Emit AHRS update event to React Native
 *
 * Formats the EKF output data and emits it via the React Native event system.
 * Reads data directly from EKF using getter functions.
 *
 * @param timestamp Timestamp in microseconds
 */
- (void)emitAhrsUpdateWithTimestamp:(uint64_t)timestamp;

/**
 * @brief Emit playback state change to React Native
 *
 * @param status - Playback status ("started", "stopped", "completed")
 * @param filename - Recording filename associated with the state change
 * @param reason - Optional reason for the transition (e.g. "end_of_file", "user")
 */
- (void)emitPlaybackStateChangeWithStatus:(NSString *)status
                                 filename:(nullable NSString *)filename
                                   reason:(nullable NSString *)reason;

/**
 * @brief Emit X-Plane connection state change to React Native
 *
 * @param connected - Whether X-Plane is connected (YES) or disconnected (NO)
 * @param host - The hostname/IP of the X-Plane computer
 */
- (void)emitXPlaneConnectionChangedWithConnected:(BOOL)connected
                                            host:(nullable NSString *)host;

@end

NS_ASSUME_NONNULL_END
