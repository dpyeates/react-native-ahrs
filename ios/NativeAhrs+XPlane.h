
#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * X-Plane WebSocket connection category for NativeAhrs
 * 
 * Handles WebSocket connection to X-Plane plugin for receiving simulated sensor data.
 * When connected, real device sensors are bypassed and X-Plane data feeds the EKF.
 */
@interface NativeAhrs (XPlane) <NSURLSessionWebSocketDelegate>

/**
 * Connects to X-Plane plugin via WebSocket
 * 
 * @param host - Hostname or IP address of X-Plane computer (e.g., "192.168.1.100")
 * 
 * Connection is made to ws://host:8765
 * When connected successfully, real sensors are bypassed and X-Plane data feeds EKF.
 */
- (void)connectToXPlane:(NSString *)host;

/**
 * Disconnects from X-Plane plugin
 * 
 * Closes WebSocket connection and returns to using real device sensors.
 */
- (void)disconnectFromXPlane;

/**
 * Handles incoming X-Plane WebSocket message
 * 
 * @param json - JSON string containing sensor data
 * 
 * Parses JSON and feeds data to EKF, identical to playback processing.
 */
- (void)handleXPlaneMessage:(NSString *)json;

/**
 * Emits X-Plane connection state change event to React Native
 * 
 * @param connected - Whether connected (YES) or disconnected (NO)
 */
- (void)emitXPlaneConnectionChanged:(BOOL)connected;

@end

NS_ASSUME_NONNULL_END
