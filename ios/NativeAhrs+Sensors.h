
#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * @category NativeAhrs (Sensors)
 * @brief Sensor data processing and EKF updates
 *
 * Handles processing of device motion data (IMU sensors) and barometer data,
 * transforms them to body frame, and updates the EKF.
 */
@interface NativeAhrs (Sensors)

/**
 * @brief Process device motion data from Core Motion
 *
 * Transforms IMU sensor data (accelerometer, gyroscope, magnetometer) to body frame,
 * initializes EKF attitude if needed, and updates the EKF with sensor data.
 *
 * @param motion CMDeviceMotion object containing IMU sensor data
 */
- (void)ProcessDeviceMotionData:(CMDeviceMotion *)motion;

/**
 * @brief Process altitude data from barometer
 *
 * Processes barometric pressure data, calibrates against GPS if available,
 * and stores the data for EKF fusion.
 *
 * @param altitudeData CMAltitudeData object containing barometric pressure
 */
- (void)ProcessAltitudeData:(CMAltitudeData *)altitudeData;

/**
 * @brief Update magnetic declination from XYZgeomag if position moved far enough
 *
 * Recalculates declination when the filter position moves more than 10 nautical miles
 * from the last declination calculation point to avoid heavy computation each update.
 *
 * @param latRad Current latitude in radians
 * @param lonRad Current longitude in radians
 * @param altM Current altitude in meters (MSL)
 */
- (void)updateMagneticDeclinationIfNeededForLatRad:(double)latRad
                                            lonRad:(double)lonRad
                                              altM:(double)altM;

@end

NS_ASSUME_NONNULL_END

