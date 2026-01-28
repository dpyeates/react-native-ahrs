#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * @category NativeAhrs (Location)
 * @brief CLLocationManager delegate methods and GPS data processing
 *
 * Handles all location-related functionality:
 * - GPS location updates
 * - Heading updates (for initial heading)
 * - Location authorization
 * - GPS data validation and processing
 */
@interface NativeAhrs (Location)

/**
 * @brief Check if GPS data has acceptable accuracy for EKF updates
 *
 * Uses fixed accuracy thresholds:
 * - Horizontal accuracy: ≤ 5.0 meters
 * - Vertical accuracy: ≤ 10.0 meters (if available)
 *
 * @param gps GPS position data structure to check
 * @return YES if accuracy is acceptable, NO otherwise
 */
- (BOOL)hasAcceptableGpsAccuracy:(gps_position_t)gps;

@end

NS_ASSUME_NONNULL_END














