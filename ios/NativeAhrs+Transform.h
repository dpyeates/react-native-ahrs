#import "NativeAhrs.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * @category NativeAhrs (Transform)
 * @brief Coordinate frame transformation utilities
 *
 * Provides helper functions for transforming sensor data from iOS device frame
 * to aviation body frame, and for normalizing heading values.
 * These are internal helper functions used within the implementation.
 */
@interface NativeAhrs (Transform)
@end

void transformToBodyFrame(double ios_x, double ios_y, double ios_z, AhrsRotation rotation, double *out_x, double *out_y, double *out_z);
float normalizeHeadingDegrees(float heading_deg);
float normalizeAngleDifferenceDegrees(float angle_deg);

NS_ASSUME_NONNULL_END

