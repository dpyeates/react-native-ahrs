#import "NativeAhrs+Transform.h"

/* =============================================================================
 * COORDINATE FRAME TRANSFORMATIONS
 * =============================================================================
 */

/**
 * @brief Transform coordinates from iOS device frame to aviation body frame
 *
 * iOS device frame:
 * - X: Right (positive = right)
 * - Y: Forward (positive = forward, toward top of device)
 * - Z: Up (positive = up, out of screen)
 *
 * Aviation body frame (NED convention):
 * - X: Forward (positive = forward, nose direction)
 * - Y: Right (positive = right wing)
 * - Z: Down (positive = down, toward ground)
 *
 * The transformation depends on device rotation/mounting:
 * - Vertical (Portrait): Device held vertically, top up
 * - Left (Landscape Left): Rotated 90° CCW from portrait
 * - Right (Landscape Right): Rotated 90° CW from portrait
 *
 * @param ios_x Input X coordinate in iOS device frame
 * @param ios_y Input Y coordinate in iOS device frame
 * @param ios_z Input Z coordinate in iOS device frame
 * @param rotation Device rotation/mounting orientation
 * @param out_x Output: transformed X in aviation body frame
 * @param out_y Output: transformed Y in aviation body frame
 * @param out_z Output: transformed Z in aviation body frame
 */
void transformToBodyFrame(double ios_x, double ios_y, double ios_z, AhrsRotation rotation, double *out_x, double *out_y, double *out_z) {
  // iOS device frame: x=right, y=forward (top edge), z=up (out of screen)
  // Aviation body NED: x=forward, y=right, z=down
  //
  // Rotation describes how the device is mounted (relative to portrait):
  // - Vertical: portrait (top of device up)
  // - Left: 90° CCW (top of device rotated left)
  // - Right: 90° CW (top of device rotated right)
  //
  // Axis mapping summary (device -> portrait-equivalent):
  //   Vertical: x_p =  ios_x, y_p =  ios_y, z_p =  ios_z
  //   Left:     x_p = -ios_y, y_p =  ios_x, z_p =  ios_z
  //   Right:    x_p =  ios_y, y_p = -ios_x, z_p =  ios_z
  //
  // Portrait-equivalent -> body (NED):
  //   body_x = -z_p (forward), body_y = x_p (right), body_z = -y_p (down)
  //
  // Implementation strategy:
  // 1) rotate the device measurement into a portrait-equivalent device frame
  // 2) apply the fixed portrait mapping dev->body for \"screen facing user, top edge up\":
  //    body = [-z, x, -y]  (forward through screen, right is device-right, down is -device-up)
  double x_p = ios_x, y_p = ios_y, z_p = ios_z;
  switch (rotation) {
    case AhrsRotationLeft:   // device rotated 90° CCW -> rotate measurements 90° CW
      // Left rotation: re-orient device axes into the portrait-equivalent frame
      // - Forward (device +Y) becomes portrait -X (left)
      // - Right (device +X) becomes portrait +Y (forward)
      // - Up (device +Z) stays +Z
      x_p = -ios_y;
      y_p = ios_x;
      z_p = ios_z;
      break;
    case AhrsRotationRight:  // device rotated 90° CW -> rotate measurements 90° CCW
      // Right rotation: re-orient device axes into the portrait-equivalent frame
      // - Forward (device +Y) becomes portrait +X (right)
      // - Right (device +X) becomes portrait -Y (backward)
      // - Up (device +Z) stays +Z
      x_p = ios_y;
      y_p = -ios_x;
      z_p = ios_z;
      break;
    case AhrsRotationVertical:
    default:
      break;
  }
  
  // Portrait-equivalent device frame -> aviation body frame (NED)
  *out_x = -z_p;  // forward (out of screen)
  *out_y = x_p;   // right
  *out_z = -y_p;  // down
}

/**
 * @brief Normalize heading angle to [0, 360) degrees
 *
 * Wraps heading values outside the [0, 360) range to within it.
 * Used for displaying heading values in UI.
 *
 * @param heading_deg Heading in degrees (may be outside [0, 360))
 * @return Normalized heading in [0, 360) degrees
 */
float normalizeHeadingDegrees(float heading_deg) {
  while (heading_deg < 0.0f) heading_deg += 360.0f;
  while (heading_deg >= 360.0f) heading_deg -= 360.0f;
  return heading_deg;
}

/**
 * Normalize an angle difference to [-180, 180) degrees
 * Used for calculating sideslip/crab angle (groundTrack - heading)
 *
 * @param angle_deg Angle difference in degrees
 * @return Normalized angle in [-180, 180) degrees
 */
float normalizeAngleDifferenceDegrees(float angle_deg) {
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  while (angle_deg <= -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

