#include <string.h>
#include <stdio.h>
#ifdef ANDROID
#include <android/log.h>
#endif
#include "FusionBridge.h"

static inline double normalizeHeading(double heading);
static inline FusionQuaternion FusionQuaternionConjugate(FusionQuaternion quaternion);
static inline FusionQuaternion FusionEulerToQuaternion(const FusionEuler euler);
static inline float SmoothHeading(float new_heading);

FusionQuaternion zeroReference;
FusionOffset offset;
FusionAhrs ahrs;
FusionEuler euler;
FusionAxesAlignment alignment;
float gain;
float heading;
int platform;
bool useQOffset = false;

/**
 * @brief Initialises (or re-initialises) the AHRS model
 * @param platform either ios == 0 or android == 1
 * @param rotation is the desired user interface orientation, 0 == None/Portrait, -1 == Left, 1 == Right.
 * @param gain is the model gain to apply. Low values give slower response.
 */
void initAhrs(int platform, int rotation, float gain) {
  platform = platform;
  setAhrsInterfaceRotation(rotation);
  const FusionAhrsSettings settings = {
      .convention = FusionConventionEnu,
      .gain = gain,
      .gyroscopeRange = 0.0f,
      .accelerationRejection = 90.0f,
      .magneticRejection = 90.0f,
      .recoveryTriggerPeriod = 0,
  };
  zeroReference = FUSION_IDENTITY_QUATERNION;
  FusionAhrsSetSettings(&ahrs, &settings);
  FusionAhrsReset(&ahrs);
}

/**
 * @brief The main AHRS model function.
 * @param deltaTime the time in seconds since update was last called (usually
 * 1/60)
 * @param accel an array holding the [x, y, z] accelerometer readings
 * @param gyro an array holding the [x, y, z] gyroscope readings
 * @param mag an array holding the [x, y, z] magnetometer readings
 */
void updateAhrs(float deltaTime, float accel[3], float gyro[3], float mag[3]) {
  // Swap axis
  FusionVector gyroscope = FusionAxesSwap(*(FusionVector *)gyro, alignment);
  FusionVector accelerometer = FusionAxesSwap(*(FusionVector *)accel, alignment);
  FusionVector magnetometer = FusionAxesSwap(*(FusionVector *)mag, alignment);

  // Update AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

  // Apply installation offset if needed
  if (useQOffset) {
    FusionQuaternion q =
        FusionQuaternionMultiply(FusionAhrsGetQuaternion(&ahrs), zeroReference);
    euler = FusionQuaternionToEuler(q);
  } else {
    euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  }

  // Apply a moving average filter to the heading to avoid jitter
  float h = FusionCompassCalculateHeading(ahrs.settings.convention, accelerometer, magnetometer);
  heading = SmoothHeading(h);
}

/**
 * @brief Compensates for installation errors by making the current 'angles' the
 * zero point. It does this by creating an inverse quaternion and saving that
 * off as a zero reference that is then applied to the AHRS calculated values
 * via quaternion multiplication.
 */
void zeroAhrs(void) {
  // Extract the current roll, pitch, and yaw
  const FusionQuaternion currentQ = FusionAhrsGetQuaternion(&ahrs);
  FusionEuler currentEuler = FusionQuaternionToEuler(currentQ);

  // Build a Quaternion for only roll and pitch.
  // Set yaw to zero, keep roll and pitch as is.
  FusionEuler rpEuler;
  rpEuler.angle.roll = currentEuler.angle.roll;
  rpEuler.angle.pitch = currentEuler.angle.pitch;
  rpEuler.angle.yaw = 0.0f;
  FusionQuaternion rpQ = FusionEulerToQuaternion(rpEuler);

  // Compute the Conjugate (Inverse) of the Roll/Pitch Quaternion
  zeroReference = FusionQuaternionConjugate(rpQ);

  useQOffset = !useQOffset;
};

float getAhrsRoll(void) { return ahrs.initialising ? 0.0f : euler.angle.roll; };

float getAhrsPitch(void) { return  ahrs.initialising ? 0.0f : euler.angle.pitch; };

float getAhrsYaw(void) { return  ahrs.initialising ? 0.0f : euler.angle.yaw; };

float getAhrsHeading(void) { return ahrs.initialising ? 0.0f : heading; };

/**
 * @brief Configures the alignment of the sensors based upon the devices rotation
 * @param rotation representing the rotation. 0 == none, -1 == left or 1 == right
 */
void setAhrsInterfaceRotation(int rotation) {
  if (rotation == 0) { // no rotation (portrait)
  #ifdef ANDROID
      alignment = FusionAxesAlignmentNZPXPY;
  #else
      alignment = FusionAxesAlignmentNZPXNY;
  #endif
  } else if (rotation == -1) { // device rotated left
#ifdef ANDROID
    alignment = FusionAxesAlignmentPZNYNX;
#else
    alignment = FusionAxesAlignmentNZPYPX;
#endif
  } else if (rotation == 1) { // device rotated right
#ifdef ANDROID
    alignment = FusionAxesAlignmentPZPYPX;
#else
    alignment = FusionAxesAlignmentNZNYNX;
#endif
  }
};

/**
 * @brief Normalises a heading value between 0-360
 * @param heading value to be normalised
 * @return the normalised heading value
 */
static inline double normalizeHeading(double heading) {
  double angle = fmod(heading, 360.0);
  if (angle < 0) {
    angle += 360.0;
  }
  return angle;
}

/**
 * @brief Computes conjugate of a quaternion (inverse rotation)
 * @param quaternion Input quaternion
 * @return Conjugate quaternion
 */
static inline FusionQuaternion FusionQuaternionConjugate(
    FusionQuaternion quaternion) {
  return (FusionQuaternion){.element.w = quaternion.element.w,
                            .element.x = -quaternion.element.x,
                            .element.y = -quaternion.element.y,
                            .element.z = -quaternion.element.z};
}

/**
 * @brief Converts a ZYX Euler angles in degrees to a quaternion
 * @param euler Euler angles in degrees..
 * @return Quaternion representing the Euler angles.
 */
static inline FusionQuaternion FusionEulerToQuaternion(
    const FusionEuler euler) {
#define E euler.angle
  const float cr = cos(FusionDegreesToRadians(E.roll) * 0.5);
  const float sr = sin(FusionDegreesToRadians(E.roll) * 0.5);
  const float cp = cos(FusionDegreesToRadians(E.pitch) * 0.5);
  const float sp = sin(FusionDegreesToRadians(E.pitch) * 0.5);
  const float cy = cos(FusionDegreesToRadians(E.yaw) * 0.5);
  const float sy = sin(FusionDegreesToRadians(E.yaw) * 0.5);
  const FusionQuaternion result = {
      .element = {.w = cr * cp * cy + sr * sp * sy,
                  .x = sr * cp * cy - cr * sp * sy,
                  .y = cr * sp * cy + sr * cp * sy,
                  .z = cr * cp * sy - sr * sp * cy}};
  return result;
#undef E
}

/**
 * @brief Applies a moving average filter to the heading
 * @param new_heading the new normalised (noisy) heading value
 * @return the filtered heading value
 */
static inline float SmoothHeading(float new_heading) {
  static int initialised = 0;
  static float smooth_x = 0, smooth_y = 0;
  const float alpha = 0.01f;  // Smoothing factor
  float rad = FusionDegreesToRadians(new_heading);
  float x = cos(rad);
  float y = sin(rad);
  if (!initialised) {
    smooth_x = x;
    smooth_y = y;
    initialised = 1;
  } else {
    smooth_x = alpha * x + (1 - alpha) * smooth_x;
    smooth_y = alpha * y + (1 - alpha) * smooth_y;
  }
  float avg_rad = atan2(smooth_y, smooth_x);
#ifdef ANDROID
return normalizeHeading(FusionRadiansToDegrees(avg_rad) + 90);
#else
  return normalizeHeading(FusionRadiansToDegrees(avg_rad) - 90);
#endif
}
