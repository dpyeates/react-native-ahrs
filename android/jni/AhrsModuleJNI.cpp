#include <jni.h>
#include <jsi/jsi.h>
#include <memory>

#include <android/log.h>

#include "../fusion/FusionBridge.h"

using namespace facebook::jsi;

extern "C" JNIEXPORT void JNICALL Java_com_ahrs_AhrsModule_initAhrs(JNIEnv *env, jobject, jint platform, jint rotation, jfloat gain) {
  initAhrs(platform, rotation, gain);
}

extern "C" JNIEXPORT void JNICALL Java_com_ahrs_AhrsModule_updateAhrs(JNIEnv *env, jobject,
                                                           jfloat deltaTime,
                                                           jfloatArray accel,
                                                           jfloatArray gyro,
                                                           jfloatArray mag) {
  jfloat *cArrayAccel = env->GetFloatArrayElements(accel, nullptr);
  jfloat *cArrayGyro = env->GetFloatArrayElements(gyro, nullptr);
  jfloat *cArrayNag = env->GetFloatArrayElements(mag, nullptr);
  updateAhrs(deltaTime, cArrayAccel, cArrayGyro, cArrayNag);
  env->ReleaseFloatArrayElements(accel, cArrayAccel, JNI_ABORT);
  env->ReleaseFloatArrayElements(gyro, cArrayGyro, JNI_ABORT);
  env->ReleaseFloatArrayElements(mag, cArrayNag, JNI_ABORT);
}

extern "C" JNIEXPORT void JNICALL Java_com_ahrs_AhrsModule_zeroAhrs(JNIEnv *env, jobject) {
  zeroAhrs();
}

extern "C" JNIEXPORT void JNICALL Java_com_ahrs_AhrsModule_setAhrsInterfaceRotation(JNIEnv *, jobject, jint rotation) {
  setAhrsInterfaceRotation(rotation);
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_ahrs_AhrsModule_getAhrsRoll(JNIEnv *, jobject) {
  return getAhrsRoll();
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_ahrs_AhrsModule_getAhrsPitch(JNIEnv *, jobject) {
  return getAhrsPitch();
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_ahrs_AhrsModule_getAhrsYaw(JNIEnv *, jobject) {
  return getAhrsYaw();
}

extern "C" JNIEXPORT jfloat JNICALL Java_com_ahrs_AhrsModule_getAhrsHeading(JNIEnv *, jobject) {
  return getAhrsHeading();
}
