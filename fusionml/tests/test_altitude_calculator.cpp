/*
 * Unit tests for AltitudeCalculator
 */

#include <cmath>
#include <cstdio>
#include <limits>

#include "AltitudeCalculator.h"

static int g_failed = 0;

static void expectTrue(bool condition, const char *message) {
  if (!condition) {
    g_failed++;
    std::printf("✗ %s\n", message);
  } else {
    std::printf("✓ %s\n", message);
  }
}

static void expectNear(float actual, float expected, float tolerance, const char *message) {
  float error = std::fabs(actual - expected);
  if (error > tolerance || !std::isfinite(actual)) {
    g_failed++;
    std::printf("✗ %s (actual=%.4f expected=%.4f tol=%.4f)\n",
                message, actual, expected, tolerance);
  } else {
    std::printf("✓ %s\n", message);
  }
}

int main() {
  std::printf("AltitudeCalculator Unit Tests\n");

  // QNE at standard pressure should be ~0m
  float qne0 = AltitudeCalculator::calculateQNE_m(1013.25f);
  expectNear(qne0, 0.0f, 0.5f, "QNE at 1013.25 hPa is near 0m");

  // Lower pressure should yield positive altitude
  float qne900 = AltitudeCalculator::calculateQNE_m(900.0f);
  expectTrue(std::isfinite(qne900) && qne900 > 0.0f,
             "QNE at 900 hPa is finite and positive");

  // QNH invalid falls back to QNE
  float qnhInvalid = AltitudeCalculator::calculateQNH_m(900.0f, 0.0f);
  expectNear(qnhInvalid, qne900, 0.01f, "QNH invalid falls back to QNE");

  // Invalid pressure returns NaN
  float qneInvalid = AltitudeCalculator::calculateQNE_m(-1.0f);
  expectTrue(std::isnan(qneInvalid), "QNE invalid pressure returns NaN");

  float qnhInvalidPressure = AltitudeCalculator::calculateQNH_m(0.0f, 1013.25f);
  expectTrue(std::isnan(qnhInvalidPressure), "QNH invalid pressure returns NaN");

  std::printf("\nAltitudeCalculator: %s\n", g_failed == 0 ? "PASS ✓" : "FAIL ✗");
  return g_failed == 0 ? 0 : 1;
}
