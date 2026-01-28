/*
 * AltitudeCalculator.cpp
 *
 * Implementation of barometric altitude calculations
 */

#include "AltitudeCalculator.h"
#include <cmath>
#include <limits>

float AltitudeCalculator::calculateQNE_m(float pressure_hpa) {
  // Validate input
  if (pressure_hpa <= 0.0f || !std::isfinite(pressure_hpa)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  // Calculate altitude using standard atmosphere (1013.25 hPa)
  float pressure_ratio = pressure_hpa / STANDARD_PRESSURE_HPA;
  
  // Check for invalid pressure ratio
  if (pressure_ratio <= 0.0f || !std::isfinite(pressure_ratio)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  // Barometric altitude formula: h = 44330 * (1 - (P/P0)^(1/5.255))
  // where P is measured pressure, P0 is reference pressure (1013.25 for QNE)
  float altitude_m = ALTITUDE_CONSTANT * (1.0f - std::pow(pressure_ratio, ALTITUDE_EXPONENT));
  
  return altitude_m;
}

float AltitudeCalculator::calculateQNH_m(float pressure_hpa, float qnh_hpa) {
  // Validate inputs
  if (pressure_hpa <= 0.0f || !std::isfinite(pressure_hpa)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  // If QNH not provided or invalid, use standard atmosphere
  if (qnh_hpa <= 0.0f || !std::isfinite(qnh_hpa)) {
    return calculateQNE_m(pressure_hpa);
  }
  
  // Calculate altitude using user-provided QNH
  float pressure_ratio = pressure_hpa / qnh_hpa;
  
  // Check for invalid pressure ratio
  if (pressure_ratio <= 0.0f || !std::isfinite(pressure_ratio)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  // Barometric altitude formula: h = 44330 * (1 - (P/QNH)^0.1903)
  float altitude_m = ALTITUDE_CONSTANT * (1.0f - std::pow(pressure_ratio, ALTITUDE_EXPONENT));
  
  return altitude_m;
}
