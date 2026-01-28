/*
 * AltitudeCalculator.h
 *
 * Barometric altitude calculations for QNE and QNH
 * Independent from uNavINS filter
 *
 * Formula: altitude = 44330 * (1 - (P/P0)^(1/5.255))
 * where P is measured barometric pressure in hPa, P0 is reference pressure in hPa
 * 
 * QNE: uses standard atmosphere (1013.25 hPa) as P0
 * QNH: uses local sea-level pressure (user-provided QNH) as P0
 */

#ifndef ALTITUDE_CALCULATOR_H
#define ALTITUDE_CALCULATOR_H

#include <cmath>
#include <limits>

class AltitudeCalculator {
public:
  /**
   * Calculate QNE altitude (standard atmosphere, 1013.25 hPa)
   * @param pressure_hpa Barometric pressure in hectopascals (hPa)
   * @return Altitude in meters, or NaN if invalid input
   */
  static float calculateQNE_m(float pressure_hpa);
  
  /**
   * Calculate QNH altitude (user-provided sea level pressure)
   * @param pressure_hpa Barometric pressure in hectopascals (hPa)
   * @param qnh_hpa QNH (sea level pressure) in hectopascals (hPa)
   * @return Altitude in meters, or NaN if invalid input
   */
  static float calculateQNH_m(float pressure_hpa, float qnh_hpa);
  
private:
  // Standard atmosphere reference pressure (hPa)
  static constexpr float STANDARD_PRESSURE_HPA = 1013.25f;
  
  // Barometric altitude formula constant
  static constexpr float ALTITUDE_CONSTANT = 44330.0f;
  
  // Barometric altitude formula exponent: 1.0 / 5.255
  static constexpr float ALTITUDE_EXPONENT = 1.0f / 5.255f;
};

#endif // ALTITUDE_CALCULATOR_H
