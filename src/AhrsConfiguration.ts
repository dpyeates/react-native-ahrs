import NativeAhrs from './NativeAhrs';
import type { AhrsRotation } from './NativeAhrs';
import { logger } from './AhrsLogger';

/**
 * Manages AHRS configuration settings
 *
 * Handles rate, rotation, magnetic declination, and QNH settings.
 */
export class AhrsConfiguration {
  /**
   * Sets the output emission rate
   *
   * Controls how often data is sent to JavaScript listeners.
   * Independent of internal sensor rate (60Hz).
   * Lower rates save battery and reduce JavaScript load.
   *
   * @param newRate - Rate in Hz, range [1, 60]
   *                  Default: 5 Hz
   */
  public setRate(newRate: number): void {
    if (newRate < 1 || newRate > 60) {
      logger.warn('AHRS rate should be between 1-60 Hz');
      return;
    }
    try {
      logger.log(`⚡ Setting AHRS rate to ${newRate} Hz`);
      NativeAhrs.setAhrsRate(newRate);
    } catch (error) {
      logger.error('❌ Error setting AHRS rate:', error);
    }
  }

  /**
   * Sets device rotation/mounting orientation
   *
   * Configures coordinate frame transformations from device sensors
   * to aviation body frame. Must match physical device mounting.
   *
   * Changing rotation resets the AHRS filter to re-initialize with new orientation.
   *
   * @param rotation - Device orientation:
   *                   "none" - Portrait/vertical (top edge up, default)
   *                   "left" - Landscape left (rotated 90° CCW from portrait)
   *                   "right" - Landscape right (rotated 90° CW from portrait)
   */
  public setRotation(rotation: AhrsRotation): void {
    if (rotation !== 'none' && rotation !== 'left' && rotation !== 'right') {
      logger.warn(
        `AHRS rotation must be 'none' | 'left' | 'right', got: ${String(rotation)}`
      );
      return;
    }
    try {
      logger.log(`🔄 Setting AHRS rotation to ${rotation}`);
      NativeAhrs.setAhrsRotation(rotation);
    } catch (error) {
      logger.error('Error setting AHRS rotation:', error);
    }
  }

  /**
   * Sets QNH pressure for altitude correction
   *
   * QNH is local sea-level pressure used by aviation.
   * Required for accurate altitude calculations from barometric pressure.
   *
   * @param qnh - Pressure in hectopascals (hPa = mbar)
   *             Range: [900, 1100] hPa
   *             Standard: 1013.25 hPa
   */
  public setQNH(qnh: number): void {
    try {
      NativeAhrs.setQNH(qnh);
    } catch (error) {
      logger.error('❌ Error setting QNH', error);
    }
  }
}
