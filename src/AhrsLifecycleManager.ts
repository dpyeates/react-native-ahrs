import NativeAhrs from './NativeAhrs';
import { logger } from './AhrsLogger';

/**
 * Manages AHRS lifecycle operations
 *
 * Handles starting, stopping, resetting, and leveling the AHRS system.
 */
export class AhrsLifecycleManager {
  private isRunning = false;

  /**
   * Gets the current running state
   */
  public getRunning(): boolean {
    return this.isRunning;
  }

  /**
   * Starts AHRS sensor processing
   *
   * Begins collecting and fusing sensor data at 60Hz internally.
   * Data is emitted to registered listeners at the configured rate (default 5Hz).
   *
   * @throws {Error} If AHRS is not supported or initialization fails
   */
  public start(): void {
    if (this.isRunning) return;
    try {
      logger.log('▶️ Starting AHRS with callback...');
      NativeAhrs.startAhrs();
      this.isRunning = true;
      logger.log('✅ AHRS started successfully');
    } catch (error) {
      logger.error('❌ Failed to start AHRS:', error);
      throw error;
    }
  }

  /**
   * Stops AHRS sensor processing
   *
   * Halts all sensor updates to save battery.
   * Can be restarted later with `start()`.
   */
  public stop(): void {
    if (!this.isRunning) return;
    try {
      logger.log('⏹️ Stopping AHRS...');
      NativeAhrs.stopAhrs();
      this.isRunning = false;
    } catch (error) {
      logger.error('❌ Error stopping AHRS:', error);
    }
  }

  /**
   * Resets the EKF filter to initial state
   *
   * Clears all state estimates and covariances.
   * The filter will re-initialize from current sensor readings.
   * Requires 2-3 seconds for reconvergence.
   *
   * Useful when:
   * - Device orientation changes significantly
   * - Filter has diverged
   * - Starting a new flight/session
   */
  public reset(): void {
    try {
      logger.log('🔄 Resetting AHRS...');
      NativeAhrs.resetAhrs();
    } catch (error) {
      logger.error('❌ Error resetting AHRS:', error);
    }
  }

  /**
   * Levels the attitude reference
   *
   * Captures the current attitude as the "zero" reference.
   * Call this when the device is level to set roll=0° and pitch=0°.
   *
   * Does not reset heading or other states.
   */
  public level(): void {
    try {
      logger.log('📏 Leveling AHRS...');
      NativeAhrs.levelAhrs();
    } catch (error) {
      logger.error('❌ Error leveling AHRS:', error);
    }
  }
}
