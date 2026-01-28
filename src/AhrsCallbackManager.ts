import type { AhrsData } from './NativeAhrs';
import { logger } from './AhrsLogger';

/**
 * Callback function type for AHRS data updates
 *
 * @param data - The latest AHRS data from the sensor fusion algorithm
 */
export type AhrsCallback = (data: AhrsData) => void;

/**
 * Manages AHRS callback listeners
 *
 * Handles registration, removal, and distribution of data to listeners.
 * Isolates errors so one failing callback doesn't break others.
 */
export class AhrsCallbackManager {
  private callbacks = new Set<AhrsCallback>();

  /**
   * Internal callback that distributes data to all registered listeners
   * Isolates errors so one failing callback doesn't break others
   *
   * @param data - AHRS data from native module
   */
  public distributeData(data: AhrsData): void {
    this.callbacks.forEach((cb) => {
      try {
        cb(data);
      } catch (error) {
        logger.error('Error in AHRS callback:', error);
      }
    });
  }

  /**
   * Registers a callback to receive AHRS data updates
   *
   * @param callback - Function to call with AHRS data updates
   * @returns Unsubscribe function to remove this listener
   */
  public addListener(callback: AhrsCallback): () => void {
    this.callbacks.add(callback);
    return () => {
      this.callbacks.delete(callback);
      logger.log(`Removed AHRS listener. Remaining: ${this.callbacks.size}`);
    };
  }

  /**
   * Removes all registered listeners
   */
  public removeAllListeners(): void {
    logger.log('Removing all AHRS listeners');
    this.callbacks.clear();
  }

  /**
   * Gets the number of registered listeners
   */
  public getListenerCount(): number {
    return this.callbacks.size;
  }

  /**
   * Checks if any listeners are registered
   */
  public hasListeners(): boolean {
    return this.callbacks.size > 0;
  }
}
