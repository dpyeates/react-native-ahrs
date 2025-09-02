import NativeAhrs, { type AhrsData, type Rotation } from './NativeAhrs';

// Re-export types for consumers of the library
export type { AhrsData, Rotation } from './NativeAhrs';

export type AhrsCallback = (data: AhrsData) => void;

class AhrsSensor {
  private callbacks = new Set<AhrsCallback>();
  private isRunning = false;

  constructor() {
    NativeAhrs?.onAhrsUpdate(this.masterCallback);
  }

  public async isSupported(): Promise<boolean> {
    try {
      const supported = await NativeAhrs.isSupported();
      if (supported) {
        console.log('✅ AHRS supported on this device');
      } else {
        console.warn('❌ AHRS not supported on this device');
      }
      return supported;
    } catch (e) {
      console.warn('Failed to query AHRS support', e);
      return false;
    }
  }

  private masterCallback = (data: AhrsData) => {
    this.callbacks.forEach((cb) => {
      try {
        cb(data);
      } catch (error) {
        console.error('Error in AHRS callback:', error);
      }
    });
  };

  public addListener(callback: AhrsCallback) {
    this.callbacks.add(callback);
    return () => {
      this.callbacks.delete(callback);
      console.log(`Removed AHRS listener. Remaining: ${this.callbacks.size}`);
      if (this.callbacks.size === 0) {
        this.stop();
      }
    };
  }

  public start(): void {
    if (this.isRunning) return;
    if (this.callbacks.size === 0) {
      console.warn(
        '❌ Attempt to start Ahrs without any callbacks registered. Call addListener before starting.'
      );
      return;
    }
    try {
      if (__DEV__) {
        console.log('▶️ Starting AHRS with callback...');
      }
      NativeAhrs.startAhrs();
      this.isRunning = true;
      if (__DEV__) {
        console.log('✅ AHRS started successfully');
      }
    } catch (error) {
      console.error('❌ Failed to start AHRS:', error);
      throw error;
    }
  }

  public stop(): void {
    if (!this.isRunning) return;
    try {
      if (__DEV__) {
        console.log('⏹️ Stopping AHRS...');
      }
      NativeAhrs.stopAhrs();
      this.isRunning = false;
    } catch (error) {
      console.error('❌ Error stopping AHRS:', error);
    }
  }

  public reset(): void {
    try {
      if (__DEV__) {
        console.log('🔄 Resetting AHRS...');
      }
      NativeAhrs.resetAhrs();
    } catch (error) {
      console.error('❌ Error resetting AHRS:', error);
    }
  }

  public level(): void {
    try {
      if (__DEV__) {
        console.log('📏 Leveling AHRS...');
      }
      NativeAhrs.levelAhrs();
    } catch (error) {
      console.error('❌ Error leveling AHRS:', error);
    }
  }

  public setRate(newRate: number): void {
    if (newRate < 1 || newRate > 60) {
      console.warn('AHRS rate should be between 1-60 Hz');
      return;
    }
    try {
      if (__DEV__) {
        console.log(`⚡ Setting AHRS rate to ${newRate} Hz`);
      }
      NativeAhrs.setAhrsRate(newRate);
    } catch (error) {
      console.error('❌ Error setting AHRS rate:', error);
    }
  }

  public setGain(gain: number): void {
    if (!Number.isFinite(gain)) {
      console.warn('AHRS gain must be a finite number');
      return;
    }
    try {
      if (__DEV__) {
        console.log(`🎛️ Setting AHRS gain to ${gain}`);
      }
      NativeAhrs.setAhrsGain(gain);
    } catch (error) {
      console.error('❌ Error setting AHRS gain:', error);
    }
  }

  public setRotation(rotation: Rotation): void {
    if (rotation !== 'none' && rotation !== 'left' && rotation !== 'right') {
      console.warn(
        `AHRS rotation must be 'none' | 'left' | 'right', got: ${String(rotation)}`
      );
      return;
    }
    try {
      if (__DEV__) {
        console.log(`🔄 Setting AHRS rotation to ${rotation}`);
      }
      NativeAhrs.setAhrsRotation(rotation);
    } catch (error) {
      console.error('Error setting AHRS rotation:', error);
    }
  }

  public removeAllListeners() {
    if (__DEV__) {
      console.log('Removing all AHRS listeners');
    }
    this.callbacks.clear();
    this.stop();
  }

  public getStatus() {
    return {
      isRunning: this.isRunning,
      listenerCount: this.callbacks.size,
    };
  }
}

export const Ahrs = new AhrsSensor();
