import { AhrsLifecycleManager } from '../AhrsLifecycleManager';
import NativeAhrs from '../NativeAhrs';
import { logger } from '../AhrsLogger';

// Mock dependencies
jest.mock('../NativeAhrs', () => ({
  __esModule: true,
  default: {
    startAhrs: jest.fn(),
    stopAhrs: jest.fn(),
    resetAhrs: jest.fn(),
    levelAhrs: jest.fn(),
  },
}));

jest.mock('../AhrsLogger', () => ({
  logger: {
    log: jest.fn(),
    warn: jest.fn(),
    error: jest.fn(),
    info: jest.fn(),
    debug: jest.fn(),
  },
}));

describe('AhrsLifecycleManager', () => {
  let manager: AhrsLifecycleManager;

  beforeEach(() => {
    manager = new AhrsLifecycleManager();
    jest.clearAllMocks();
  });

  describe('start', () => {
    it('should start AHRS and set running state', () => {
      manager.start();

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(1);
      expect(manager.getRunning()).toBe(true);
      expect(logger.log).toHaveBeenCalledWith(
        '▶️ Starting AHRS with callback...'
      );
      expect(logger.log).toHaveBeenCalledWith('✅ AHRS started successfully');
    });

    it('should not start if already running', () => {
      manager.start();
      manager.start();

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(1);
    });

    it('should throw error if native module fails', () => {
      const error = new Error('Start failed');
      (NativeAhrs.startAhrs as jest.Mock).mockImplementationOnce(() => {
        throw error;
      });

      expect(() => manager.start()).toThrow('Start failed');
      expect(manager.getRunning()).toBe(false);
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Failed to start AHRS:',
        error
      );
    });
  });

  describe('stop', () => {
    it('should stop AHRS and clear running state', () => {
      manager.start();
      manager.stop();

      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(1);
      expect(manager.getRunning()).toBe(false);
      expect(logger.log).toHaveBeenCalledWith('⏹️ Stopping AHRS...');
    });

    it('should not stop if not running', () => {
      manager.stop();

      expect(NativeAhrs.stopAhrs).not.toHaveBeenCalled();
    });

    it('should handle native module errors gracefully', () => {
      manager.start();
      (NativeAhrs.stopAhrs as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Stop failed');
      });

      expect(() => manager.stop()).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Error stopping AHRS:',
        expect.any(Error)
      );
    });
  });

  describe('reset', () => {
    it('should reset AHRS filter', () => {
      manager.reset();

      expect(NativeAhrs.resetAhrs).toHaveBeenCalledTimes(1);
      expect(logger.log).toHaveBeenCalledWith('🔄 Resetting AHRS...');
    });

    it('should handle native module errors gracefully', () => {
      (NativeAhrs.resetAhrs as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Reset failed');
      });

      expect(() => manager.reset()).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Error resetting AHRS:',
        expect.any(Error)
      );
    });

    it('should work whether running or not', () => {
      // Reset when not running
      manager.reset();
      expect(NativeAhrs.resetAhrs).toHaveBeenCalledTimes(1);

      // Reset when running
      manager.start();
      manager.reset();
      expect(NativeAhrs.resetAhrs).toHaveBeenCalledTimes(2);
    });
  });

  describe('level', () => {
    it('should level AHRS attitude reference', () => {
      manager.level();

      expect(NativeAhrs.levelAhrs).toHaveBeenCalledTimes(1);
      expect(logger.log).toHaveBeenCalledWith('📏 Leveling AHRS...');
    });

    it('should handle native module errors gracefully', () => {
      (NativeAhrs.levelAhrs as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Level failed');
      });

      expect(() => manager.level()).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Error leveling AHRS:',
        expect.any(Error)
      );
    });

    it('should work whether running or not', () => {
      // Level when not running
      manager.level();
      expect(NativeAhrs.levelAhrs).toHaveBeenCalledTimes(1);

      // Level when running
      manager.start();
      manager.level();
      expect(NativeAhrs.levelAhrs).toHaveBeenCalledTimes(2);
    });
  });

  describe('getRunning', () => {
    it('should return false initially', () => {
      expect(manager.getRunning()).toBe(false);
    });

    it('should return true after start', () => {
      manager.start();
      expect(manager.getRunning()).toBe(true);
    });

    it('should return false after stop', () => {
      manager.start();
      manager.stop();
      expect(manager.getRunning()).toBe(false);
    });

    it('should remain false after failed start', () => {
      (NativeAhrs.startAhrs as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Start failed');
      });

      try {
        manager.start();
      } catch {
        // Expected
      }

      expect(manager.getRunning()).toBe(false);
    });
  });

  describe('lifecycle sequences', () => {
    it('should handle start -> stop -> start -> stop correctly', () => {
      manager.start();
      expect(manager.getRunning()).toBe(true);

      manager.stop();
      expect(manager.getRunning()).toBe(false);

      manager.start();
      expect(manager.getRunning()).toBe(true);

      manager.stop();
      expect(manager.getRunning()).toBe(false);

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(2);
      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(2);
    });

    it('should handle reset during running state', () => {
      manager.start();
      manager.reset();
      manager.level();

      expect(manager.getRunning()).toBe(true);
      expect(NativeAhrs.resetAhrs).toHaveBeenCalledTimes(1);
      expect(NativeAhrs.levelAhrs).toHaveBeenCalledTimes(1);
    });
  });
});
