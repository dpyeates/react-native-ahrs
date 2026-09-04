import { AhrsCallbackManager, type AhrsCallback } from '../AhrsCallbackManager';
import type { AhrsData } from '../NativeAhrs';

// Mock the logger
jest.mock('../AhrsLogger', () => ({
  logger: {
    log: jest.fn(),
    warn: jest.fn(),
    error: jest.fn(),
    info: jest.fn(),
    debug: jest.fn(),
  },
}));

import { logger } from '../AhrsLogger';

describe('AhrsCallbackManager', () => {
  let manager: AhrsCallbackManager;

  const createMockAhrsData = (overrides?: Partial<AhrsData>): AhrsData => ({
    roll: 0,
    pitch: 0,
    heading: 0,
    magneticDeclination: 0,
    groundTrack: 0,
    groundSpeed: 0,
    flightPathAngle: 0,
    horizontalFlightPathAngle: 0,
    altitude: 0,
    altitudeQNE: 0,
    altitudeQNH: 0,
    verticalSpeed: 0,
    barometricPressure: 1013.25,
    velocityNorth: 0,
    velocityEast: 0,
    velocityDown: 0,
    latitude: 0,
    longitude: 0,
    flightPhase: 0,
    flightPhaseConfidence: 0,
    attitudeValid: true,
    altitudeValid: true,
    positionValid: true,
    flightPhaseValid: false,
    filterHealthStatus: 0,
    ...overrides,
  });

  beforeEach(() => {
    manager = new AhrsCallbackManager();
    jest.clearAllMocks();
  });

  describe('addListener', () => {
    it('should add a listener and return an unsubscribe function', () => {
      const callback = jest.fn();
      const unsubscribe = manager.addListener(callback);

      expect(typeof unsubscribe).toBe('function');
      expect(manager.getListenerCount()).toBe(1);
    });

    it('should add multiple listeners', () => {
      manager.addListener(jest.fn());
      manager.addListener(jest.fn());
      manager.addListener(jest.fn());

      expect(manager.getListenerCount()).toBe(3);
    });

    it('should remove listener when unsubscribe is called', () => {
      const callback = jest.fn();
      const unsubscribe = manager.addListener(callback);

      expect(manager.getListenerCount()).toBe(1);

      unsubscribe();

      expect(manager.getListenerCount()).toBe(0);
      expect(logger.log).toHaveBeenCalledWith(
        'Removed AHRS listener. Remaining: 0'
      );
    });

    it('should not remove other listeners when one unsubscribes', () => {
      const callback1 = jest.fn();
      const callback2 = jest.fn();
      const unsubscribe1 = manager.addListener(callback1);
      manager.addListener(callback2);

      unsubscribe1();

      expect(manager.getListenerCount()).toBe(1);
    });

    it('should handle duplicate unsubscribe calls gracefully', () => {
      const callback = jest.fn();
      const unsubscribe = manager.addListener(callback);

      unsubscribe();
      unsubscribe(); // Should not throw

      expect(manager.getListenerCount()).toBe(0);
    });
  });

  describe('distributeData', () => {
    it('should call all registered callbacks with the data', () => {
      const callback1 = jest.fn();
      const callback2 = jest.fn();
      manager.addListener(callback1);
      manager.addListener(callback2);

      const data = createMockAhrsData({ roll: 15, pitch: 10 });
      manager.distributeData(data);

      expect(callback1).toHaveBeenCalledWith(data);
      expect(callback2).toHaveBeenCalledWith(data);
    });

    it('should not call unsubscribed callbacks', () => {
      const callback1 = jest.fn();
      const callback2 = jest.fn();
      const unsubscribe1 = manager.addListener(callback1);
      manager.addListener(callback2);

      unsubscribe1();

      const data = createMockAhrsData();
      manager.distributeData(data);

      expect(callback1).not.toHaveBeenCalled();
      expect(callback2).toHaveBeenCalledWith(data);
    });

    it('should isolate errors so one failing callback does not affect others', () => {
      const errorCallback = jest.fn(() => {
        throw new Error('Callback error');
      });
      const successCallback = jest.fn();

      manager.addListener(errorCallback);
      manager.addListener(successCallback);

      const data = createMockAhrsData();
      manager.distributeData(data);

      expect(errorCallback).toHaveBeenCalled();
      expect(successCallback).toHaveBeenCalled();
      expect(logger.error).toHaveBeenCalledWith(
        'Error in AHRS callback:',
        expect.any(Error)
      );
    });

    it('should handle no listeners gracefully', () => {
      const data = createMockAhrsData();

      expect(() => manager.distributeData(data)).not.toThrow();
    });

    it('should distribute data in order of registration', () => {
      const callOrder: number[] = [];
      const callback1: AhrsCallback = () => callOrder.push(1);
      const callback2: AhrsCallback = () => callOrder.push(2);
      const callback3: AhrsCallback = () => callOrder.push(3);

      manager.addListener(callback1);
      manager.addListener(callback2);
      manager.addListener(callback3);

      manager.distributeData(createMockAhrsData());

      expect(callOrder).toEqual([1, 2, 3]);
    });
  });

  describe('removeAllListeners', () => {
    it('should remove all listeners', () => {
      manager.addListener(jest.fn());
      manager.addListener(jest.fn());
      manager.addListener(jest.fn());

      expect(manager.getListenerCount()).toBe(3);

      manager.removeAllListeners();

      expect(manager.getListenerCount()).toBe(0);
      expect(logger.log).toHaveBeenCalledWith('Removing all AHRS listeners');
    });

    it('should handle removing when no listeners exist', () => {
      expect(() => manager.removeAllListeners()).not.toThrow();
      expect(manager.getListenerCount()).toBe(0);
    });
  });

  describe('hasListeners', () => {
    it('should return false when no listeners are registered', () => {
      expect(manager.hasListeners()).toBe(false);
    });

    it('should return true when listeners are registered', () => {
      manager.addListener(jest.fn());
      expect(manager.hasListeners()).toBe(true);
    });

    it('should return false after all listeners are removed', () => {
      const unsubscribe = manager.addListener(jest.fn());
      expect(manager.hasListeners()).toBe(true);

      unsubscribe();
      expect(manager.hasListeners()).toBe(false);
    });
  });

  describe('getListenerCount', () => {
    it('should return 0 when no listeners', () => {
      expect(manager.getListenerCount()).toBe(0);
    });

    it('should return correct count as listeners are added/removed', () => {
      const unsub1 = manager.addListener(jest.fn());
      expect(manager.getListenerCount()).toBe(1);

      const unsub2 = manager.addListener(jest.fn());
      expect(manager.getListenerCount()).toBe(2);

      unsub1();
      expect(manager.getListenerCount()).toBe(1);

      unsub2();
      expect(manager.getListenerCount()).toBe(0);
    });
  });
});
