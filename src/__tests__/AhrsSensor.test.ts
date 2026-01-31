import {
  Ahrs,
  type AhrsData,
  type AhrsRotation,
  type RecordingFile,
  type PlaybackStateEvent,
  type XPlaneConnectionEvent,
} from '../index';
import NativeAhrs from '../NativeAhrs';

// Store callbacks that persist across jest.clearAllMocks()
let storedAhrsCallback: ((data: AhrsData) => void) | null = null;
let storedPlaybackCallback: ((event: PlaybackStateEvent) => void) | null = null;
let storedXPlaneCallback:
  | ((event: { connected: boolean; host: string }) => void)
  | null = null;

// Mock the native module with callback capture
jest.mock('../NativeAhrs', () => ({
  __esModule: true,
  default: {
    onAhrsUpdate: jest.fn((cb: (data: AhrsData) => void) => {
      storedAhrsCallback = cb;
    }),
    startAhrs: jest.fn(),
    stopAhrs: jest.fn(),
    resetAhrs: jest.fn(),
    levelAhrs: jest.fn(),
    setAhrsRate: jest.fn(),
    setAhrsRotation: jest.fn(),
    setQNH: jest.fn(),
    isSupported: jest.fn(),
    isPositionReliable: jest.fn(),
    onPlaybackStateChanged: jest.fn(
      (cb: (event: PlaybackStateEvent) => void) => {
        storedPlaybackCallback = cb;
      }
    ),
    onXPlaneConnectionChanged: jest.fn(
      (cb: (event: { connected: boolean; host: string }) => void) => {
        storedXPlaneCallback = cb;
      }
    ),
    startRecording: jest.fn(),
    stopRecording: jest.fn(),
    playbackRecording: jest.fn(),
    stopPlayback: jest.fn(),
    getRecordingFiles: jest.fn(),
    deleteRecording: jest.fn(),
    connectToXPlane: jest.fn(),
    disconnectFromXPlane: jest.fn(),
  },
}));

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

// Helper functions to invoke stored callbacks
const emitAhrsData = (data: Partial<AhrsData>) => {
  if (storedAhrsCallback) {
    storedAhrsCallback(data as AhrsData);
  }
};

const emitPlaybackEvent = (event: PlaybackStateEvent) => {
  if (storedPlaybackCallback) {
    storedPlaybackCallback(event);
  }
};

const emitXPlaneEvent = (event: { connected: boolean; host: string }) => {
  if (storedXPlaneCallback) {
    storedXPlaneCallback(event);
  }
};

describe('AhrsSensor', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Reset the singleton by clearing listeners
    Ahrs.removeAllListeners();
    // Ensure NativeAhrs mock is properly set up
    (NativeAhrs.onAhrsUpdate as jest.Mock).mockClear();
    (NativeAhrs.startAhrs as jest.Mock).mockResolvedValue(undefined);
    (NativeAhrs.stopAhrs as jest.Mock).mockResolvedValue(undefined);
  });

  describe('isSupported', () => {
    it('should return true when AHRS is supported', async () => {
      (NativeAhrs.isSupported as jest.Mock).mockResolvedValue(true);

      const result = await Ahrs.isSupported();

      expect(result).toBe(true);
      expect(NativeAhrs.isSupported).toHaveBeenCalledTimes(1);
    });

    it('should return false when AHRS is not supported', async () => {
      (NativeAhrs.isSupported as jest.Mock).mockResolvedValue(false);

      const result = await Ahrs.isSupported();

      expect(result).toBe(false);
    });

    it('should return false and log warning on error', async () => {
      (NativeAhrs.isSupported as jest.Mock).mockRejectedValue(
        new Error('Test error')
      );

      const result = await Ahrs.isSupported();

      expect(result).toBe(false);
    });
  });

  describe('addListener', () => {
    it('should register a callback', () => {
      const callback = jest.fn();
      const unsubscribe = Ahrs.addListener(callback);

      expect(typeof unsubscribe).toBe('function');
    });

    it('should call registered callback when data is received', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      // Simulate native module emitting data
      const mockData: AhrsData = {
        roll: 10,
        pitch: 5,
        heading: 90,
        magneticDeclination: 0.5,
        groundTrack: 90,
        groundSpeed: 20,
        flightPathAngle: 0,
        horizontalFlightPathAngle: 0,
        altitude: 100,
        altitudeQNE: 100,
        altitudeQNH: 100,
        verticalSpeed: 1,
        barometricPressure: 1013.25,
        velocityNorth: 10,
        velocityEast: 0,
        velocityDown: -1,
        latitude: 51.0,
        longitude: 0.0,
        flightPhase: 0,
        flightPhaseConfidence: 1,
        attitudeValid: true,
        altitudeValid: true,
        positionValid: true,
        flightPhaseValid: false,
      };

      emitAhrsData(mockData);

      expect(callback).toHaveBeenCalledWith(mockData);
    });

    it('should handle multiple listeners', () => {
      const callback1 = jest.fn();
      const callback2 = jest.fn();
      Ahrs.addListener(callback1);
      Ahrs.addListener(callback2);

      const mockData: Partial<AhrsData> = { roll: 10, pitch: 5, heading: 90 };
      emitAhrsData(mockData);

      expect(callback1).toHaveBeenCalled();
      expect(callback2).toHaveBeenCalled();
    });

    it('should isolate errors in callbacks', () => {
      const callback1 = jest.fn(() => {
        throw new Error('Callback error');
      });
      const callback2 = jest.fn();
      Ahrs.addListener(callback1);
      Ahrs.addListener(callback2);

      const mockData: Partial<AhrsData> = { roll: 10 };
      emitAhrsData(mockData);

      // Both should be called despite error in first
      expect(callback1).toHaveBeenCalled();
      expect(callback2).toHaveBeenCalled();
    });

    it('should remove listener when unsubscribe is called', () => {
      const callback = jest.fn();
      const unsubscribe = Ahrs.addListener(callback);

      unsubscribe();

      const mockData: Partial<AhrsData> = { roll: 10 };
      emitAhrsData(mockData);

      expect(callback).not.toHaveBeenCalled();
    });

    it('should stop AHRS when last listener is removed', () => {
      const callback = jest.fn();
      const unsubscribe = Ahrs.addListener(callback);
      Ahrs.start();
      unsubscribe();

      expect(NativeAhrs.stopAhrs).toHaveBeenCalled();
    });
  });

  describe('addPlaybackListener', () => {
    it('should register and receive playback events', () => {
      const listener = jest.fn();
      const unsubscribe = Ahrs.addPlaybackListener(listener);

      expect(typeof unsubscribe).toBe('function');

      const startedEvent: PlaybackStateEvent = {
        status: 'started',
        filename: 'test.json.gz',
      };
      emitPlaybackEvent(startedEvent);
      expect(listener).toHaveBeenCalledWith(startedEvent);

      unsubscribe();

      const completedEvent: PlaybackStateEvent = {
        status: 'completed',
        filename: 'test.json.gz',
      };
      emitPlaybackEvent(completedEvent);
      expect(listener).toHaveBeenCalledTimes(1);
    });
  });

  describe('start', () => {
    it('should start AHRS when listeners are registered', () => {
      Ahrs.addListener(jest.fn());
      Ahrs.start();

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(1);
    });

    it('should not start if already running', () => {
      Ahrs.addListener(jest.fn());
      Ahrs.start();
      Ahrs.start();

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(1);
    });

    it('should warn and not start if no listeners registered', () => {
      Ahrs.start();

      expect(NativeAhrs.startAhrs).not.toHaveBeenCalled();
    });

    it('should throw error if native module fails', () => {
      Ahrs.addListener(jest.fn());
      (NativeAhrs.startAhrs as jest.Mock).mockImplementation(() => {
        throw new Error('Start failed');
      });

      expect(() => Ahrs.start()).toThrow('Start failed');

      // Reset the mock after this test to prevent affecting other tests
      (NativeAhrs.startAhrs as jest.Mock).mockReset();
      (NativeAhrs.startAhrs as jest.Mock).mockResolvedValue(undefined);
    });
  });

  describe('stop', () => {
    it('should stop AHRS when running', () => {
      Ahrs.addListener(jest.fn());
      Ahrs.start();
      Ahrs.stop();

      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(1);
    });

    it('should not stop if not running', () => {
      Ahrs.stop();

      expect(NativeAhrs.stopAhrs).not.toHaveBeenCalled();
    });
  });

  describe('reset', () => {
    it('should call native resetAhrs', () => {
      Ahrs.reset();

      expect(NativeAhrs.resetAhrs).toHaveBeenCalledTimes(1);
    });

    it('should handle errors gracefully', () => {
      (NativeAhrs.resetAhrs as jest.Mock).mockImplementation(() => {
        throw new Error('Reset failed');
      });

      // Should not throw
      expect(() => Ahrs.reset()).not.toThrow();
    });
  });

  describe('level', () => {
    it('should call native levelAhrs', () => {
      Ahrs.level();

      expect(NativeAhrs.levelAhrs).toHaveBeenCalledTimes(1);
    });
  });


  describe('setQNH', () => {
    it('should call native setQNH', () => {
      Ahrs.setQNH(1013.25);

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1013.25);
    });
  });

  describe('setRate', () => {
    it('should call native setAhrsRate with valid rate', () => {
      Ahrs.setRate(10);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(10);
    });

    it('should warn and not set rate if out of range (too low)', () => {
      Ahrs.setRate(0);

      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });

    it('should warn and not set rate if out of range (too high)', () => {
      Ahrs.setRate(100);

      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });
  });

  describe('setRotation', () => {
    it('should call native setAhrsRotation with valid rotation', () => {
      Ahrs.setRotation('left');

      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('left');
    });

    it('should warn and not set rotation if invalid', () => {
      Ahrs.setRotation('invalid' as AhrsRotation);

      expect(NativeAhrs.setAhrsRotation).not.toHaveBeenCalled();
    });
  });

  describe('removeAllListeners', () => {
    it('should remove all listeners and stop', () => {
      Ahrs.addListener(jest.fn());
      Ahrs.addListener(jest.fn());
      Ahrs.start();
      Ahrs.removeAllListeners();

      expect(NativeAhrs.stopAhrs).toHaveBeenCalled();
      expect(Ahrs.getStatus().listenerCount).toBe(0);
    });
  });

  describe('getStatus', () => {
    it('should return current status', () => {
      const status = Ahrs.getStatus();

      expect(status).toHaveProperty('isRunning');
      expect(status).toHaveProperty('listenerCount');
      expect(typeof status.isRunning).toBe('boolean');
      expect(typeof status.listenerCount).toBe('number');
    });

    it('should reflect running state', () => {
      Ahrs.addListener(jest.fn());
      expect(Ahrs.getStatus().isRunning).toBe(false);

      Ahrs.start();
      expect(Ahrs.getStatus().isRunning).toBe(true);

      Ahrs.stop();
      expect(Ahrs.getStatus().isRunning).toBe(false);
    });

    it('should reflect listener count', () => {
      expect(Ahrs.getStatus().listenerCount).toBe(0);

      const unsubscribe1 = Ahrs.addListener(jest.fn());
      expect(Ahrs.getStatus().listenerCount).toBe(1);

      Ahrs.addListener(jest.fn());
      expect(Ahrs.getStatus().listenerCount).toBe(2);

      unsubscribe1();
      expect(Ahrs.getStatus().listenerCount).toBe(1);
    });
  });

  describe('Recording functionality', () => {
    it('should start recording', () => {
      Ahrs.startRecording();

      expect(NativeAhrs.startRecording).toHaveBeenCalledTimes(1);
    });

    it('should stop recording', () => {
      Ahrs.stopRecording();

      expect(NativeAhrs.stopRecording).toHaveBeenCalledTimes(1);
    });

    it('should get recording files', async () => {
      const mockFiles: RecordingFile[] = [
        { filename: 'test1.json.gz', size: 1024, date: Date.now() },
        { filename: 'test2.json.gz', size: 2048, date: Date.now() - 1000 },
      ];
      (NativeAhrs.getRecordingFiles as jest.Mock).mockResolvedValue(mockFiles);

      const files = await Ahrs.getRecordingFiles();

      expect(files).toEqual(mockFiles);
      expect(NativeAhrs.getRecordingFiles).toHaveBeenCalledTimes(1);
    });

    it('should delete recording', () => {
      Ahrs.deleteRecording('test.json.gz');

      expect(NativeAhrs.deleteRecording).toHaveBeenCalledWith('test.json.gz');
    });
  });

  describe('Playback functionality', () => {
    it('should start playback', () => {
      Ahrs.playbackRecording('test.json.gz');

      expect(NativeAhrs.playbackRecording).toHaveBeenCalledWith('test.json.gz');
    });

    it('should stop playback', () => {
      Ahrs.stopPlayback();

      expect(NativeAhrs.stopPlayback).toHaveBeenCalledTimes(1);
    });

    it('should track playback active state', () => {
      Ahrs.addPlaybackListener(jest.fn()); // Initialize subscription
      expect(Ahrs.isPlaybackActive()).toBe(false);

      emitPlaybackEvent({ status: 'started', filename: 'test.json.gz' });
      expect(Ahrs.isPlaybackActive()).toBe(true);

      emitPlaybackEvent({ status: 'completed', filename: 'test.json.gz' });
      expect(Ahrs.isPlaybackActive()).toBe(false);
    });

    it('should handle playback stopped event', () => {
      const listener = jest.fn();
      Ahrs.addPlaybackListener(listener);

      const stoppedEvent: PlaybackStateEvent = {
        status: 'stopped',
        filename: 'test.json.gz',
        reason: 'User requested stop',
      };
      emitPlaybackEvent(stoppedEvent);

      expect(listener).toHaveBeenCalledWith(stoppedEvent);
      expect(Ahrs.isPlaybackActive()).toBe(false);
    });
  });

  describe('X-Plane connection', () => {
    it('should connect to X-Plane', () => {
      Ahrs.addListener(jest.fn()); // Initialize native subscription
      Ahrs.connectToXPlane('192.168.1.100');

      expect(NativeAhrs.connectToXPlane).toHaveBeenCalledWith('192.168.1.100');
    });

    it('should disconnect from X-Plane', () => {
      // Clear mock calls from beforeEach's removeAllListeners
      (NativeAhrs.disconnectFromXPlane as jest.Mock).mockClear();

      Ahrs.disconnectFromXPlane();

      expect(NativeAhrs.disconnectFromXPlane).toHaveBeenCalledTimes(1);
    });

    it('should track X-Plane connection state', () => {
      Ahrs.addListener(jest.fn()); // Initialize native subscription

      expect(Ahrs.isXPlaneConnected()).toBe(false);
      expect(Ahrs.getXPlaneHost()).toBeNull();

      const connectedEvent: XPlaneConnectionEvent = {
        connected: true,
        host: '192.168.1.100',
      };
      emitXPlaneEvent(connectedEvent);

      expect(Ahrs.isXPlaneConnected()).toBe(true);
      expect(Ahrs.getXPlaneHost()).toBe('192.168.1.100');
    });

    it('should handle X-Plane disconnection', () => {
      Ahrs.addListener(jest.fn()); // Initialize native subscription

      // First connect
      emitXPlaneEvent({ connected: true, host: '192.168.1.100' });
      expect(Ahrs.isXPlaneConnected()).toBe(true);

      // Then disconnect
      emitXPlaneEvent({ connected: false, host: '' });
      expect(Ahrs.isXPlaneConnected()).toBe(false);
      expect(Ahrs.getXPlaneHost()).toBeNull();
    });

    it('should add and remove X-Plane connection listener', () => {
      Ahrs.addListener(jest.fn()); // Initialize native subscription

      const listener = jest.fn();
      const unsubscribe = Ahrs.addXPlaneConnectionListener(listener);

      const event: XPlaneConnectionEvent = {
        connected: true,
        host: '192.168.1.100',
      };
      emitXPlaneEvent(event);

      expect(listener).toHaveBeenCalledWith(event);

      unsubscribe();

      emitXPlaneEvent({ connected: false, host: '' });
      expect(listener).toHaveBeenCalledTimes(1); // Should not be called again
    });
  });

  describe('removeAllListeners comprehensive', () => {
    it('should clear all state including playback and X-Plane', () => {
      Ahrs.addListener(jest.fn());
      Ahrs.addPlaybackListener(jest.fn());
      Ahrs.addXPlaneConnectionListener(jest.fn());
      Ahrs.start();

      // Simulate active playback and X-Plane connection
      emitPlaybackEvent({ status: 'started', filename: 'test.json.gz' });
      emitXPlaneEvent({ connected: true, host: '192.168.1.100' });

      expect(Ahrs.isPlaybackActive()).toBe(true);
      expect(Ahrs.isXPlaneConnected()).toBe(true);

      Ahrs.removeAllListeners();

      expect(Ahrs.getStatus().listenerCount).toBe(0);
      expect(Ahrs.getStatus().isRunning).toBe(false);
      expect(Ahrs.isPlaybackActive()).toBe(false);
      expect(Ahrs.isXPlaneConnected()).toBe(false);
      expect(Ahrs.getXPlaneHost()).toBeNull();
      expect(NativeAhrs.disconnectFromXPlane).toHaveBeenCalled();
    });
  });

  describe('edge cases', () => {
    it('should handle rapid start/stop cycles', () => {
      Ahrs.addListener(jest.fn());

      for (let i = 0; i < 10; i++) {
        Ahrs.start();
        Ahrs.stop();
      }

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(10);
      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(10);
    });

    it('should handle multiple unsubscribe calls gracefully', () => {
      const unsubscribe = Ahrs.addListener(jest.fn());

      unsubscribe();
      unsubscribe(); // Should not throw
      unsubscribe();

      expect(Ahrs.getStatus().listenerCount).toBe(0);
    });

    it('should handle callbacks that modify listener list during iteration', () => {
      let unsubscribe2: (() => void) | null = null;

      const callback1 = jest.fn(() => {
        if (unsubscribe2) {
          unsubscribe2();
        }
      });
      const callback2 = jest.fn();

      Ahrs.addListener(callback1);
      unsubscribe2 = Ahrs.addListener(callback2);

      const mockData: Partial<AhrsData> = { roll: 10 };

      // Should not throw even though callback1 modifies listeners
      expect(() => emitAhrsData(mockData)).not.toThrow();
    });

    it('should handle all valid rotation values', () => {
      const rotations: AhrsRotation[] = ['none', 'left', 'right'];

      for (const rotation of rotations) {
        Ahrs.setRotation(rotation);
        expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith(rotation);
      }
    });

    it('should handle rate boundary values', () => {
      // Valid boundary values
      Ahrs.setRate(1);
      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(1);

      Ahrs.setRate(60);
      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(60);

      // Invalid boundary values
      (NativeAhrs.setAhrsRate as jest.Mock).mockClear();
      Ahrs.setRate(0.9);
      Ahrs.setRate(60.1);
      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });
  });
});
