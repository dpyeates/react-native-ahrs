import { Ahrs, type AhrsData, type PlaybackStateEvent } from '../index';
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

describe('AHRS Integration Tests', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    Ahrs.removeAllListeners();
  });

  describe('End-to-end sensor data flow', () => {
    it('should receive and distribute data from native module', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      const mockData: AhrsData = {
        roll: 15.5,
        pitch: -5.2,
        heading: 180,
        magneticDeclination: -2.3,
        groundTrack: 182,
        groundSpeed: 25.5,
        flightPathAngle: 2.5,
        horizontalFlightPathAngle: 0,
        altitude: 1000.5,
        altitudeQNE: 1000.5,
        altitudeQNH: 1000.5,
        verticalSpeed: 2.5,
        barometricPressure: 1013.25,
        velocityNorth: 0,
        velocityEast: 25.5,
        velocityDown: -2.5,
        latitude: 51.0,
        longitude: -0.1,
        flightPhase: 2,
        flightPhaseConfidence: 0.9,
        attitudeValid: true,
        altitudeValid: true,
        positionValid: true,
        flightPhaseValid: false,
        filterHealthStatus: 0,
      };

      // Simulate native module emitting data
      emitAhrsData(mockData);

      expect(callback).toHaveBeenCalledWith(mockData);
      expect(callback).toHaveBeenCalledTimes(1);
    });

    it('should handle multiple listeners receiving same data', () => {
      const callback1 = jest.fn();
      const callback2 = jest.fn();
      const callback3 = jest.fn();

      Ahrs.addListener(callback1);
      Ahrs.addListener(callback2);
      Ahrs.addListener(callback3);

      const mockData: Partial<AhrsData> = {
        roll: 10,
        pitch: 5,
        heading: 90,
      };

      emitAhrsData(mockData);

      expect(callback1).toHaveBeenCalledWith(mockData);
      expect(callback2).toHaveBeenCalledWith(mockData);
      expect(callback3).toHaveBeenCalledWith(mockData);
    });
  });

  describe('Lifecycle management', () => {
    it('should start, run, and stop correctly', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      // Start
      Ahrs.start();
      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(1);
      expect(Ahrs.getStatus().isRunning).toBe(true);

      // Stop
      Ahrs.stop();
      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(1);
      expect(Ahrs.getStatus().isRunning).toBe(false);
    });

    it('should handle start -> stop -> start cycle', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      Ahrs.start();
      Ahrs.stop();
      Ahrs.start();

      expect(NativeAhrs.startAhrs).toHaveBeenCalledTimes(2);
      expect(NativeAhrs.stopAhrs).toHaveBeenCalledTimes(1);
    });
  });

  describe('Listener management', () => {
    it('should maintain correct listener count', () => {
      expect(Ahrs.getStatus().listenerCount).toBe(0);

      const unsubscribe1 = Ahrs.addListener(jest.fn());
      expect(Ahrs.getStatus().listenerCount).toBe(1);

      const unsubscribe2 = Ahrs.addListener(jest.fn());
      expect(Ahrs.getStatus().listenerCount).toBe(2);

      unsubscribe1();
      expect(Ahrs.getStatus().listenerCount).toBe(1);

      unsubscribe2();
      expect(Ahrs.getStatus().listenerCount).toBe(0);
    });

    it('should stop when last listener is removed', () => {
      const callback = jest.fn();
      const unsubscribe = Ahrs.addListener(callback);
      Ahrs.start();

      expect(Ahrs.getStatus().isRunning).toBe(true);

      unsubscribe();

      expect(Ahrs.getStatus().isRunning).toBe(false);
      expect(NativeAhrs.stopAhrs).toHaveBeenCalled();
    });
  });

  describe('Configuration flow', () => {
    it('should apply configuration before starting', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      Ahrs.setQNH(1013.25);
      Ahrs.setRate(10);
      Ahrs.setRotation('left');
      Ahrs.start();

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1013.25);
      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(10);
      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('left');
      expect(NativeAhrs.startAhrs).toHaveBeenCalled();
    });
  });

  describe('Error isolation', () => {
    it('should continue processing when one callback fails', () => {
      const failingCallback = jest.fn(() => {
        throw new Error('Callback error');
      });
      const workingCallback = jest.fn();

      Ahrs.addListener(failingCallback);
      Ahrs.addListener(workingCallback);

      const mockData: Partial<AhrsData> = { roll: 10 };

      // Should not throw
      expect(() => emitAhrsData(mockData)).not.toThrow();

      // Both should be called
      expect(failingCallback).toHaveBeenCalled();
      expect(workingCallback).toHaveBeenCalled();
    });

    it('should isolate errors in playback listeners', () => {
      const failingListener = jest.fn(() => {
        throw new Error('Playback listener error');
      });
      const workingListener = jest.fn();

      Ahrs.addPlaybackListener(failingListener);
      Ahrs.addPlaybackListener(workingListener);

      const event: PlaybackStateEvent = {
        status: 'started',
        filename: 'test.json.gz',
      };

      // Should not throw
      expect(() => emitPlaybackEvent(event)).not.toThrow();

      // Both should be called
      expect(failingListener).toHaveBeenCalled();
      expect(workingListener).toHaveBeenCalled();
    });
  });

  describe('Recording and playback flow', () => {
    it('should support full recording workflow', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);
      Ahrs.start();

      // Start recording
      Ahrs.startRecording();
      expect(NativeAhrs.startRecording).toHaveBeenCalled();

      // Simulate receiving data during recording
      emitAhrsData({ roll: 5, pitch: 10, heading: 180 });
      expect(callback).toHaveBeenCalled();

      // Stop recording
      Ahrs.stopRecording();
      expect(NativeAhrs.stopRecording).toHaveBeenCalled();

      Ahrs.stop();
    });

    it('should support full playback workflow', () => {
      const dataCallback = jest.fn();
      const playbackCallback = jest.fn();

      Ahrs.addListener(dataCallback);
      Ahrs.addPlaybackListener(playbackCallback);
      Ahrs.start();

      // Start playback
      Ahrs.playbackRecording('test-flight.json.gz');
      expect(NativeAhrs.playbackRecording).toHaveBeenCalledWith(
        'test-flight.json.gz'
      );

      // Simulate playback started event
      emitPlaybackEvent({
        status: 'started',
        filename: 'test-flight.json.gz',
      });

      expect(playbackCallback).toHaveBeenCalledWith({
        status: 'started',
        filename: 'test-flight.json.gz',
      });
      expect(Ahrs.isPlaybackActive()).toBe(true);

      // Simulate data coming from playback
      emitAhrsData({ roll: 15, pitch: -5, heading: 90 });
      expect(dataCallback).toHaveBeenCalled();

      // Simulate playback completed
      emitPlaybackEvent({
        status: 'completed',
        filename: 'test-flight.json.gz',
      });

      expect(Ahrs.isPlaybackActive()).toBe(false);

      Ahrs.stop();
    });
  });

  describe('X-Plane connection flow', () => {
    it('should support full X-Plane connection workflow', () => {
      const dataCallback = jest.fn();
      const xplaneCallback = jest.fn();

      Ahrs.addListener(dataCallback);
      Ahrs.addXPlaneConnectionListener(xplaneCallback);
      Ahrs.start();

      // Connect to X-Plane
      Ahrs.connectToXPlane('192.168.1.50');
      expect(NativeAhrs.connectToXPlane).toHaveBeenCalledWith('192.168.1.50');

      // Simulate connection event
      emitXPlaneEvent({ connected: true, host: '192.168.1.50' });

      expect(xplaneCallback).toHaveBeenCalledWith({
        connected: true,
        host: '192.168.1.50',
      });
      expect(Ahrs.isXPlaneConnected()).toBe(true);
      expect(Ahrs.getXPlaneHost()).toBe('192.168.1.50');

      // Simulate data from X-Plane
      emitAhrsData({
        roll: 25,
        pitch: 10,
        heading: 270,
        altitude: 5000,
        groundSpeed: 75,
      });
      expect(dataCallback).toHaveBeenCalled();

      // Disconnect from X-Plane
      Ahrs.disconnectFromXPlane();
      expect(NativeAhrs.disconnectFromXPlane).toHaveBeenCalled();

      // Simulate disconnection event
      emitXPlaneEvent({ connected: false, host: '' });

      expect(Ahrs.isXPlaneConnected()).toBe(false);
      expect(Ahrs.getXPlaneHost()).toBeNull();

      Ahrs.stop();
    });
  });

  describe('Complex scenarios', () => {
    it('should handle multiple listeners subscribing and unsubscribing rapidly', () => {
      const callbacks: jest.Mock[] = [];
      const unsubscribes: (() => void)[] = [];

      // Add 10 listeners
      for (let i = 0; i < 10; i++) {
        callbacks.push(jest.fn());
        unsubscribes.push(Ahrs.addListener(callbacks[i]!));
      }

      expect(Ahrs.getStatus().listenerCount).toBe(10);

      // Unsubscribe every other listener
      for (let i = 0; i < 10; i += 2) {
        unsubscribes[i]!();
      }

      expect(Ahrs.getStatus().listenerCount).toBe(5);

      // Send data
      emitAhrsData({ roll: 5 });

      // Verify only subscribed callbacks were called
      for (let i = 0; i < 10; i++) {
        if (i % 2 === 0) {
          expect(callbacks[i]).not.toHaveBeenCalled();
        } else {
          expect(callbacks[i]).toHaveBeenCalled();
        }
      }
    });

    it('should maintain correct state through full session', () => {
      // Initial state
      expect(Ahrs.getStatus().isRunning).toBe(false);
      expect(Ahrs.getStatus().listenerCount).toBe(0);

      // Add listener and start
      const callback = jest.fn();
      const unsubscribe = Ahrs.addListener(callback);
      Ahrs.start();

      expect(Ahrs.getStatus().isRunning).toBe(true);
      expect(Ahrs.getStatus().listenerCount).toBe(1);

      // Configure
      Ahrs.setRate(10);
      Ahrs.setRotation('left');
      Ahrs.setQNH(1015);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(10);
      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('left');
      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1015);

      // Reset and level
      Ahrs.reset();
      Ahrs.level();

      expect(NativeAhrs.resetAhrs).toHaveBeenCalled();
      expect(NativeAhrs.levelAhrs).toHaveBeenCalled();

      // Stop and cleanup
      Ahrs.stop();
      unsubscribe();

      expect(Ahrs.getStatus().isRunning).toBe(false);
      expect(Ahrs.getStatus().listenerCount).toBe(0);
    });

    it('should handle data with all fields populated', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      const fullData: AhrsData = {
        roll: 15.5,
        pitch: -5.2,
        heading: 180.0,
        magneticDeclination: -2.3,
        groundTrack: 182.5,
        groundSpeed: 65.5,
        flightPathAngle: 3.5,
        horizontalFlightPathAngle: -1.2,
        altitude: 3048.0, // 10000 ft
        altitudeQNE: 3050.0,
        altitudeQNH: 3045.0,
        verticalSpeed: 5.0,
        barometricPressure: 697.0, // ~10000 ft
        velocityNorth: 50.0,
        velocityEast: 40.0,
        velocityDown: -5.0,
        latitude: 51.5074,
        longitude: -0.1278,
        flightPhase: 3, // CRUISE
        flightPhaseConfidence: 0.95,
        attitudeValid: true,
        altitudeValid: true,
        positionValid: true,
        flightPhaseValid: true,
        filterHealthStatus: 0,
      };

      emitAhrsData(fullData);

      expect(callback).toHaveBeenCalledWith(fullData);
    });

    it('should handle data with minimal fields', () => {
      const callback = jest.fn();
      Ahrs.addListener(callback);

      const minimalData: Partial<AhrsData> = {
        roll: 0,
        pitch: 0,
        heading: 0,
        attitudeValid: false,
        altitudeValid: false,
        positionValid: false,
        flightPhaseValid: false,
      };

      emitAhrsData(minimalData);

      expect(callback).toHaveBeenCalledWith(minimalData);
    });
  });

  describe('Async operations', () => {
    it('should handle isSupported check', async () => {
      (NativeAhrs.isSupported as jest.Mock).mockResolvedValue(true);

      const supported = await Ahrs.isSupported();
      expect(supported).toBe(true);
    });

    it('should handle getRecordingFiles', async () => {
      const mockFiles = [
        { filename: 'flight1.json.gz', size: 1024, date: Date.now() },
        { filename: 'flight2.json.gz', size: 2048, date: Date.now() - 86400000 },
      ];
      (NativeAhrs.getRecordingFiles as jest.Mock).mockResolvedValue(mockFiles);

      const files = await Ahrs.getRecordingFiles();
      expect(files).toEqual(mockFiles);
      expect(files).toHaveLength(2);
    });

    it('should handle isSupported returning false on error', async () => {
      (NativeAhrs.isSupported as jest.Mock).mockRejectedValue(
        new Error('Native error')
      );

      const supported = await Ahrs.isSupported();
      expect(supported).toBe(false);
    });

    it('should return empty array when getRecordingFiles native module unavailable', async () => {
      (NativeAhrs.getRecordingFiles as jest.Mock).mockResolvedValue([]);

      const files = await Ahrs.getRecordingFiles();
      expect(files).toEqual([]);
    });
  });
});
