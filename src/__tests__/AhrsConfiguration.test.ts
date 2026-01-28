import { AhrsConfiguration } from '../AhrsConfiguration';
import NativeAhrs from '../NativeAhrs';
import { logger } from '../AhrsLogger';

// Mock dependencies
jest.mock('../NativeAhrs', () => ({
  __esModule: true,
  default: {
    setAhrsRate: jest.fn(),
    setAhrsRotation: jest.fn(),
    setQNH: jest.fn(),
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

describe('AhrsConfiguration', () => {
  let config: AhrsConfiguration;

  beforeEach(() => {
    config = new AhrsConfiguration();
    jest.clearAllMocks();
  });

  describe('setRate', () => {
    it('should set rate when within valid range (1-60 Hz)', () => {
      config.setRate(10);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(10);
      expect(logger.log).toHaveBeenCalledWith('⚡ Setting AHRS rate to 10 Hz');
    });

    it('should set minimum rate (1 Hz)', () => {
      config.setRate(1);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(1);
    });

    it('should set maximum rate (60 Hz)', () => {
      config.setRate(60);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(60);
    });

    it('should warn and not set rate below minimum', () => {
      config.setRate(0);

      expect(logger.warn).toHaveBeenCalledWith(
        'AHRS rate should be between 1-60 Hz'
      );
      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });

    it('should warn and not set rate above maximum', () => {
      config.setRate(61);

      expect(logger.warn).toHaveBeenCalledWith(
        'AHRS rate should be between 1-60 Hz'
      );
      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });

    it('should warn for negative rate', () => {
      config.setRate(-5);

      expect(logger.warn).toHaveBeenCalledWith(
        'AHRS rate should be between 1-60 Hz'
      );
      expect(NativeAhrs.setAhrsRate).not.toHaveBeenCalled();
    });

    it('should handle native module errors gracefully', () => {
      (NativeAhrs.setAhrsRate as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Native error');
      });

      expect(() => config.setRate(10)).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Error setting AHRS rate:',
        expect.any(Error)
      );
    });

    it('should accept decimal rates within range', () => {
      config.setRate(5.5);

      expect(NativeAhrs.setAhrsRate).toHaveBeenCalledWith(5.5);
    });
  });

  describe('setRotation', () => {
    it('should set rotation to "none"', () => {
      config.setRotation('none');

      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('none');
      expect(logger.log).toHaveBeenCalledWith('🔄 Setting AHRS rotation to none');
    });

    it('should set rotation to "left"', () => {
      config.setRotation('left');

      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('left');
      expect(logger.log).toHaveBeenCalledWith('🔄 Setting AHRS rotation to left');
    });

    it('should set rotation to "right"', () => {
      config.setRotation('right');

      expect(NativeAhrs.setAhrsRotation).toHaveBeenCalledWith('right');
      expect(logger.log).toHaveBeenCalledWith(
        '🔄 Setting AHRS rotation to right'
      );
    });

    it('should warn and not set invalid rotation', () => {
      config.setRotation('invalid' as never);

      expect(logger.warn).toHaveBeenCalledWith(
        "AHRS rotation must be 'none' | 'left' | 'right', got: invalid"
      );
      expect(NativeAhrs.setAhrsRotation).not.toHaveBeenCalled();
    });

    it('should warn for empty string rotation', () => {
      config.setRotation('' as never);

      expect(logger.warn).toHaveBeenCalled();
      expect(NativeAhrs.setAhrsRotation).not.toHaveBeenCalled();
    });

    it('should handle native module errors gracefully', () => {
      (NativeAhrs.setAhrsRotation as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Native error');
      });

      expect(() => config.setRotation('left')).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        'Error setting AHRS rotation:',
        expect.any(Error)
      );
    });
  });

  describe('setQNH', () => {
    it('should set standard QNH (1013.25 hPa)', () => {
      config.setQNH(1013.25);

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1013.25);
    });

    it('should set low QNH value', () => {
      config.setQNH(970);

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(970);
    });

    it('should set high QNH value', () => {
      config.setQNH(1050);

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1050);
    });

    it('should handle native module errors gracefully', () => {
      (NativeAhrs.setQNH as jest.Mock).mockImplementationOnce(() => {
        throw new Error('Native error');
      });

      expect(() => config.setQNH(1013.25)).not.toThrow();
      expect(logger.error).toHaveBeenCalledWith(
        '❌ Error setting QNH',
        expect.any(Error)
      );
    });

    it('should accept decimal QNH values', () => {
      config.setQNH(1013.567);

      expect(NativeAhrs.setQNH).toHaveBeenCalledWith(1013.567);
    });
  });
});
