import { logger } from '../AhrsLogger';

describe('AhrsLogger', () => {
  let consoleSpy: {
    log: jest.SpyInstance;
    warn: jest.SpyInstance;
    error: jest.SpyInstance;
    info: jest.SpyInstance;
    debug: jest.SpyInstance;
  };

  beforeEach(() => {
    consoleSpy = {
      log: jest.spyOn(console, 'log').mockImplementation(),
      warn: jest.spyOn(console, 'warn').mockImplementation(),
      error: jest.spyOn(console, 'error').mockImplementation(),
      info: jest.spyOn(console, 'info').mockImplementation(),
      debug: jest.spyOn(console, 'debug').mockImplementation(),
    };
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  describe('in development mode (__DEV__ = true)', () => {
    beforeAll(() => {
      (global as unknown as { __DEV__: boolean }).__DEV__ = true;
    });

    it('should log messages with log()', () => {
      logger.log('test message');
      expect(consoleSpy.log).toHaveBeenCalledWith('test message');
    });

    it('should log warnings with warn()', () => {
      logger.warn('warning message');
      expect(consoleSpy.warn).toHaveBeenCalledWith('warning message');
    });

    it('should log errors with error()', () => {
      logger.error('error message');
      expect(consoleSpy.error).toHaveBeenCalledWith('error message');
    });

    it('should log info with info()', () => {
      logger.info('info message');
      expect(consoleSpy.info).toHaveBeenCalledWith('info message');
    });

    it('should log debug with debug()', () => {
      logger.debug('debug message');
      expect(consoleSpy.debug).toHaveBeenCalledWith('debug message');
    });

    it('should handle multiple arguments', () => {
      logger.log('message', { key: 'value' }, 123);
      expect(consoleSpy.log).toHaveBeenCalledWith(
        'message',
        { key: 'value' },
        123
      );
    });

    it('should handle Error objects', () => {
      const error = new Error('test error');
      logger.error('An error occurred:', error);
      expect(consoleSpy.error).toHaveBeenCalledWith('An error occurred:', error);
    });

    it('should handle undefined and null arguments', () => {
      logger.log(undefined, null);
      expect(consoleSpy.log).toHaveBeenCalledWith(undefined, null);
    });
  });

  describe('in production mode (__DEV__ = false)', () => {
    beforeAll(() => {
      (global as unknown as { __DEV__: boolean }).__DEV__ = false;
    });

    afterAll(() => {
      (global as unknown as { __DEV__: boolean }).__DEV__ = true;
    });

    it('should not log messages with log()', () => {
      logger.log('test message');
      expect(consoleSpy.log).not.toHaveBeenCalled();
    });

    it('should not log warnings with warn()', () => {
      logger.warn('warning message');
      expect(consoleSpy.warn).not.toHaveBeenCalled();
    });

    it('should not log errors with error()', () => {
      logger.error('error message');
      expect(consoleSpy.error).not.toHaveBeenCalled();
    });

    it('should not log info with info()', () => {
      logger.info('info message');
      expect(consoleSpy.info).not.toHaveBeenCalled();
    });

    it('should not log debug with debug()', () => {
      logger.debug('debug message');
      expect(consoleSpy.debug).not.toHaveBeenCalled();
    });
  });
});
