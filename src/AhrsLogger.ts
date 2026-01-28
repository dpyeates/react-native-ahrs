/**
 * AHRS Logger utility
 *
 * All logging methods are noop in production builds (when __DEV__ is false).
 * This prevents console output in production while allowing debug logs during development.
 */

type LogLevel = 'log' | 'warn' | 'error' | 'info' | 'debug';

class Logger {
  /**
   * Internal method to log a message at the specified level
   * Only outputs in development builds
   *
   * @param level - Log level (log, warn, error, info, debug)
   * @param args - Arguments to log
   */
  private logInternal(level: LogLevel, ...args: unknown[]): void {
    if (__DEV__) {
      // eslint-disable-next-line no-console
      console[level](...args);
    }
  }

  /**
   * Logs an informational message
   * Use for general information during development
   */
  log(...args: unknown[]): void {
    this.logInternal('log', ...args);
  }

  /**
   * Logs a warning message
   * Use for warnings that don't prevent functionality
   */
  warn(...args: unknown[]): void {
    this.logInternal('warn', ...args);
  }

  /**
   * Logs an error message
   * Use for actual errors (always logged, even in production if needed)
   * Note: Currently still respects __DEV__, but can be modified to always log errors
   */
  error(...args: unknown[]): void {
    this.logInternal('error', ...args);
  }

  /**
   * Logs an info message
   * Use for informational messages
   */
  info(...args: unknown[]): void {
    this.logInternal('info', ...args);
  }

  /**
   * Logs a debug message
   * Use for detailed debugging information
   */
  debug(...args: unknown[]): void {
    this.logInternal('debug', ...args);
  }
}

export const logger = new Logger();
