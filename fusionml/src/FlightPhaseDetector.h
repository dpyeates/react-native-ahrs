/**
 * FlightPhaseDetector.h
 *
 * Flight Phase Detection Finite State Machine (FSM) for aircraft telemetry analysis.
 *
 * Classifies aircraft flight phases from GPS and sensor telemetry data. Designed
 * for 1Hz updates (throttled internally). Independent from uNavINS filter - uses
 * raw GPS data and optional accelerometer input.
 *
 * Features:
 *   - 9 flight phases covering complete flight cycle
 *   - Deterministic FSM with priority-based transitions
 *   - Persistence timers prevent rapid oscillations
 *   - Startup in-flight detection (first 30 seconds)
 *   - Recovery transitions for go-arounds and unusual maneuvers
 *   - Confidence scoring for phase reliability
 *
 * Flight Phases:
 *   0: GROUND - Aircraft on ground, stationary or slow taxi
 *   1: TAKEOFF_ROLL - Acceleration on ground before takeoff
 *   2: TAKEOFF - Airborne, initial climb phase
 *   3: CLIMB - Sustained climb to cruise altitude
 *   4: CRUISE - Level flight at cruise altitude
 *   5: DESCENT - Descending toward destination
 *   6: APPROACH - Terminal area, final approach
 *   7: LANDING - Flare and short final
 *   8: LANDING_ROLL - Deceleration on ground after landing
 *
 * Transition Priority Order (checked in sequence):
 *   1. Emergency Recovery (ANY → GROUND) - Highest priority
 *   2. Startup In-Flight Detection (first 30 seconds only)
 *   3. Recovery Transitions (go-arounds, unusual maneuvers)
 *   4. Normal Forward Transitions (standard flight progression)
 *   5. Stay in current phase (no transition)
 *
 * Input Requirements:
 *   - GPS altitude (MSL), vertical speed, groundspeed (required)
 *   - GPS position (lat/lon) for departure tracking (required)
 *   - Ground elevation (optional, improves AGL calculations)
 *   - Forward acceleration (optional, improves takeoff roll detection)
 *
 * Thread Safety: Not thread-safe. Use external synchronization if calling from
 * multiple threads.
 *
 * See FlightPhaseDetector.cpp for detailed transition criteria documentation.
 */

#ifndef FLIGHT_PHASE_DETECTOR_H
#define FLIGHT_PHASE_DETECTOR_H

#include <cmath>
#include <limits>
#include <vector>
#include <cstdint>
#include <cstring>

/**
 * @enum FlightPhase
 * @brief Flight phase enumeration
 *
 * Represents the current phase of flight. Phases progress in order during
 * normal flight, but can transition in any direction for recovery scenarios.
 */
enum FlightPhase {
  PHASE_GROUND = 0,        ///< Aircraft on ground (stationary or taxi)
  PHASE_TAKEOFF_ROLL = 1, ///< Acceleration on ground before takeoff
  PHASE_TAKEOFF = 2,      ///< Airborne, initial climb
  PHASE_CLIMB = 3,        ///< Sustained climb to cruise altitude
  PHASE_CRUISE = 4,       ///< Level flight at cruise altitude
  PHASE_DESCENT = 5,      ///< Descending toward destination
  PHASE_APPROACH = 6,     ///< Terminal area, final approach
  PHASE_LANDING = 7,      ///< Flare and short final
  PHASE_LANDING_ROLL = 8  ///< Deceleration on ground after landing
};

/**
 * @class CircularBuffer
 * @brief Simple circular buffer for time-series data storage
 *
 * Fixed-size circular buffer for storing historical values (altitude, groundspeed).
 * Used for computing trends and averages over time windows.
 *
 * @tparam T Element type (float, double, etc.)
 * @tparam N Buffer size (number of elements)
 */
template<typename T, size_t N>
class CircularBuffer {
public:
  /**
   * @brief Constructs empty circular buffer
   */
  CircularBuffer() : head_(0), size_(0) {
    std::memset(data_, 0, sizeof(data_));
  }
  
  /**
   * @brief Adds new value to buffer (overwrites oldest if full)
   * @param value Value to add
   */
  void push(T value) {
    data_[head_] = value;
    head_ = (head_ + 1) % N;
    if (size_ < N) size_++;
  }
  
  /**
   * @brief Gets value at index (0 = oldest, size-1 = newest)
   * @param index Index in buffer (0 to size-1)
   * @return Value at index, or T(0) if index out of range
   */
  T get(size_t index) const {
    if (index >= size_) return T(0);
    size_t pos = (head_ - size_ + index + N) % N;
    return data_[pos];
  }
  
  /**
   * @brief Gets oldest value in buffer
   * @return Oldest value, or T(0) if buffer empty
   */
  T getOldest() const {
    if (size_ == 0) return T(0);
    size_t pos = (head_ - size_) % N;
    return data_[pos];
  }
  
  /**
   * @brief Computes mean of all values in buffer
   * @return Mean value, or 0.0f if buffer empty
   */
  float mean() const {
    if (size_ == 0) return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < size_; i++) {
      sum += get(i);
    }
    return sum / size_;
  }
  
  /**
   * @brief Returns number of elements currently in buffer
   * @return Current size (0 to N)
   */
  size_t size() const { return size_; }
  
  /**
   * @brief Clears all data from buffer
   */
  void clear() { head_ = 0; size_ = 0; }
  
private:
  T data_[N];      ///< Fixed-size array for data storage
  size_t head_;    ///< Index of next write position
  size_t size_;    ///< Current number of elements (0 to N)
};

/**
 * @class FlightPhaseDetector
 * @brief Flight phase detection finite state machine
 *
 * Classifies aircraft flight phases from telemetry data using a deterministic
 * FSM with priority-based transitions. Designed for 1Hz updates (throttled
 * internally to prevent rapid updates).
 *
 * Algorithm:
 *   1. Smooths vertical speed and groundspeed using exponential filter
 *   2. Maintains circular buffers for altitude and groundspeed history
 *   3. Computes derived features (altitude trends, AGL, etc.)
 *   4. Checks transitions in priority order (emergency → startup → recovery → normal)
 *   5. Updates confidence based on phase persistence and data quality
 *
 * State Management:
 *   - Tracks departure location and altitude for relative altitude calculations
 *   - Records takeoff speed for approach detection
 *   - Maintains persistence timers for each transition to prevent oscillations
 *   - Provides confidence score indicating phase reliability
 */
class FlightPhaseDetector {
public:
  /**
   * @brief Constructs flight phase detector
   *
   * Initializes detector in GROUND phase with all timers reset.
   */
  FlightPhaseDetector();
  
  /**
   * @brief Destructor (no cleanup needed)
   */
  ~FlightPhaseDetector();
  
  /**
   * @brief Process new telemetry sample
   *
   * Main update function. Processes telemetry data and updates flight phase.
   * Throttled to 1Hz internally (skips updates if less than 1 second elapsed).
   *
   * Processing steps:
   *   1. First sample initialization (captures departure location/altitude)
   *   2. Smooth vertical speed and groundspeed (exponential filter)
   *   3. Update circular buffers (altitude, groundspeed history)
   *   4. Compute derived features (altitude trends, AGL, etc.)
   *   5. Check transitions (priority order: emergency → startup → recovery → normal)
   *   6. Increment timers
   *   7. Update confidence
   *
   * @param gps_alt_msl GPS altitude in meters MSL (required)
   * @param vertical_speed Vertical speed in m/s, positive = climb (required)
   * @param groundspeed_ms Groundspeed in m/s, converted to knots internally (required)
   * @param lat_deg Latitude in degrees (required)
   * @param lon_deg Longitude in degrees (required)
   * @param timestamp_us Timestamp in microseconds (required)
   * @param ground_elev_msl Ground elevation MSL in meters (optional, pass NAN if unavailable)
   *                        Used for AGL calculations if provided
   * @param accel_forward_mss Forward acceleration in m/s², body frame, positive = forward (optional, pass NAN if unavailable)
   *                          Used for takeoff roll detection if provided
   */
  void update(float gps_alt_msl, float vertical_speed, float groundspeed_ms,
              float lat_deg, float lon_deg, uint64_t timestamp_us,
              float ground_elev_msl = std::numeric_limits<float>::quiet_NaN(),
              float accel_forward_mss = std::numeric_limits<float>::quiet_NaN());
  
  /**
   * @brief Get current flight phase
   *
   * @return Phase number (0-8):
   *   - 0 = GROUND
   *   - 1 = TAKEOFF_ROLL
   *   - 2 = TAKEOFF
   *   - 3 = CLIMB
   *   - 4 = CRUISE
   *   - 5 = DESCENT
   *   - 6 = APPROACH
   *   - 7 = LANDING
   *   - 8 = LANDING_ROLL
   */
  int getFlightPhase() const;
  
  /**
   * @brief Get confidence in current phase detection
   *
   * Confidence is based on:
   *   - Phase persistence (time in current phase)
   *   - Data quality (assumed good if receiving updates)
   *   - Phase stability (higher for stable phases like CRUISE, GROUND)
   *
   * @return Confidence value 0.0-1.0, where 1.0 = highest confidence
   */
  float getConfidence() const;
  
  /**
   * @brief Check if phase detection is valid
   *
   * Phase detection is considered valid after:
   *   - Startup window (30 seconds) has elapsed, OR
   *   - Transition from GROUND phase has occurred
   *
   * @return true if phase is reliable and should be used, false during startup
   */
  bool isValid() const;
  
  /**
   * @brief Reset all state to initial values
   *
   * Resets detector to GROUND phase and clears all history, timers, and
   * departure tracking. Call this when starting a new flight or after
   * long periods of inactivity.
   */
  void reset();
  
private:
  // ============================================================================
  // TUNING CONSTANTS - Groundspeed Thresholds (knots)
  // ============================================================================
  // All groundspeed values are in knots (converted from m/s internally)
  static constexpr float EMERGENCY_GROUND_GS = 15.0f;
  static constexpr float LANDING_GROUND_GS_SLOW = 15.0f;
  static constexpr float LANDING_GROUND_GS = 25.0f;
  static constexpr float TAKEOFF_ROLL_MIN_GS = 10.0f; // Minimum groundspeed to start takeoff roll detection
  static constexpr float TAKEOFF_ROLL_ACCEL_MSS = 1.5f; // Forward acceleration threshold (m/s²) for takeoff roll detection
  static constexpr float TAKEOFF_MIN_GS = 35.0f;
  static constexpr float LANDING_ROLL_DECEL = -2.0f; // Groundspeed deceleration threshold (kt/s) for landing roll detection
  static constexpr float LANDING_ROLL_STOP_GS = 5.0f; // Groundspeed threshold to transition from landing roll to ground
  static constexpr float RECOVERY_MIN_GS = 60.0f;
  static constexpr float APPROACH_LANDING_GS = 70.0f;
  static constexpr float CRUISE_MIN_GS = 70.0f;
  static constexpr float STARTUP_CRUISE_GS = 80.0f;
  static constexpr float APPROACH_SPEED_MULTIPLIER = 1.2f; // Approach speed should be < takeoff_speed * 1.2
  
  // ============================================================================
  // TUNING CONSTANTS - Vertical Speed Thresholds (m/s)
  // ============================================================================
  // Positive values = climb, negative values = descent
  static constexpr float GROUND_VS = 0.3f;
  static constexpr float STABLE_VS = 0.5f;
  static constexpr float CRUISE_VS = 1.5f;
  static constexpr float TAKEOFF_VS = 2.0f;
  static constexpr float STARTUP_CLIMB_VS = 2.0f;
  static constexpr float DESCENT_VS = 2.0f;
  static constexpr float GENTLE_DESCENT_VS = 1.0f; // Gentle descent threshold (~200 fpm) for detecting small descents
  static constexpr float CLIMB_VS = 3.0f;
  static constexpr float RECOVERY_DESCENT_VS = 3.0f;
  static constexpr float RECOVERY_CLIMB_VS = 4.0f;
  
  // ============================================================================
  // TUNING CONSTANTS - Altitude Trends (meters)
  // ============================================================================
  // Thresholds for altitude change detection over time windows
  static constexpr float GROUND_ALT_STABLE = 8.0f;
  static constexpr size_t BUFFER_INIT_THRESHOLD = 10;
  static constexpr float LANDING_ALT_STABLE_HIGH = 15.0f;
  static constexpr float TAKEOFF_ALT_GAIN = 15.0f;
  static constexpr float TAKEOFF_ALT_GAIN_FAST = 5.0f; // Faster detection using 5s window
  static constexpr float LANDING_ALT_LOSS = 20.0f;
  static constexpr float STARTUP_ALT_GAIN = 20.0f;
  static constexpr float CLIMB_ALT_GAIN = 30.0f;
  static constexpr float CRUISE_ALT_STABLE = 40.0f;
  
  // ============================================================================
  // TUNING CONSTANTS - Persistence Timers (seconds)
  // ============================================================================
  // Timers prevent rapid oscillations by requiring conditions to persist
  // before allowing transitions. Higher values = more stable but slower response.
  static constexpr int TAKEOFF_ROLL_TIMER = 2; // Faster detection for takeoff roll
  static constexpr int TAKEOFF_TIMER = 3;
  static constexpr int CLIMB_TIMER = 3;
  static constexpr int CLIMB_AGL_TIMER = 2;
  static constexpr int APPROACH_TIMER_SHORT = 3; // Reduced from 5s to 3s for faster detection
  static constexpr int LANDING_TIMER = 3; // Reduced from 5s to 3s for faster detection
  static constexpr int STARTUP_CLIMB_TIMER = 5;
  static constexpr int GO_AROUND_TIMER = 5;
  static constexpr int APPROACH_TIMER = 5; // Reduced from 8s to 5s for faster detection
  static constexpr int RECOVERY_TIMER = 5; // Reduced from 8s to 5s for faster detection
  static constexpr int DESCENT_TIMER = 3; // Reduced from 10s to 3s for faster detection
  static constexpr int LANDING_GROUND_TIMER_SHORT = 5; // Reduced from 10s to 5s for faster detection
  static constexpr int STARTUP_CRUISE_TIMER = 5; // Reduced from 10s to 5s for faster detection
  static constexpr int LANDING_GROUND_TIMER = 8; // Reduced from 12s to 8s for faster detection
  static constexpr int CRUISE_TIMER = 10; // Reduced from 15s to 10s for faster detection
  static constexpr int LANDING_GROUND_TIMER_LONG = 15; // Reduced from 20s to 15s for faster detection
  static constexpr int EMERGENCY_GROUND_TIMER = 25;
  static constexpr int LANDING_ROLL_TIMER = 2; // Faster detection for landing roll
  static constexpr int LANDING_ROLL_TO_GROUND_TIMER = 3; // Timer for landing roll to ground transition
  
  // ============================================================================
  // TUNING CONSTANTS - AGL (Above Ground Level) Thresholds (meters)
  // ============================================================================
  // AGL calculated from GPS altitude - ground elevation (if available)
  static constexpr float GROUND_AGL = 5.0f;
  static constexpr float CLIMB_MIN_AGL = 61.0f; // approx 200 ft
  static constexpr float LANDING_AGL = 61.0f; // approx 200 ft
  static constexpr float CRUISE_MIN_AGL = 152.0f; // approx 500ft
  static constexpr float APPROACH_AGL = 350.0f; // approx 1100ft
  
  // ============================================================================
  // TUNING CONSTANTS - Relative Altitude Thresholds (meters)
  // ============================================================================
  // Altitude relative to departure field (takeoff location)
  static constexpr float CRUISE_MIN_RELATIVE_ALT = 200.0f;
  static constexpr float APPROACH_MAX_RELATIVE_ALT = 300.0f;
  static constexpr float DESCENT_MAX_RELATIVE_ALT = 500.0f;
  
  // ============================================================================
  // FILTER CONSTANTS
  // ============================================================================
  // Exponential smoothing parameters for noise reduction
  static constexpr float LOWPASS_ALPHA = 0.8187f; // exp(-1/5)
  static constexpr size_t ALT_BUFFER_SIZE = 20;
  static constexpr size_t GS_TREND_BUFFER_SIZE = 5;
  static constexpr float GS_TREND_THRESHOLD = 5.0f;
  static constexpr int STARTUP_WINDOW = 30;
  
  // ============================================================================
  // STATE VARIABLES
  // ============================================================================
  
  // Current FSM state
  int current_phase_;             ///< Current flight phase (0-8, see FlightPhase enum)
  float vs_smooth_;               ///< Smoothed vertical speed (m/s, exponential filter)
  float gs_smooth_;               ///< Smoothed groundspeed (knots, exponential filter)
  float current_confidence_;      ///< Confidence in current phase (0.0-1.0)
  bool is_valid_;                 ///< Phase detection validity flag
  
  // Circular buffers for trend analysis
  CircularBuffer<float, ALT_BUFFER_SIZE> alt_history_;      ///< Altitude history (20 samples = 20s at 1Hz)
  CircularBuffer<float, GS_TREND_BUFFER_SIZE> gs_history_;  ///< Groundspeed history (5 samples = 5s at 1Hz)
  
  // Departure data (captured at takeoff)
  float departure_lat_;           ///< Departure latitude (degrees, captured at first sample)
  float departure_lon_;           ///< Departure longitude (degrees, captured at first sample)
  float departure_field_alt_;     ///< Field altitude at takeoff (meters MSL, captured at TAKEOFF_ROLL)
  float takeoff_speed_kt_;        ///< Groundspeed at takeoff (knots, captured at TAKEOFF phase)
  
  // System state
  bool has_ground_ref_;           ///< True if ground elevation data is available
  int startup_timer_;             ///< Seconds since first sample (0-30, used for startup detection)
  uint64_t last_update_time_us_;  ///< Timestamp of last processed update (for 1Hz throttling)
  bool first_sample_;             ///< True if this is the first sample (needs initialization)
  
  // Derived values (computed each update)
  float delta_alt_20s_;           ///< Altitude change over 20s window (meters)
  float delta_alt_5s_;            ///< Altitude change over 5s window (meters, for faster takeoff detection)
  float gs_acceleration_;         ///< Groundspeed acceleration (kt/s, computed from 5s trend)
  float accel_forward_mss_;       ///< Forward acceleration from accelerometer (m/s², for takeoff roll detection)
  float pseudo_agl_;              ///< Pseudo AGL = GPS altitude - ground elevation (meters, NAN if no ground ref)
  bool gs_increasing_;            ///< True if groundspeed is increasing (trend analysis)
  float current_gps_alt_msl_;     ///< Current GPS altitude MSL (meters, cached for transition checks)
  float current_ground_elev_msl_; ///< Current ground elevation MSL (meters, NAN if unavailable)
  
  // ============================================================================
  // TRANSITION TIMERS (one per transition condition)
  // ============================================================================
  // Each timer counts seconds that transition condition has been met.
  // Transition executes when timer reaches threshold. Timer resets to 0 when
  // condition is not met or after transition executes.
  int timer_emergency_ground_;
  int timer_startup_cruise_;
  int timer_startup_climb_;
  int timer_startup_approach_;
  int timer_startup_descent_;
  int timer_cruise_to_takeoff_;
  int timer_approach_to_climb_;
  int timer_ground_to_takeoff_roll_;
  int timer_takeoff_roll_to_takeoff_;
  int timer_takeoff_to_climb_;
  int timer_climb_to_cruise_;
  int timer_climb_to_descent_; // Direct transition from climb to descent (skipping cruise)
  int timer_cruise_to_descent_;
  int timer_descent_to_climb_; // Recovery from descent (climb again)
  int timer_descent_to_cruise_; // Level off during descent
  int timer_descent_to_approach_;
  int timer_approach_to_cruise_; // Level off from approach back to cruise
  int timer_approach_to_landing_;
  int timer_landing_to_landing_roll_;
  int timer_landing_roll_to_takeoff_roll_; // For touch-and-go scenarios
  int timer_landing_roll_to_ground_;
  
  // ============================================================================
  // HELPER METHODS
  // ============================================================================
  
  /**
   * @brief Initialize detector with first sample
   *
   * Captures departure location and altitude, initializes buffers and state.
   * Called automatically on first update() call.
   */
  void initializeFirstSample(float gps_alt_msl, float vertical_speed, float groundspeed_kt,
                             float lat_deg, float lon_deg);
  
  /**
   * @brief Update smoothed values using exponential filter
   *
   * Applies exponential smoothing to vertical speed and groundspeed to reduce noise.
   * Filter time constant: 5 seconds (α = exp(-1/5) ≈ 0.8187)
   */
  void updateSmoothing(float vertical_speed, float groundspeed_kt);
  
  /**
   * @brief Update circular buffers and compute trends
   *
   * Adds new values to altitude and groundspeed buffers, then computes:
   * - Altitude trends (20s and 5s windows)
   * - Groundspeed acceleration and trend
   */
  void updateBuffers(float gps_alt_msl, float gs_smooth);
  
  /**
   * @brief Compute derived features from raw inputs
   *
   * Computes pseudo-AGL (if ground elevation available) and other derived
   * values used in transition logic.
   */
  void computeDerivedFeatures(float gps_alt_msl, float lat_deg, float lon_deg, float ground_elev_msl);
  
  /**
   * @brief Check and execute state transitions
   *
   * Main transition logic. Checks transitions in priority order:
   * 1. Emergency recovery
   * 2. Startup in-flight detection
   * 3. Recovery transitions
   * 4. Normal forward transitions
   *
   * Each transition has persistence timer to prevent oscillations.
   */
  void checkTransitions(float gs_smooth, float vs_smooth, float gps_alt_msl);
  
  /**
   * @brief Execute transition to new phase
   *
   * Updates current phase, resets all timers, captures takeoff data if needed,
   * and logs transition for debugging.
   *
   * @param new_phase Target phase (0-8)
   * @param reason Human-readable reason for transition (for logging)
   * @param gs_smooth Current smoothed groundspeed (for logging)
   * @param vs_smooth Current smoothed vertical speed (for logging)
   */
  void executeTransition(int new_phase, const char* reason, float gs_smooth, float vs_smooth);
  
  /**
   * @brief Increment all active timers
   *
   * Called each update cycle. Individual timers are incremented in
   * checkTransitions() when their conditions are met.
   */
  void incrementTimers();
  
  /**
   * @brief Reset all transition timers to zero
   *
   * Called after each transition to prevent carry-over of timer state.
   */
  void resetAllTimers();
  
  /**
   * @brief Update confidence and validity flags
   *
   * Computes confidence based on phase persistence, data quality, and phase
   * stability. Updates is_valid_ flag based on startup window.
   */
  void updateConfidence();
  
  /**
   * @brief Convert meters per second to knots
   * @param mps Speed in m/s
   * @return Speed in knots
   */
  float mpsToKnots(float mps) const { return mps * 1.94384f; }
  
  /**
   * @brief Check if value is NaN or infinite
   * @param value Value to check
   * @return true if NaN or infinite, false otherwise
   */
  bool isNaN(float value) const { return std::isnan(value) || !std::isfinite(value); }
};

#endif // FLIGHT_PHASE_DETECTOR_H
