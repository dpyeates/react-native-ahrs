/**
 * FlightPhaseDetector.cpp
 *
 * Implementation of Flight Phase Detection Finite State Machine.
 *
 * This file contains the complete FSM implementation with detailed transition
 * logic. Transitions are checked in priority order to handle edge cases:
 *
 * Priority 1: Emergency Recovery
 *   - ANY → GROUND: Detects emergency landing or loss of power
 *
 * Priority 2: Startup In-Flight Detection (first 30 seconds)
 *   - Detects if app started mid-flight (cruise, climb, descent, approach)
 *   - Uses more aggressive thresholds since we don't have departure context
 *
 * Priority 3: Recovery Transitions
 *   - Go-arounds (APPROACH → CLIMB)
 *
 * Priority 4: Normal Forward Transitions
 *   - Standard flight progression (GROUND → TAKEOFF_ROLL → TAKEOFF → CLIMB → CRUISE → DESCENT → APPROACH → LANDING → LANDING_ROLL → GROUND)
 *
 * All transitions use persistence timers to prevent rapid oscillations from
 * noisy sensor data. See individual transition comments for detailed criteria.
 */

#include "FlightPhaseDetector.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

// Provide definitions for static constexpr members used in ODR contexts (C++14).
constexpr int FlightPhaseDetector::STARTUP_WINDOW;

FlightPhaseDetector::FlightPhaseDetector()
: current_phase_(PHASE_GROUND)  // Start in GROUND phase
, vs_smooth_(0.0f)              // Smoothed vertical speed (initialized to 0)
, gs_smooth_(0.0f)              // Smoothed groundspeed (initialized to 0)
, current_confidence_(0.0f)    // No confidence until data accumulates
, is_valid_(false)              // Invalid until startup window passes or transition occurs
, departure_lat_(0.0f)          // Will be set on first sample
, departure_lon_(0.0f)          // Will be set on first sample
, departure_field_alt_(0.0f)   // Will be set at TAKEOFF_ROLL transition
, takeoff_speed_kt_(55.0f)      // Default 55 knots - overwritten at TAKEOFF, but needed for approach detection if starting in cruise
, has_ground_ref_(false)        // Set to true if ground elevation provided
, startup_timer_(0)              // Seconds since first sample (0-30)
, last_update_time_us_(0)       // Timestamp of last processed update (for 1Hz throttling)
, first_sample_(true)            // True until first sample is processed
, delta_alt_20s_(0.0f)          // Altitude change over 20s window
, pseudo_agl_(std::numeric_limits<float>::quiet_NaN())  // AGL (computed if ground elevation available)
, gs_increasing_(false)         // Groundspeed trend (true if increasing)
, gs_acceleration_(0.0f)       // Groundspeed acceleration (kt/s)
, accel_forward_mss_(0.0f)       // Forward acceleration (m/s², from accelerometer)
, delta_alt_5s_(0.0f)           // Altitude change over 5s window (for faster takeoff detection)
, current_gps_alt_msl_(0.0f)    // Current GPS altitude (cached for transition checks)
, current_ground_elev_msl_(std::numeric_limits<float>::quiet_NaN())  // Current ground elevation (if available)
// Initialize all transition timers to zero
, timer_emergency_ground_(0)
, timer_startup_cruise_(0)
, timer_startup_climb_(0)
, timer_startup_approach_(0)
, timer_startup_descent_(0)
, timer_cruise_to_takeoff_(0)
, timer_approach_to_climb_(0)
, timer_ground_to_takeoff_roll_(0)
, timer_takeoff_roll_to_takeoff_(0)
, timer_takeoff_to_climb_(0)
, timer_climb_to_cruise_(0)
, timer_climb_to_descent_(0)
, timer_cruise_to_descent_(0)
, timer_descent_to_climb_(0)
, timer_descent_to_cruise_(0)
, timer_descent_to_approach_(0)
, timer_approach_to_cruise_(0)
, timer_approach_to_landing_(0)
, timer_landing_to_landing_roll_(0)
, timer_landing_roll_to_takeoff_roll_(0)
, timer_landing_roll_to_ground_(0)
{
  // Constructor complete - detector ready for first update() call
}

FlightPhaseDetector::~FlightPhaseDetector() {
  // No dynamic memory or resources to clean up
  // All data structures are stack-allocated
}

void FlightPhaseDetector::update(float gps_alt_msl, float vertical_speed, float groundspeed_ms,
                                 float lat_deg, float lon_deg, uint64_t timestamp_us,
                                 float ground_elev_msl, float accel_forward_mss) {
  // ===== Throttle to 1Hz =====
  // Only process updates if at least 1 second has passed since last update.
  // This prevents rapid updates from causing timer issues and ensures
  // consistent 1Hz processing rate.
  if (last_update_time_us_ > 0) {
    uint64_t dt_us = timestamp_us - last_update_time_us_;
    if (dt_us < 1000000ULL) { // Less than 1 second (1,000,000 microseconds)
      return; // Skip this update - too soon
    }
  }
  
  // Convert groundspeed from m/s to knots (internal calculations use knots)
  float groundspeed_kt = mpsToKnots(groundspeed_ms);
  
  // Check if ground elevation data is available (for AGL calculations)
  has_ground_ref_ = !isNaN(ground_elev_msl);
  
  // ===== Step 1: First Sample Initialization =====
  // On first sample, capture departure location and initialize state.
  // Don't process transitions on first sample (need history for trends).
  if (first_sample_) {
    initializeFirstSample(gps_alt_msl, vertical_speed, groundspeed_kt, lat_deg, lon_deg);
    first_sample_ = false;
    last_update_time_us_ = timestamp_us;
    return; // Exit early - no transitions on first sample
  }
  
  // ===== Step 2: Update Filtered Values =====
  // Apply exponential smoothing to reduce noise in vertical speed and groundspeed.
  // Filter time constant: 5 seconds (α = exp(-1/5) ≈ 0.8187)
  updateSmoothing(vertical_speed, groundspeed_kt);
  
  // ===== Step 3: Update Buffers =====
  // Add new values to circular buffers and compute trends:
  // - Altitude trends (20s and 5s windows)
  // - Groundspeed acceleration and trend
  updateBuffers(gps_alt_msl, gs_smooth_);
  
  // Cache current values for transition checks (used throughout checkTransitions)
  current_gps_alt_msl_ = gps_alt_msl;
  current_ground_elev_msl_ = ground_elev_msl;
  accel_forward_mss_ = isNaN(accel_forward_mss) ? 0.0f : accel_forward_mss; // Default to 0 if unavailable
  
  // ===== Step 4: Compute Derived Features =====
  // Compute pseudo-AGL and other derived values used in transition logic.
  computeDerivedFeatures(gps_alt_msl, lat_deg, lon_deg, ground_elev_msl);
  
  // ===== Step 5: Check Transitions (Priority Order) =====
  // Check transitions in priority order:
  // 1. Emergency recovery (highest priority)
  // 2. Startup in-flight detection
  // 3. Recovery transitions
  // 4. Normal forward transitions
  checkTransitions(gs_smooth_, vs_smooth_, gps_alt_msl);
  
  // ===== Step 6: Increment Timers =====
  // Update startup timer and transition timers (individual timers incremented in checkTransitions)
  incrementTimers();
  
  // ===== Step 7: Update Confidence =====
  // Compute confidence based on phase persistence, data quality, and phase stability.
  // Update is_valid_ flag based on startup window.
  updateConfidence();
  
  // Record timestamp for next throttling check
  last_update_time_us_ = timestamp_us;
}

void FlightPhaseDetector::initializeFirstSample(float gps_alt_msl, float vertical_speed,
                                                float groundspeed_kt, float lat_deg, float lon_deg) {
  // Initialize detector state with first sample
  // Assumes starting on ground (will be corrected by startup in-flight detection if needed)
  current_phase_ = PHASE_GROUND;
  
  // Initialize smoothed values with first sample (no smoothing yet)
  vs_smooth_ = vertical_speed;
  gs_smooth_ = groundspeed_kt;
  
  // Capture departure location (used for relative altitude calculations)
  departure_lat_ = lat_deg;
  departure_lon_ = lon_deg;
  departure_field_alt_ = gps_alt_msl; // Initial field altitude (may be updated at TAKEOFF_ROLL)
  
  // Reset timers and buffers
  startup_timer_ = 0;
  resetAllTimers();
  alt_history_.clear();
  gs_history_.clear();
  
  // Seed buffers with first sample
  alt_history_.push(gps_alt_msl);
  gs_history_.push(gs_smooth_);
  
  // Initialize confidence and validity (low confidence until data accumulates)
  current_confidence_ = 0.0f;
  is_valid_ = false;
}

void FlightPhaseDetector::updateSmoothing(float vertical_speed, float groundspeed_kt) {
  // Apply exponential smoothing to reduce noise
  // Filter: y[n] = α * y[n-1] + (1-α) * x[n]
  // Where α = exp(-1/τ) = exp(-1/5) ≈ 0.8187
  // Time constant τ = 5 seconds (LOWPASS_TAU)
  // This gives ~63% response in 5 seconds, ~95% in 15 seconds
  vs_smooth_ = LOWPASS_ALPHA * vs_smooth_ + (1.0f - LOWPASS_ALPHA) * vertical_speed;
  gs_smooth_ = LOWPASS_ALPHA * gs_smooth_ + (1.0f - LOWPASS_ALPHA) * groundspeed_kt;
}

void FlightPhaseDetector::updateBuffers(float gps_alt_msl, float gs_smooth) {
  // Add new values to circular buffers
  alt_history_.push(gps_alt_msl);  // 20-sample buffer (20 seconds at 1Hz)
  gs_history_.push(gs_smooth);     // 5-sample buffer (5 seconds at 1Hz)
  
  // ===== Compute Altitude Trends =====
  // Need at least BUFFER_INIT_THRESHOLD (10) samples before computing trends
  // to avoid false positives from initial noise
  if (alt_history_.size() < BUFFER_INIT_THRESHOLD) {
    delta_alt_20s_ = 0.0f; // Suppress until sufficient data
    delta_alt_5s_ = 0.0f;
  } else {
    // 20-second trend: current altitude - mean altitude over 20s window
    // Positive = climbing, negative = descending
    float mean_alt = alt_history_.mean();
    delta_alt_20s_ = gps_alt_msl - mean_alt;
    
    // 5-second trend: current altitude - altitude 5 seconds ago
    // Used for faster takeoff detection (TAKEOFF_ALT_GAIN_FAST threshold)
    // get(0) is oldest, get(size-1) is newest, so get(size-5) is 5 samples ago
    if (alt_history_.size() >= 5) {
      size_t idx_5s_ago = alt_history_.size() - 5;
      float alt_5s_ago = alt_history_.get(idx_5s_ago);
      delta_alt_5s_ = gps_alt_msl - alt_5s_ago;
    } else {
      delta_alt_5s_ = 0.0f;
    }
  }
  
  // ===== Compute Groundspeed Trend and Acceleration =====
  // Need at least 5 samples (5 seconds) for trend analysis
  if (gs_history_.size() >= GS_TREND_BUFFER_SIZE) {
    float gs_5s_ago = gs_history_.getOldest(); // Oldest value in buffer (5s ago)
    
    // Check if groundspeed is increasing (trend analysis)
    // Threshold: must increase by at least GS_TREND_THRESHOLD (5 kt) over 5s
    gs_increasing_ = (gs_smooth - gs_5s_ago) > GS_TREND_THRESHOLD;
    
    // Calculate groundspeed acceleration (kt/s)
    // Since updates are at 1Hz, change over 5 samples = 5 seconds
    // Acceleration = (current_speed - speed_5s_ago) / 5.0 seconds
    gs_acceleration_ = (gs_smooth - gs_5s_ago) / 5.0f; // kt/s
  } else {
    // Not enough data yet - assume no trend
    gs_increasing_ = false;
    gs_acceleration_ = 0.0f;
  }
}

void FlightPhaseDetector::computeDerivedFeatures(float gps_alt_msl, float lat_deg, float lon_deg, float ground_elev_msl) {
  // ===== Compute Pseudo AGL (Above Ground Level) =====
  // AGL = GPS altitude - ground elevation
  // Used for phase detection when ground elevation data is available
  // (e.g., from terrain database or user input)
  if (has_ground_ref_ && !isNaN(ground_elev_msl)) {
    pseudo_agl_ = gps_alt_msl - ground_elev_msl;
  } else {
    // No ground elevation available - AGL cannot be computed
    pseudo_agl_ = std::numeric_limits<float>::quiet_NaN();
  }
  
  // Note: Distance-based logic was removed in favor of altitude-based transitions
  // All transitions now use altitude trends, AGL, or other sensor-derived conditions
}

/**
 * Check and execute state transitions based on current telemetry data
 * 
 * Transition Priority Order:
 * 1. EMERGENCY RECOVERY TRANSITIONS (highest priority)
 * 2. STARTUP IN-FLIGHT DETECTION (first 30 seconds only)
 * 3. RECOVERY TRANSITIONS (unusual maneuvers, go-arounds)
 * 4. NORMAL FORWARD TRANSITIONS (standard flight phases)
 * 5. Stay in current phase (no transition)
 * 
 * @param gs_smooth Smoothed groundspeed in knots
 * @param vs_smooth Smoothed vertical speed in m/s (positive = climb)
 * @param gps_alt_msl Current GPS altitude MSL in meters
 */
void FlightPhaseDetector::checkTransitions(float gs_smooth, float vs_smooth, float gps_alt_msl) {
  // Priority order: Emergency → Startup → Recovery → Normal → Stay
  
  // ============================================================================
  // 1. EMERGENCY RECOVERY TRANSITIONS (Highest Priority - Check First)
  // ============================================================================
  
  /**
   * ANY → GROUND: Emergency Ground Detection
   * 
   * Detects emergency landing or loss of power from any phase except:
   * - GROUND (already on ground)
   * - TAKEOFF_ROLL (normal ground operation)
   * - TAKEOFF (normal takeoff)
   * - LANDING (normal landing)
   * - LANDING_ROLL (normal rollout)
   * 
   * Conditions (ALL required):
   * - Groundspeed < 15 kt
   * - Vertical speed < 0.3 m/s (stable)
   * - Altitude change (20s window) < 8 m (stable)
   * 
   * Persistence: 25 seconds
   * Reason: "emergency ground detected"
   */
  if (current_phase_ != PHASE_GROUND && current_phase_ != PHASE_TAKEOFF_ROLL && 
      current_phase_ != PHASE_TAKEOFF && current_phase_ != PHASE_LANDING && 
      current_phase_ != PHASE_LANDING_ROLL) {
    if (gs_smooth < EMERGENCY_GROUND_GS &&
        std::abs(vs_smooth) < GROUND_VS &&
        std::abs(delta_alt_20s_) < GROUND_ALT_STABLE) {
      timer_emergency_ground_++;
      if (timer_emergency_ground_ >= EMERGENCY_GROUND_TIMER) {
        executeTransition(PHASE_GROUND, "emergency ground detected", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_emergency_ground_ = 0;
    }
  } else {
    timer_emergency_ground_ = 0;
  }
  
  // ============================================================================
  // 2. STARTUP IN-FLIGHT DETECTION (First 30 Seconds Only)
  // ============================================================================
  // Detects if app starts during flight (not on ground)
  
  if (startup_timer_ < STARTUP_WINDOW && current_phase_ == PHASE_GROUND) {
    /**
     * GROUND → CRUISE: Startup Detected Cruise
     * 
     * App starts during cruise flight
     * 
     * Conditions (ANY option):
     * - Option A: Startup timer < 30 seconds AND groundspeed > 80 kt AND 
     *             vertical speed < 2.0 m/s (stable)
     * - Option B (immediate): Startup timer < 10 seconds AND groundspeed > 70 kt AND 
     *                        vertical speed < 1.5 m/s AND altitude > departure + 200m
     *                        (aggressive detection for reset-in-flight)
     * 
     * Persistence: 10 seconds
     * Reason: "startup detected cruise"
     */
    bool cruise_optionA = (gs_smooth > STARTUP_CRUISE_GS && std::abs(vs_smooth) < TAKEOFF_VS);
    bool cruise_optionB = (startup_timer_ < 10 && gs_smooth > CRUISE_MIN_GS && 
                           std::abs(vs_smooth) < CRUISE_VS && 
                           gps_alt_msl > departure_field_alt_ + CRUISE_MIN_RELATIVE_ALT);
    
    if (cruise_optionA || cruise_optionB) {
      timer_startup_cruise_++;
      if (timer_startup_cruise_ >= STARTUP_CRUISE_TIMER) {
        executeTransition(PHASE_CRUISE, "startup detected cruise", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_startup_cruise_ = 0;
    }
    
    /**
     * GROUND → CLIMB: Startup Detected Climb
     * 
     * App starts during climb phase
     * 
     * Conditions (ANY option):
     * - Option A (fast): Startup timer < 30 seconds AND vertical speed > 2.0 m/s AND 
     *                    altitude gain (5s window) > 5 m (faster detection after reset)
     * - Option B (slow): Startup timer < 30 seconds AND vertical speed > 2.0 m/s AND 
     *                    altitude gain (20s window) > 20 m (when buffer is full)
     * - Option C (immediate, no buffer): Startup timer < 10 seconds AND vertical speed > 3.0 m/s AND 
     *                                   groundspeed > 35 kt AND altitude > departure + 50m
     *                                   (aggressive detection for reset-in-flight, no buffer needed)
     * 
     * Persistence: 5 seconds
     * Reason: "startup detected climb"
     */
    bool climb_optionA = (vs_smooth > STARTUP_CLIMB_VS && delta_alt_5s_ > TAKEOFF_ALT_GAIN_FAST);
    bool climb_optionB = (vs_smooth > STARTUP_CLIMB_VS && delta_alt_20s_ > STARTUP_ALT_GAIN);
    bool climb_optionC = (startup_timer_ < 10 && vs_smooth > CLIMB_VS && gs_smooth > TAKEOFF_MIN_GS &&
                          gps_alt_msl > departure_field_alt_ + 50.0f);
    
    if (climb_optionA || climb_optionB || climb_optionC) {
      timer_startup_climb_++;
      if (timer_startup_climb_ >= STARTUP_CLIMB_TIMER) {
        executeTransition(PHASE_CLIMB, "startup detected climb", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_startup_climb_ = 0;
    }
    
    /**
     * GROUND → APPROACH: Startup Detected Approach
     * 
     * App starts during approach
     * 
     * Conditions (ALL required):
     * - Startup timer < 30 seconds
     * - Vertical speed < -2.0 m/s (descending)
     * - Ground reference available AND AGL < 1000 m (low altitude approach)
     * 
     * Persistence: 8 seconds
     * Reason: "startup detected approach"
     */
    if (vs_smooth < -DESCENT_VS && has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < APPROACH_AGL) {
      timer_startup_approach_++;
      if (timer_startup_approach_ >= APPROACH_TIMER) {
        executeTransition(PHASE_APPROACH, "startup detected approach", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_startup_approach_ = 0;
    }
    
    /**
     * GROUND → DESCENT: Startup Detected Descent
     * 
     * App starts during descent (no destination)
     * 
     * Conditions (ANY option):
     * - Option A (fast): Startup timer < 30 seconds AND vertical speed < -1.0 m/s (gentle descent) AND 
     *                    altitude loss (5s window) > 5 m (faster detection after reset)
     * - Option B (slow): Startup timer < 30 seconds AND vertical speed < -2.0 m/s AND 
     *                    altitude loss (20s window) > 20 m (when buffer is full)
     * - Option C (immediate, no buffer): Startup timer < 10 seconds AND vertical speed < -3.0 m/s AND 
     *                                    groundspeed > 35 kt AND altitude > departure + 50m
     *                                    (aggressive detection for reset-in-flight, no buffer needed)
     * 
     * Persistence: 10 seconds
     * Reason: "startup detected descent"
     */
    bool descent_optionA = (vs_smooth < -GENTLE_DESCENT_VS && delta_alt_5s_ < -TAKEOFF_ALT_GAIN_FAST);
    bool descent_optionB = (vs_smooth < -DESCENT_VS && delta_alt_20s_ < -LANDING_ALT_LOSS);
    bool descent_optionC = (startup_timer_ < 10 && vs_smooth < -RECOVERY_DESCENT_VS && gs_smooth > TAKEOFF_MIN_GS &&
                            gps_alt_msl > departure_field_alt_ + 50.0f);
    
    if (descent_optionA || descent_optionB || descent_optionC) {
      timer_startup_descent_++;
      if (timer_startup_descent_ >= DESCENT_TIMER) {
        executeTransition(PHASE_DESCENT, "startup detected descent", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_startup_descent_ = 0;
    }
  } else {
    timer_startup_cruise_ = 0;
    timer_startup_climb_ = 0;
    timer_startup_approach_ = 0;
    timer_startup_descent_ = 0;
  }
  
  // ============================================================================
  // 3. RECOVERY TRANSITIONS (Check After Startup)
  // ============================================================================
  // Handles unusual maneuvers and go-arounds
  
  /**
   * CLIMB → DESCENT: Rapid Descent from Climb
   * 
   * Aircraft descending rapidly during climb phase (unusual maneuver, e.g., emergency descent)
   * 
   * Conditions (ALL required):
   * - Groundspeed > 60 kt
   * - Vertical speed < -3.0 m/s (rapid descent)
   * 
   * Persistence: 5 seconds (RECOVERY_TIMER)
   * Reason: "rapid descent from climb"
   */
  if (current_phase_ == PHASE_CLIMB) {
    if (gs_smooth > RECOVERY_MIN_GS && vs_smooth < -RECOVERY_DESCENT_VS) {
      timer_climb_to_descent_++;
      if (timer_climb_to_descent_ >= RECOVERY_TIMER) {
        executeTransition(PHASE_DESCENT, "rapid descent from climb", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_climb_to_descent_ = 0;
    }
  } else {
    timer_climb_to_descent_ = 0;
  }
  
  /**
   * CRUISE → CLIMB: Rapid Climb from Cruise
   * 
   * Aircraft climbing rapidly from cruise (unusual maneuver, e.g., go-around or emergency climb)
   * 
   * Conditions (ALL required):
   * - Groundspeed > 60 kt
   * - Vertical speed > +4.0 m/s (rapid climb)
   * 
   * Persistence: 8 seconds
   * Reason: "rapid climb from cruise"
   */
  if (current_phase_ == PHASE_CRUISE) {
    if (gs_smooth > RECOVERY_MIN_GS && vs_smooth > RECOVERY_CLIMB_VS) {
      timer_cruise_to_takeoff_++;
      if (timer_cruise_to_takeoff_ >= RECOVERY_TIMER) {
        executeTransition(PHASE_CLIMB, "rapid climb from cruise", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_cruise_to_takeoff_ = 0;
    }
  } else {
    timer_cruise_to_takeoff_ = 0;
  }
  
  /**
   * APPROACH → CLIMB: Go-Around (Missed Approach)
   * 
   * Pilot executes missed approach/go-around
   * 
   * Conditions (ALL required):
   * - Vertical speed > +1.5 m/s (climbing)
   * - Groundspeed increasing (trend over 5 seconds)
   * 
   * Persistence: 5 seconds
   * Reason: "go-around initiated"
   */
  if (current_phase_ == PHASE_APPROACH) {
    if (vs_smooth > CRUISE_VS && gs_increasing_) {
      timer_approach_to_climb_++;
      if (timer_approach_to_climb_ >= GO_AROUND_TIMER) {
        executeTransition(PHASE_CLIMB, "go-around initiated", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_approach_to_climb_ = 0;
    }
  } else {
    timer_approach_to_climb_ = 0;
  }
  
  /**
   * APPROACH → CRUISE: Level Off from Approach
   * 
   * Aircraft levels off from approach back to cruise (e.g., missed approach that levels off,
   * or approach aborted and returning to cruise altitude)
   * 
   * Conditions (ALL required):
   * - Vertical speed between -0.5 and +0.5 m/s (level flight)
   * - Groundspeed >= 70 kt (cruise speed)
   * - Altitude condition (ANY):
   *   - Ground reference available AND AGL >= 300 m, OR
   *   - No ground reference AND GPS altitude >= departure_field_alt + 200 m
   * 
   * Persistence: 10 seconds
   * Reason: "level off to cruise"
   */
  if (current_phase_ == PHASE_APPROACH) {
    bool level_flight = (std::abs(vs_smooth) < STABLE_VS);
    bool cruise_speed = (gs_smooth >= CRUISE_MIN_GS);
    bool alt_condition = false;
    if (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ >= CRUISE_MIN_AGL) {
      alt_condition = true;
    } else if (!has_ground_ref_ && gps_alt_msl >= departure_field_alt_ + CRUISE_MIN_RELATIVE_ALT) {
      alt_condition = true;
    }
    
    if (level_flight && cruise_speed && alt_condition) {
      timer_approach_to_cruise_++;
      if (timer_approach_to_cruise_ >= CRUISE_TIMER) {
        executeTransition(PHASE_CRUISE, "level off to cruise", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_approach_to_cruise_ = 0;
    }
  } else {
    timer_approach_to_cruise_ = 0;
  }
  
  // ============================================================================
  // 4. NORMAL FORWARD TRANSITIONS
  // ============================================================================
  // Standard flight phase progression: GROUND → TAKEOFF_ROLL → TAKEOFF → CLIMB → 
  // CRUISE → DESCENT → APPROACH → LANDING → LANDING_ROLL → GROUND
  
  /**
   * GROUND → TAKEOFF_ROLL: Takeoff Roll Detection
   * 
   * Detects acceleration on ground before becoming airborne
   * Uses accelerometer data when available for faster, more accurate detection
   * 
   * Conditions (ALL required):
   * - Must be on ground (AGL < 30m OR altitude relative to departure < 50m)
   * - Groundspeed > 10 kt
   * - Forward acceleration detected:
   *   - Option A (preferred): Accelerometer forward acceleration > 1.5 m/s²
   *   - Option B (fallback): Groundspeed acceleration > 2.9 kt/s
   * 
   * Persistence: 2 seconds
   * Reason: "takeoff roll detected"
   */
  if (current_phase_ == PHASE_GROUND) {
    // Must be on ground - check AGL or relative altitude
    bool on_ground = false;
    if (!isNaN(pseudo_agl_)) {
      on_ground = (pseudo_agl_ < GROUND_AGL);
    } else {
      // Fallback: check altitude relative to departure
      float relative_alt = gps_alt_msl - departure_field_alt_;
      on_ground = (relative_alt < 50.0f);
    }
    
    if (!on_ground) {
      // Not on ground - reset timer and skip takeoff roll detection
      timer_ground_to_takeoff_roll_ = 0;
    } else {
      // Detect takeoff roll based on forward acceleration from accelerometer
      // Use accelerometer if available, otherwise fall back to groundspeed acceleration
      bool accelerating = false;
      if (!isNaN(accel_forward_mss_) && accel_forward_mss_ > 0.0f) {
        // Use accelerometer reading (forward acceleration in m/s²)
        accelerating = (gs_smooth > TAKEOFF_ROLL_MIN_GS && accel_forward_mss_ > TAKEOFF_ROLL_ACCEL_MSS);
      } else {
        // Fallback to groundspeed acceleration (kt/s)
        // Convert m/s² threshold to kt/s: 1.5 m/s² ≈ 2.9 kt/s
        const float TAKEOFF_ROLL_ACCEL_KT_S = TAKEOFF_ROLL_ACCEL_MSS * 1.94384f; // Convert m/s² to kt/s
        accelerating = (gs_smooth > TAKEOFF_ROLL_MIN_GS && gs_acceleration_ > TAKEOFF_ROLL_ACCEL_KT_S);
      }
      
      if (accelerating) {
        timer_ground_to_takeoff_roll_++;
        if (timer_ground_to_takeoff_roll_ >= TAKEOFF_ROLL_TIMER) {
          executeTransition(PHASE_TAKEOFF_ROLL, "takeoff roll detected", gs_smooth, vs_smooth);
          return;
        }
      } else {
        timer_ground_to_takeoff_roll_ = 0;
      }
    }
  } else {
    timer_ground_to_takeoff_roll_ = 0;
  }
  
  /**
   * TAKEOFF_ROLL → TAKEOFF: Takeoff Roll Complete (Airborne)
   * 
   * Aircraft becomes airborne after takeoff roll
   * Requires sustained positive rate of climb
   * 
   * Conditions:
   * - Vertical speed > 1.0 m/s (sustained positive rate of climb)
   * 
   * Persistence: 2 seconds
   * Reason: "takeoff roll complete"
   */
  if (current_phase_ == PHASE_TAKEOFF_ROLL) {
    // Detect sustained positive rate of climb (aircraft is airborne)
    // Require minimum 1.0 m/s to avoid false positives from small fluctuations
    if (vs_smooth > 1.0f) {
      timer_takeoff_roll_to_takeoff_++;
      if (timer_takeoff_roll_to_takeoff_ >= 2) { // 2 seconds persistence
        executeTransition(PHASE_TAKEOFF, "takeoff roll complete", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_takeoff_roll_to_takeoff_ = 0;
    }
  } else {
    timer_takeoff_roll_to_takeoff_ = 0;
  }
  
  /**
   * TAKEOFF → CLIMB: Climb Established
   * 
   * Aircraft establishes sustained climb after takeoff
   * 
   * Conditions (ANY option):
   * - Option A: Vertical speed > 3.0 m/s AND altitude gain (20s window) > 30 m
   * - Option B: Ground reference available AND AGL > 120 m
   * 
   * Persistence: 
   *   - Option A: 3 seconds
   *   - Option B: 2 seconds
   * Reason: "climb established"
   */
  if (current_phase_ == PHASE_TAKEOFF) {
    bool optionA = (vs_smooth > CLIMB_VS && delta_alt_20s_ > CLIMB_ALT_GAIN);
    bool optionB = (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ > CLIMB_MIN_AGL);
    
    if (optionA) {
      timer_takeoff_to_climb_++;
      if (timer_takeoff_to_climb_ >= CLIMB_TIMER) {
        executeTransition(PHASE_CLIMB, "climb established", gs_smooth, vs_smooth);
        return;
      }
    } else if (optionB) {
      timer_takeoff_to_climb_++;
      if (timer_takeoff_to_climb_ >= CLIMB_AGL_TIMER) {
        executeTransition(PHASE_CLIMB, "climb established", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_takeoff_to_climb_ = 0;
    }
  } else {
    timer_takeoff_to_climb_ = 0;
  }
  
  /**
   * CLIMB → CRUISE: Level Flight Established
   * 
   * Aircraft levels off at cruise altitude
   * 
   * Conditions (ALL required):
   * - Groundspeed > 70 kt
   * - Vertical speed < 1.5 m/s (stable)
   * - Altitude change (20s window) < 40 m (stable)
   * - Altitude condition (ANY):
   *   - No ground reference available, OR
   *   - AGL > 300 m, OR
   *   - GPS altitude > departure_field_alt + 200 m
   * 
   * Persistence: 15 seconds
   * Reason: "level flight established"
   */
  if (current_phase_ == PHASE_CLIMB) {
    bool alt_condition = false;
    if (!has_ground_ref_) {
      alt_condition = true; // No ground ref, use other conditions
    } else if (!isNaN(pseudo_agl_) && pseudo_agl_ > CRUISE_MIN_AGL) {
      alt_condition = true;
    } else if (gps_alt_msl > departure_field_alt_ + CRUISE_MIN_RELATIVE_ALT) {
      alt_condition = true;
    }
    
    // Allow small vertical speeds (gentle climbs/descents) to transition to cruise
    // This handles cases where aircraft is mostly level with small variations
    // CRUISE_VS (1.5 m/s) already covers gentle descents, so we just check for level flight
    bool level_flight = (std::abs(vs_smooth) < CRUISE_VS);
    
    if (gs_smooth > CRUISE_MIN_GS &&
        level_flight &&
        std::abs(delta_alt_20s_) < CRUISE_ALT_STABLE &&
        alt_condition) {
      timer_climb_to_cruise_++;
      if (timer_climb_to_cruise_ >= CRUISE_TIMER) {
        executeTransition(PHASE_CRUISE, "level flight established", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_climb_to_cruise_ = 0;
    }
  } else {
    timer_climb_to_cruise_ = 0;
  }
  
  /**
   * CLIMB → DESCENT: Direct Descent from Climb
   * 
   * Aircraft begins descent directly from climb phase (skipping cruise)
   * Common scenarios: pattern work, training maneuvers, traffic avoidance
   * 
   * Conditions (ALL required):
   * - Vertical speed < -1.0 m/s (gentle descent) OR < -2.0 m/s (steep descent)
   * - Altitude condition (ANY):
   *   - GPS altitude < departure_field_alt + 500 m, OR
   *   - Ground reference available AND AGL < 1000 m
   * 
   * Persistence: 10 seconds
   * Reason: "descent initiated from climb"
   */
  if (current_phase_ == PHASE_CLIMB) {
    bool alt_condition = false;
    if (gps_alt_msl < departure_field_alt_ + DESCENT_MAX_RELATIVE_ALT) {
      alt_condition = true;
    } else if (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < APPROACH_AGL) {
      alt_condition = true;
    }
    
    // Use gentle descent threshold to detect small descents (e.g., -200 fpm)
    bool gentle_descent = (vs_smooth < -GENTLE_DESCENT_VS && alt_condition);
    bool steep_descent = (vs_smooth < -DESCENT_VS);
    
    if ((gentle_descent || steep_descent) && alt_condition) {
      timer_climb_to_descent_++;
      if (timer_climb_to_descent_ >= DESCENT_TIMER) {
        executeTransition(PHASE_DESCENT, "descent initiated from climb", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_climb_to_descent_ = 0;
    }
  } else {
    timer_climb_to_descent_ = 0;
  }
  
  /**
   * CRUISE → DESCENT: Descent Initiated
   * 
   * Aircraft begins descent
   * 
   * Conditions (ALL required):
   * - Vertical speed < -1.0 m/s (gentle descent) OR < -2.0 m/s (steep descent)
   * - Altitude condition (ANY):
   *   - GPS altitude < departure_field_alt + 500 m, OR
   *   - Ground reference available AND AGL < 1000 m
   * 
   * Persistence: 10 seconds
   * Reason: "descent initiated"
   */
  if (current_phase_ == PHASE_CRUISE) {
    bool alt_condition = false;
    if (gps_alt_msl < departure_field_alt_ + DESCENT_MAX_RELATIVE_ALT) {
      alt_condition = true;
    } else if (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < APPROACH_AGL) {
      alt_condition = true;
    }
    
    // Use gentle descent threshold to detect small descents (e.g., -200 fpm)
    // Check for either gentle descent with altitude condition, or steeper descent
    bool gentle_descent = (vs_smooth < -GENTLE_DESCENT_VS && alt_condition);
    bool steep_descent = (vs_smooth < -DESCENT_VS);
    
    if ((gentle_descent || steep_descent) && alt_condition) {
      timer_cruise_to_descent_++;
      if (timer_cruise_to_descent_ >= DESCENT_TIMER) {
        executeTransition(PHASE_DESCENT, "descent initiated", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_cruise_to_descent_ = 0;
    }
  } else {
    timer_cruise_to_descent_ = 0;
  }
  
  /**
   * DESCENT → CLIMB: Climb from Descent
   * 
   * Aircraft begins climbing from descent phase (e.g., go-around, missed approach, traffic avoidance)
   * 
   * Conditions (ALL required):
   * - Vertical speed > 2.0 m/s (sustained climb)
   * - Groundspeed > 60 kt
   * 
   * Persistence: 3 seconds
   * Reason: "climb from descent"
   */
  if (current_phase_ == PHASE_DESCENT) {
    if (vs_smooth > CLIMB_VS && gs_smooth > RECOVERY_MIN_GS) {
      timer_descent_to_climb_++;
      if (timer_descent_to_climb_ >= CLIMB_TIMER) {
        executeTransition(PHASE_CLIMB, "climb from descent", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_descent_to_climb_ = 0;
    }
  } else {
    timer_descent_to_climb_ = 0;
  }
  
  /**
   * DESCENT → APPROACH: Approach Phase
   * 
   * Aircraft enters terminal area for approach
   * 
   * Only transitions from descent to approach when:
   * - Sustained descent (vertical speed < -1.0 m/s, gentle descent threshold)
   * - Deceleration from cruise (groundspeed acceleration < 0, indicating slowing down)
   * - Low altitude conditions met
   * 
   * Conditions (ALL required for Option A, Option B):
   * - Option A: Vertical speed < -1.0 m/s (gentle descent) AND groundspeed decelerating (acceleration < 0) AND
   *              ground reference available AND AGL < 1000 m
   * - Option B: Vertical speed < -1.0 m/s (gentle descent) AND groundspeed decelerating (acceleration < 0) AND
   *              no ground reference AND GPS altitude < departure_field_alt + 300 m
   * 
   * Persistence:
   *   - Option A: 5 seconds
   *   - Option B: 8 seconds
   * Reason: "approach phase"
   */
  if (current_phase_ == PHASE_DESCENT) {
    bool decelerating = (gs_acceleration_ < 0.0f); // Negative acceleration = deceleration
    
    // Use gentle descent threshold to detect small descents during approach
    bool descending = (vs_smooth < -GENTLE_DESCENT_VS); // Accept gentle descents
    
    // Check if groundspeed is within approach range relative to takeoff speed
    // Approach speed should be: takeoff_speed <= gs_smooth < takeoff_speed * 1.3
    bool approach_speed_range = (gs_smooth >= takeoff_speed_kt_ && gs_smooth < takeoff_speed_kt_ * APPROACH_SPEED_MULTIPLIER);
    
    bool optionA = (descending && decelerating && approach_speed_range && has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < APPROACH_AGL);
    bool optionB = (descending && decelerating && approach_speed_range && !has_ground_ref_ && gps_alt_msl < departure_field_alt_ + APPROACH_MAX_RELATIVE_ALT);
    
    if (optionA) {
      timer_descent_to_approach_++;
      if (timer_descent_to_approach_ >= APPROACH_TIMER_SHORT) {
        executeTransition(PHASE_APPROACH, "approach phase", gs_smooth, vs_smooth);
        return;
      }
    } else if (optionB) {
      timer_descent_to_approach_++;
      if (timer_descent_to_approach_ >= APPROACH_TIMER) {
        executeTransition(PHASE_APPROACH, "approach phase", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_descent_to_approach_ = 0;
    }
  } else {
    timer_descent_to_approach_ = 0;
  }
  
  /**
   * APPROACH → LANDING: Landing Flare
   * 
   * Aircraft is on short final or in flare
   * 
   * Conditions (ANY option):
   * - Option A: Groundspeed < 70 kt AND vertical speed > -1.0 m/s (reduced descent) AND 
   *              altitude loss (20s window) > 20 m
   * - Option B: Ground reference available AND AGL < 150 m AND groundspeed < 80 kt
   * 
   * Persistence:
   *   - Option A: 5 seconds
   *   - Option B: 3 seconds
   * Reason: "landing flare"
   */
  if (current_phase_ == PHASE_APPROACH) {
    bool optionA = (gs_smooth < APPROACH_LANDING_GS && vs_smooth > -STABLE_VS && delta_alt_20s_ < -LANDING_ALT_LOSS);
    bool optionB = (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < LANDING_AGL && gs_smooth < STARTUP_CRUISE_GS);
    
    if (optionA) {
      timer_approach_to_landing_++;
      if (timer_approach_to_landing_ >= LANDING_TIMER) {
        executeTransition(PHASE_LANDING, "landing flare", gs_smooth, vs_smooth);
        return;
      }
    } else if (optionB) {
      timer_approach_to_landing_++;
      if (timer_approach_to_landing_ >= CLIMB_AGL_TIMER) { // 3s per spec
        executeTransition(PHASE_LANDING, "landing flare", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_approach_to_landing_ = 0;
    }
  } else {
    timer_approach_to_landing_ = 0;
  }
  
  /**
   * LANDING → LANDING_ROLL: Landing Roll Detection
   * 
   * Detects deceleration on ground after landing
   * 
   * Conditions (ALL required):
   * - Groundspeed < 25 kt
   * - Deceleration detected: groundspeed acceleration < -2.0 kt/s
   * 
   * Persistence: 2 seconds
   * Reason: "landing roll detected"
   */
  if (current_phase_ == PHASE_LANDING) {
    // Detect landing roll based on deceleration while on ground
    bool decelerating = (gs_smooth < LANDING_GROUND_GS && gs_acceleration_ < LANDING_ROLL_DECEL);
    
    if (decelerating) {
      timer_landing_to_landing_roll_++;
      if (timer_landing_to_landing_roll_ >= LANDING_ROLL_TIMER) {
        executeTransition(PHASE_LANDING_ROLL, "landing roll detected", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_landing_to_landing_roll_ = 0;
    }
  } else {
    timer_landing_to_landing_roll_ = 0;
  }
  
  /**
   * LANDING_ROLL → TAKEOFF_ROLL: Touch-and-Go Detection
   * 
   * Aircraft accelerates for takeoff without fully stopping (touch-and-go)
   * 
   * Conditions (ALL required):
   * - Groundspeed > 10 kt
   * - Forward acceleration detected:
   *   - Option A (preferred): Accelerometer forward acceleration > 1.5 m/s²
   *   - Option B (fallback): Groundspeed acceleration > 2.9 kt/s
   * 
   * Persistence: 2 seconds
   * Reason: "touch-and-go detected"
   * 
   * Note: This transition is checked BEFORE LANDING_ROLL → GROUND to prioritize
   *       touch-and-go scenarios over full stop landings
   */
  if (current_phase_ == PHASE_LANDING_ROLL) {
    // Detect acceleration for touch-and-go (aircraft accelerates without stopping)
    bool accelerating = false;
    if (!isNaN(accel_forward_mss_) && accel_forward_mss_ > 0.0f) {
      // Use accelerometer reading (forward acceleration in m/s²)
      accelerating = (gs_smooth > TAKEOFF_ROLL_MIN_GS && accel_forward_mss_ > TAKEOFF_ROLL_ACCEL_MSS);
    } else {
      // Fallback to groundspeed acceleration (kt/s)
      const float TAKEOFF_ROLL_ACCEL_KT_S = TAKEOFF_ROLL_ACCEL_MSS * 1.94384f; // Convert m/s² to kt/s
      accelerating = (gs_smooth > TAKEOFF_ROLL_MIN_GS && gs_acceleration_ > TAKEOFF_ROLL_ACCEL_KT_S);
    }
    
    if (accelerating) {
      timer_landing_roll_to_takeoff_roll_++;
      if (timer_landing_roll_to_takeoff_roll_ >= TAKEOFF_ROLL_TIMER) {
        executeTransition(PHASE_TAKEOFF_ROLL, "touch-and-go detected", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_landing_roll_to_takeoff_roll_ = 0;
    }
  } else {
    timer_landing_roll_to_takeoff_roll_ = 0;
  }
  
  /**
   * LANDING_ROLL → GROUND: Rollout Complete
   * 
   * Aircraft has stopped or is moving very slowly after landing
   * 
   * Conditions (ANY option):
   * - Option A: Groundspeed < 5 kt AND vertical speed < 0.5 m/s (stable) AND 
   *              altitude change (20s window) < 8 m (stable)
   * - Option B: Groundspeed < 15 kt AND altitude change (20s window) < 15 m (stable)
   * - Option C: Ground reference available AND AGL < 30 m AND groundspeed < 10 kt
   * 
   * Persistence:
   *   - Option A: 3 seconds
   *   - Option B: 20 seconds
   *   - Option C: 10 seconds
   * Reason: "rollout complete"
   */
  if (current_phase_ == PHASE_LANDING_ROLL) {
    bool optionA = (gs_smooth < LANDING_ROLL_STOP_GS && std::abs(vs_smooth) < STABLE_VS && 
                    std::abs(delta_alt_20s_) < GROUND_ALT_STABLE);
    bool optionB = (gs_smooth < LANDING_GROUND_GS_SLOW && std::abs(delta_alt_20s_) < LANDING_ALT_STABLE_HIGH);
    bool optionC = (has_ground_ref_ && !isNaN(pseudo_agl_) && pseudo_agl_ < GROUND_AGL && 
                    gs_smooth < TAKEOFF_ROLL_MIN_GS);
    
    if (optionA) {
      timer_landing_roll_to_ground_++;
      if (timer_landing_roll_to_ground_ >= LANDING_ROLL_TO_GROUND_TIMER) {
        executeTransition(PHASE_GROUND, "rollout complete", gs_smooth, vs_smooth);
        return;
      }
    } else if (optionB) {
      timer_landing_roll_to_ground_++;
      if (timer_landing_roll_to_ground_ >= LANDING_GROUND_TIMER_LONG) {
        executeTransition(PHASE_GROUND, "rollout complete", gs_smooth, vs_smooth);
        return;
      }
    } else if (optionC) {
      timer_landing_roll_to_ground_++;
      if (timer_landing_roll_to_ground_ >= LANDING_GROUND_TIMER_SHORT) {
        executeTransition(PHASE_GROUND, "rollout complete", gs_smooth, vs_smooth);
        return;
      }
    } else {
      timer_landing_roll_to_ground_ = 0;
    }
  } else {
    timer_landing_roll_to_ground_ = 0;
  }
  
  // 5. Stay in current phase (no transition)
}

void FlightPhaseDetector::executeTransition(int new_phase, const char* reason,
                                            float gs_smooth, float vs_smooth) {
  // Execute transition to new phase
  int old_phase = current_phase_;
  current_phase_ = new_phase;
  
  // Reset all transition timers after transition
  // Prevents carry-over of timer state from previous phase
  resetAllTimers();
  
  // ===== Capture Flight Plan Data on Specific Transitions =====
  
  // Transition to TAKEOFF_ROLL (from GROUND)
  // Capture field altitude early during takeoff roll, before aircraft climbs significantly.
  // This ensures we capture the actual field altitude rather than a slightly elevated
  // altitude if the aircraft has already started climbing.
  if (new_phase == PHASE_TAKEOFF_ROLL && old_phase == PHASE_GROUND) {
    departure_field_alt_ = current_gps_alt_msl_;
  }
  
  // Transition to TAKEOFF (from TAKEOFF_ROLL)
  // Capture takeoff speed (groundspeed when aircraft becomes airborne).
  // Used later for approach detection (approach speed should be < takeoff_speed * 1.2).
  // Note: Field altitude already captured at TAKEOFF_ROLL transition.
  if (new_phase == PHASE_TAKEOFF) {
    takeoff_speed_kt_ = gs_smooth;
  }
  
  // ===== Log Transition for Debugging =====
  // Format: "Phase {old} → {new}: GS={gs:.1f}kt VS={vs:.1f}m/s Δalt={delta:.0f}m AGL={agl:.0f}m ({reason})"
  const char* phase_names[] = {
    "GROUND", "TAKEOFF_ROLL", "TAKEOFF", "CLIMB", "CRUISE",
    "DESCENT", "APPROACH", "LANDING", "LANDING_ROLL"
  };
  
  // Format AGL for display (handle NaN case)
  float agl_display = isNaN(pseudo_agl_) ? 0.0f : pseudo_agl_;
  
  printf("✈️ Flight Phase Transition: %s → %s: GS=%.1fkt VS=%.1f m/s Δalt=%.0fm AGL=%.0fm (%s)\n",
         phase_names[old_phase],
         phase_names[new_phase],
         gs_smooth,
         vs_smooth,
         delta_alt_20s_,
         agl_display,
         reason);
}

void FlightPhaseDetector::incrementTimers() {
  // Increment startup timer (capped at STARTUP_WINDOW = 30 seconds)
  // Used for startup in-flight detection and confidence calculation
  startup_timer_ = std::min(startup_timer_ + 1, STARTUP_WINDOW);
  
  // Note: Individual transition timers are incremented in checkTransitions()
  // when their specific conditions are met. They are reset to 0 when conditions
  // are not met or after a transition executes.
}

void FlightPhaseDetector::resetAllTimers() {
  // Reset all transition timers to zero
  // Called after each transition to prevent carry-over of timer state.
  // Individual timers are incremented in checkTransitions() when their
  // conditions are met, and reset here or when conditions are not met.
  timer_emergency_ground_ = 0;
  timer_startup_cruise_ = 0;
  timer_startup_climb_ = 0;
  timer_startup_approach_ = 0;
  timer_startup_descent_ = 0;
  timer_cruise_to_takeoff_ = 0;
  timer_approach_to_climb_ = 0;
  timer_approach_to_cruise_ = 0;
  timer_ground_to_takeoff_roll_ = 0;
  timer_takeoff_roll_to_takeoff_ = 0;
  timer_takeoff_to_climb_ = 0;
  timer_climb_to_cruise_ = 0;
  timer_climb_to_descent_ = 0;
  timer_cruise_to_descent_ = 0;
  timer_descent_to_climb_ = 0;
  timer_descent_to_cruise_ = 0;
  timer_descent_to_approach_ = 0;
  timer_approach_to_landing_ = 0;
  timer_landing_to_landing_roll_ = 0;
  timer_landing_roll_to_takeoff_roll_ = 0;
  timer_landing_roll_to_ground_ = 0;
}

void FlightPhaseDetector::updateConfidence() {
  // ===== Compute Confidence Score =====
  // Confidence is a product of three factors:
  // 1. Persistence factor: How long we've been running (0.0 to 1.0)
  //    - Increases from 0 to 1 over 30 seconds (startup window)
  //    - Higher confidence after startup window passes
  float persistence_factor = std::min(1.0f, startup_timer_ / 30.0f);
  
  // 2. Data quality: Assumed good if we're receiving updates
  //    - Could be enhanced to check GPS accuracy, sensor health, etc.
  float data_quality = 1.0f;
  
  // 3. Phase stability: Some phases are inherently more stable
  //    - CRUISE and GROUND are stable phases (confidence = 1.0)
  //    - Transient phases (TAKEOFF, LANDING, etc.) have lower confidence (0.8)
  float phase_stability = 1.0f;
  if (current_phase_ == PHASE_CRUISE || current_phase_ == PHASE_GROUND) {
    phase_stability = 1.0f; // Stable phases
  } else {
    phase_stability = 0.8f; // Transient phases
  }
  
  // Final confidence: product of all factors (0.0 to 1.0)
  current_confidence_ = persistence_factor * data_quality * phase_stability;
  
  // ===== Update Validity Flag =====
  // Phase detection is considered valid after:
  // - Startup window (30 seconds) has elapsed, OR
  // - Transition from GROUND phase has occurred (we've detected flight activity)
  // This prevents false phase reports during initial startup on ground
  is_valid_ = (startup_timer_ >= STARTUP_WINDOW || current_phase_ != PHASE_GROUND);
}

int FlightPhaseDetector::getFlightPhase() const {
  // Return current flight phase (0-8, see FlightPhase enum)
  return current_phase_;
}

float FlightPhaseDetector::getConfidence() const {
  // Return confidence in current phase detection (0.0-1.0)
  // Higher values indicate more reliable phase detection
  return current_confidence_;
}

bool FlightPhaseDetector::isValid() const {
  // Return true if phase detection is valid and should be used
  // False during startup window (first 30 seconds) if still on ground
  return is_valid_;
}

void FlightPhaseDetector::reset() {
  // Reset all state to initial values
  // Call this when starting a new flight or after long periods of inactivity
  
  // Reset FSM state
  current_phase_ = PHASE_GROUND;
  vs_smooth_ = 0.0f;
  gs_smooth_ = 0.0f;
  current_confidence_ = 0.0f;
  is_valid_ = false;
  
  // Reset flight plan data
  departure_lat_ = 0.0f;
  departure_lon_ = 0.0f;
  departure_field_alt_ = 0.0f;
  takeoff_speed_kt_ = 55.0f; // Reset to default 55 knots
  
  // Reset system state
  has_ground_ref_ = false;
  startup_timer_ = 0;
  last_update_time_us_ = 0;
  first_sample_ = true; // Next update() will trigger initialization
  
  // Reset derived values
  delta_alt_20s_ = 0.0f;
  delta_alt_5s_ = 0.0f;
  pseudo_agl_ = std::numeric_limits<float>::quiet_NaN();
  gs_increasing_ = false;
  gs_acceleration_ = 0.0f;
  accel_forward_mss_ = 0.0f;
  current_gps_alt_msl_ = 0.0f;
  current_ground_elev_msl_ = std::numeric_limits<float>::quiet_NaN();
  
  // Clear circular buffers
  alt_history_.clear();
  gs_history_.clear();
  
  // Reset all transition timers
  resetAllTimers();
}
