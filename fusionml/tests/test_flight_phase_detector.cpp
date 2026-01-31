/*
 * Unit tests for FlightPhaseDetector using realistic scenario data.
 *
 * Scenarios use typical GA (general aviation) numbers:
 *   - Field elevation ~100 m MSL
 *   - Takeoff roll: 15–25 kt accelerating, forward accel ~1.5–2 m/s²
 *   - Takeoff: 55–65 kt, climb 3–4 m/s
 *   - Cruise: 95–110 kt, level
 *   - Descent: 85–95 kt, -2 to -3 m/s
 *   - Approach: speed between takeoff_speed and takeoff_speed*1.2, decelerating
 *   - Landing: 55–65 kt then decel to <25 kt (roll), then <5 kt (ground)
 *
 * All phases (0–8) and key transitions are tested. Update rate is 1 Hz (detector throttle).
 */

#include <cstdio>
#include <cstdint>
#include <cmath>

#include "FlightPhaseDetector.h"

static int g_failed = 0;

static void expectEqual(int actual, int expected, const char *message) {
  if (actual != expected) {
    g_failed++;
    std::printf("✗ %s (actual=%d expected=%d)\n", message, actual, expected);
  } else {
    std::printf("✓ %s\n", message);
  }
}

static void expectTrue(bool condition, const char *message) {
  if (!condition) {
    g_failed++;
    std::printf("✗ %s\n", message);
  } else {
    std::printf("✓ %s\n", message);
  }
}

static float knotsToMps(float kt) {
  return kt / 1.94384f;
}

struct ScenarioRunner {
  FlightPhaseDetector detector;
  uint64_t time_us = 0;
  float alt_msl = 100.0f;
  float ground_elev = 100.0f;

  void reset(float alt_msl_init, float ground_elev_init) {
    detector.reset();
    time_us = 0;
    alt_msl = alt_msl_init;
    ground_elev = ground_elev_init;
  }

  void tick(float vs_mps, float gs_kt, float accel_forward_mss, bool update_alt = true) {
    if (update_alt) {
      alt_msl += vs_mps;
    }
    detector.update(
      alt_msl,
      vs_mps,
      knotsToMps(gs_kt),
      51.0f,
      -0.1f,
      time_us,
      ground_elev,
      accel_forward_mss
    );
    time_us += 1000000ULL;
  }

  void runForSeconds(int seconds, float vs_mps, float gs_kt, float accel_forward_mss, bool update_alt = true) {
    for (int i = 0; i < seconds; i++) {
      tick(vs_mps, gs_kt, accel_forward_mss, update_alt);
    }
  }

  void advanceStartupWindow() {
    runForSeconds(31, 0.0f, 0.0f, 0.0f, false);
  }

  void rampGroundspeed(int seconds, float vs_mps, float start_kt, float end_kt, float accel_forward_mss) {
    float step = (end_kt - start_kt) / (float)std::max(1, seconds - 1);
    for (int i = 0; i < seconds; i++) {
      float gs = start_kt + step * (float)i;
      tick(vs_mps, gs, accel_forward_mss, true);
    }
  }
};

static const char* phaseName(int phase) {
  switch (phase) {
    case PHASE_GROUND: return "GROUND";
    case PHASE_TAKEOFF_ROLL: return "TAKEOFF_ROLL";
    case PHASE_TAKEOFF: return "TAKEOFF";
    case PHASE_CLIMB: return "CLIMB";
    case PHASE_CRUISE: return "CRUISE";
    case PHASE_DESCENT: return "DESCENT";
    case PHASE_APPROACH: return "APPROACH";
    case PHASE_LANDING: return "LANDING";
    case PHASE_LANDING_ROLL: return "LANDING_ROLL";
    default: return "?";
  }
}

static void testFullFlightProfile() {
  std::printf("\n=== Test: Full Flight Profile (all phases and transitions) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  expectEqual(runner.detector.getFlightPhase(), PHASE_GROUND, "after startup: GROUND (0)");

  // --- GROUND -> TAKEOFF_ROLL: accelerate on ground, forward accel > 1.5 m/s², 2s ---
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF_ROLL, "GROUND -> TAKEOFF_ROLL (1)");

  // Build speed on roll (still below sustained climb)
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);

  // Hold 60 kt with low vs so gs_smooth = 60 when we transition (takeoff_speed_kt_ sets approach range [60,72))
  runner.runForSeconds(8, 0.5f, 60.0f, 0.5f);
  // --- TAKEOFF_ROLL -> TAKEOFF: vs > 1.0 m/s sustained, 2s ---
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF, "TAKEOFF_ROLL -> TAKEOFF (2)");

  // --- TAKEOFF -> CLIMB: AGL > 61 m (2s) or vs > 3 m/s and alt gain (3s) ---
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "TAKEOFF -> CLIMB (3)");

  // Climb to cruise altitude (AGL > 152 m), then level off
  runner.runForSeconds(32, 5.0f, 85.0f, 0.0f);

  // --- CLIMB -> CRUISE: level, gs > 70, |vs|<1.5, stable alt, 10s (vs_smooth decay needs ~6s then 10s timer) ---
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "CLIMB -> CRUISE (4)");

  // --- CRUISE -> DESCENT: vs < -1.0 or -2.0, 3s ---
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_DESCENT, "CRUISE -> DESCENT (5)");

  // --- DESCENT -> APPROACH: decelerating, approach_speed_range (takeoff_speed <= gs < takeoff_speed*1.2), AGL < 350, 3s ---
  // takeoff_speed_kt_ = 60, so 60 <= gs_smooth < 72. Smoothing lags: hold approach speed long enough for gs_smooth to settle.
  runner.rampGroundspeed(12, -1.5f, 75.0f, 62.0f, 0.0f);
  runner.runForSeconds(12, -1.5f, 62.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "DESCENT -> APPROACH (6)");

  // Descend to short final (AGL < 61 for LANDING). Option B: AGL < 61, gs < 80, 3s
  runner.runForSeconds(55, -3.0f, 60.0f, 0.0f);
  runner.runForSeconds(3, -0.2f, 60.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING, "APPROACH -> LANDING (7)");

  // --- LANDING -> LANDING_ROLL: gs < 25 kt, decel < -2 kt/s, 2s ---
  runner.rampGroundspeed(6, -0.3f, 60.0f, 5.0f, 0.0f);
  runner.runForSeconds(5, -0.2f, 5.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING_ROLL, "LANDING -> LANDING_ROLL (8)");

  // --- LANDING_ROLL -> GROUND: gs < 5, stable vs/alt, 3s ---
  runner.runForSeconds(25, 0.0f, 0.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_GROUND, "LANDING_ROLL -> GROUND (0)");

  expectTrue(runner.detector.isValid(), "detector valid after full profile");
  std::printf("  Confidence: %.2f\n", runner.detector.getConfidence());
}

static void testGoAround() {
  std::printf("\n=== Test: Go-Around (APPROACH -> CLIMB) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(8, 0.5f, 60.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 85.0f, 0.0f);
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  runner.rampGroundspeed(12, -1.5f, 75.0f, 62.0f, 0.0f);
  runner.runForSeconds(12, -1.5f, 62.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "approach reached for go-around");

  runner.rampGroundspeed(10, 3.5f, 65.0f, 95.0f, 0.0f);
  runner.runForSeconds(5, 3.5f, 100.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "APPROACH -> CLIMB (go-around)");
}

static void testTouchAndGo() {
  std::printf("\n=== Test: Touch-and-Go (LANDING_ROLL -> TAKEOFF_ROLL) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(8, 0.5f, 60.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 85.0f, 0.0f);
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  runner.rampGroundspeed(12, -1.5f, 75.0f, 62.0f, 0.0f);
  runner.runForSeconds(12, -1.5f, 62.0f, 0.0f);
  runner.runForSeconds(55, -3.0f, 60.0f, 0.0f);
  runner.runForSeconds(3, -0.2f, 60.0f, 0.0f);
  runner.rampGroundspeed(6, -0.3f, 60.0f, 5.0f, 0.0f);
  runner.runForSeconds(5, -0.2f, 5.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING_ROLL, "landing roll reached");

  runner.runForSeconds(2, 0.5f, 25.0f, 2.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF_ROLL, "LANDING_ROLL -> TAKEOFF_ROLL (touch-and-go)");
}

static void testRecoveryTransitions() {
  std::printf("\n=== Test: Recovery Transitions ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "climb reached");

  runner.runForSeconds(80, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(6, 0.0f, 90.0f, 0.0f, false);

  runner.runForSeconds(6, -6.0f, 85.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_DESCENT, "CLIMB -> DESCENT (direct)");

  runner.runForSeconds(16, 4.0f, 90.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "DESCENT -> CLIMB (recovery)");

  runner.runForSeconds(32, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(20, 0.0f, 110.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "CLIMB -> CRUISE");

  runner.runForSeconds(12, 5.0f, 80.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "CRUISE -> CLIMB (rapid climb)");
}

static void testApproachToCruiseMissedApproach() {
  std::printf("\n=== Test: Missed Approach (APPROACH -> CRUISE) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(8, 0.5f, 60.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 85.0f, 0.0f);
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  runner.runForSeconds(3, -2.5f, 90.0f, 0.0f);
  runner.rampGroundspeed(10, -1.5f, 75.0f, 65.0f, 0.0f);
  runner.runForSeconds(10, -1.5f, 65.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "approach reached");

  // Level off at approach alt; need AGL >= 152 for APPROACH->CRUISE. Cruise higher (270 m) so after descent we stay >= 252.
  runner.reset(100.0f, 100.0f);
  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(8, 0.5f, 60.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  runner.runForSeconds(36, 5.0f, 85.0f, 0.0f);
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  runner.runForSeconds(5, -2.0f, 72.0f, 0.0f);
  runner.rampGroundspeed(8, -1.0f, 72.0f, 65.0f, 0.0f);
  runner.runForSeconds(10, -1.0f, 65.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "approach at sufficient AGL (>=252m)");
  runner.runForSeconds(20, 0.0f, 95.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "APPROACH -> CRUISE (missed approach)");
}

static void testEmergencyRecovery() {
  std::printf("\n=== Test: Emergency Recovery (ANY -> GROUND) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 55.0f, 0.5f);
  runner.runForSeconds(10, 3.0f, 60.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 75.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 85.0f, 0.0f);
  runner.runForSeconds(25, 0.0f, 100.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "cruise reached");

  // Simulate emergency: gs<15, |vs|<0.3, |delta_alt_20s|<8 for 25s. gs_smooth decays from 100 so need ~10s + 25s.
  runner.runForSeconds(40, 0.0f, 10.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_GROUND, "CRUISE -> GROUND (emergency recovery)");
}

static void testStartupInFlightDetection() {
  std::printf("\n=== Test: Startup In-Flight Detection (first 30s) ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  // Do NOT advance startup window. Simulate app started while climbing (vs>2, delta_alt_5s>5 for 5s).
  runner.runForSeconds(15, 4.0f, 85.0f, 0.0f);
  int phase = runner.detector.getFlightPhase();
  expectTrue(phase == PHASE_CLIMB || phase == PHASE_CRUISE,
             "within 30s: detect CLIMB or CRUISE when in-flight (startup in-flight)");
  std::printf("  Phase after 15s climb: %s\n", phaseName(phase));

  runner.reset(100.0f, 100.0f);
  // First sample at cruise speed so we don't accelerate into TAKEOFF_ROLL
  runner.tick(0.0f, 95.0f, 0.0f, false);
  runner.runForSeconds(8, 0.0f, 95.0f, 0.0f, false);
  phase = runner.detector.getFlightPhase();
  expectTrue(phase == PHASE_CRUISE || phase == PHASE_GROUND,
             "within 30s: detect CRUISE or GROUND when level at cruise speed");
  std::printf("  Phase after 8s level cruise: %s\n", phaseName(phase));
}

static void testConfidenceAndValidity() {
  std::printf("\n=== Test: Confidence and Validity ===\n");

  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  runner.tick(0.0f, 0.0f, 0.0f);
  expectTrue(!runner.detector.isValid(), "invalid during first second (startup)");

  runner.advanceStartupWindow();
  expectTrue(runner.detector.isValid(), "valid after 31s on ground (startup passed)");

  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(3, 3.0f, 60.0f, 0.5f);
  float conf = runner.detector.getConfidence();
  expectTrue(conf >= 0.0f && conf <= 1.0f, "confidence in [0,1]");
  std::printf("  Confidence after takeoff: %.2f\n", conf);
}

int main() {
  std::printf("FlightPhaseDetector Unit Tests\n");
  std::printf("(Realistic GA scenarios, 1 Hz updates)\n");

  testFullFlightProfile();
  testGoAround();
  testTouchAndGo();
  testRecoveryTransitions();
  testApproachToCruiseMissedApproach();
  testEmergencyRecovery();
  testStartupInFlightDetection();
  testConfidenceAndValidity();

  std::printf("\nFlightPhaseDetector: %s\n", g_failed == 0 ? "PASS ✓" : "FAIL ✗");
  return g_failed == 0 ? 0 : 1;
}
