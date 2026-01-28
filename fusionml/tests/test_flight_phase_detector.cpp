/*
 * Unit tests for FlightPhaseDetector using realistic scenario data.
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
    // Keep the aircraft stationary long enough to bypass startup detection logic.
    runForSeconds(31, 0.0f, 0.0f, 0.0f, false);
  }

  void rampGroundspeed(int seconds, float vs_mps, float start_kt, float end_kt, float accel_forward_mss) {
    float step = (end_kt - start_kt) / std::max(1, seconds - 1);
    for (int i = 0; i < seconds; i++) {
      float gs = start_kt + step * i;
      tick(vs_mps, gs, accel_forward_mss, true);
    }
  }
};

static void testFullFlightProfile() {
  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  // Initialization sample
  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  expectEqual(runner.detector.getFlightPhase(), PHASE_GROUND, "initial phase is GROUND");

  // Takeoff roll: accelerate on ground (>=2s with smoothing)
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF_ROLL, "GROUND -> TAKEOFF_ROLL");

  // Build speed while still on roll (below takeoff VS threshold)
  runner.runForSeconds(5, 0.5f, 90.0f, 0.5f);

  // Takeoff: positive rate of climb (>=2s with smoothing)
  runner.runForSeconds(3, 3.0f, 90.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF, "TAKEOFF_ROLL -> TAKEOFF");

  // Climb: AGL > 61m (2s)
  runner.runForSeconds(15, 4.0f, 85.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "TAKEOFF -> CLIMB");

  // Climb to cruise altitude (AGL > 152m)
  runner.runForSeconds(32, 5.0f, 95.0f, 0.0f);

  // Cruise: level flight with high groundspeed, stable altitude (>=10s)
  runner.runForSeconds(25, 0.0f, 110.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "CLIMB -> CRUISE");

  // Descent: steady descent (>=3s)
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_DESCENT, "CRUISE -> DESCENT");

  // Approach: decelerating and descending (>=5s)
  runner.rampGroundspeed(8, -1.5f, 80.0f, 70.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "DESCENT -> APPROACH");

  // Landing: low AGL, slower speed (>=3s)
  runner.runForSeconds(55, -3.0f, 60.0f, 0.0f);
  runner.runForSeconds(3, -0.2f, 60.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING, "APPROACH -> LANDING");

  // Landing roll: strong deceleration on ground (>=2s)
  runner.rampGroundspeed(6, -0.3f, 60.0f, 5.0f, 0.0f);
  runner.runForSeconds(5, -0.2f, 5.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING_ROLL, "LANDING -> LANDING_ROLL");

  // Ground: slow and stable (>=3s, keep stable for 20s)
  runner.runForSeconds(25, 0.0f, 0.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_GROUND, "LANDING_ROLL -> GROUND");
}

static void testGoAround() {
  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  // Reach APPROACH quickly: takeoff -> climb -> cruise -> descent -> approach
  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 90.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 90.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 85.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(20, 0.0f, 110.0f, 0.0f, false);
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  runner.rampGroundspeed(8, -1.5f, 80.0f, 70.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_APPROACH, "approach reached for go-around");

  // Go-around: climb with increasing groundspeed (>=5s)
  runner.rampGroundspeed(10, 3.5f, 65.0f, 95.0f, 0.0f);
  runner.runForSeconds(5, 3.5f, 100.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "APPROACH -> CLIMB (go-around)");
}

static void testTouchAndGo() {
  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  // Reach LANDING_ROLL
  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 90.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 90.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 85.0f, 0.5f);
  runner.runForSeconds(32, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(20, 0.0f, 110.0f, 0.0f, false);
  runner.runForSeconds(5, -2.5f, 90.0f, 0.0f);
  runner.rampGroundspeed(8, -1.5f, 80.0f, 70.0f, 0.0f);
  runner.runForSeconds(55, -3.0f, 60.0f, 0.0f);
  runner.runForSeconds(3, -0.2f, 60.0f, 0.0f);
  runner.runForSeconds(3, -0.2f, 60.0f, 0.0f);
  runner.rampGroundspeed(10, -0.3f, 60.0f, 5.0f, 0.0f);
  runner.runForSeconds(5, -0.2f, 5.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_LANDING_ROLL, "landing roll reached");

  // Touch-and-go: accelerate on roll (>=2s)
  runner.runForSeconds(2, 0.5f, 25.0f, 2.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_TAKEOFF_ROLL, "LANDING_ROLL -> TAKEOFF_ROLL");
}

static void testRecoveryTransitions() {
  ScenarioRunner runner;
  runner.reset(100.0f, 100.0f);

  // Reach CLIMB
  runner.tick(0.0f, 0.0f, 0.0f);
  runner.advanceStartupWindow();
  runner.runForSeconds(5, 0.2f, 25.0f, 2.0f);
  runner.runForSeconds(5, 0.5f, 90.0f, 0.5f);
  runner.runForSeconds(3, 3.0f, 90.0f, 0.5f);
  runner.runForSeconds(15, 4.0f, 85.0f, 0.5f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "climb reached");

  // Climb higher so AGL stays well above approach window during recovery tests
  runner.runForSeconds(80, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(6, 0.0f, 90.0f, 0.0f, false);

  // CLIMB -> DESCENT (direct)
  runner.runForSeconds(6, -6.0f, 85.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_DESCENT, "CLIMB -> DESCENT");

  // DESCENT -> CLIMB recovery
  runner.runForSeconds(16, 4.0f, 90.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "DESCENT -> CLIMB");

  // CLIMB -> CRUISE
  runner.runForSeconds(32, 5.0f, 95.0f, 0.0f);
  runner.runForSeconds(20, 0.0f, 110.0f, 0.0f, false);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CRUISE, "CLIMB -> CRUISE (recovery test)");

  // CRUISE -> CLIMB rapid climb
  runner.runForSeconds(12, 5.0f, 80.0f, 0.0f);
  expectEqual(runner.detector.getFlightPhase(), PHASE_CLIMB, "CRUISE -> CLIMB rapid climb");
}

int main() {
  std::printf("FlightPhaseDetector Unit Tests\n");

  testFullFlightProfile();
  testGoAround();
  testTouchAndGo();
  testRecoveryTransitions();

  std::printf("\nFlightPhaseDetector: %s\n", g_failed == 0 ? "PASS ✓" : "FAIL ✗");
  return g_failed == 0 ? 0 : 1;
}
