/*
 * uNavINS tests for the iOS GPS/INS plant.
 *
 * IMU at 60 Hz (Core Motion). GPS at 1 Hz on the ground and 5 Hz in the air
 * (Core Location). First IMU sample uses dt = 0, matching NativeAhrs.
 *
 * Suites:
 *   Init/safety — first sample, bad accel, missing mag, independent instances
 *   Static      — desk rest, handheld ZUPT, pickup
 *   Walking     — pedestrian speed, corner, mag interference
 *   Car         — cruise, level yaw turn, stoplight, urban GPS
 *   Aircraft    — level, coordinated turn, 360° horizon, climb/baro, GPS outage
 *   Outputs     — vertical/horizontal FPA, ground track, NED vel, lat/lon
 */

#include <cmath>
#include <cstdio>
#include <cstdint>

#include "uNavINS.h"

static int g_failed = 0;
static int g_passed = 0;

static void expectTrue(bool condition, const char *message) {
  if (!condition) {
    g_failed++;
    std::printf("✗ %s\n", message);
  } else {
    g_passed++;
    std::printf("✓ %s\n", message);
  }
}

static void expectNear(float actual, float expected, float tolerance, const char *message) {
  float diff = std::fabs(actual - expected);
  if (diff > tolerance) {
    g_failed++;
    std::printf("✗ %s (actual=%.4f expected=%.4f diff=%.4f tol=%.4f)\n",
                message, actual, expected, diff, tolerance);
  } else {
    g_passed++;
    std::printf("✓ %s (actual=%.4f)\n", message, actual);
  }
}

static void expectNearDeg(float actual_rad, float expected_deg, float tolerance_deg,
                          const char *message) {
  float actual_deg = actual_rad * 180.0f / (float)M_PI;
  float diff = std::fabs(actual_deg - expected_deg);
  if (diff > 180.0f) diff = 360.0f - diff;
  if (diff > tolerance_deg) {
    g_failed++;
    std::printf("✗ %s (actual=%.2f° expected=%.2f° diff=%.2f°)\n",
                message, actual_deg, expected_deg, diff);
  } else {
    g_passed++;
    std::printf("✓ %s (actual=%.2f°)\n", message, actual_deg);
  }
}

static const float G = 9.807f;
static const float DEG_TO_RAD = (float)M_PI / 180.0f;
static const float RAD_TO_DEG = 180.0f / (float)M_PI;
static const float IMU_DT = 1.0f / 60.0f;
static const int IMU_HZ = 60;
static const double EARTH_R = 6378137.0;
// NED field matching World::bn/be/bd (nT / 1000). Used for body-frame mag.
static const float MAG_N_UT = 19.0f;
static const float MAG_E_UT = -1.0f;
static const float MAG_D_UT = 45.0f;

// Rotate NED mag into body (X fwd, Y right, Z down). C_N2B = Rx(roll)*Ry(pitch)*Rz(yaw)
// with Ry chosen so a level pitched-up IMU reads ax = G*sin(pitch).
static void nedMagToBody(float heading_deg, float pitch_deg, float roll_deg,
                         float *mx, float *my, float *mz) {
  float yaw = heading_deg * DEG_TO_RAD;
  float th = pitch_deg * DEG_TO_RAD;
  float ph = roll_deg * DEG_TO_RAD;
  float cy = std::cos(yaw), sy = std::sin(yaw);
  float ct = std::cos(th), st = std::sin(th);
  float cp = std::cos(ph), sp = std::sin(ph);
  float n = MAG_N_UT, e = MAG_E_UT, d = MAG_D_UT;
  float xn = cy * n + sy * e;
  float yn = -sy * n + cy * e;
  float zn = d;
  float xp = ct * xn - st * zn;
  float yp = yn;
  float zp = st * xn + ct * zn;
  *mx = xp;
  *my = cp * yp + sp * zp;
  *mz = -sp * yp + cp * zp;
}

struct SimulatedSensors {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

static float wrap360(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f) deg += 360.0f;
  return deg;
}

// Level, z-down specific force. Mag is the World NED field rotated into body.
static SimulatedSensors levelSensors(float heading_deg, float yaw_rate_rads = 0.0f,
                                     float ax = 0.0f, float ay = 0.0f) {
  SimulatedSensors s;
  s.ax = ax;
  s.ay = ay;
  s.az = -G;
  s.gx = 0.0f;
  s.gy = 0.0f;
  s.gz = yaw_rate_rads;
  nedMagToBody(heading_deg, 0.0f, 0.0f, &s.mx, &s.my, &s.mz);
  return s;
}

static SimulatedSensors coordinatedTurnSensors(float heading_deg, float bank_deg,
                                               float turn_rate_deg_s) {
  SimulatedSensors s;
  float bank_rad = bank_deg * DEG_TO_RAD;
  float turn_rate_rad = turn_rate_deg_s * DEG_TO_RAD;
  float load_factor = 1.0f / std::cos(bank_rad);
  s.ax = 0.0f;
  s.ay = 0.0f;
  s.az = -load_factor * G;
  s.gx = 0.0f;
  s.gy = turn_rate_rad * std::sin(bank_rad);
  s.gz = turn_rate_rad * std::cos(bank_rad);
  nedMagToBody(heading_deg, 0.0f, bank_deg, &s.mx, &s.my, &s.mz);
  return s;
}

// Shared IMU/GPS plant. GPS lat/lon/vel freeze between fixes, as on iOS.
struct World {
  uNavINS filter;
  unsigned long tow_ms = 0;
  double lat_rad = 51.5 * DEG_TO_RAD;
  double lon_rad = -0.1 * DEG_TO_RAD;
  double alt_m = 50.0;
  double vn = 0.0;
  double ve = 0.0;
  double vd = 0.0;
  float heading_deg = 0.0f;
  float hacc = 5.0f;
  float vacc = 8.0f;
  float sacc = 0.5f;
  float baro_hpa = -1.0f;
  float qnh = 1013.25f;
  float bn = 19000.0f;
  float be = -1000.0f;
  float bd = 45000.0f;
  int gps_every = IMU_HZ;  // 1 Hz
  int k = 0;
  bool begun = false;
  double gps_vn = 0.0, gps_ve = 0.0, gps_vd = 0.0;
  double gps_lat = 51.5 * DEG_TO_RAD;
  double gps_lon = -0.1 * DEG_TO_RAD;
  double gps_alt = 50.0;

  void set_gps_hz(int hz) {
    gps_every = IMU_HZ / hz;
    if (gps_every < 1) gps_every = 1;
  }

  void set_course(float speed_ms, float hdg_deg) {
    heading_deg = wrap360(hdg_deg);
    float h = heading_deg * DEG_TO_RAD;
    vn = speed_ms * std::cos(h);
    ve = speed_ms * std::sin(h);
  }

  // Velocity along a ground track that may differ from heading (crab / sideslip).
  void set_velocity_track(float speed_ms, float track_deg) {
    float t = wrap360(track_deg) * DEG_TO_RAD;
    vn = speed_ms * std::cos(t);
    ve = speed_ms * std::sin(t);
  }

  void begin() {
    gps_vn = vn;
    gps_ve = ve;
    gps_vd = vd;
    gps_lat = lat_rad;
    gps_lon = lon_rad;
    gps_alt = alt_m;
    SimulatedSensors s = levelSensors(heading_deg);
    // iOS first IMU sample has dt = 0
    filter.update(0.0, tow_ms, gps_vn, gps_ve, gps_vd, gps_lat, gps_lon, gps_alt,
                  s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.mx, s.my, s.mz,
                  bn, be, bd, hacc, vacc, sacc, baro_hpa, qnh);
    begun = true;
  }

  void step(const SimulatedSensors &s) {
    if (!begun) begin();
    lat_rad += vn * (double)IMU_DT / EARTH_R;
    lon_rad += ve * (double)IMU_DT / (EARTH_R * std::cos(lat_rad));
    alt_m -= vd * (double)IMU_DT;
    if (k % gps_every == 0) {
      tow_ms += (unsigned long)((double)gps_every * IMU_DT * 1000.0 + 0.5);
      gps_vn = vn;
      gps_ve = ve;
      gps_vd = vd;
      gps_lat = lat_rad;
      gps_lon = lon_rad;
      gps_alt = alt_m;
    }
    k++;
    filter.update((double)IMU_DT, tow_ms, gps_vn, gps_ve, gps_vd, gps_lat, gps_lon,
                  gps_alt, s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.mx, s.my, s.mz,
                  bn, be, bd, hacc, vacc, sacc, baro_hpa, qnh);
  }

  void run_level(int n, float ax = 0.0f, float ay = 0.0f, float yaw_rate = 0.0f) {
    for (int i = 0; i < n; i++) {
      step(levelSensors(heading_deg, yaw_rate, ax, ay));
    }
  }

  float speed() {
    return std::sqrt((float)(filter.getVelNorth_ms() * filter.getVelNorth_ms() +
                             filter.getVelEast_ms() * filter.getVelEast_ms()));
  }

  float speed_3d() {
    double vn_i = filter.getVelNorth_ms();
    double ve_i = filter.getVelEast_ms();
    double vd_i = filter.getVelDown_ms();
    return std::sqrt((float)(vn_i * vn_i + ve_i * ve_i + vd_i * vd_i));
  }

  float fpa_deg() { return filter.getFlightPathAngle_rad() * RAD_TO_DEG; }
  float hfpa_deg() { return filter.getHorizontalFlightPathAngle_rad() * RAD_TO_DEG; }
  float track_deg() { return filter.getGroundTrack_rad() * RAD_TO_DEG; }
};

static SimulatedSensors walkSensors(float heading_deg, int i, float yaw_rate = 0.0f) {
  SimulatedSensors s = levelSensors(heading_deg, yaw_rate);
  float step_phase = (float)i * 2.0f * (float)M_PI * 1.8f / (float)IMU_HZ;
  s.az += 1.2f * std::sin(step_phase);
  s.ax += 0.35f * std::sin(step_phase);
  s.gx += 0.03f * std::sin(step_phase * 0.5f);
  s.gy += 0.02f * std::cos(step_phase * 0.4f);
  return s;
}

static SimulatedSensors pitchedSensors(float heading_deg, float pitch_deg,
                                       float pitch_rate = 0.0f, float yaw_rate = 0.0f) {
  SimulatedSensors s = levelSensors(heading_deg, yaw_rate);
  float th = pitch_deg * DEG_TO_RAD;
  s.ax = G * std::sin(th);
  s.ay = 0.0f;
  s.az = -G * std::cos(th);
  s.gy = pitch_rate;
  s.gz = yaw_rate * std::cos(th);
  nedMagToBody(heading_deg, pitch_deg, 0.0f, &s.mx, &s.my, &s.mz);
  return s;
}

static SimulatedSensors carVibeSensors(float heading_deg, int i, float yaw_rate = 0.0f,
                                       float ax = 0.0f, float ay = 0.0f) {
  SimulatedSensors s = levelSensors(heading_deg, yaw_rate, ax, ay);
  s.gx += 0.04f * std::sin((float)i * 0.7f);
  s.gy += 0.035f * std::cos((float)i * 0.55f);
  s.gz += 0.03f * std::sin((float)i * 0.4f);
  s.ax += 0.6f * std::sin((float)i * 0.9f);
  s.ay += 0.5f * std::cos((float)i * 0.8f);
  return s;
}

// ============================================================================
// Init / safety
// ============================================================================

static void testInitFirstSampleDtZero() {
  std::printf("\n=== Init: first iOS sample dt=0 ===\n");
  uNavINS filter;
  SimulatedSensors s = levelSensors(0.0f);
  filter.update(0.0, 0, 0, 0, 0, 0.9, 0.1, 20,
                s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.mx, s.my, s.mz,
                19000, -1000, 45000);
  expectTrue(filter.isInitialized(), "Initializes on dt=0 first sample");
  expectNearDeg(filter.getRoll_rad(), 0.0f, 5.0f, "Roll near zero at init");
  expectNearDeg(filter.getPitch_rad(), 0.0f, 5.0f, "Pitch near zero at init");
}

static void testInitRejectsBadAccel() {
  std::printf("\n=== Init: reject |ax| > g ===\n");
  uNavINS filter;
  SimulatedSensors s = levelSensors(0.0f);
  s.ax = 20.0f;
  filter.update(0.016, 0, 0, 0, 0, 0.9, 0.1, 20,
                s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.mx, s.my, s.mz,
                19000, -1000, 45000);
  expectTrue(!filter.isInitialized(), "Does not initialize on |ax| > g");
}

static void testInitZeroMag() {
  std::printf("\n=== Init: missing magnetometer ===\n");
  uNavINS filter;
  SimulatedSensors s = levelSensors(0.0f);
  filter.update(0.0, 16, 0, 0, 0, 0.9, 0.1, 20,
                s.gx, s.gy, s.gz, s.ax, s.ay, s.az, 0.0f, 0.0f, 0.0f,
                19000, -1000, 45000);
  expectTrue(filter.isInitialized(), "Initializes without mag");
  expectTrue(std::isfinite(filter.getHeading_rad()), "Heading finite without mag");
  expectTrue(std::isfinite(filter.getRoll_rad()) && std::isfinite(filter.getPitch_rad()),
             "Roll/pitch finite without mag");
}

static void testInitHeadingCardinals() {
  std::printf("\n=== Init: cardinal magnetic headings ===\n");
  const float headings[] = {0.0f, 90.0f, 180.0f, 270.0f};
  for (int i = 0; i < 4; i++) {
    float hdg = headings[i];
    uNavINS filter;
    SimulatedSensors s = levelSensors(hdg);
    filter.update(0.0, 0, 0, 0, 0, 0.9, 0.1, 20,
                  s.gx, s.gy, s.gz, s.ax, s.ay, s.az, s.mx, s.my, s.mz,
                  MAG_N_UT * 1000.0f, MAG_E_UT * 1000.0f, MAG_D_UT * 1000.0f);
    char msg[80];
    std::snprintf(msg, sizeof(msg), "Init heading at %.0f°", hdg);
    expectNearDeg(filter.getHeading_rad(), hdg, 8.0f, msg);
  }
}

static void testInitIndependentInstances() {
  std::printf("\n=== Init: independent instances ===\n");
  World a;
  a.set_course(0.0f, 0.0f);
  a.run_level(120);
  World b;
  b.set_course(0.0f, 0.0f);
  b.begin();
  expectTrue(b.filter.isInitialized(), "Second instance initializes independently");
  expectTrue(std::isfinite(b.filter.getHeading_rad()), "Second instance heading is finite");
}

// ============================================================================
// Static
// ============================================================================

static void testStaticDesk() {
  std::printf("\n=== Static: phone on a desk ===\n");
  World w;
  w.alt_m = 20.0;
  w.set_course(0.0f, 0.0f);
  const float gyro_bias = 0.02f;  // ~1.1 deg/s

  for (int i = 0; i < 300; i++) {  // 5 s
    SimulatedSensors s = levelSensors(0.0f);
    s.gx = gyro_bias + 0.001f * std::sin((float)i * 0.1f);
    s.gy = 0.001f * std::cos((float)i * 0.1f);
    s.gz = 0.001f * std::sin((float)i * 0.15f);
    w.step(s);
  }

  expectTrue(w.filter.isAtRest(), "Rest detection on a still desk");
  expectTrue(w.filter.isZuptActive(), "ZUPT after 3 s of GPS speed 0");
  expectTrue(w.speed_3d() < 0.5f, "Velocity stays near zero on a desk");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 5.0f, "Roll level on a desk");
  expectNearDeg(w.filter.getPitch_rad(), 0.0f, 5.0f, "Pitch level on a desk");
  expectTrue(w.filter.getHealthStatus() <= 1, "Health OK while static");
  expectNear(w.fpa_deg(), 0.0f, 1.0f, "Vertical FPA is zero when stationary");
  expectNear(w.hfpa_deg(), 0.0f, 1.0f, "Horizontal FPA is zero when stationary");
}

static void testStaticHandheld() {
  std::printf("\n=== Static: standing, phone in hand ===\n");
  World w;
  w.set_course(0.0f, 0.0f);

  bool zupt_at_2s = false;
  bool zupt_at_4s = false;
  for (int i = 0; i < 300; i++) {  // 5 s
    SimulatedSensors s = levelSensors(0.0f);
    s.gx += 0.03f * std::sin((float)i * 0.2f);
    s.gy += 0.02f * std::cos((float)i * 0.3f);
    s.gz += 0.01f * std::sin((float)i * 0.15f);
    s.ax += 0.8f * std::sin((float)i * 0.1f);
    s.ay += 0.6f * std::cos((float)i * 0.12f);
    w.step(s);
    if (i == 120) zupt_at_2s = w.filter.isZuptActive();
    if (i == 240) zupt_at_4s = w.filter.isZuptActive();
  }

  expectTrue(!zupt_at_2s, "ZUPT does not fire before 3 s");
  expectTrue(zupt_at_4s, "ZUPT fires while handheld and GPS says stopped");
  expectTrue(!w.filter.isAtRest(), "Hand tremor is not variance-based rest");
  expectTrue(w.speed_3d() < 1.0f, "Handheld ZUPT keeps speed small");
}

static void testStaticPickup() {
  std::printf("\n=== Static: pick up the phone ===\n");
  World w;
  w.set_course(0.0f, 0.0f);
  w.run_level(120);
  expectTrue(w.filter.isAtRest(), "At rest before pickup");

  for (int i = 0; i < 60; i++) {
    SimulatedSensors s = levelSensors(0.0f);
    s.gx = 0.15f;
    s.gy = 0.10f;
    s.gz = 0.05f;
    w.vn = 0.3;
    w.step(s);
  }
  expectTrue(!w.filter.isAtRest(), "Rest exits when the phone is picked up");
}

// ============================================================================
// Walking
// ============================================================================

static void testWalkingStraight() {
  std::printf("\n=== Walking: straight at pedestrian pace ===\n");
  World w;
  w.hacc = 6.0f;
  w.vacc = 10.0f;
  w.sacc = 0.8f;
  w.set_course(1.4f, 0.0f);

  for (int i = 0; i < 600; i++) {  // 10 s
    w.step(walkSensors(w.heading_deg, i));
  }

  expectTrue(!w.filter.isZuptActive(), "ZUPT stays off while walking");
  expectTrue(!w.filter.isAtRest(), "Walking is not rest");
  expectNear(w.speed(), 1.4f, 0.8f, "Ground speed tracks ~1.4 m/s");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 12.0f, "Roll roughly level while walking");
  expectNearDeg(w.filter.getPitch_rad(), 0.0f, 12.0f, "Pitch roughly level while walking");
  expectTrue(w.filter.getHealthStatus() <= 2, "Health not critical while walking");
  expectNearDeg(w.filter.getGroundTrack_rad(), 0.0f, 25.0f, "Ground track follows walking heading");
}

static void testWalkingCorner() {
  std::printf("\n=== Walking: 90° street corner ===\n");
  World w;
  w.set_course(1.4f, 0.0f);
  for (int i = 0; i < 180; i++) {
    w.step(walkSensors(w.heading_deg, i));
  }

  const float yaw_rate = 45.0f * DEG_TO_RAD;  // 90° in 2 s
  for (int i = 0; i < 120; i++) {
    w.heading_deg = wrap360(w.heading_deg + 45.0f * IMU_DT);
    w.set_course(1.4f, w.heading_deg);
    w.step(walkSensors(w.heading_deg, i, yaw_rate));
  }
  for (int i = 0; i < 180; i++) {
    w.step(walkSensors(w.heading_deg, i));
  }

  expectNearDeg(w.filter.getHeading_rad(), 90.0f, 25.0f, "Heading follows a walking turn");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 15.0f, "Roll stays near level around a corner");
  expectTrue(!w.filter.isZuptActive(), "ZUPT stays off around a walking corner");
}

static void testWalkingMagInterference() {
  std::printf("\n=== Walking: pass a metal object ===\n");
  World w;
  w.set_course(1.4f, 0.0f);
  for (int i = 0; i < 120; i++) {
    w.step(walkSensors(0.0f, i));
  }
  float heading_before = w.filter.getHeading_rad();

  for (int i = 0; i < 60; i++) {
    SimulatedSensors s = walkSensors(0.0f, i);
    s.mx *= 3.0f;
    s.my *= 3.0f;
    s.mz *= 3.0f;
    w.step(s);
  }
  for (int i = 0; i < 120; i++) {
    w.step(walkSensors(0.0f, i));
  }

  float delta = std::fabs(w.filter.getHeading_rad() - heading_before) * RAD_TO_DEG;
  if (delta > 180.0f) delta = 360.0f - delta;
  std::printf("  Heading change through 3x mag: %.2f°\n", delta);
  expectTrue(delta < 20.0f, "Heading stable through mag spike");
}

// ============================================================================
// Car
// ============================================================================

static void testCarCruise() {
  std::printf("\n=== Car: cruise at 20 m/s ===\n");
  World w;
  w.alt_m = 40.0;
  w.hacc = 4.0f;
  w.set_course(20.0f, 0.0f);
  for (int i = 0; i < 600; i++) {  // 10 s
    w.step(carVibeSensors(w.heading_deg, i));
  }
  expectNear(w.speed(), 20.0f, 2.0f, "Speed tracks 20 m/s cruise");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 8.0f, "Roll level in a car");
  expectNearDeg(w.filter.getPitch_rad(), 0.0f, 8.0f, "Pitch level in a car");
  expectTrue(!w.filter.isZuptActive(), "ZUPT off while driving");
  expectTrue(!w.filter.isAtRest(), "Engine vibration is not rest while moving");
  expectTrue(w.filter.getHealthStatus() <= 1, "Health OK in a car");
  expectNearDeg(w.filter.getGroundTrack_rad(), 0.0f, 15.0f, "Ground track northbound in a car");
  expectNear((float)w.filter.getVelNorth_ms(), 20.0f, 2.5f, "North velocity matches car cruise");
  expectNear((float)w.filter.getVelEast_ms(), 0.0f, 2.5f, "East velocity near zero northbound");
  expectNear(w.fpa_deg(), 0.0f, 4.0f, "Vertical FPA near zero on level road");
  expectNear(w.hfpa_deg(), 0.0f, 12.0f, "Horizontal FPA near zero when track matches heading");
}

static void testCarLevelTurn() {
  std::printf("\n=== Car: 90° turn, wheels on the ground ===\n");
  World w;
  const float speed = 15.0f;
  const float radius = 50.0f;
  const float yaw_rate = speed / radius;  // rad/s, no bank
  w.set_course(speed, 0.0f);
  for (int i = 0; i < 180; i++) {
    w.step(carVibeSensors(w.heading_deg, i));
  }

  float max_roll = 0.0f;
  int turn_iters = (int)((0.5f * (float)M_PI / yaw_rate) / IMU_DT);
  for (int i = 0; i < turn_iters; i++) {
    w.heading_deg = wrap360(w.heading_deg + yaw_rate * IMU_DT * RAD_TO_DEG);
    w.set_course(speed, w.heading_deg);
    float ay = speed * yaw_rate;  // centripetal in body y
    w.step(carVibeSensors(w.heading_deg, i, yaw_rate, 0.0f, ay));
    float roll = std::fabs(w.filter.getRoll_rad() * RAD_TO_DEG);
    if (roll > max_roll) max_roll = roll;
  }
  std::printf("  Max roll during level car turn: %.2f°\n", max_roll);
  expectTrue(max_roll < 15.0f, "Car turn does not produce an aircraft bank");

  for (int i = 0; i < 180; i++) {
    w.step(carVibeSensors(w.heading_deg, i));
  }
  expectNearDeg(w.filter.getHeading_rad(), 90.0f, 20.0f, "Heading follows a 90° car turn");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 10.0f, "Roll returns to level after car turn");
}

static void testCarStoplight() {
  std::printf("\n=== Car: stop at a light, then pull away ===\n");
  World w;
  w.set_course(12.0f, 0.0f);
  for (int i = 0; i < 120; i++) {
    w.step(carVibeSensors(w.heading_deg, i));
  }
  expectTrue(!w.filter.isZuptActive(), "ZUPT off at 12 m/s");

  for (int i = 0; i < 120; i++) {  // 2 s brake
    float speed = 12.0f * (1.0f - (float)i / 120.0f);
    w.set_course(speed, 0.0f);
    w.step(carVibeSensors(0.0f, i, 0.0f, -2.0f, 0.0f));
  }
  w.set_course(0.0f, 0.0f);
  bool zupt_at_2s = false;
  bool zupt_at_4s = false;
  for (int i = 0; i < 300; i++) {
    w.step(carVibeSensors(0.0f, i));
    if (i == 120) zupt_at_2s = w.filter.isZuptActive();
    if (i == 240) zupt_at_4s = w.filter.isZuptActive();
  }
  expectTrue(!zupt_at_2s, "ZUPT not yet at 2 s stopped");
  expectTrue(zupt_at_4s, "ZUPT on after 3 s at a stoplight");
  expectTrue(w.speed_3d() < 1.0f, "Speed near zero at a stoplight");

  w.set_course(8.0f, 0.0f);
  for (int i = 0; i < 120; i++) {
    w.step(carVibeSensors(0.0f, i, 0.0f, 1.5f, 0.0f));
  }
  expectTrue(!w.filter.isZuptActive(), "ZUPT clears when pulling away");
}

static void testCarUrbanGps() {
  std::printf("\n=== Car: urban canyon GPS ===\n");
  World good;
  good.hacc = 3.0f;
  good.vacc = 5.0f;
  good.sacc = 0.4f;
  good.set_course(15.0f, 0.0f);
  for (int i = 0; i < 300; i++) {
    good.step(carVibeSensors(0.0f, i));
  }

  World poor;
  poor.hacc = 20.0f;
  poor.vacc = 30.0f;
  poor.sacc = 2.5f;
  poor.set_course(15.0f, 0.0f);
  for (int i = 0; i < 300; i++) {
    poor.step(carVibeSensors(0.0f, i));
  }

  float std_good = good.filter.getPositionNorthStd_m();
  float std_poor = poor.filter.getPositionNorthStd_m();
  std::printf("  Pos std good GPS: %.2fm  poor GPS: %.2fm\n", std_good, std_poor);
  expectTrue(std_poor > std_good * 1.5f, "Poor GPS increases position uncertainty");
  expectNear(poor.speed(), 15.0f, 4.0f, "Speed still usable with 20 m hacc");
  expectNearDeg(poor.filter.getRoll_rad(), 0.0f, 10.0f, "Horizon holds with poor GPS");
}

static void testCarAccelerate() {
  std::printf("\n=== Car: acceleration ===\n");
  World w;
  const float a = 2.0f;
  float speed = 5.0f;
  w.set_course(speed, 0.0f);
  for (int i = 0; i < 60; i++) {
    w.step(carVibeSensors(0.0f, i));
  }
  for (int i = 0; i < 300; i++) {  // 5 s → 15 m/s
    speed += a * IMU_DT;
    w.set_course(speed, 0.0f);
    w.step(carVibeSensors(0.0f, i, 0.0f, a, 0.0f));
  }
  expectNear(w.speed(), 15.0f, 2.5f, "Speed tracks a 5→15 m/s pull");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 8.0f, "Roll stays level while accelerating");
  expectTrue(!w.filter.isZuptActive(), "ZUPT off while accelerating");
}

static void testCarDecelerate() {
  std::printf("\n=== Car: deceleration ===\n");
  World w;
  const float a = -2.0f;
  float speed = 20.0f;
  w.set_course(speed, 0.0f);
  for (int i = 0; i < 60; i++) {
    w.step(carVibeSensors(0.0f, i));
  }
  for (int i = 0; i < 300; i++) {  // 5 s → 10 m/s
    speed += a * IMU_DT;
    w.set_course(speed, 0.0f);
    w.step(carVibeSensors(0.0f, i, 0.0f, a, 0.0f));
  }
  expectNear(w.speed(), 10.0f, 2.5f, "Speed tracks a 20→10 m/s brake");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 8.0f, "Roll stays level while braking");
  expectTrue(!w.filter.isZuptActive(), "ZUPT off while still moving after a brake");
}

static void testCar360() {
  std::printf("\n=== Car: 360° level turn ===\n");
  World w;
  const float speed = 15.0f;
  const float radius = 50.0f;
  const float yaw_rate = speed / radius;
  w.set_course(speed, 0.0f);
  for (int i = 0; i < 120; i++) {
    w.step(carVibeSensors(w.heading_deg, i));
  }
  float max_roll = 0.0f;
  int turn_iters = (int)((2.0f * (float)M_PI / yaw_rate) / IMU_DT);
  for (int i = 0; i < turn_iters; i++) {
    w.heading_deg = wrap360(w.heading_deg + yaw_rate * IMU_DT * RAD_TO_DEG);
    w.set_course(speed, w.heading_deg);
    float ay = speed * yaw_rate;
    w.step(carVibeSensors(w.heading_deg, i, yaw_rate, 0.0f, ay));
    float roll = std::fabs(w.filter.getRoll_rad() * RAD_TO_DEG);
    if (roll > max_roll) max_roll = roll;
  }
  std::printf("  Max roll during 360° car turn: %.2f°  heading: %.1f°\n",
              max_roll, w.filter.getHeading_rad() * RAD_TO_DEG);
  expectTrue(max_roll < 15.0f, "360° car turn does not bank like an aircraft");
  expectNearDeg(w.filter.getHeading_rad(), 0.0f, 35.0f, "Heading returns after a 360° car turn");
  expectNear(w.speed(), speed, 3.0f, "Speed holds through a 360° car turn");
}

// ============================================================================
// Aircraft
// ============================================================================

static void testAircraftLevel() {
  std::printf("\n=== Aircraft: straight and level ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.hacc = 3.0f;
  w.vacc = 4.0f;
  w.sacc = 0.3f;
  w.set_course(50.0f, 0.0f);
  w.run_level(600);  // 10 s
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 3.0f, "Roll level in cruise");
  expectNearDeg(w.filter.getPitch_rad(), 0.0f, 3.0f, "Pitch level in cruise");
  expectNear(w.speed(), 50.0f, 2.0f, "Speed tracks 50 m/s");
  expectTrue(w.filter.getPositionNorthStd_m() < 10.0f, "Position uncertainty bounded");
  expectTrue(w.filter.getRollStd_rad() * RAD_TO_DEG < 5.0f, "Attitude uncertainty reasonable");
  expectTrue(w.filter.getHealthStatus() <= 1, "Health OK in cruise");
  expectNear(w.fpa_deg(), 0.0f, 3.0f, "Vertical FPA near zero in level cruise");
  expectNear(w.hfpa_deg(), 0.0f, 8.0f, "Horizontal FPA near zero when coordinated");
  expectNearDeg(w.filter.getGroundTrack_rad(), 0.0f, 10.0f, "Ground track northbound in cruise");
  expectNear((float)w.filter.getVelNorth_ms(), 50.0f, 2.5f, "North velocity matches airspeed");
  expectNear((float)w.filter.getVelEast_ms(), 0.0f, 2.5f, "East velocity near zero northbound");
  expectNear((float)w.filter.getVelDown_ms(), 0.0f, 1.5f, "Down velocity near zero in level flight");
  expectTrue(w.filter.getLatitude_rad() > 51.5 * DEG_TO_RAD,
             "Latitude increases when flying north");
  expectTrue(std::isfinite(w.filter.getGyroBiasX_rads()) &&
                 std::isfinite(w.filter.getAccelBiasX_mss()) &&
                 std::isfinite(w.filter.getMagBiasX_nT()),
             "Bias estimates are finite in cruise");
  expectTrue(w.filter.getVelocityNorthStd_ms() < 3.0f, "North velocity uncertainty bounded");
}

static void runAircraftRollIn(World &w, float target_bank, float roll_rate_deg_s,
                              float turn_rate_deg_s) {
  float bank = 0.0f;
  int n = (int)(std::fabs(target_bank) / roll_rate_deg_s / IMU_DT);
  if (n < 1) n = 1;
  float bank_step = target_bank / (float)n;
  float airspeed = std::sqrt((float)(w.vn * w.vn + w.ve * w.ve));
  for (int i = 0; i < n; i++) {
    bank += bank_step;
    float partial_turn = turn_rate_deg_s * (bank / target_bank);
    SimulatedSensors s;
    float bank_rad = bank * DEG_TO_RAD;
    float load = 1.0f / std::cos(bank_rad);
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = -load * G;
    s.gx = bank_step / IMU_DT * DEG_TO_RAD;
    s.gy = partial_turn * DEG_TO_RAD * std::sin(bank_rad);
    s.gz = partial_turn * DEG_TO_RAD * std::cos(bank_rad);
    w.heading_deg = wrap360(w.heading_deg + partial_turn * IMU_DT);
    nedMagToBody(w.heading_deg, 0.0f, bank, &s.mx, &s.my, &s.mz);
    w.set_course(airspeed, w.heading_deg);
    w.step(s);
  }
}

static void runAircraftTurn(World &w, int n, float bank_deg, float turn_rate_deg_s) {
  float airspeed = std::sqrt((float)(w.vn * w.vn + w.ve * w.ve));
  for (int i = 0; i < n; i++) {
    SimulatedSensors s = coordinatedTurnSensors(w.heading_deg, bank_deg, turn_rate_deg_s);
    w.heading_deg = wrap360(w.heading_deg + turn_rate_deg_s * IMU_DT);
    w.set_course(airspeed, w.heading_deg);
    w.step(s);
  }
}

static void runAircraftRollOut(World &w, float current_bank, float roll_rate_deg_s,
                               float turn_rate_deg_s) {
  int n = (int)(std::fabs(current_bank) / roll_rate_deg_s / IMU_DT);
  if (n < 1) n = 1;
  float bank_step = -current_bank / (float)n;
  float bank = current_bank;
  float airspeed = std::sqrt((float)(w.vn * w.vn + w.ve * w.ve));
  for (int i = 0; i < n; i++) {
    bank += bank_step;
    SimulatedSensors s;
    float bank_rad = bank * DEG_TO_RAD;
    float load = (std::fabs(bank) > 1.0f) ? (1.0f / std::cos(bank_rad)) : 1.0f;
    float frac = std::fabs(bank / current_bank);
    float decaying = turn_rate_deg_s * frac * DEG_TO_RAD;
    s.ax = 0.0f;
    s.ay = 0.0f;
    s.az = -load * G;
    s.gx = bank_step / IMU_DT * DEG_TO_RAD;
    s.gy = decaying * std::sin(bank_rad);
    s.gz = decaying * std::cos(bank_rad);
    w.heading_deg = wrap360(w.heading_deg + turn_rate_deg_s * frac * IMU_DT);
    nedMagToBody(w.heading_deg, 0.0f, bank, &s.mx, &s.my, &s.mz);
    w.set_course(airspeed, w.heading_deg);
    w.step(s);
  }
}

static void testAircraftCoordinatedTurn() {
  std::printf("\n=== Aircraft: coordinated 30° turn ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);

  const float bank = 30.0f;
  const float turn_rate = 6.0f;
  const float roll_rate = 15.0f;
  runAircraftRollIn(w, bank, roll_rate, turn_rate);
  runAircraftTurn(w, 300, bank, turn_rate);
  std::printf("  Roll during turn: %.2f°\n", w.filter.getRoll_rad() * RAD_TO_DEG);
  expectNearDeg(w.filter.getRoll_rad(), bank, 10.0f, "Roll tracks 30° bank");
  expectNear(w.hfpa_deg(), 0.0f, 12.0f, "Horizontal FPA near zero in a coordinated turn");

  runAircraftRollOut(w, bank, roll_rate, turn_rate);
  w.run_level(300);
  std::printf("  Roll after rollout: %.2f°\n", w.filter.getRoll_rad() * RAD_TO_DEG);
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 10.0f, "Roll returns to level after turn");
}

static void testAircraftHorizon360() {
  std::printf("\n=== Aircraft: 360° sustained turn ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);
  const float bank = 30.0f;
  const float turn_rate = 6.0f;
  runAircraftRollIn(w, bank, 15.0f, turn_rate);
  float heading_start = w.filter.getHeading_rad() * RAD_TO_DEG;
  float max_err = 0.0f;
  for (int seg = 0; seg < 6; seg++) {
    runAircraftTurn(w, 600, bank, turn_rate);
    float err = std::fabs(w.filter.getRoll_rad() * RAD_TO_DEG - bank);
    if (err > max_err) max_err = err;
    std::printf("  Segment %d roll error: %.2f°\n", seg + 1, err);
  }
  expectTrue(max_err < 30.0f, "Horizon stays within 30° over a 360° turn");
  expectNearDeg(w.filter.getHeading_rad(), heading_start, 35.0f,
                "Heading returns after a 360° aircraft turn");
}

static void testAircraftClimb() {
  std::printf("\n=== Aircraft: climb ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);

  const float pitch = 6.0f;
  const float vs = 5.0f;  // m/s up
  w.vd = -vs;
  for (int i = 0; i < 600; i++) {  // 10 s → +50 m
    w.step(pitchedSensors(0.0f, pitch));
  }
  std::printf("  Alt: %.1fm  pitch: %.2f°  vs: %.2f m/s\n",
              w.filter.getAltitude_m(), w.filter.getPitch_rad() * RAD_TO_DEG,
              (float)(-w.filter.getVelDown_ms()));
  expectNear((float)w.filter.getAltitude_m(), 1050.0f, 12.0f, "Altitude rises ~50 m in a climb");
  expectTrue(w.filter.getPitch_rad() * RAD_TO_DEG > 2.0f, "Pitch is nose-up in a climb");
  expectTrue((-w.filter.getVelDown_ms()) > 2.0f, "Vertical speed is positive in a climb");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 8.0f, "Wings level in a straight climb");
  // atan2(5, 50) ≈ 5.7°
  expectNear(w.fpa_deg(), 5.7f, 3.5f, "Vertical FPA is nose-up in a climb");
  expectTrue(w.fpa_deg() > 2.0f, "Vertical FPA sign is climb");
}

static void testAircraftDescent() {
  std::printf("\n=== Aircraft: descent ===\n");
  World w;
  w.alt_m = 1200.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);

  const float pitch = -5.0f;
  const float vs = -4.0f;  // m/s down
  w.vd = -vs;
  for (int i = 0; i < 600; i++) {  // 10 s → -40 m
    w.step(pitchedSensors(0.0f, pitch));
  }
  std::printf("  Alt: %.1fm  pitch: %.2f°  vs: %.2f m/s\n",
              w.filter.getAltitude_m(), w.filter.getPitch_rad() * RAD_TO_DEG,
              (float)(-w.filter.getVelDown_ms()));
  expectNear((float)w.filter.getAltitude_m(), 1160.0f, 12.0f, "Altitude drops ~40 m in a descent");
  expectTrue(w.filter.getPitch_rad() * RAD_TO_DEG < -1.5f, "Pitch is nose-down in a descent");
  expectTrue((-w.filter.getVelDown_ms()) < -1.5f, "Vertical speed is negative in a descent");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 8.0f, "Wings level in a straight descent");
  // atan2(-4, 50) ≈ -4.6°
  expectNear(w.fpa_deg(), -4.6f, 3.5f, "Vertical FPA is nose-down in a descent");
  expectTrue(w.fpa_deg() < -1.5f, "Vertical FPA sign is descent");
}

static void testAircraftAccelerate() {
  std::printf("\n=== Aircraft: acceleration ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  float speed = 40.0f;
  w.set_course(speed, 0.0f);
  w.run_level(60);
  const float a = 1.5f;
  for (int i = 0; i < 300; i++) {  // 5 s → 47.5 m/s
    speed += a * IMU_DT;
    w.set_course(speed, 0.0f);
    w.step(levelSensors(0.0f, 0.0f, a, 0.0f));
  }
  expectNear(w.speed(), 47.5f, 3.0f, "Speed tracks a 40→47.5 m/s acceleration");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 6.0f, "Wings level while accelerating");
}

static void testAircraftDecelerate() {
  std::printf("\n=== Aircraft: deceleration ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  float speed = 55.0f;
  w.set_course(speed, 0.0f);
  w.run_level(60);
  const float a = -1.5f;
  for (int i = 0; i < 300; i++) {  // 5 s → 47.5 m/s
    speed += a * IMU_DT;
    w.set_course(speed, 0.0f);
    w.step(levelSensors(0.0f, 0.0f, a, 0.0f));
  }
  expectNear(w.speed(), 47.5f, 3.0f, "Speed tracks a 55→47.5 m/s deceleration");
  expectNearDeg(w.filter.getRoll_rad(), 0.0f, 6.0f, "Wings level while decelerating");
}

static void testAircraftClimbingTurn() {
  std::printf("\n=== Aircraft: climbing turn ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);
  double alt0 = w.filter.getAltitude_m();
  const float bank = 25.0f;
  const float turn_rate = 6.0f;
  w.vd = -3.0;  // 3 m/s climb
  runAircraftRollIn(w, bank, 15.0f, turn_rate);
  runAircraftTurn(w, 300, bank, turn_rate);  // 5 s, +15 m, ~30° heading
  std::printf("  Roll: %.1f°  heading: %.1f°  alt: %.1fm  vs: %.2f\n",
              w.filter.getRoll_rad() * RAD_TO_DEG, w.filter.getHeading_rad() * RAD_TO_DEG,
              w.filter.getAltitude_m(), (float)(-w.filter.getVelDown_ms()));
  expectNearDeg(w.filter.getRoll_rad(), bank, 12.0f, "Roll tracks bank in a climbing turn");
  expectTrue(w.filter.getAltitude_m() > alt0 + 8.0, "Altitude increases in a climbing turn");
  expectTrue((-w.filter.getVelDown_ms()) > 1.0f, "Climbing turn has positive vertical speed");
}

static void testAircraftDescendingTurn() {
  std::printf("\n=== Aircraft: descending turn ===\n");
  World w;
  w.alt_m = 1200.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);
  double alt0 = w.filter.getAltitude_m();
  const float bank = 25.0f;
  const float turn_rate = 6.0f;
  w.vd = 3.0;  // 3 m/s descent
  runAircraftRollIn(w, bank, 15.0f, turn_rate);
  runAircraftTurn(w, 300, bank, turn_rate);
  std::printf("  Roll: %.1f°  heading: %.1f°  alt: %.1fm  vs: %.2f\n",
              w.filter.getRoll_rad() * RAD_TO_DEG, w.filter.getHeading_rad() * RAD_TO_DEG,
              w.filter.getAltitude_m(), (float)(-w.filter.getVelDown_ms()));
  expectNearDeg(w.filter.getRoll_rad(), bank, 12.0f, "Roll tracks bank in a descending turn");
  expectTrue(w.filter.getAltitude_m() < alt0 - 8.0, "Altitude decreases in a descending turn");
  expectTrue((-w.filter.getVelDown_ms()) < -1.0f, "Descending turn has negative vertical speed");
}

static void testAircraftClimbBaro() {
  std::printf("\n=== Aircraft: climb with poor GPS vertical ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.hacc = 3.0f;
  w.vacc = 25.0f;
  w.sacc = 0.5f;
  w.set_course(50.0f, 0.0f);
  w.vd = -1.0;  // NED down negative = climb

  for (int i = 0; i < 600; i++) {
    double t = (double)i * IMU_DT;
    double true_alt = 1000.0 + t * 1.0;
    w.alt_m = true_alt;
    w.baro_hpa = 1013.25f * std::exp((float)(-(true_alt + 0.5 * std::sin((float)i * 0.1f)) / 8430.0));
    SimulatedSensors s = levelSensors(0.0f);
    w.step(s);
  }
  double expected = 1010.0;
  std::printf("  Filter alt: %.1fm expected ~%.1fm\n", w.filter.getAltitude_m(), expected);
  expectNear((float)w.filter.getAltitude_m(), (float)expected, 8.0f,
             "Baro holds altitude in a climb when vacc is poor");

  World good;
  good.alt_m = 1000.0;
  good.set_gps_hz(5);
  good.hacc = 3.0f;
  good.vacc = 5.0f;
  good.set_course(50.0f, 0.0f);
  good.baro_hpa = 1013.25f * std::exp(-1000.0f / 8430.0f);
  good.run_level(300);
  expectNear((float)good.filter.getAltitude_m(), 1000.0f, 3.0f,
             "Baro does not pull altitude when vacc is good");
}

static void testAircraftGpsOutage() {
  std::printf("\n=== Aircraft: 5 s GPS outage ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(120);
  float roll_before = w.filter.getRoll_rad();
  unsigned long frozen = w.tow_ms;
  for (int i = 0; i < 300; i++) {
    w.lat_rad += w.vn * (double)IMU_DT / EARTH_R;
    w.lon_rad += w.ve * (double)IMU_DT / (EARTH_R * std::cos(w.lat_rad));
    SimulatedSensors s = levelSensors(0.0f);
    w.filter.update((double)IMU_DT, frozen, w.gps_vn, w.gps_ve, w.gps_vd,
                    w.gps_lat, w.gps_lon, w.gps_alt, s.gx, s.gy, s.gz, s.ax, s.ay, s.az,
                    s.mx, s.my, s.mz, w.bn, w.be, w.bd, w.hacc, w.vacc, w.sacc);
  }
  expectNearDeg(w.filter.getRoll_rad(), roll_before * RAD_TO_DEG, 4.0f,
                "Roll stable during GPS outage");
  expectTrue(w.filter.getPositionNorthStd_m() > 3.0f, "Position uncertainty grows without GPS");
}

static void testAircraftMagIgnoredWhenBanked() {
  std::printf("\n=== Aircraft: mag ignored when banked ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.set_course(50.0f, 0.0f);
  w.run_level(60);
  float heading_before = w.filter.getHeading_rad();
  for (int i = 0; i < 180; i++) {
    SimulatedSensors s = coordinatedTurnSensors(0.0f, 35.0f, 0.0f);
    s.mx = 0.0f;
    s.my = 20.0f;
    s.mz = 40.0f;
    w.step(s);
  }
  float delta = std::fabs(w.filter.getHeading_rad() - heading_before) * RAD_TO_DEG;
  if (delta > 180.0f) delta = 360.0f - delta;
  std::printf("  Heading change while banked with bad mag: %.2f°\n", delta);
  expectTrue(delta < 20.0f, "Banked mag error does not yank heading");
}

static void testAircraftCrab() {
  std::printf("\n=== Aircraft: crab / horizontal FPA ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  w.heading_deg = 0.0f;
  w.set_velocity_track(40.0f, 15.0f);  // heading north, track 15°
  w.run_level(600);
  std::printf("  HFPA: %.2f°  track: %.2f°  heading: %.2f°\n",
              w.hfpa_deg(), w.track_deg(), w.filter.getHeading_rad() * RAD_TO_DEG);
  expectNear(w.hfpa_deg(), 15.0f, 8.0f, "Horizontal FPA is +crab (drifting right)");
  expectTrue(w.hfpa_deg() > 5.0f, "Positive crab has positive horizontal FPA");
  expectNearDeg(w.filter.getGroundTrack_rad(), 15.0f, 10.0f, "Ground track follows velocity, not heading");
  expectNearDeg(w.filter.getHeading_rad(), 0.0f, 20.0f, "Heading stays near nose-north during crab");

  World left;
  left.alt_m = 1000.0;
  left.set_gps_hz(5);
  left.heading_deg = 0.0f;
  left.set_velocity_track(40.0f, 345.0f);  // -15°
  left.run_level(600);
  std::printf("  Left HFPA: %.2f°\n", left.hfpa_deg());
  expectNear(left.hfpa_deg(), -15.0f, 8.0f, "Horizontal FPA is -crab (drifting left)");
  expectTrue(left.hfpa_deg() < -5.0f, "Negative crab has negative horizontal FPA");
}

static void testAircraftEastbound() {
  std::printf("\n=== Aircraft: eastbound track and longitude ===\n");
  World w;
  w.alt_m = 1000.0;
  w.set_gps_hz(5);
  double lon0 = w.lon_rad;
  w.set_course(50.0f, 90.0f);
  w.run_level(600);
  expectNearDeg(w.filter.getGroundTrack_rad(), 90.0f, 12.0f, "Ground track eastbound");
  expectNearDeg(w.filter.getHeading_rad(), 90.0f, 15.0f, "Heading eastbound");
  expectNear((float)w.filter.getVelEast_ms(), 50.0f, 3.0f, "East velocity matches airspeed");
  expectNear((float)w.filter.getVelNorth_ms(), 0.0f, 3.0f, "North velocity near zero eastbound");
  expectTrue(w.filter.getLongitude_rad() > lon0, "Longitude increases when flying east");
  expectNear(w.hfpa_deg(), 0.0f, 10.0f, "Horizontal FPA near zero when heading matches track");
}

int main() {
  std::printf("uNavINS use-case tests\n");
  std::printf("==========================================\n");

  testInitFirstSampleDtZero();
  testInitRejectsBadAccel();
  testInitZeroMag();
  testInitHeadingCardinals();
  testInitIndependentInstances();

  testStaticDesk();
  testStaticHandheld();
  testStaticPickup();

  testWalkingStraight();
  testWalkingCorner();
  testWalkingMagInterference();

  testCarCruise();
  testCarAccelerate();
  testCarDecelerate();
  testCarLevelTurn();
  testCar360();
  testCarStoplight();
  testCarUrbanGps();

  testAircraftLevel();
  testAircraftAccelerate();
  testAircraftDecelerate();
  testAircraftClimb();
  testAircraftDescent();
  testAircraftCrab();
  testAircraftEastbound();
  testAircraftCoordinatedTurn();
  testAircraftClimbingTurn();
  testAircraftDescendingTurn();
  testAircraftHorizon360();
  testAircraftClimbBaro();
  testAircraftGpsOutage();
  testAircraftMagIgnoredWhenBanked();

  std::printf("\n==========================================\n");
  std::printf("uNavINS: %d passed, %d failed — %s\n",
              g_passed, g_failed, g_failed == 0 ? "PASS ✓" : "FAIL ✗");
  return g_failed == 0 ? 0 : 1;
}
