# fusionml Tests

Standalone C++ tests for `fusionml` (uNavINS, AltitudeCalculator, JsonRecorder, FlightPhaseDetector, XYZgeomag).

## Run

From the repo root:

```bash
yarn test:fusion
# or
python3 fusionml/tests/run_fusion_tests.py
```

Options:

- `--list` — list test names and exit
- `--build-only` — build all tests but do not run them
- `-v`, `--verbose` — print compiler and runner commands

Requires `g++` or `clang++` and zlib development headers.

## uNavINS (`test_unavins.cpp`)

The filter is driven like iOS: 60 Hz IMU, GPS frozen between Core Location fixes (1 Hz on the ground, 5 Hz in the air), first sample `dt = 0`. Specific force is z-down (`az = -G` when level).

| Suite | What it covers |
| --- | --- |
| Init / safety | `dt = 0` first sample, reject `|ax| > g`, missing mag, independent instances |
| Static | Desk rest + ZUPT, handheld still (tremor, no rest), pickup |
| Walking | 1.4 m/s pedestrian, 90° corner, mag spike |
| Car | 20 m/s cruise, level 90° turn (must not bank), stoplight ZUPT, urban GPS |
| Aircraft | Cruise, coordinated turn, 360° horizon, baro climb, GPS outage, mag gated when banked |
