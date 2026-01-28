# fusionml Tests

Standalone C++ unit tests for the `fusionml` components (AltitudeCalculator, JsonRecorder, FlightPhaseDetector, XYZgeomag).

## Run via Python wrapper (recommended)

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

Requires `clang++` or `g++` and zlib development headers (e.g. `brew install zlib` on macOS).

## Manual build and run (macOS/Linux)

From the repo root:

```bash
clang++ -std=c++14 -Ifusionml/src fusionml/tests/test_altitude_calculator.cpp \
  fusionml/src/AltitudeCalculator.cpp -o /tmp/test_altitude && /tmp/test_altitude

clang++ -std=c++14 -Ifusionml/src fusionml/tests/test_json_recorder.cpp \
  fusionml/src/JsonRecorder.cpp -lz -o /tmp/test_json && /tmp/test_json

clang++ -std=c++14 -Ifusionml/src fusionml/tests/test_flight_phase_detector.cpp \
  fusionml/src/FlightPhaseDetector.cpp -o /tmp/test_flight_phase && /tmp/test_flight_phase

clang++ -std=c++14 -Ifusionml/src fusionml/tests/test_xyzgeomag.cpp \
  -o /tmp/test_xyzgeomag && /tmp/test_xyzgeomag
```
